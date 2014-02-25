/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2013 by Boris Houska, Hans Joachim Ferreau,
 *    Milan Vukov, Rien Quirynen, KU Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC)
 *    under supervision of Moritz Diehl. All rights reserved.
 *
 *    ACADO Toolkit is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with ACADO Toolkit; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */



 /**
 *    \file examples/function/symmetricAD.cpp
 *    \author Boris Houska, Rien Quirynen
 *    \date 2014
 */

#include <time.h>

#include <acado/utils/acado_utils.hpp>
#include <acado/user_interaction/user_interaction.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/function/function.hpp>
#include <acado/code_generation/export_algorithm_factory.hpp>

USING_NAMESPACE_ACADO

// ---------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------
// SIMPLE AUXILIARY FUNCTION:
Expression symmetricDoubleProduct( const Expression& expr, const Expression& arg ) {

	uint dim = arg.getNumCols();
	uint dim2 = arg.getNumRows();

	IntermediateState inter_res = zeros<double>(dim2,dim);
	for( uint i = 0; i < dim; i++ ) {
		for( uint k1 = 0; k1 < dim2; k1++ ) {
			for( uint k2 = 0; k2 <= k1; k2++ ) {
				inter_res(k1,i) += expr(k1,k2)*arg(k2,i);
			}
			for( uint k2 = k1+1; k2 < dim2; k2++ ) {
				inter_res(k1,i) += expr(k2,k1)*arg(k2,i);
			}
		}
	}

	Expression new_expr( "", dim, dim );
	for( uint i = 0; i < dim; i++ ) {
		for( uint j = 0; j <= i; j++ ) {
			Expression new_tmp = 0;
			for( uint k1 = 0; k1 < dim2; k1++ ) {
				new_tmp = new_tmp+arg(k1,i)*inter_res(k1,j);
			}
			new_expr(i,j) = new_tmp;
			new_expr(j,i) = new_tmp;
		}
	}
	return new_expr;
}

Expression returnLowerTriangular( const Expression& expr ) {
	ASSERT( expr.getNumRows() == expr.getNumCols() );

	Expression new_expr;
	for( uint i = 0; i < expr.getNumRows(); i++ ) {
		for( uint j = 0; j <= i; j++ ) {
			new_expr << expr(i,j);
		}
	}
	return new_expr;
}
// ---------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------


/* >>> start tutorial code >>> */
int main( ){
    
    DifferentialState   xT;     // the trolley position
    DifferentialState   vT;     // the trolley velocity
    IntermediateState   aT;     // the trolley acceleration
    DifferentialState   xL;     // the cable length
    DifferentialState   vL;     // the cable velocity
    IntermediateState   aL;     // the cable acceleration
    DifferentialState   phi;    // the excitation angle
    DifferentialState   omega;  // the angular velocity
        
    DifferentialState   uT;     // trolley velocity control
    DifferentialState   uL;     // cable velocity control

    Control             duT;
    Control             duL;

    //
    // DEFINE THE PARAMETERS:
    //
    const double      tau1 = 0.012790605943772;
    const double      a1   = 0.047418203070092;
    const double      tau2 = 0.024695192379264;
    const double      a2   = 0.034087337273386;
    const double      g = 9.81;       		
    const double      c = 0.2;        		
    const double      m = 1318.0;     		

    //
    // DEFINE THE MODEL EQUATIONS:
    //
    DifferentialEquation   f, f2, test_expr;
    ExportAcadoFunction    fun, fun2;
    
    aT = -1.0 / tau1 * vT + a1 / tau1 * uT;
    aL = -1.0 / tau2 * vL + a2 / tau2 * uL;

    Expression states;
    states << xT;
    states << vT;
    states << xL;
    states << vL;
    states << phi;
    states << omega;
    states << uT;
    states << uL;
    
    Expression controls;
    controls << duT;
    controls << duL;
    
    Expression arg;
    arg << states;
    arg << controls;
    
    int NX = states.getDim();
    int NU = controls.getDim();
    
    IntermediateState expr(2);
    
    expr(0) = - 1.0/xL*(-g*sin(phi)-aT*cos(phi)-2*vL*omega-c*omega/(m*xL)) + log(duT/duL)*pow(xL,2);
    expr(1) = - 1.0/xL*(-g*tan(phi)-aT*acos(phi)-2*atan(vL)*omega-c*asin(omega)/exp(xL)) + duT/pow(omega,3);
    //~ expr(0) = - 1.0/xL*(-g*sin(phi));
    //~ expr(1) = duT/pow(omega,3);
						
	DifferentialState lambda("", expr.getDim(),1);
	DifferentialState Sx("", states.getDim(),states.getDim());
	DifferentialState Su("", states.getDim(),controls.getDim());
	Expression S = Sx;
	S.appendCols(Su);
	
	// SYMMETRIC DERIVATIVES
	Expression S_tmp = S;
	S_tmp.appendRows(zeros<double>(NU,NX).appendCols(eye<double>(NU)));
    
	IntermediateState dfS,dl;
	
    Expression f_tmp = symmetricDerivative( expr, arg, S_tmp, lambda, &dfS, &dl );
    f << returnLowerTriangular( f_tmp );
    
    fun.init(f, "symmetricDerivative", NX*(1+NX+NU)+expr.getDim(), 0, 2, 0, 0, 0);
    
	// ALTERNATIVE DERIVATIVES
    IntermediateState tmp = backwardDerivative( expr, states, lambda );
    IntermediateState tmp2 = forwardDerivative( tmp, states );
    Expression tmp3 = backwardDerivative( expr, controls, lambda );
	Expression tmp4 = multipleForwardDerivative( tmp3, states, Su );
	Expression tmp5 = tmp4 + tmp4.transpose() + forwardDerivative( tmp3, controls );
		
	Expression f2_tmp1;
	f2_tmp1 = symmetricDoubleProduct( tmp2, Sx );  f2_tmp1.appendCols( zeros<double>(NX,NU) );
    
    Expression f2_tmp2;
    f2_tmp2 = Su.transpose()*tmp2*Sx + multipleForwardDerivative( tmp3, states, Sx );
    f2_tmp2.appendCols( symmetricDoubleProduct( tmp2, Su ) + tmp5 );
    
    f2_tmp1.appendRows( f2_tmp2 );
    f2 << returnLowerTriangular( f2_tmp1 );
    
    
    fun2.init(f2, "alternativeSymmetric", NX*(1+NX+NU)+expr.getDim(), 0, 2, 0, 0, 0);
    
    Function f1;
    
    f1 << dfS;
    f1 << dl;
    
    std::ofstream stream2( "ADtest/ADsymbolic_output2.c" );
    stream2 << f1;
    stream2.close();
    
    std::ofstream stream( "ADtest/ADsymbolic_output.c" );
    fun.exportCode( stream, "double" );
    fun2.exportCode( stream, "double" );
    
    test_expr << expr;
    stream << test_expr;
    
    stream.close();

    return 0;
}
/* <<< end tutorial code <<< */

