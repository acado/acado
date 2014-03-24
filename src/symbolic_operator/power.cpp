/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2014 by Boris Houska, Hans Joachim Ferreau,
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
 *    \file src/symbolic_operator/power.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */


#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>



BEGIN_NAMESPACE_ACADO




Power::Power( ):BinaryOperator( ){ }

Power::Power( const SharedOperator &_argument1, const SharedOperator &_argument2 )
      :BinaryOperator( _argument1, _argument2 ){ }

Power::Power( const Power &arg ):BinaryOperator( arg ){ }

Power::~Power(){ }

returnValue Power::evaluate( EvaluationBase *x ){

    x->power(*a1,*a2);
    return SUCCESSFUL_RETURN;
}


SharedOperator Power::substitute( SharedOperatorMap &sub ){

    return SharedOperator( new Power( a1->substitute(sub),
                                      a2->substitute(sub) ) );
}


std::ostream& Power::print( std::ostream &stream, StringMap &name ) const{

	if ( acadoIsEqual( a2->getValue(),0.5 ) == BT_TRUE )
	{
		stream << "(sqrt(";
		a1->print(stream,name);
		stream << "))"; 
		return  stream;
	}
	else
	{
		if ( acadoIsEqual( a2->getValue(),-0.5 ) == BT_TRUE )
		{
			stream << "(1.0/sqrt(";
			a1->print(stream,name);
			stream << "))"; 
			return  stream;
		}
		else
		{
			stream << "(pow(";
			a1->print(stream,name);
			stream << ",";
			a2->print(stream,name);
			stream << "))";
			return  stream;
		}
	}
}


returnValue Power::initDerivative() {

    if( d1 != 0 ) { return SUCCESSFUL_RETURN; }

    SharedOperator oneTmp  = SharedOperator( new DoubleConstant(1.0, NE_ONE));  // 1
    SharedOperator yminone = mySubtract( a2, oneTmp );                          // y-1

    SharedOperator xpowy       = convert2TreeProjection( myPower(a1,a2)     );  // x^y
    SharedOperator xpowyminone = convert2TreeProjection( myPower(a1,yminone));  // x^{y-1}
    SharedOperator logx        = convert2TreeProjection( myLogarithm(a1)    );  // log(x)

    d1 = convert2TreeProjection(myProd(xpowyminone,a2));                        // x^{y-1}*y
    d2 = convert2TreeProjection(myProd(logx,xpowy));                            // log(x)*x^y 

    SharedOperator twoTmp       = SharedOperator( new DoubleConstant(2.0,NE_NEITHER_ONE_NOR_ZERO)); // 2
    SharedOperator subTmp2      = mySubtract( a2, twoTmp );                                         // y-2
    SharedOperator prodTmp      = myProd( a2, yminone );                                            // y*(y-1)
    SharedOperator prodTmp2     = myProd( a2, logx );                                               // y*log(x)
    SharedOperator oneplusylogx = myAdd( oneTmp, prodTmp2 );                                        // 1+y*log(x)
    SharedOperator xpowymintwo  = myPower( a1, subTmp2);                                            // x^(y-2)

    d11 = convert2TreeProjection( myProd( prodTmp, xpowymintwo      ) );                            // y*(y-1)*x^(y-2)
    d12 = convert2TreeProjection( myProd( oneplusylogx, xpowyminone ) );                            // (1+y*log(x))*x^(y-1)
    d22 = convert2TreeProjection( myProd(logx,d2) );                                                // log(x)^2*x^y
    
    a1->initDerivative();
    return a2->initDerivative();
}


CLOSE_NAMESPACE_ACADO

// end of file.
