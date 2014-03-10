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
      :BinaryOperator( _argument1, _argument2 ){

}

Power::Power( const Power &arg ):BinaryOperator( arg ){

    derivative01 = arg.derivative01;
    derivative02 = arg.derivative02;
    derivative12 = arg.derivative12;
    derivative21 = arg.derivative21;
    derivative22 = arg.derivative22;
    derivative23 = arg.derivative23;
}


Power::~Power(){

}


Power& Power::operator=( const Power &arg ){

    if( this != &arg ){

        BinaryOperator::operator=( arg );
    }
    return *this;
}


returnValue Power::evaluate( int number, double *x, double *result ){

    if( number >= bufferSize ){
        bufferSize += number;
        argument1_result  = (double*)realloc( argument1_result,bufferSize*sizeof(double));
        argument2_result  = (double*)realloc( argument2_result,bufferSize*sizeof(double));
        dargument1_result = (double*)realloc(dargument1_result,bufferSize*sizeof(double));
        dargument2_result = (double*)realloc(dargument2_result,bufferSize*sizeof(double));
    }

    argument1->evaluate( number, x , &argument1_result[number] );
    argument2->evaluate( number, x , &argument2_result[number] );

    result[0] = pow( argument1_result[number], argument2_result[number] );

    return SUCCESSFUL_RETURN;
}


returnValue Power::evaluate( EvaluationBase *x ){

    x->power(*argument1,*argument2);
    return SUCCESSFUL_RETURN;
}


SharedOperator Power::differentiate( int index ){

	dargument1 = argument1->differentiate( index );
	dargument2 = argument2->differentiate( index );

	SharedOperator prodTmp1 = myProd( SharedOperator( new Power(*this) ), derivative02);
	SharedOperator prodTmp2 = myProd( dargument2, prodTmp1 );
	SharedOperator prodTmp4 = myProd( dargument1, derivative12 );

	SharedOperator result = myAdd( prodTmp2, prodTmp4 );

	return result;

}


SharedOperator Power::AD_forward( int dim,
                               VariableType *varType,
                               int *component,
                               SharedOperator *seed,
                               std::vector<SharedOperator> &newIS ){

    dargument1 = argument1->AD_forward(dim,varType,component,seed,newIS);
    dargument2 = argument2->AD_forward(dim,varType,component,seed,newIS);

    SharedOperator prodTmp1 = myProd( SharedOperator( new Power(*this) ), derivative02);
    SharedOperator prodTmp2 = myProd( dargument2, prodTmp1 );
    SharedOperator prodTmp4 = myProd( dargument1, derivative12 );

    SharedOperator result = myAdd( prodTmp2, prodTmp4 );

    return result;
}


returnValue Power::AD_backward( int           dim      , /**< number of directions  */
                                        VariableType *varType  , /**< the variable types    */
                                        int          *component, /**< and their components  */
                                        SharedOperator  &seed     , /**< the backward seed     */
                                        SharedOperator    *df       , /**< the result            */
                                        std::vector<SharedOperator> &newIS  /**< the new IS-pointer    */ ){

	if( seed->isOneOrZero() != NE_ZERO ){

		SharedOperator tmp = convert2TreeProjection(seed);
		SharedOperator prodTmp4 = myProd( tmp, derivative12 );

		argument1->AD_backward( dim, varType, component, prodTmp4, df, newIS );

		SharedOperator prodTmp1 = myProd( SharedOperator(new Power(*this)), derivative02);
		SharedOperator prodTmp2 = myProd( tmp, prodTmp1 );

		argument2->AD_backward( dim, varType, component, prodTmp2, df, newIS );
	}
    return SUCCESSFUL_RETURN;
}


returnValue Power::AD_symmetric( int            dim       , /**< number of directions  */
                                        VariableType  *varType   , /**< the variable types    */
                                        int           *component , /**< and their components  */
                                 SharedOperator      &l         , /**< the backward seed     */
                                 SharedOperator     *S         , /**< forward seed matrix   */
                                        int            dimS      , /**< dimension of forward seed             */
                                  SharedOperator     *dfS       , /**< first order foward result             */
                                  SharedOperator     *ldf       , /**< first order backward result           */
                                  SharedOperator     *H         , /**< upper trianglular part of the Hessian */
                                      std::vector<SharedOperator> &newLIS , /**< the new LIS-pointer   */
                                      std::vector<SharedOperator> &newSIS , /**< the new SIS-pointer   */
                                      std::vector<SharedOperator> &newHIS   /**< the new HIS-pointer   */ ){
    
    SharedOperator dx = convert2TreeProjection(derivative12);
    SharedOperator dy = SharedOperator( new Product( SharedOperator( new Power(*this) ), derivative02));
    SharedOperator dxx = convert2TreeProjection( derivative22 );
    SharedOperator dxy = convert2TreeProjection( derivative23 );
    SharedOperator dyy = SharedOperator( new Product( dy, derivative02 ));
    
    return ADsymCommon2( argument1,argument2,dx,dy,dxx,dxy,dyy, dim, varType, component, l, S, dimS, dfS,
			  ldf, H, newLIS, newSIS, newHIS );
}


returnValue Power::initDerivative() {

	if( derivative01 != 0 ) {
		return SUCCESSFUL_RETURN;
	}

	SharedOperator oneTmp = SharedOperator( new DoubleConstant(1.0, NE_ONE));
	SharedOperator subTmp = mySubtract( argument2, oneTmp );

	derivative01 = convert2TreeProjection( myPower( argument1, subTmp) );
	derivative02 = convert2TreeProjection( myLogarithm( argument1 )    );

	derivative12 = convert2TreeProjection(myProd( derivative01, argument2 ));

	SharedOperator twoTmp = SharedOperator( new DoubleConstant(2.0,NE_NEITHER_ONE_NOR_ZERO));
	SharedOperator subTmp2 = mySubtract( argument2, twoTmp );
	SharedOperator prodTmp = myProd( argument2, subTmp );
	SharedOperator prodTmp2 = myProd( argument2, derivative02 );
	SharedOperator addTmp = myAdd( oneTmp, prodTmp2 );

	derivative21 = convert2TreeProjection(myPower( argument1, subTmp2));
	derivative22 = convert2TreeProjection(myProd( prodTmp, derivative21 ));
	derivative23 = convert2TreeProjection(myProd( derivative01, addTmp ));

	argument1->initDerivative();
	return argument2->initDerivative();
}


SharedOperator Power::substitute( int index, const SharedOperator &sub ){

    return SharedOperator( new Power( argument1->substitute( index , sub ),
                                      argument2->substitute( index , sub ) ) );
}


BooleanType Power::isLinearIn( int dim,
                                 VariableType *varType,
                                 int *component,
                                 BooleanType   *implicit_dep ){

    if( argument2->isOneOrZero() == NE_ZERO ){
        return BT_TRUE;
    }

    if( argument1->isOneOrZero() == NE_ONE ){
        return BT_TRUE;
    }

    if( argument1->isOneOrZero() == NE_ZERO ){
        return BT_TRUE;
    }

    if( argument1->isLinearIn( dim, varType, component, implicit_dep ) == BT_TRUE &&
        argument2->isOneOrZero() == NE_ONE ){
        return BT_TRUE;
    }

    if( isDependingOn( dim, varType, component, implicit_dep) == BT_FALSE ){
        return BT_TRUE;
    }

    return BT_FALSE;
}


BooleanType Power::isPolynomialIn( int dim,
                                     VariableType *varType,
                                     int *component,
                                     BooleanType   *implicit_dep ){

    if(  argument1->isPolynomialIn( dim, varType, component, implicit_dep ) == BT_TRUE  &&
         argument2->isDependingOn( dim, varType, component, implicit_dep )  == BT_FALSE ){
        return BT_TRUE;
    }

    return BT_FALSE;
}


BooleanType Power::isRationalIn( int dim,
                                   VariableType *varType,
                                   int *component,
                                   BooleanType   *implicit_dep ){

    if(  argument1->isRationalIn( dim, varType, component, implicit_dep )   == BT_TRUE  &&
         argument2->isDependingOn( dim, varType, component, implicit_dep )  == BT_FALSE ){
        return BT_TRUE;
    }

    return BT_FALSE;
}


MonotonicityType Power::getMonotonicity( ){

    if( monotonicity != MT_UNKNOWN )  return monotonicity;

    const MonotonicityType m1 = argument1->getMonotonicity();
    const MonotonicityType m2 = argument2->getMonotonicity();

    if( m1 == MT_CONSTANT ){

        if( m2 == MT_CONSTANT      )  return MT_CONSTANT;

        double res;
        argument1->evaluate(0,0,&res);

        if( res >= 1.0 ) return m2;

        if( m2 == MT_NONDECREASING )  return MT_NONINCREASING;
        if( m2 == MT_NONINCREASING )  return MT_NONDECREASING;
    }


    if( m2 == MT_CONSTANT ){

        double res;
        argument2->evaluate(0,0,&res);

        if( res >= 0.0 ) return m1;

        if( res < 0.0 ){

            if( m1 == MT_NONDECREASING )  return MT_NONINCREASING;
            if( m1 == MT_NONINCREASING )  return MT_NONDECREASING;
        }
    }

    return MT_NONMONOTONIC;
}


CurvatureType Power::getCurvature( ){

    if( curvature != CT_UNKNOWN )  return curvature;

    const CurvatureType c1 = argument1->getCurvature();
    const CurvatureType c2 = argument2->getCurvature();

    if( c1 == CT_CONSTANT ){

        if( c2 == CT_CONSTANT )  return CT_CONSTANT;
        if( c2 == CT_AFFINE   )  return CT_CONVEX  ;

        if( c2 == CT_CONVEX  ){

            double res;
            argument1->evaluate(0,0,&res);

            if( res >= 1.0 )  return CT_CONVEX;
        }
        if( c2 == CT_CONCAVE  ){

            double res;
            argument1->evaluate(0,0,&res);

            if( res < 1.0 )  return CT_CONVEX;
        }

        return CT_NEITHER_CONVEX_NOR_CONCAVE;
    }

    if( c2 == CT_CONSTANT ){

        double res;
        argument2->evaluate(0,0,&res);

        if( c1 == CT_AFFINE ){

            if( res >= 1.0 || res <  0.0 ) return CT_CONVEX ;
            if( res <  1.0 && res >= 0.0 ) return CT_CONCAVE;
        }

        if( c1 == CT_CONVEX ){

            if( res >= 1.0 ) return CT_CONVEX;
        }

        if( c1 == CT_CONCAVE ){

            if( res <  0.0               ) return CT_CONVEX ;
            if( res <= 1.0 && res >= 0.0 ) return CT_CONCAVE;
        }
    }

    return CT_NEITHER_CONVEX_NOR_CONCAVE;
}


returnValue Power::AD_forward( int number, double *x, double *seed,
                               double *f, double *df ){

    if( number >= bufferSize ){
        bufferSize += number;
        argument1_result  = (double*)realloc( argument1_result,bufferSize*sizeof(double));
        argument2_result  = (double*)realloc( argument2_result,bufferSize*sizeof(double));
        dargument1_result = (double*)realloc(dargument1_result,bufferSize*sizeof(double));
        dargument2_result = (double*)realloc(dargument2_result,bufferSize*sizeof(double));
    }

    argument1->AD_forward( number, x, seed, &argument1_result[number],
                           &dargument1_result[number] );
    argument2->AD_forward( number, x, seed, &argument2_result[number],
                           &dargument2_result[number] );

      f[0] = pow(argument1_result[number],argument2_result[number]);
     df[0] = argument2_result[number]*pow(argument1_result[number],argument2_result[number]
             -1.0)*dargument1_result[number]
             +f[0]*log(argument1_result[number])*dargument2_result[number];

     return SUCCESSFUL_RETURN;
}


returnValue Power::AD_forward( int number, double *seed, double *df ){

    argument1->AD_forward( number, seed, &dargument1_result[number] );
    argument2->AD_forward( number, seed, &dargument2_result[number] );

     df[0] = argument2_result[number]*pow(argument1_result[number],argument2_result[number]
             -1.0)*dargument1_result[number]
             +pow(argument1_result[number],argument2_result[number])
             *log(argument1_result[number])*dargument2_result[number];

     return SUCCESSFUL_RETURN;
}


returnValue Power::AD_backward( int number, double seed, double *df ){

    argument1->AD_backward(number, argument2_result[number]*pow(argument1_result[number],
                           argument2_result[number]-1.0)*seed, df );

    argument2->AD_backward(number, pow(argument1_result[number],argument2_result[number])
                           *log(argument1_result[number])*seed, df );

    return SUCCESSFUL_RETURN;
}



returnValue Power::AD_forward2( int number, double *seed, double *dseed,
                                double *df, double *ddf ){

    double      ddargument1_result;
    double      ddargument2_result;
    double      dargument_result1 ;
    double      dargument_result2 ;

    argument1->AD_forward2( number, seed, dseed,
                            &dargument_result1, &ddargument1_result);
    argument2->AD_forward2( number, seed, dseed,
                            &dargument_result2, &ddargument2_result);

    const double nn1 = pow( argument1_result[number], argument2_result[number]     );
    const double nn2 = pow( argument1_result[number], argument2_result[number]-1.0 );
    const double nn3 = log( argument1_result[number] );
    const double nn4 = nn2*argument2_result[number];
    const double nn5 = nn1*nn3;
    const double nn6 = nn2*(argument2_result[number]*nn3 + 1.0);

     df[0] = nn4*dargument_result1
            +nn5*dargument_result2;
    ddf[0] = nn4*ddargument1_result
            +nn5*ddargument2_result
            +argument2_result[number]*(argument2_result[number]-1.0)
                 *pow(argument1_result[number],argument2_result[number]-2.0)
                 *dargument1_result[number]*dargument_result1
            +nn5*nn3*dargument2_result[number]*dargument_result2
            +nn6*dargument_result1*dargument2_result[number]
            +nn6*dargument1_result[number]*dargument_result2;

    return SUCCESSFUL_RETURN;
}

returnValue Power::AD_backward2( int number, double seed1, double seed2,
                                       double *df, double *ddf ){

    const double nn1 = pow( argument1_result[number], argument2_result[number]     );
    const double nn2 = pow( argument1_result[number], argument2_result[number]-1.0 );
    const double nn3 = log( argument1_result[number] );
    const double nn4 = nn2*argument2_result[number];
    const double nn5 = nn1*nn3;
    const double nn6 = nn2*(argument2_result[number]*nn3 + 1.0);

    argument1->AD_backward2(  number,
                              seed1*nn4,
                              seed2*nn4 + seed1*(
                                  argument2_result[number]*(argument2_result[number]-1.0)*
                                  pow( argument1_result[number],
                                       argument2_result[number]-2.0 )*
                                  dargument1_result[number]
                                + nn6*dargument2_result[number] ),
                              df, ddf );

    argument2->AD_backward2(  number,
                              seed1*nn5,
                              seed2*nn5 + seed1*(
                                  nn5*nn3*dargument2_result[number]
                                + nn6*dargument1_result[number]   ),
                              df, ddf );

    return SUCCESSFUL_RETURN;
}


std::ostream& Power::print( std::ostream &stream ) const{

	if ( acadoIsEqual( argument2->getValue(),0.5 ) == BT_TRUE )
	{
		return stream << "(sqrt(" << *argument1 << "))";
	}
	else
	{
		if ( acadoIsEqual( argument2->getValue(),-0.5 ) == BT_TRUE )
		{
			return stream << "(1.0/sqrt(" << *argument1 << "))";
		}
		else
		{
			return stream << "(pow(" << *argument1 << "," << *argument2 << "))";
		}
	}
}


OperatorName Power::getName(){

    return ON_POWER;

}


CLOSE_NAMESPACE_ACADO

// end of file.
