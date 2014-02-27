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




Power::Power( ):BinaryOperator( ){}

Power::Power( Operator *_argument1, Operator *_argument2 )
      :BinaryOperator( _argument1, _argument2 ){

}

Power::Power( const Power &arg ):BinaryOperator( arg ){ }


Power::~Power(){ }


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


Operator* Power::differentiate( int index ){

  dargument1 = argument1->differentiate( index );
  dargument2 = argument2->differentiate( index );

  Operator *logTmp = myLogarithm( argument1 );
  Operator *prodTmp1 = myProd( dargument2, logTmp );
  Operator *prodTmp2 = myProd( this, prodTmp1 );

  Operator *oneTmp = new DoubleConstant(1.0, NE_ONE);
  Operator *addTmp1 = mySubtract( argument2, oneTmp );
  Operator *powerTmp1 = myPower( argument1, addTmp1 );
  Operator *prodTmp3 = myProd( powerTmp1, dargument1 );
  Operator *prodTmp4 = myProd( argument2, prodTmp3 );

  Operator *result = myAdd( prodTmp2, prodTmp4 );

  delete logTmp;
  delete prodTmp1;
  delete prodTmp2;
  delete oneTmp;
  delete addTmp1;
  delete powerTmp1;
  delete prodTmp3;
  delete prodTmp4;

  return result;

}


Operator* Power::AD_forward( int dim,
                               VariableType *varType,
                               int *component,
                               Operator **seed,
                               int &nNewIS,
                               TreeProjection ***newIS ){

    if( dargument1 != 0 )
        delete dargument1;

    if( dargument2 != 0 )
        delete dargument2;

    dargument1 = argument1->AD_forward(dim,varType,component,seed,nNewIS,newIS);
    dargument2 = argument2->AD_forward(dim,varType,component,seed,nNewIS,newIS);

    Operator *logTmp = myLogarithm( argument1 );
    Operator *prodTmp1 = myProd( dargument2, logTmp );
    Operator *prodTmp2 = myProd( this, prodTmp1 );

    Operator *oneTmp = new DoubleConstant(1.0, NE_ONE);
    Operator *addTmp1 = mySubtract( argument2, oneTmp );
    Operator *powerTmp1 = myPower( argument1, addTmp1 );
    Operator *prodTmp3 = myProd( powerTmp1, dargument1 );
    Operator *prodTmp4 = myProd( argument2, prodTmp3 );

    Operator *result = myAdd( prodTmp2, prodTmp4 );

    delete logTmp;
    delete prodTmp1;
    delete prodTmp2;
    delete oneTmp;
    delete addTmp1;
    delete powerTmp1;
    delete prodTmp3;
    delete prodTmp4;

    return result;
}


returnValue Power::AD_backward( int           dim      , /**< number of directions  */
                                        VariableType *varType  , /**< the variable types    */
                                        int          *component, /**< and their components  */
                                        Operator     *seed     , /**< the backward seed     */
                                        Operator    **df       , /**< the result            */
                                        int           &nNewIS  , /**< the number of new IS  */
                                        TreeProjection ***newIS  /**< the new IS-pointer    */ ){

	if( seed->isOneOrZero() != NE_ZERO ){

		TreeProjection tmp;
		tmp = *seed;

	    Operator *oneTmp = new DoubleConstant(1.0, NE_ONE);
		Operator *addTmp1 = mySubtract( argument2, oneTmp );
		Operator *powerTmp1 = myPower( argument1, addTmp1 );
		Operator *prodTmp3 = myProd( powerTmp1, &tmp );
		Operator *prodTmp4 = myProd( argument2, prodTmp3 );

		argument1->AD_backward( dim, varType, component, prodTmp4->clone(), df, nNewIS, newIS );

		Operator *logTmp = myLogarithm( argument1 );
		Operator *prodTmp1 = myProd( &tmp, logTmp );
		Operator *prodTmp2 = myProd( this, prodTmp1 );

		argument2->AD_backward( dim, varType, component, prodTmp2->clone(), df, nNewIS, newIS );

		delete logTmp;
		delete prodTmp1;
		delete prodTmp2;
		delete oneTmp;
		delete addTmp1;
		delete powerTmp1;
		delete prodTmp3;
		delete prodTmp4;
	}

    delete seed;
    return SUCCESSFUL_RETURN;
}


returnValue Power::ADsymmetric( int            dim       , /**< number of directions  */
                                        VariableType  *varType   , /**< the variable types    */
                                        int           *component , /**< and their components  */
                                        Operator      *l         , /**< the backward seed     */
                                        Operator     **S         , /**< forward seed matrix   */
                                        int            dimS      , /**< dimension of forward seed             */
                                        Operator     **dfS       , /**< first order foward result             */
                                        Operator     **ldf       , /**< first order backward result           */
                                        Operator     **H         , /**< upper trianglular part of the Hessian */
                                      int            &nNewLIS  , /**< the number of newLIS  */
                                      TreeProjection ***newLIS , /**< the new LIS-pointer   */
                                      int            &nNewSIS  , /**< the number of newSIS  */
                                      TreeProjection ***newSIS , /**< the new SIS-pointer   */
                                      int            &nNewHIS  , /**< the number of newHIS  */
                                      TreeProjection ***newHIS   /**< the new HIS-pointer   */ ){
  
    TreeProjection dx,dy,dxx,dxy,dyy;
    TreeProjection tmp1,tmp2,tmp3,tmp4,tmp5;
    
    tmp1 = Subtraction( argument2->clone(), new DoubleConstant(1.0,NE_ONE ) );
    tmp2 = Power(argument1->clone(),argument2->clone());
    tmp3 = Power(argument1->clone(),tmp1.clone());
    tmp4 = Logarithm(argument1->clone());
    tmp5 = Power(
                argument1->clone(),
		 new Subtraction( argument2->clone(), new DoubleConstant(2.0,NE_NEITHER_ONE_NOR_ZERO ))
	   );
    
    dx  = Product( argument2->clone(), tmp3.clone() );
    dy  = Product( tmp2.clone(), tmp4.clone() );
    dxx = Product( new Product( argument2->clone(), tmp1.clone() ), tmp5.clone() );
    dxy = Product( tmp3.clone(), new Addition( new DoubleConstant(1.0,NE_ONE ), tmp4.clone() ) );
    dyy = Product( dy.clone(), tmp4.clone() );
    
    return ADsymCommon2( argument1,argument2,dx,dy,dxx,dxy,dyy, dim, varType, component, l, S, dimS, dfS,
			  ldf, H, nNewLIS, newLIS, nNewSIS, newSIS, nNewHIS, newHIS );
}



Operator* Power::substitute( int index, const Operator *sub ){

    return new Power( argument1->substitute( index , sub ),
                      argument2->substitute( index , sub ) );

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


Operator* Power::clone() const{

    return new Power(*this);
}


OperatorName Power::getName(){

    return ON_POWER;

}


CLOSE_NAMESPACE_ACADO

// end of file.
