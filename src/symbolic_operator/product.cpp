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
 *    \file src/symbolic_operator/product.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */


#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>



BEGIN_NAMESPACE_ACADO




Product::Product():BinaryOperator(){ }

Product::Product( const SharedOperator &_argument1, const SharedOperator &_argument2 )
        :BinaryOperator( _argument1, _argument2 ){

}


Product::Product( const Product &arg ):BinaryOperator( arg ){

}


Product::~Product(){

}

Product& Product::operator=( const Product &arg ){

    if( this != &arg ){

        BinaryOperator::operator=( arg );
    }

    return *this;
}



returnValue Product::evaluate( int number, double *x, double *result ){

    if( number >= bufferSize ){
        bufferSize += number;
        argument1_result  = (double*)realloc( argument1_result,bufferSize*sizeof(double));
        argument2_result  = (double*)realloc( argument2_result,bufferSize*sizeof(double));
        dargument1_result = (double*)realloc(dargument1_result,bufferSize*sizeof(double));
        dargument2_result = (double*)realloc(dargument2_result,bufferSize*sizeof(double));
    }

    argument1->evaluate( number, x , &argument1_result[number] );
    argument2->evaluate( number, x , &argument2_result[number] );

    result[0] = argument1_result[number] * argument2_result[number];

    return SUCCESSFUL_RETURN;
}


returnValue Product::evaluate( EvaluationBase *x ){
 
    x->product(*argument1,*argument2);
    return SUCCESSFUL_RETURN;
}


SharedOperator Product::differentiate( int index ){

	dargument1 = argument1->differentiate( index );
	dargument2 = argument2->differentiate( index );

	SharedOperator prodTmp1 = myProd(dargument1, argument2);
	SharedOperator prodTmp2 = myProd(argument1, dargument2);
	SharedOperator result = myAdd(prodTmp1, prodTmp2);

	return result;
}


SharedOperator Product::AD_forward( int dim,
                                 VariableType *varType,
                                 int *component,
                                 SharedOperator *seed,
                                 std::vector<SharedOperator> &newIS ){

    dargument1 = argument1->AD_forward(dim,varType,component,seed,newIS);
    dargument2 = argument2->AD_forward(dim,varType,component,seed,newIS);

    SharedOperator prodTmp1 = myProd(dargument1, argument2);
    SharedOperator prodTmp2 = myProd(argument1, dargument2);
    SharedOperator result = myAdd(prodTmp1, prodTmp2);

    return result;
}


returnValue Product::AD_backward( int           dim      , /**< number of directions  */
                                        VariableType *varType  , /**< the variable types    */
                                        int          *component, /**< and their components  */
                                        SharedOperator     &seed     , /**< the backward seed     */
                                        SharedOperator    *df       , /**< the result            */
                                        std::vector<SharedOperator> &newIS  /**< the new IS-pointer    */ ){


    if( seed->isOneOrZero() != NE_ZERO ){
	
        SharedOperator tmp = convert2TreeProjection(seed);
        SharedOperator prodTmp1 = myProd(argument2, tmp);

        argument1->AD_backward( dim, varType, component, prodTmp1,
                                df, newIS );

        SharedOperator prodTmp2 = myProd(argument1, tmp);

        argument2->AD_backward( dim, varType, component, prodTmp2,
                                df, newIS );
    }
    return SUCCESSFUL_RETURN;
}


returnValue Product::AD_symmetric( int            dim       , /**< number of directions  */
                                        VariableType  *varType   , /**< the variable types    */
                                        int           *component , /**< and their components  */
                                        SharedOperator   &l         , /**< the backward seed     */
                                        SharedOperator     *S         , /**< forward seed matrix   */
                                        int            dimS      , /**< dimension of forward seed             */
                                        SharedOperator     *dfS       , /**< first order foward result             */
                                        SharedOperator     *ldf       , /**< first order backward result           */
                                        SharedOperator     *H         , /**< upper trianglular part of the Hessian */
                                      std::vector<SharedOperator> &newLIS , /**< the new LIS-pointer   */
                                      std::vector<SharedOperator> &newSIS , /**< the new SIS-pointer   */
                                      std::vector<SharedOperator> &newHIS   /**< the new HIS-pointer   */ ){
  
    SharedOperator dx,dy,dxx,dxy,dyy;
    
    dx  = convert2TreeProjection( argument2 );
    dy  = convert2TreeProjection( argument1 );
    dxx = SharedOperator( new DoubleConstant(0.0,NE_ZERO) );
    dxy = SharedOperator( new DoubleConstant(1.0,NE_ONE ) );
    dyy = SharedOperator( new DoubleConstant(0.0,NE_ZERO) );
    
    return ADsymCommon2( argument1,argument2,dx,dy,dxx,dxy,dyy, dim, varType, component, l, S, dimS, dfS,
			  ldf, H, newLIS, newSIS, newHIS );
}




SharedOperator Product::substitute( int index, const SharedOperator &sub ){

    return SharedOperator( new Product( argument1->substitute( index , sub ),
                                        argument2->substitute( index , sub ) ));

}


BooleanType Product::isLinearIn( int dim,
                                   VariableType *varType,
                                   int *component,
                                   BooleanType   *implicit_dep ){

    if(  argument1->isLinearIn( dim, varType, component, implicit_dep )    == BT_TRUE &&
         argument2->isDependingOn( dim, varType, component, implicit_dep ) == BT_FALSE ){
        return BT_TRUE;
    }

    if(  argument2->isLinearIn( dim, varType, component, implicit_dep )    == BT_TRUE &&
         argument1->isDependingOn( dim, varType, component, implicit_dep ) == BT_FALSE ){
        return BT_TRUE;
    }

    return BT_FALSE;
}


BooleanType Product::isPolynomialIn( int dim,
                                       VariableType *varType,
                                       int *component,
                                       BooleanType   *implicit_dep ){

    if(  argument1->isPolynomialIn( dim, varType, component, implicit_dep )    == BT_TRUE  &&
         argument2->isPolynomialIn( dim, varType, component, implicit_dep )    == BT_TRUE ){
        return BT_TRUE;
    }

    return BT_FALSE;
}


BooleanType Product::isRationalIn( int dim,
                                     VariableType *varType,
                                     int *component,
                                     BooleanType   *implicit_dep ){

    if(  argument1->isRationalIn( dim, varType, component, implicit_dep )    == BT_TRUE  &&
         argument2->isRationalIn( dim, varType, component, implicit_dep )    == BT_TRUE ){
        return BT_TRUE;
    }

    return BT_FALSE;
}


MonotonicityType Product::getMonotonicity( ){

    if( monotonicity != MT_UNKNOWN )  return monotonicity;

    MonotonicityType m1, m2;

    m1 = argument1->getMonotonicity();
    m2 = argument2->getMonotonicity();

    if( m1 == MT_CONSTANT ){

        if( m2 == MT_CONSTANT )  return MT_CONSTANT;

        double res;
        argument1->evaluate(0,0,&res);

        if( res >= 0.0 ) return m2;

        if( m2 == MT_NONDECREASING ) return MT_NONINCREASING;
        if( m2 == MT_NONINCREASING ) return MT_NONDECREASING;

        return MT_NONMONOTONIC;
    }

    if( m2 == MT_CONSTANT ){

        double res;
        argument2->evaluate(0,0,&res);

        if( res >= 0.0 ) return m1;

        if( m1 == MT_NONDECREASING ) return MT_NONINCREASING;
        if( m1 == MT_NONINCREASING ) return MT_NONDECREASING;

        return MT_NONMONOTONIC;
    }

    return MT_NONMONOTONIC;
}


CurvatureType Product::getCurvature( ){

    if( curvature != CT_UNKNOWN )  return curvature;

    CurvatureType c1, c2;

    c1 = argument1->getCurvature();
    c2 = argument2->getCurvature();

    if( c1 == CT_CONSTANT ){

        if( c2 == CT_CONSTANT )  return CT_CONSTANT;

        double res;
        argument1->evaluate(0,0,&res);

        if( res >= 0.0 ) return c2;

        if( c2 == CT_AFFINE  ) return CT_AFFINE ;
        if( c2 == CT_CONVEX  ) return CT_CONCAVE;
        if( c2 == CT_CONCAVE ) return CT_CONVEX ;

        return CT_NEITHER_CONVEX_NOR_CONCAVE;
    }

    if( c2 == CT_CONSTANT ){

        if( c1 == CT_CONSTANT )  return CT_CONSTANT;

        double res;
        argument2->evaluate(0,0,&res);

        if( res >= 0.0 ) return c1;

        if( c1 == CT_AFFINE  ) return CT_AFFINE ;
        if( c1 == CT_CONVEX  ) return CT_CONCAVE;
        if( c1 == CT_CONCAVE ) return CT_CONVEX ;

        return CT_NEITHER_CONVEX_NOR_CONCAVE;
    }

    return CT_NEITHER_CONVEX_NOR_CONCAVE;
}


double Product::getValue() const
{ 
	if ( ( argument1 == 0 ) || ( argument2 == 0 ) )
		return INFTY;
		
	if ( ( acadoIsEqual( argument1->getValue(),INFTY ) == BT_TRUE ) ||
		 ( acadoIsEqual( argument2->getValue(),INFTY ) == BT_TRUE ) )
		return INFTY;
		
	return (argument1->getValue() * argument2->getValue());
}


returnValue Product::AD_forward( int number, double *x, double *seed,
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
    argument2->AD_forward( number,
                           x, seed, &argument2_result[number], &dargument2_result[number] );

      f[0] =  argument1_result[number]*argument2_result[number];
     df[0] =  argument2_result[number]*dargument1_result[number] +
              argument1_result[number]*dargument2_result[number];

     return SUCCESSFUL_RETURN;
}



returnValue Product::AD_forward( int number, double *seed, double *df ){

    argument1->AD_forward( number, seed, &dargument1_result[number] );
    argument2->AD_forward( number, seed, &dargument2_result[number] );

     df[0] =  argument2_result[number]*dargument1_result[number] +
              argument1_result[number]*dargument2_result[number];

     return SUCCESSFUL_RETURN;
}


returnValue Product::AD_backward( int number, double seed, double *df ){

    argument1->AD_backward( number, argument2_result[number]*seed, df );
    argument2->AD_backward( number, argument1_result[number]*seed, df );

    return SUCCESSFUL_RETURN;
}


returnValue Product::AD_forward2( int number, double *seed, double *dseed,
                                  double *df, double *ddf ){

    double      ddargument1_result;
    double      ddargument2_result;
    double      dargument_result1 ;
    double      dargument_result2 ;

    argument1->AD_forward2( number, seed, dseed,
                            &dargument_result1, &ddargument1_result);
    argument2->AD_forward2( number, seed, dseed,
                            &dargument_result2, &ddargument2_result);

     df[0] =  argument2_result[number]*dargument_result1
             +argument1_result[number]*dargument_result2;
    ddf[0] =  argument2_result[number]*ddargument1_result
             +argument1_result[number]*ddargument2_result
             +dargument_result1*dargument2_result[number]
             +dargument_result2*dargument1_result[number];

    return SUCCESSFUL_RETURN;
}


returnValue Product::AD_backward2( int number, double seed1, double seed2,
                                       double *df, double *ddf ){

    argument1->AD_backward2(  number,
                              seed1*argument2_result[number],
                              seed2*argument2_result[number] +
                              seed1*dargument2_result[number],
                              df, ddf );

    argument2->AD_backward2(  number,
                              seed1*argument1_result[number],
                              seed2*argument1_result[number] +
                              seed1*dargument1_result[number] ,
                              df, ddf );

    return SUCCESSFUL_RETURN;
}



std::ostream& Product::print( std::ostream &stream ) const{

	if ( ( acadoIsFinite( argument1->getValue() ) == BT_FALSE ) ||
		 ( acadoIsFinite( argument2->getValue() ) == BT_FALSE ) )
	{
		return  (((((stream << "(") << *argument1) << "*") << *argument2) << ")");
	}
	else
	{
		return stream << "((real_t)(" << ((argument1->getValue()) * (argument2->getValue())) << "))";
	}
}


//
// PROTECTED MEMBER FUNCTIONS:
// ---------------------------

OperatorName Product::getName(){

    return ON_PRODUCT;
}



CLOSE_NAMESPACE_ACADO

// end of file.
