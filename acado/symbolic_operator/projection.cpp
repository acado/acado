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
 *    \file src/symbolic_operator/projection.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */


#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>



BEGIN_NAMESPACE_ACADO


Projection::Projection()
           :SmoothOperator( ){

    scale          = 1.0           ;
    curvature      = CT_AFFINE     ;
    monotonicity = MT_NONDECREASING;
    operatorName = ON_VARIABLE     ;

    nCount = 0;
}


Projection::Projection( const std::string &name_ )
           :SmoothOperator( ){

    scale          = 1.0             ;
    curvature      = CT_AFFINE       ;
    monotonicity   = MT_NONDECREASING;
    operatorName   = ON_VARIABLE     ;
    name           = name_           ;

    nCount = 0;
}


Projection::Projection( VariableType variableType_, int vIndex_, const std::string &name_ ) :SmoothOperator( )
{
    variableType   = variableType_ ;
    vIndex         = vIndex_       ;
    variableIndex  = vIndex        ;

    scale          = 1.0           ;
    curvature      = CT_AFFINE     ;
    monotonicity = MT_NONDECREASING;
    operatorName = ON_VARIABLE     ;

    std::stringstream ss;
    switch(variableType){

         case VT_DIFFERENTIAL_STATE:
              ss << "xd" << "[" << vIndex <<"]";
              break;

         case VT_ALGEBRAIC_STATE:
              ss << "xa" << "[" << vIndex <<"]";
              break;

         case VT_CONTROL:
              ss << "u" << "[" << vIndex <<"]";
              break;

         case VT_INTEGER_CONTROL:
              ss << "v" << "[" << vIndex <<"]";
              break;

         case VT_PARAMETER:
              ss << "p" << "[" << vIndex <<"]";
              break;

         case VT_ONLINE_DATA:
              ss << "od" << "[" << vIndex <<"]";
              break;

         case VT_INTEGER_PARAMETER:
              ss << "q" << "[" << vIndex <<"]";
              break;

         case VT_DISTURBANCE:
              ss << "w" << "[" << vIndex <<"]";
              break;

         case VT_TIME:
              ss << "t" << "[" << vIndex <<"]";
              break;

         case VT_INTERMEDIATE_STATE:
              ss << "a" << "[" << vIndex <<"]";
              break;

         case VT_DDIFFERENTIAL_STATE:
              ss << "dx" << "[" << vIndex <<"]";
              break;

         default: break;
    }
    name = ss.str();
    nCount = 0;
}


Projection::~Projection(){ }


Projection::Projection( const Projection& arg ){

    copy( arg );
}


Operator* Projection::clone() const{

    return new Projection(*this);
}


void Projection::copy( const Projection &arg ){

    if( this != &arg ){

        variableType   = arg.variableType ;
        variableIndex  = arg.variableIndex;
        vIndex         = arg.vIndex       ;
        scale          = arg.scale        ;
        name           = arg.name         ;
        operatorName   = arg.operatorName ;
        curvature      = arg.curvature    ;
        monotonicity   = arg.monotonicity ;

        nCount = 0;
    }
}


returnValue Projection::evaluate( int number, double *x, double *result ){

    result[0] = x[variableIndex];
    return SUCCESSFUL_RETURN;
}


returnValue Projection::evaluate( EvaluationBase *x ){

    x->project(variableIndex);
    return SUCCESSFUL_RETURN;
}


Operator* Projection::differentiate( int index ){

    if( variableIndex == index ){
        return new DoubleConstant( 1.0 , NE_ONE );
    }
    else{
        return new DoubleConstant( 0.0 , NE_ZERO );
    }
}


Operator* Projection::AD_forward( int dim,
                                          VariableType *varType,
                                          int *component,
                                          Operator **seed,
                                          int &nNewIS,
                                          TreeProjection ***newIS ){

    return ADforwardProtected( dim, varType, component, seed, nNewIS, newIS );
}



returnValue Projection::AD_backward( int           dim      , /**< number of directions  */
                                        VariableType *varType  , /**< the variable types    */
                                        int          *component, /**< and their components  */
                                        Operator     *seed     , /**< the backward seed     */
                                        Operator    **df       , /**< the result            */
                                        int           &nNewIS  , /**< the number of new IS  */
                                        TreeProjection ***newIS  /**< the new IS-pointer    */ ){

    return ADbackwardProtected( dim, varType, component, seed, df, nNewIS, newIS );
}


returnValue Projection::AD_symmetric( int            dim       , /**< number of directions  */
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
  
	return ADsymmetricProtected( dim, varType, component, l, S, dimS, dfS, ldf, H, nNewLIS, newLIS, nNewSIS, newSIS, nNewHIS, newHIS );
}



Operator* Projection::substitute( int index, const Operator *sub ){

    if( variableIndex == index ){
        return sub->clone();
    }
    else{
        return clone();
    }
}



NeutralElement Projection::isOneOrZero() const{

    return NE_NEITHER_ONE_NOR_ZERO;
}




BooleanType Projection::isDependingOn( VariableType var ) const{

    if( variableType == var )  return BT_TRUE;
    return BT_FALSE;
}




BooleanType Projection::isDependingOn( int dim,
                                               VariableType *varType,
                                               int *component,
                                               BooleanType   *implicit_dep ){

    int run1 = 0;

    while( run1 < dim ){

        if( varType[run1] == variableType && component[run1] == vIndex ){
            return BT_TRUE;
        }
        run1++;
    }
    return BT_FALSE;
}


BooleanType Projection::isLinearIn( int dim,
                                            VariableType *varType,
                                            int *component,
                                            BooleanType   *implicit_dep ){

    return BT_TRUE;
}


BooleanType Projection::isPolynomialIn( int dim,
                                                VariableType *varType,
                                                int *component,
                                                BooleanType   *implicit_dep ){

    return BT_TRUE;
}


BooleanType Projection::isRationalIn( int dim,
                                              VariableType *varType,
                                              int *component,
                                              BooleanType   *implicit_dep ){

    return BT_TRUE;
}


MonotonicityType Projection::getMonotonicity( ){

    return monotonicity;
}


CurvatureType Projection::getCurvature( ){

    return curvature;
}


returnValue Projection::setMonotonicity( MonotonicityType monotonicity_ ){

    monotonicity = monotonicity_;
    return SUCCESSFUL_RETURN;
}


returnValue Projection::setCurvature( CurvatureType curvature_ ){

    curvature = curvature_;
    return SUCCESSFUL_RETURN;
}


returnValue Projection::AD_forward( int number, double *x, double *seed,
                                          double *f, double *df ){

      f[0] =  x[variableIndex];
     df[0] =  seed[variableIndex];

     return SUCCESSFUL_RETURN;
}


returnValue Projection::AD_forward( int number, double *seed, double *df ){

     df[0] =  seed[variableIndex];
     return SUCCESSFUL_RETURN;
}


returnValue Projection::AD_backward( int number, double seed, double *df ){

    df[variableIndex] = df[variableIndex] + seed;
    return SUCCESSFUL_RETURN;
}


returnValue Projection::AD_forward2( int number, double *seed, double *dseed,
                                           double *df, double *ddf ){

     df[0] = seed [variableIndex];
    ddf[0] = dseed[variableIndex];
    return SUCCESSFUL_RETURN;
}

returnValue Projection::AD_backward2( int number, double seed1, double seed2,
                                            double *df, double *ddf ){

     df[variableIndex] =  df[variableIndex] + seed1;
    ddf[variableIndex] = ddf[variableIndex] + seed2;

    return SUCCESSFUL_RETURN;
}



std::ostream& Projection::print( std::ostream &stream ) const{

    return stream << name;
}


returnValue Projection::clearBuffer(){

    return SUCCESSFUL_RETURN;
}


OperatorName Projection::getName(){

    return operatorName;
}


double Projection::getScale() const{

    return scale;
}


returnValue Projection::setScale( const double &scale_ ){

    scale = scale_;
    return SUCCESSFUL_RETURN;
}


BooleanType Projection::isVariable( VariableType &varType, int &component ) const
{
  varType   = variableType;
  component = vIndex    ;

  return BT_TRUE;
}

returnValue Projection::enumerateVariables( SymbolicIndexList *indexList ){

  variableIndex = indexList->determineVariableIndex( variableType,
						     vIndex, scale         );
  return SUCCESSFUL_RETURN;
}


int Projection::getVariableIndex( ) const{

    return variableIndex;
}


int Projection::getGlobalIndex() const{

    return vIndex;
}


VariableType Projection::getType( ) const
{
	return variableType;
}


returnValue Projection::loadIndices( SymbolicIndexList *indexList ){

    indexList->addNewElement( variableType, vIndex );
    return SUCCESSFUL_RETURN;
}


BooleanType Projection::isSymbolic() const{

    return BT_TRUE;
}


Operator* Projection::ADforwardProtected( int dim,
                                                  VariableType *varType,
                                                  int *component,
                                                  Operator **seed,
                                                  int &nNewIS,
                                                  TreeProjection ***newIS ){

    int run1 = 0;

    while( run1 < dim ){

        if( varType[run1] == variableType && component[run1] == vIndex ){
            return seed[run1]->clone();
        }
        run1++;
    }

    return new DoubleConstant( 0.0 , NE_ZERO );
}



returnValue Projection::ADbackwardProtected( int           dim      , /**< number of directions  */
                                        VariableType *varType  , /**< the variable types    */
                                        int          *component, /**< and their components  */
                                        Operator     *seed     , /**< the backward seed     */
                                        Operator    **df       , /**< the result            */
                                        int           &nNewIS  , /**< the number of new IS  */
                                        TreeProjection ***newIS  /**< the new IS-pointer    */ ){

    int run1 = 0;

    while( run1 < dim ){

        if( varType[run1] == variableType && component[run1] == vIndex ){

            if(   df[run1]->isOneOrZero() == NE_ZERO ){
                  delete df[run1];
                  df[run1] = seed->clone();
            }
            else{

                if( seed-> isOneOrZero() != NE_ZERO ){

                    Operator *tmp = df[run1]->clone();
                    delete df[run1];
                    df[run1] = new Addition(tmp->clone(),seed->clone());
                    delete tmp;
                }
            }

            break;
        }
        run1++;
    }

    delete seed;
    return SUCCESSFUL_RETURN;
}


returnValue Projection::ADsymmetricProtected( int            dim       , /**< number of directions  */
                                        VariableType  *varType   , /**< the variable types    */
                                        int           *component , /**< and their components  */
                                        Operator      *l         , /**< the backward seed     */
                                        Operator     **S         , /**< forward seed matrix   */
                                        int            dimS      , /**< dimension of forward seed             */
                                        Operator     **dfS       , /**< first order forward result             */
                                        Operator     **ldf       , /**< first order backward result           */
                                        Operator     **H         , /**< upper triangular part of the Hessian */
                                        int            &nNewLIS  , /**< the number of newLIS  */
                                        TreeProjection ***newLIS , /**< the new LIS-pointer   */
                                        int            &nNewSIS  , /**< the number of newSIS  */
                                        TreeProjection ***newSIS , /**< the new SIS-pointer   */
                                        int            &nNewHIS  , /**< the number of newHIS  */
                                        TreeProjection ***newHIS   /**< the new HIS-pointer   */ ){

	int run1 = 0;

	while( run1 < dim ){

		if( varType[run1] == variableType && component[run1] == vIndex ){

			int run2;
			for( run2 = 0; run2 < dimS; run2++ ){
				delete dfS[run2];
				dfS[run2] = S[run1*dimS+run2]->clone();
			}

			if(   ldf[run1]->isOneOrZero() == NE_ZERO ){
				delete ldf[run1];
				ldf[run1] = l->clone();
			}
			else{

				if( l->isOneOrZero() != NE_ZERO ){

					Operator *tmp = ldf[run1]->clone();
					delete ldf[run1];
					ldf[run1] = new Addition(tmp->clone(),l->clone());
					delete tmp;
				}
			}
			break;
		}
		run1++;
	}

	delete l;
	return SUCCESSFUL_RETURN;
}

returnValue Projection::setVariableExportName( const VariableType &_type, const std::vector<std::string > &_name ){

	if (variableType == _type)
	{
		this->name = _name[vIndex];
	}
	return Operator::setVariableExportName(_type, _name);
}


BooleanType Projection::isTrivial() const {
	return BT_TRUE;
}


CLOSE_NAMESPACE_ACADO

// end of file.
