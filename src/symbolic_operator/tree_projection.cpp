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
*    \file src/symbolic_operator/tree_projection.cpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date 2008
*/


#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/symbolic_expression/constraint_component.hpp>


BEGIN_NAMESPACE_ACADO



int TreeProjection::count = 0;

TreeProjection::TreeProjection( )
               :Projection(){

    variableType   = VT_INTERMEDIATE_STATE ;
    vIndex         = 0                     ;
    variableIndex  = 0                     ;
    Operator *tmp  = 0                     ;
    argument       = SharedOperator(tmp)   ;
    ne             = NE_ZERO               ;
}


TreeProjection::TreeProjection( const std::string &name_ )
               :Projection( name_ ){

    variableType   = VT_INTERMEDIATE_STATE ;
    vIndex         = 0                     ;
    variableIndex  = 0                     ;
    Operator *tmp  = 0                     ;
    argument       = SharedOperator(tmp)   ;
    ne             = NE_ZERO               ;
}



TreeProjection::TreeProjection( const TreeProjection &arg )
               :Projection(){

    copy(arg);
    argument = arg.argument;
    ne = arg.ne;
}


TreeProjection::~TreeProjection(){ }



Operator& TreeProjection::operator=( const SharedOperator &arg ){

    	Operator *arg_tmp = arg.get();
    	TreeProjection *tp = dynamic_cast<TreeProjection *>(arg_tmp);
    	Projection *p = dynamic_cast<Projection *>(arg_tmp);
    	if( tp != 0 ) {
    		// special case: argument is a treeprojection
    		if( tp->argument != 0 ) {
    			argument = tp->argument;
    			ne = argument->isOneOrZero();
    		}
    		else {
			Operator *tmp = 0;
    			argument = SharedOperator( tmp );
    			ne = NE_NEITHER_ONE_NOR_ZERO;
    		}
		copy(*tp);
    	}
    	else {
    		if( p != 0 ) {
        		// special case: argument is a projection
			Operator *tmp = 0;
    			argument = SharedOperator( tmp );
    			ne = NE_NEITHER_ONE_NOR_ZERO;
    			copy(*p);
    		}
    		else {
    			// no special case: create a new treeprojection
    			argument = arg;
    			vIndex   = count++;
    			variableIndex  = vIndex ;
			variableType   = VT_INTERMEDIATE_STATE ;

    			curvature      = CT_UNKNOWN; // argument->getCurvature();
    			monotonicity   = MT_UNKNOWN; // argument->getMonotonicity();

    			ne = argument->isOneOrZero();
    		}
    	}
    return *this;
}


Operator& TreeProjection::operator=( const Expression &arg ){

	ASSERT( arg.getDim() == 1 );

	argument = arg.getOperatorClone(0);

	vIndex         = count++;
	variableIndex  = vIndex ;

	curvature      = CT_UNKNOWN; // argument->getCurvature();
	monotonicity   = MT_UNKNOWN; // argument->getMonotonicity();

	ne = argument->isOneOrZero();

	return *this;
}


Operator& TreeProjection::operator=( const double& arg ){

    Expression tmp;
    return this->operator=( tmp.convert(arg) );
}


Operator& TreeProjection::operator+=( const Expression& arg ){
  
return operator=( myAdd( SharedOperator(new TreeProjection(*this)), arg.getOperatorClone(0) ) );
}

Operator& TreeProjection::operator-=( const Expression& arg ){
  
return operator=( mySubtract( SharedOperator(new TreeProjection(*this)), arg.getOperatorClone(0) ) );
}

Operator& TreeProjection::operator*=( const Expression& arg ){
  
return operator=( myProd( SharedOperator(new TreeProjection(*this)), arg.getOperatorClone(0) ) );
}

Operator& TreeProjection::operator/=( const Expression& arg ){
  
return operator=( SharedOperator( new Quotient( SharedOperator(new TreeProjection(*this)), arg.getOperatorClone(0) ) ));
}


SharedOperator TreeProjection::ADforwardProtected( int dim,
                                                   VariableType *varType,
                                                   int *component,
                                                   SharedOperator *seed,
                                                   std::vector<SharedOperator> &newIS ){
  
    if (argument == 0) {
        return Projection::ADforwardProtected( dim, varType, component, seed, newIS );
    }

    int run1 = 0;

    while( run1 < dim ){

        if( varType[run1] == variableType && component[run1] == vIndex ){
            return seed[run1];
        }
        run1++;
    }
    
    newIS.resize(vIndex+1);

    if( newIS[vIndex] != 0 ) return newIS[vIndex];
    
    SharedOperator tmp = argument->AD_forward(dim,varType,component,seed,newIS);

    newIS[vIndex] = SharedOperator(new TreeProjection());
    newIS[vIndex]->operator=(tmp);
    newIS[vIndex]->setCurvature( CT_UNKNOWN );

    return newIS[vIndex];
}



returnValue TreeProjection::ADbackwardProtected( int           dim      , /**< number of directions  */
                                        VariableType *varType  , /**< the variable types    */
                                        int          *component, /**< and their components  */
                                        SharedOperator &seed     , /**< the backward seed     */
                                        SharedOperator    *df       , /**< the result            */
                                        std::vector<SharedOperator> &newIS ){

	if (argument == 0) {
		return Projection::ADbackwardProtected( dim, varType, component, seed, df, newIS );
	}
	else {

    int run1;

    newIS.resize((vIndex+1)*dim);
    
    if( newIS[vIndex*dim] == 0 ){
      
        SharedOperator *results = new SharedOperator[dim];
        for( run1 = 0; run1 < dim; run1++ ){
            results[run1] = SharedOperator( new DoubleConstant(0.0,NE_ZERO) );
        }
        SharedOperator aux = SharedOperator( new DoubleConstant(1.0,NE_ONE) );
        argument->AD_backward(dim,varType,component, aux, results, newIS );
        for( run1 = 0; run1 < dim; run1++ ){  
	      newIS[vIndex*dim+run1] = SharedOperator( new TreeProjection() );
	      newIS[vIndex*dim+run1]->operator=(results[run1]);
	      newIS[vIndex*dim+run1]->setCurvature( CT_UNKNOWN );
	}
        delete[] results;
    }

    if( seed->isOneOrZero() != NE_ZERO ){
      
        for( run1 = 0; run1 < dim; run1++ ){
	  
	    SharedOperator tmp = df[run1];
	    
	    if( seed->isOneOrZero() == NE_ONE ){
	        df[run1] = myAdd( newIS[vIndex*dim+run1], tmp );
	    }
	    else{
                SharedOperator projTmp = myProd(newIS[vIndex*dim+run1],seed);
                df[run1] = myAdd( projTmp, tmp );
	    }
        }
    }
    return SUCCESSFUL_RETURN;
  }
}



returnValue TreeProjection::ADsymmetricProtected( int            dim       , /**< number of directions  */
                                        VariableType  *varType   , /**< the variable types    */
                                        int           *component , /**< and their components  */
                                        SharedOperator    &l         , /**< the backward seed     */
                                        SharedOperator    *S         , /**< forward seed matrix   */
                                        int            dimS      , /**< dimension of forward seed             */
                                        SharedOperator     *dfS       , /**< first order forward result             */
                                        SharedOperator     *ldf       , /**< first order backward result           */
                                        SharedOperator     *H         , /**< upper triangular part of the Hessian */
                                        std::vector<SharedOperator> &newLIS , /**< the new LIS-pointer   */
                                        std::vector<SharedOperator> &newSIS , /**< the new SIS-pointer   */
                                        std::vector<SharedOperator> &newHIS   /**< the new HIS-pointer   */ ){
  
	if (argument == 0) {
		return Projection::ADsymmetricProtected( dim, varType, component, l, S, dimS, dfS, ldf, H, newLIS, newSIS, newHIS );
	}
	else {

	int run1;

	newLIS.resize((vIndex+1)*dim );
	newSIS.resize((vIndex+1)*dimS);
	newHIS.resize((vIndex+1)*dimS*dimS);
  
  // ============================================================================

	if( newLIS[vIndex*dim] == 0 ){

		SharedOperator *lres = new SharedOperator[dim];
		for( run1 = 0; run1 < dim; run1++ ){
			lres[run1] = SharedOperator( new DoubleConstant(0.0,NE_ZERO));
		}
		SharedOperator aux = SharedOperator( new DoubleConstant(1.0,NE_ONE) );

		SharedOperator *Sres = new SharedOperator[dimS];
		for( run1 = 0; run1 < dimS; run1++ ){
			Sres[run1] = SharedOperator( new DoubleConstant(0.0,NE_ZERO) );
		}

		SharedOperator *Hres = new SharedOperator[dimS*dimS];
		for( run1 = 0; run1 < dimS*dimS; run1++ ){
			Hres[run1] = SharedOperator( new DoubleConstant(0.0,NE_ZERO) );
		}

		argument->AD_symmetric( dim, varType, component, aux, S, dimS,
				Sres, lres, Hres,
				newLIS, newSIS, newHIS );


		for( run1 = 0; run1 < dim; run1++ ){
			newLIS[vIndex*dim+run1] = SharedOperator( new TreeProjection() );
			newLIS[vIndex*dim+run1]->operator=(lres[run1]);
			newLIS[vIndex*dim+run1]->setCurvature( CT_UNKNOWN );
		}
		delete[] lres;

		for( run1 = 0; run1 < dimS; run1++ ){
			newSIS[vIndex*dimS+run1] = SharedOperator( new TreeProjection() );
			newSIS[vIndex*dimS+run1]->operator=(Sres[run1]);
			newSIS[vIndex*dimS+run1]->setCurvature( CT_UNKNOWN );
		}
		delete[] Sres;

		for( run1 = 0; run1 < dimS*dimS; run1++ ){
			newHIS[vIndex*dimS*dimS+run1] = SharedOperator( new TreeProjection() );
			newHIS[vIndex*dimS*dimS+run1]->operator=(Hres[run1]);
			newHIS[vIndex*dimS*dimS+run1]->setCurvature( CT_UNKNOWN );
		}
		delete[] Hres;
	}

  // ============================================================================
  
	if( l->isOneOrZero() != NE_ZERO ){

		for( run1 = 0; run1 < dim; run1++ ){

			SharedOperator tmp = ldf[run1];

			if( l->isOneOrZero() == NE_ONE ){
				ldf[run1] = myAdd( newLIS[vIndex*dim+run1], tmp );
			}
			else{
				SharedOperator projTmp = myProd( newLIS[vIndex*dim+run1], l );
				ldf[run1] = myAdd( projTmp, tmp );
			}
		}
	}

	for( run1 = 0; run1 < dimS; run1++ ){
		dfS[run1] = newSIS[vIndex*dimS+run1];
	}

	for( run1 = 0; run1 < dimS*dimS; run1++ ){

		if( l->isOneOrZero() == NE_ONE ){
			H[run1] = newHIS[vIndex*dimS*dimS+run1];
		}
		else{
			H[run1] = myProd(newHIS[vIndex*dimS*dimS+run1],l);
		}
	}
	return SUCCESSFUL_RETURN;
	}
}
  

  
returnValue TreeProjection::loadIndices( SymbolicIndexList *indexList ){

    returnValue returnvalue = SUCCESSFUL_RETURN;

    if( argument == 0 ){
        return Projection::loadIndices( indexList );
    }

    if( indexList->addNewElement( VT_INTERMEDIATE_STATE, vIndex ) == BT_TRUE ){

        returnvalue = argument->loadIndices( indexList );

        indexList->addOperatorPointer( argument, vIndex );
    }

    if (name.empty())
    {
    	std::stringstream ss;
        ss << "a" << "[" << vIndex << "]";
        name = ss.str();
    }

    return returnvalue;
}


BooleanType TreeProjection::isDependingOn( int dim,
                                                VariableType *varType,
                                                int *component,
                                                BooleanType   *implicit_dep ){

	if( argument == 0 ){
		return Projection::isDependingOn( dim, varType, component, implicit_dep );
	}
	return implicit_dep[vIndex];
}


BooleanType TreeProjection::isLinearIn( int dim,
                                             VariableType *varType,
                                             int *component,
                                             BooleanType   *implicit_dep ){

	if( argument == 0 ){
		return Projection::isLinearIn( dim, varType, component, implicit_dep );
	}
    return implicit_dep[vIndex];
}


BooleanType TreeProjection::isPolynomialIn( int dim,
                                                 VariableType *varType,
                                                 int *component,
                                                 BooleanType   *implicit_dep ){

	if( argument == 0 ){
		return Projection::isPolynomialIn( dim, varType, component, implicit_dep );
	}
    return implicit_dep[vIndex];
}


BooleanType TreeProjection::isRationalIn( int dim,
                                               VariableType *varType,
                                               int *component,
                                               BooleanType   *implicit_dep ){

	if( argument == 0 ){
		return Projection::isRationalIn( dim, varType, component, implicit_dep );
	}
	return implicit_dep[vIndex];
}


returnValue TreeProjection::clearStaticCounters(){

    count     = 0;
    return SUCCESSFUL_RETURN;
}


SharedOperator TreeProjection::getArgument() const{

    return argument;
}


NeutralElement TreeProjection::isOneOrZero() const{

    return ne;
}



void TreeProjection::copy( const Projection &arg ){

    Projection::copy(arg);
}


SharedOperator TreeProjection::passArgument() const{

    return argument;
}


returnValue TreeProjection::setVariableExportName( const VariableType &_type,
						    const std::vector< std::string >& _name )
{
	if (argument != 0 && argument->getName() == ON_POWER_INT)
		argument->setVariableExportName(_type, _name);
	
	return Projection::setVariableExportName(_type, _name);
}


BooleanType TreeProjection::isTrivial() const {

	return argument != 0;
}


returnValue TreeProjection::initDerivative() {

	if( argument != 0 ) {
		return argument->initDerivative();
	}
	else {
		return SUCCESSFUL_RETURN;
	}
}


CLOSE_NAMESPACE_ACADO

// end of file.
