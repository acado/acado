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
    argument       = 0                     ;
    ne             = NE_ZERO               ;
}


TreeProjection::TreeProjection( const std::string &name_ )
               :Projection( name_ ){

    variableType   = VT_INTERMEDIATE_STATE ;
    vIndex         = 0                     ;
    variableIndex  = 0                     ;
    argument       = 0                     ;
    ne             = NE_ZERO               ;
}



TreeProjection::TreeProjection( const TreeProjection &arg )
               :Projection(){

    copy(arg);

    if( arg.argument == 0 ){
        argument = 0;
    }
    else{
        argument = arg.argument;
        argument->nCount++;
    }

    ne = arg.ne;
}


TreeProjection::~TreeProjection(){
 
    if( argument != 0 ){

        if( argument->nCount == 0 ){
            delete argument;
            argument = 0;
        }
        else{
            argument->nCount--;
        }
    }
}



Operator& TreeProjection::operator=( const Operator &arg ){

    if( this != &arg ){

        if( argument != 0 ){
            if( argument->nCount == 0 ){
                delete argument;
                argument = 0;
            }
            else{
                argument->nCount--;
            }
        }

    	Operator *arg_tmp = arg.clone();
    	TreeProjection *tp = dynamic_cast<TreeProjection *>(arg_tmp);
    	Projection *p = dynamic_cast<Projection *>(arg_tmp);
    	if( tp != 0 ) {
    		// special case: argument is a treeprojection
    		if( tp->argument != 0 ) {
    			argument = tp->argument->clone();
    			ne = argument->isOneOrZero();
    		}
    		else {
    			argument = 0;
    			ne = NE_NEITHER_ONE_NOR_ZERO;
    		}
			copy(*tp);

    	}
    	else {
    		if( p != 0 ) {
        		// special case: argument is a projection
    			argument = 0;
    			ne = NE_NEITHER_ONE_NOR_ZERO;
    			copy(*p);
    		}
    		else {
    			// no special case: create a new treeprojection
    			argument = arg.clone() ;
    			vIndex   = count++;
    			variableIndex  = vIndex ;

    			curvature      = CT_UNKNOWN; // argument->getCurvature();
    			monotonicity   = MT_UNKNOWN; // argument->getMonotonicity();

    			ne = argument->isOneOrZero();

    			if( curvature == CT_CONSTANT )
    				scale = argument->getValue();
    		}
    	}
    	delete arg_tmp;
    }

    return *this;
}


Operator& TreeProjection::operator=( const Expression &arg ){

	ASSERT( arg.getDim() == 1 );

	if( argument != 0 ){
		if( argument->nCount == 0 ){
			delete argument;
			argument = 0;
		}
		else{
			argument->nCount--;
		}
	}

	argument = arg.getOperatorClone(0);

	vIndex         = count++;
	variableIndex  = vIndex ;

	curvature      = CT_UNKNOWN; // argument->getCurvature();
	monotonicity   = MT_UNKNOWN; // argument->getMonotonicity();

	ne = argument->isOneOrZero();

	if( curvature == CT_CONSTANT )
		scale = argument->getValue();

	return *this;
}


Operator& TreeProjection::operator=( const double& arg ){

    scale = arg;

    Expression tmp( arg );
    return this->operator=( tmp );
}


TreeProjection* TreeProjection::clone() const{

    return new TreeProjection( *this );
}


TreeProjection* TreeProjection::cloneTreeProjection() const{

    return new TreeProjection( *this );
}



Operator* TreeProjection::ADforwardProtected( int dim,
                                                   VariableType *varType,
                                                   int *component,
                                                   Operator **seed,
                                                   int &nNewIS,
                                                   TreeProjection ***newIS ){

	if (argument == 0) {
		return Projection::ADforwardProtected( dim, varType, component, seed, nNewIS, newIS );
	}
	else {

    int run1 = 0;

    while( run1 < dim ){

        if( varType[run1] == variableType && component[run1] == vIndex ){
            return seed[run1]->clone();
        }
        run1++;
    }

    if( vIndex >= nNewIS ){

        *newIS = (TreeProjection**)realloc(*newIS,(vIndex+1)*sizeof(TreeProjection*));

        for( run1 = nNewIS; run1 < vIndex + 1; run1++ )
             newIS[0][run1] = 0;

        nNewIS = vIndex+1;
    }

    if( newIS[0][vIndex] != 0 ){

        return newIS[0][vIndex]->clone();
    }

    Operator *tmp = argument->AD_forward(dim,varType,component,seed,nNewIS,newIS);

    newIS[0][vIndex] = new TreeProjection();

    newIS[0][vIndex]->operator=(*tmp);

    newIS[0][vIndex]->setCurvature( CT_UNKNOWN );

    delete tmp;
    return newIS[0][vIndex]->clone();
	}
}



returnValue TreeProjection::ADbackwardProtected( int           dim      , /**< number of directions  */
                                        VariableType *varType  , /**< the variable types    */
                                        int          *component, /**< and their components  */
                                        Operator     *seed     , /**< the backward seed     */
                                        Operator    **df       , /**< the result            */
                                        int           &nNewIS  , /**< the number of new IS  */
                                        TreeProjection ***newIS  /**< the new IS-pointer    */ ){

	if (argument == 0) {
		return Projection::ADbackwardProtected( dim, varType, component, seed, df, nNewIS, newIS );
	}
	else {

    int run1;

    if( (vIndex+1)*dim-1 >= nNewIS ){

        *newIS = (TreeProjection**)realloc(*newIS,((vIndex+1)*dim)*sizeof(TreeProjection*));

        for( run1 = nNewIS; run1 < (vIndex+1)*dim; run1++ )
             newIS[0][run1] = 0;

        nNewIS = (vIndex+1)*dim;
    }
    
    if( newIS[0][vIndex*dim] == 0 ){
      
        Operator **results = new Operator*[dim];
        for( run1 = 0; run1 < dim; run1++ ){
            results[run1] = new DoubleConstant(0.0,NE_ZERO);
        }
        Operator *aux = new DoubleConstant(1.0,NE_ONE);
        argument->AD_backward(dim,varType,component, aux, results, nNewIS, newIS );
        for( run1 = 0; run1 < dim; run1++ ){  
	      newIS[0][vIndex*dim+run1] = new TreeProjection();
	      newIS[0][vIndex*dim+run1]->operator=(*results[run1]);
	      newIS[0][vIndex*dim+run1]->setCurvature( CT_UNKNOWN );
	      delete results[run1];
	}
        delete[] results;
    }

    if( seed->isOneOrZero() != NE_ZERO ){
      
        for( run1 = 0; run1 < dim; run1++ ){
	  
	    Operator *tmp = df[run1]->clone();
            delete df[run1];
	    
	    if( seed->isOneOrZero() == NE_ONE ){
	        df[run1] = myAdd( newIS[0][vIndex*dim+run1], tmp );
	    }
	    else{
                Operator *projTmp = myProd(newIS[0][vIndex*dim+run1],seed);
                df[run1] = myAdd( projTmp, tmp );
                delete projTmp;
	    }
            delete tmp;
        }
    }
    delete seed;

    return SUCCESSFUL_RETURN;
	}
}



returnValue TreeProjection::ADsymmetricProtected( int            dim       , /**< number of directions  */
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
  
  
	if (argument == 0) {
		return Projection::ADsymmetricProtected( dim, varType, component, l, S, dimS, dfS, ldf, H, nNewLIS, newLIS, nNewSIS, newSIS, nNewHIS, newHIS );
	}
	else {

	int run1;

	if( (vIndex+1)*dim-1 >= nNewLIS ){

		*newLIS = (TreeProjection**)realloc(*newLIS,((vIndex+1)*dim)*sizeof(TreeProjection*));
		for( run1 = nNewLIS; run1 < (vIndex+1)*dim; run1++ )
			newLIS[0][run1] = 0;
		nNewLIS = (vIndex+1)*dim;
	}

	if( (vIndex+1)*dimS-1 >= nNewSIS ){

		*newSIS = (TreeProjection**)realloc(*newSIS,((vIndex+1)*dimS)*sizeof(TreeProjection*));
		for( run1 = nNewSIS; run1 < (vIndex+1)*dimS; run1++ )
			newSIS[0][run1] = 0;
		nNewSIS = (vIndex+1)*dimS;
	}

	if( (vIndex+1)*dimS*dimS-1 >= nNewHIS ){

		*newHIS = (TreeProjection**)realloc(*newHIS,((vIndex+1)*dimS*dimS)*sizeof(TreeProjection*));
		for( run1 = nNewHIS; run1 < (vIndex+1)*dimS*dimS; run1++ )
			newHIS[0][run1] = 0;
		nNewHIS = (vIndex+1)*dimS*dimS;
	}
  
  // ============================================================================

	if( newLIS[0][vIndex*dim] == 0 ){

		Operator **lres = new Operator*[dim];
		for( run1 = 0; run1 < dim; run1++ ){
			lres[run1] = new DoubleConstant(0.0,NE_ZERO);
		}
		Operator *aux = new DoubleConstant(1.0,NE_ONE);

		Operator **Sres = new Operator*[dimS];
		for( run1 = 0; run1 < dimS; run1++ ){
			Sres[run1] = new DoubleConstant(0.0,NE_ZERO);
		}

		Operator **Hres = new Operator*[dimS*dimS];
		for( run1 = 0; run1 < dimS*dimS; run1++ ){
			Hres[run1] = new DoubleConstant(0.0,NE_ZERO);
		}

		argument->AD_symmetric( dim, varType, component, aux, S, dimS,
				Sres, lres, Hres,
				nNewLIS, newLIS, nNewSIS, newSIS, nNewHIS, newHIS );


		for( run1 = 0; run1 < dim; run1++ ){
			newLIS[0][vIndex*dim+run1] = new TreeProjection();
			newLIS[0][vIndex*dim+run1]->operator=(*lres[run1]);
			newLIS[0][vIndex*dim+run1]->setCurvature( CT_UNKNOWN );
			delete lres[run1];
		}
		delete[] lres;

		for( run1 = 0; run1 < dimS; run1++ ){
			newSIS[0][vIndex*dimS+run1] = new TreeProjection();
			newSIS[0][vIndex*dimS+run1]->operator=(*Sres[run1]);
			newSIS[0][vIndex*dimS+run1]->setCurvature( CT_UNKNOWN );
			delete Sres[run1];
		}
		delete[] Sres;

		for( run1 = 0; run1 < dimS*dimS; run1++ ){
			newHIS[0][vIndex*dimS*dimS+run1] = new TreeProjection();
			newHIS[0][vIndex*dimS*dimS+run1]->operator=(*Hres[run1]);
			newHIS[0][vIndex*dimS*dimS+run1]->setCurvature( CT_UNKNOWN );
			delete Hres[run1];
		}
		delete[] Hres;
	}

  // ============================================================================
  
	if( l->isOneOrZero() != NE_ZERO ){

		for( run1 = 0; run1 < dim; run1++ ){

			Operator *tmp = ldf[run1]->clone();
			delete ldf[run1];

			if( l->isOneOrZero() == NE_ONE ){
				ldf[run1] = myAdd( newLIS[0][vIndex*dim+run1], tmp );
			}
			else{
				Operator *projTmp = myProd( newLIS[0][vIndex*dim+run1], l );
				ldf[run1] = myAdd( projTmp, tmp );
				delete projTmp;
			}
			delete tmp;
		}
	}

	for( run1 = 0; run1 < dimS; run1++ ){
		delete dfS[run1];
		dfS[run1] = newSIS[0][vIndex*dimS+run1]->clone();
	}

	for( run1 = 0; run1 < dimS*dimS; run1++ ){

		delete H[run1];

		if( l->isOneOrZero() == NE_ONE ){
			H[run1] = newHIS[0][vIndex*dimS*dimS+run1]->clone();
		}
		else{
			H[run1] = myProd(newHIS[0][vIndex*dimS*dimS+run1],l);
		}
	}

	delete l;
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


Operator* TreeProjection::getArgument() const{

    if( argument != 0 ) return argument->clone();
    return 0;
}


NeutralElement TreeProjection::isOneOrZero() const{

    return ne;
}



void TreeProjection::copy( const Projection &arg ){

    Projection::copy(arg);
}


Operator* TreeProjection::passArgument() const{

    return argument;
}


returnValue TreeProjection::setVariableExportName(	const VariableType &_type,
													const std::vector< std::string >& _name
													)
{
	if (argument != 0 && argument->getName() == ON_POWER_INT)
		argument->setVariableExportName(_type, _name);

	return Projection::setVariableExportName(_type, _name);
}


BooleanType TreeProjection::isTrivial() const {

	return argument != 0;
}


returnValue TreeProjection::initDerivative() {

	if( !initialized && argument != 0 ) {
		initialized = BT_TRUE;
		return argument->initDerivative();
	}
	else {
		return SUCCESSFUL_RETURN;
	}
}


CLOSE_NAMESPACE_ACADO

// end of file.
