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


TreeProjection::TreeProjection( const String &name_ )
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



TreeProjection& TreeProjection::operator=( const Operator &arg ){

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

        Operator *tmp = arg.passArgument();

        if( tmp == 0 ) argument = arg.clone() ;
        else           argument = tmp->clone();

        vIndex         = count++;
        variableIndex  = vIndex ;

        curvature      = CT_UNKNOWN; // argument->getCurvature();
        monotonicity   = MT_UNKNOWN; // argument->getMonotonicity();

        ne = argument->isOneOrZero();

        if( curvature == CT_CONSTANT )
            scale = argument->getValue();
    }

    return *this;
}


TreeProjection& TreeProjection::operator=( const Expression &arg ){

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


TreeProjection& TreeProjection::operator=( const double& arg ){

    scale = arg;

    Expression tmp;
    return operator=( tmp.convert(arg) );
}


Operator* TreeProjection::clone() const{

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

    ASSERT( argument != 0 );

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



returnValue TreeProjection::ADbackwardProtected( int dim,
                                                    VariableType *varType,
                                                    int *component,
                                                    Operator *seed,
                                                    Operator **df         ){

    ASSERT( argument != 0 );

    Operator **results = new Operator*[dim];

    int run1;

    for( run1 = 0; run1 < dim; run1++ ){
       results[run1] = new DoubleConstant(0.0,NE_ZERO);
    }

    argument->AD_backward(dim,varType,component,seed, results );

    for( run1 = 0; run1 < dim; run1++ ){

       Operator *tmp = df[run1]->clone();
       delete df[run1];
       df[run1] = new Addition( results[run1]->clone(), tmp->clone() );
       delete tmp;
    }

    for( run1 = 0; run1 < dim; run1++ )
        delete results[run1];

    delete[] results;

    return SUCCESSFUL_RETURN;
}


returnValue TreeProjection::loadIndices( SymbolicIndexList *indexList ){

    returnValue returnvalue = SUCCESSFUL_RETURN;

    if( argument == NULL ){
        ACADOERROR(RET_INTERMEDIATE_STATE_HAS_NO_ARGUMENT);
        ASSERT( 1 == 0 );
    }

    if( indexList->addNewElement( VT_INTERMEDIATE_STATE, vIndex ) == BT_TRUE ){

        returnvalue = argument->loadIndices( indexList );

        indexList->addOperatorPointer( argument, vIndex );
    }

    if( name.isEmpty() == BT_TRUE ){
        name << "a" << "[" << vIndex << "]";
    }

    return returnvalue;
}


BooleanType TreeProjection::isDependingOn( int dim,
                                                VariableType *varType,
                                                int *component,
                                                BooleanType   *implicit_dep ){

    return implicit_dep[vIndex];
}


BooleanType TreeProjection::isLinearIn( int dim,
                                             VariableType *varType,
                                             int *component,
                                             BooleanType   *implicit_dep ){

    return implicit_dep[vIndex];
}


BooleanType TreeProjection::isPolynomialIn( int dim,
                                                 VariableType *varType,
                                                 int *component,
                                                 BooleanType   *implicit_dep ){

    return implicit_dep[vIndex];
}


BooleanType TreeProjection::isRationalIn( int dim,
                                               VariableType *varType,
                                               int *component,
                                               BooleanType   *implicit_dep ){

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


returnValue TreeProjection::setVariableExportName( const VariableType &_type, const Stream *_name )
{
	if (argument->getName() == ON_POWER_INT)
		argument->setVariableExportName(_type, _name);

	return Projection::setVariableExportName(_type, _name);
}


CLOSE_NAMESPACE_ACADO

// end of file.
