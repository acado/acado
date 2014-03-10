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
 *    \file   src/symbolic_operator/binary_operator.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2010
 */


#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>


BEGIN_NAMESPACE_ACADO


BinaryOperator::BinaryOperator( ) : SmoothOperator( ){ }

BinaryOperator::BinaryOperator( const SharedOperator &_argument1, const SharedOperator &_argument2 ) : SmoothOperator( )
{
    argument1          = _argument1                      ;
    argument2          = _argument2                      ;
    argument1_result  = (double*)calloc(1,sizeof(double));
    argument2_result  = (double*)calloc(1,sizeof(double));
    dargument1_result = (double*)calloc(1,sizeof(double));
    dargument2_result = (double*)calloc(1,sizeof(double));
    bufferSize        = 1                                ;
    curvature         = CT_UNKNOWN                       ;
    monotonicity      = MT_UNKNOWN                       ;
}


BinaryOperator::BinaryOperator( const BinaryOperator &arg ){

    argument1  = arg.argument1;
    argument2  = arg.argument2;
    copy( arg );
}


BinaryOperator::~BinaryOperator(){

    deleteAll();
}


BinaryOperator& BinaryOperator::operator=( const BinaryOperator &arg ){

    if( this != &arg ){
        deleteAll();
        copy( arg );
    }
    return *this;
}


NeutralElement BinaryOperator::isOneOrZero() const{ return NE_NEITHER_ONE_NOR_ZERO; }

BooleanType BinaryOperator::isDependingOn( VariableType var ) const{

    if( argument1->isDependingOn(var) == BT_FALSE &&
        argument2->isDependingOn(var) == BT_FALSE )
        return BT_FALSE;

    return BT_TRUE;
}


BooleanType BinaryOperator::isDependingOn( int dim,
                                           VariableType *varType,
                                           int *component,
                                           BooleanType   *implicit_dep ){

    if( argument1->isDependingOn( dim, varType, component, implicit_dep ) == BT_TRUE ||
        argument2->isDependingOn( dim, varType, component, implicit_dep ) == BT_TRUE ){
        return BT_TRUE;
    }

    return BT_FALSE;
}


returnValue BinaryOperator::setMonotonicity( MonotonicityType monotonicity_ ){

    monotonicity = monotonicity_;
    return SUCCESSFUL_RETURN;
}


returnValue BinaryOperator::setCurvature( CurvatureType curvature_ ){

    curvature = curvature_;
    return SUCCESSFUL_RETURN;
}


returnValue BinaryOperator::clearBuffer(){

    if( bufferSize > 1 ){
        bufferSize = 1;
        argument1_result  = (double*)realloc( argument1_result,bufferSize*sizeof(double));
        argument2_result  = (double*)realloc( argument2_result,bufferSize*sizeof(double));
        dargument1_result = (double*)realloc(dargument1_result,bufferSize*sizeof(double));
        dargument2_result = (double*)realloc(dargument2_result,bufferSize*sizeof(double));
    }

    return SUCCESSFUL_RETURN;
}



returnValue BinaryOperator::enumerateVariables( SymbolicIndexList *indexList ){

    returnValue returnvalue;
    returnvalue = argument1->enumerateVariables( indexList );
    if( returnvalue != SUCCESSFUL_RETURN ){
        return returnvalue;
    }

    return argument2->enumerateVariables( indexList );
}


BooleanType BinaryOperator::isVariable( VariableType &varType, int &component ) const{

    return BT_FALSE;
}


returnValue BinaryOperator::loadIndices( SymbolicIndexList *indexList ){

    returnValue returnvalue;

    returnvalue = argument1->loadIndices( indexList );

    if( returnvalue != SUCCESSFUL_RETURN ){
        return returnvalue;
    }

    return argument2->loadIndices( indexList );
}


BooleanType BinaryOperator::isSymbolic() const{

    if( argument1->isSymbolic() == BT_FALSE ) return BT_FALSE;
    if( argument2->isSymbolic() == BT_FALSE ) return BT_FALSE;

    return BT_TRUE;
}



// //
// // PROTECTED MEMBER FUNCTIONS:
// // ---------------------------


void BinaryOperator::copy( const BinaryOperator &arg ){

    int run1;

    bufferSize = arg.bufferSize;

    dargument1 = arg.dargument1;
    dargument2 = arg.dargument2;

    argument1_result  = (double*)calloc(bufferSize,sizeof(double));
    argument2_result  = (double*)calloc(bufferSize,sizeof(double));
    dargument1_result = (double*)calloc(bufferSize,sizeof(double));
    dargument2_result = (double*)calloc(bufferSize,sizeof(double));

    for( run1 = 0; run1 < bufferSize; run1++ ){

        argument1_result[run1] = arg.argument1_result[run1];
        argument2_result[run1] = arg.argument2_result[run1];
       dargument1_result[run1] = arg.dargument1_result[run1];
       dargument2_result[run1] = arg.dargument2_result[run1];

    }
    curvature         = arg.curvature   ;
    monotonicity      = arg.monotonicity;
}


void BinaryOperator::deleteAll(){

    free(  argument1_result );
    free(  argument2_result );
    free( dargument1_result );
    free( dargument2_result );
}

returnValue BinaryOperator::setVariableExportName(	const VariableType &_type,
													const std::vector< std::string >& _name
													)
{
	argument1->setVariableExportName(_type, _name);
	argument2->setVariableExportName(_type, _name);

	return Operator::setVariableExportName(_type, _name);
}


returnValue BinaryOperator::initDerivative() {

	argument1->initDerivative();
	return argument2->initDerivative();
}

CLOSE_NAMESPACE_ACADO

// end of file.
