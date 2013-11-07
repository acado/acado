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
 *    \file include/acado/symbolic_expression/expression.ipp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */


//
// PUBLIC MEMBER FUNCTIONS:
//


BEGIN_NAMESPACE_ACADO



inline unsigned int Expression::getDim( ) const{

    return dim;
}

inline unsigned int Expression::getNumRows( ) const{

    return nRows;
}

inline unsigned int Expression::getNumCols( ) const{

    return nCols;
}

inline unsigned int Expression::getComponent( const unsigned int idx ) const{

    ASSERT( idx < getDim() );
    return component + idx;
}

inline VariableType Expression::getVariableType( ) const{

    return variableType;
}

inline BooleanType Expression::isVariable( ) const{

    if( getVariableType() == VT_UNKNOWN            ) return BT_FALSE;
    if( getVariableType() == VT_INTERMEDIATE_STATE ) return BT_FALSE;
    return BT_TRUE;
}


inline Operator* Expression::getOperatorClone( uint idx ) const{

    ASSERT( idx < getDim() );

    Operator *tmp = element[idx]->passArgument();
    if( tmp == 0 ) tmp = element[idx];

    return tmp->clone();
}


inline TreeProjection Expression::getTreeProjection( const uint &idx, String name_ ) const{

    TreeProjection tmp( name_ );
    tmp.operator=( *element[idx] );
    return tmp;
}


CLOSE_NAMESPACE_ACADO

// end of file.
