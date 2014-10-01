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
 *    \file src/ocp/multi_objective_functionality.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */

#include <acado/ocp/multi_objective_functionality.hpp>
#include <acado/matrix_vector/matrix_vector.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>


BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//


MultiObjectiveFunctionality::MultiObjectiveFunctionality( ){

    nMayer     = 0;
    mayerTerms = 0;
}


MultiObjectiveFunctionality::MultiObjectiveFunctionality( const MultiObjectiveFunctionality& rhs ){

    int run1;
    nMayer = rhs.nMayer;

    if( nMayer > 0 ){
        mayerTerms = (Expression**)calloc(nMayer,sizeof(Expression*));
        for( run1 = 0; run1 < nMayer; run1++ ){
            mayerTerms[run1] = new Expression(*rhs.mayerTerms[run1]);
        }
    }
    else{
        mayerTerms = 0;
    }
}


MultiObjectiveFunctionality::~MultiObjectiveFunctionality( ){

    int run1;
    for( run1 = 0; run1 < nMayer; run1++ )
        if( mayerTerms[run1] != 0 ) delete mayerTerms[run1];

    if( mayerTerms != 0 )
        free(mayerTerms);
}


MultiObjectiveFunctionality& MultiObjectiveFunctionality::operator=( const MultiObjectiveFunctionality& rhs ){

    int run1;

    if ( this != &rhs ){

        for( run1 = 0; run1 < nMayer; run1++ )
            if( mayerTerms[run1] != 0 ) delete mayerTerms[run1];

        if( mayerTerms != 0 )
            free(mayerTerms);

        nMayer = rhs.nMayer;

        if( nMayer > 0 ){
            mayerTerms = (Expression**)calloc(nMayer,sizeof(Expression*));
            for( run1 = 0; run1 < nMayer; run1++ ){
                mayerTerms[run1] = new Expression( *rhs.mayerTerms[run1] );
            }
        }
        else{
            mayerTerms = 0;
        }
    }
    return *this;
}



returnValue MultiObjectiveFunctionality::minimizeMayerTerm( const int &multiObjectiveIdx,
                                                            const Expression& arg ){

    ASSERT( multiObjectiveIdx >= 0 );

    if( multiObjectiveIdx >= nMayer ){
        ASSERT( multiObjectiveIdx == nMayer );
        nMayer = multiObjectiveIdx+1;
        mayerTerms = (Expression**)realloc(mayerTerms,nMayer*sizeof(Expression*));
        mayerTerms[multiObjectiveIdx] = new Expression(arg);
        return SUCCESSFUL_RETURN;
    }
    mayerTerms[multiObjectiveIdx] = new Expression( *mayerTerms[multiObjectiveIdx] + arg );
    return SUCCESSFUL_RETURN;
}

int MultiObjectiveFunctionality::getNumberOfMayerTerms() const{

    return nMayer;
}

returnValue MultiObjectiveFunctionality::getObjective( const int &multiObjectiveIdx, Expression **arg ) const{

    ASSERT( arg != 0 );
    ASSERT( multiObjectiveIdx < nMayer);
    *arg = new Expression(*mayerTerms[multiObjectiveIdx]);

    return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO

// end of file.
