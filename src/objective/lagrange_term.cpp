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
 *    \file src/objective/lagrange_term.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */

#include <acado/objective/lagrange_term.hpp>



BEGIN_NAMESPACE_ACADO




LagrangeTerm::LagrangeTerm(){

    nLagrangeTerms = 0;
    lagrangeFcn    = 0;
}


returnValue LagrangeTerm::init( const Grid &grid_ ){

    grid = grid_;
    return SUCCESSFUL_RETURN;
}


LagrangeTerm::LagrangeTerm( const LagrangeTerm& rhs ){

    int run1;
    grid           = rhs.grid          ;
    nLagrangeTerms = rhs.nLagrangeTerms;

    if( rhs.lagrangeFcn != 0 ){
        lagrangeFcn = (Expression**)calloc(nLagrangeTerms,sizeof(Expression*));
        for( run1 = 0; run1 < nLagrangeTerms; run1++ )
            lagrangeFcn[run1] = new Expression(*rhs.lagrangeFcn[run1]);
    }
    else lagrangeFcn = 0;
}


LagrangeTerm::~LagrangeTerm( ){

    int run1;

    if( lagrangeFcn != 0 ){
        for( run1 = 0; run1 < nLagrangeTerms; run1++ )
            if( lagrangeFcn[run1] != 0 ) delete lagrangeFcn[run1];
        free(lagrangeFcn);
    }
}


LagrangeTerm& LagrangeTerm::operator=( const LagrangeTerm& rhs ){

    int run1;

    if( this != &rhs ){

        if( lagrangeFcn != 0 ){
            for( run1 = 0; run1 < nLagrangeTerms; run1++ )
                if( lagrangeFcn[run1] != 0 ) delete lagrangeFcn[run1];
            free(lagrangeFcn);
        }

        grid           = rhs.grid          ;
        nLagrangeTerms = rhs.nLagrangeTerms;

        if( rhs.lagrangeFcn != 0 ){
            lagrangeFcn = (Expression**)calloc(nLagrangeTerms,sizeof(Expression*));
            for( run1 = 0; run1 < nLagrangeTerms; run1++ )
                lagrangeFcn[run1] = new Expression(*rhs.lagrangeFcn[run1]);
        }
        else lagrangeFcn = 0;
    }
    return *this;
}



CLOSE_NAMESPACE_ACADO

// end of file.
