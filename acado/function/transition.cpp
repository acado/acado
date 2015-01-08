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
 *    \file src/function/transition.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *
 */

#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/function/function_.hpp>
#include <acado/function/transition.hpp>


BEGIN_NAMESPACE_ACADO




//
// PUBLIC MEMBER FUNCTIONS:
//

Transition::Transition( )
           :Function(){

    counter   = 0;
    component = 0;
}

Transition::Transition( const Transition& arg )
           :Function(arg){

    int run1;
    counter = arg.counter;
    if( arg.component != 0 ){
        component = (int*)calloc(counter,sizeof(int));
        for( run1 = 0; run1 < counter; run1++ )
            component[run1] = arg.component[run1];
    }
    else component = 0;
}

Transition::~Transition( ){

    if( component != 0 ) free(component);
}


Transition& Transition::operator=( const Transition& arg ){

    if ( this != &arg ){

        if( component != 0 )
            free(component);

        Function::operator=( arg );

        counter = arg.counter;

        if( arg.component == 0 ){
              component = 0;
        }
        else{
            int run1;
            component = (int*)calloc(counter,sizeof(int));
            for( run1 = 0; run1 < counter; run1++ ){
                component[run1] = arg.component[run1];
            }
        }
    }
    return *this;
}


Transition& Transition::operator<<( const DifferentialState& arg ){

    uint run1;
    component = (int*)realloc(component,(counter+arg.getDim())*sizeof(int));

    for( run1 = 0; run1 < arg.getDim(); run1++ ){
        counter++;
        component[counter-1] = arg.getComponent(run1);
    }
    return *this;
}


Transition& Transition::operator==( const Expression& arg ){

    Function::operator<<(arg);
    return *this;
}


Transition& Transition::operator==( const double &arg ){

    Expression tmp;
    tmp = arg;
    return operator==(tmp);
}


Transition& Transition::operator==( const DVector& arg ){

    uint run1;

    for( run1 = 0; run1 < arg.getDim(); run1++ )
        operator==( arg(run1) );

    return *this;
}


Transition& Transition::operator==( const DMatrix& arg ){

    uint run1, run2;

    for( run1 = 0; run1 < arg.getNumRows(); run1++ )
        for( run2 = 0; run2 < arg.getNumCols(); run2++ )
        operator==( arg(run1,run2) );

    return *this;
}





CLOSE_NAMESPACE_ACADO

// end of file.
