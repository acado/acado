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
 *    \file src/symbolic_expression/parameter.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2010
 */

#include <acado/symbolic_expression/parameter.hpp>


BEGIN_NAMESPACE_ACADO

int Parameter::count = 0;


Parameter::Parameter()
          :Expression( 1, 1, VT_PARAMETER, count ){

    count++;
}


Parameter::Parameter( uint nRows_, uint nCols_, String name_ )
          :Expression( nRows_, nCols_, VT_PARAMETER, (uint) count, name_ ){

    count += nRows_*nCols_;
}


Parameter::Parameter( const Parameter &arg )
          :Expression(arg){ }


Parameter::~Parameter(){ }


Parameter& Parameter::operator=( const Parameter &arg ){

    if( this != &arg ){

        Expression::operator=(arg);

    }
    return *this;
}


Expression* Parameter::clone() const{

    return new Parameter(*this);
}


returnValue Parameter::clearStaticCounters(){

    count = 0;
    return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO

// end of file.
