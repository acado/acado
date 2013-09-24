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
*    \file src/symbolic_expression/integer_parameter.cpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date 2008
*/


#include <acado/symbolic_expression/integer_parameter.hpp>



BEGIN_NAMESPACE_ACADO


int IntegerParameter::count = 0;


IntegerParameter::IntegerParameter()
                 :Expression( 1, 1, VT_INTEGER_PARAMETER, count ){

    count++;
}


IntegerParameter::IntegerParameter( uint nRows_, uint nCols_, String name_ )
                 :Expression( nRows_, nCols_, VT_INTEGER_PARAMETER, (uint) count, name_ ){

    count += nRows_*nCols_;
}


IntegerParameter::IntegerParameter( const IntegerParameter &arg )
               :Expression(arg){ }


IntegerParameter::~IntegerParameter(){ }


IntegerParameter& IntegerParameter::operator=( const IntegerParameter &arg ){

    if( this != &arg ){

        Expression::operator=(arg);

    }
    return *this;
}


Expression* IntegerParameter::clone() const{

    return new IntegerParameter(*this);
}


returnValue IntegerParameter::clearStaticCounters(){

    count = 0;
    return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO

// end of file.
