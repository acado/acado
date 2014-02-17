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
 *    \file   examples/getting_started/symbolic_algebra.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2009
 */

#include <acado_toolkit.hpp>

using namespace std;

USING_NAMESPACE_ACADO

void output(const char* name, const Expression& e)
{
    Function f;
    f << e;

    ofstream stream( name );
    stream << f;
    stream.close();
}

int main( ){

    IntermediateState y0;
    IntermediateState y1=y0*2+1;
    output("sym1.txt",y1);
    y0=4;
    output("sym2.txt",y1);
    y0=2;
    output("sym3.txt",y1);

    Expression E;
    IntermediateState e1(1,3);e1(0)=1;e1(1)=2;e1(2)=3;
    IntermediateState e2(1,3);e2(0)=-1;e2(1)=-2;e2(2)=-3;
    // Thus doesn't work with DifferentialState e1(3);
    E.appendRows(e1);
    printf("Shape %d x %d\n",E.getNumRows(),E.getNumCols());
    E.appendRows(e2);

    output("sym4.txt",E);

    return 0;
}


