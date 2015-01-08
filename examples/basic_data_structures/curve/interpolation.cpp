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
 *    \file examples/curve/interpolation.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */


#include <acado_integrators.hpp>
#include <acado_gnuplot.hpp>

USING_NAMESPACE_ACADO

/* >>> start tutorial code >>> */
int main( )
{
    // DEFINE A VARIABLES GRID:
    // ------------------------
    Grid dataGrid( 0.0, 5.0, 6 );

    VariablesGrid data;
    data.init( 2, dataGrid );

    data( 0, 0 ) = 0.0;  data( 0, 1 ) = 1.0  ;
    data( 1, 0 ) = 0.2;  data( 1, 1 ) = 0.8  ;
    data( 2, 0 ) = 0.4;  data( 2, 1 ) = 0.7  ;
    data( 3, 0 ) = 0.6;  data( 3, 1 ) = 0.65 ;
    data( 4, 0 ) = 0.8;  data( 4, 1 ) = 0.625;
    data( 5, 0 ) = 1.0;  data( 5, 1 ) = 0.613;

    // CONSTRUCT A CURVE INTERPOLATING THE DATA:
    // -----------------------------------------

    Curve c1, c2;

    c1.add( data, IM_CONSTANT );
    c2.add( data, IM_LINEAR   );


    // PLOT CURVES ON GIVEN GRID:
    // --------------------------
    GnuplotWindow window;
         window.addSubplot( c1, 0.0,5.0, "Constant data Interpolation"   );
         window.addSubplot( c2, 0.0,5.0, "Linear data Interpolation"   );
    window.plot();

    return 0;
}
/* <<< end tutorial code <<< */


