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
 *    \file examples/integrator/harmonic_oscillator.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */


#include <acado_integrators.hpp>
#include <acado_gnuplot.hpp>


int main( ){

  USING_NAMESPACE_ACADO;

  // Parameters
  double h_hw = 10;    // water level
  double A_hw = 1.0;   // amplitude of the waves
  double T_hw = 5.0;   // duration of a wave
  double rho  = 1000;  // density of water
  double A    = 1.0;   // bottom area of the buoy
  double m    = 100;   // mass of the buoy
  double g    = 9.81;  // gravitational constant

  // Free varameter
  double a = 1.0;      // take to be constant here

  // Variables
  DifferentialState h; // Position of the buoy
  DifferentialState v; // Velocity of the buoy
  DifferentialState w; // Produced wave energy
  TIME t;

  // Differential equation
  DifferentialEquation f;

  // Height of the wave
  IntermediateState hw;
  hw = h_hw + A_hw*sin(2*M_PI*t/T_hw);
  f << dot(h) ==  v;
  f << dot(v) ==  rho*A*(hw-h)/m - g - a*v;
  f << dot(w) ==  a*v*v;

  // Define an integrator:
  // ---------------------
  IntegratorRK45 integrator( f );

  // Define an initial value:
  // ------------------------
  double x_start[3];
  x_start[0] = h_hw - 2*A_hw;
  x_start[1] = 0;
  x_start[2] = 0;

  Grid timeInterval( 0.0, 25.0, 200 );

  integrator.set( INTEGRATOR_PRINTLEVEL, MEDIUM );
  integrator.integrate( timeInterval, x_start );

  VariablesGrid differentialStates;
  VariablesGrid intermediateStates;

  integrator.getX ( differentialStates );
  integrator.getI ( intermediateStates );

  GnuplotWindow window;
      window.addSubplot( differentialStates(0) );
      window.addSubplot( differentialStates(1) );
      window.addSubplot( differentialStates(2) );

  window.plot();

  return 0;
}



