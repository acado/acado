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


#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>


int main( ){

  USING_NAMESPACE_ACADO;

  // Parameters
  double h_hw = 10;    // water level
  double A_hw = 1.0;   // amplitude of the waves
  double T_hw = 5.0;   // duration of a wave
  double h_b  = 3.0;   // height of the buoy
  double rho  = 1000;  // density of water
  double A    = 1.0;   // bottom area of the buoy
  double m    = 100;   // mass of the buoy
  double g    = 9.81;  // gravitational constant

  // Free parameter
  Control u;

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
  f << dot(v) ==  rho*A*(hw-h)/m - g - u;
  f << dot(w) ==  u*v;

  // DEFINE AN OPTIMAL CONTROL PROBLEM:
  // ----------------------------------

  const double t_start    =  0.0        ;
  const double t_end      =  15;

  OCP ocp( t_start, t_end, 100 );
  ocp.maximizeMayerTerm( w );
  ocp.subjectTo( f );
  
//   double x_start[3];
//   x_start[0] = h_hw - 0*A_hw;
//   x_start[1] = 0;
//   x_start[2] = 0;

  ocp.subjectTo( AT_START, h - (h_hw-A_hw) ==  0.0 );
  ocp.subjectTo( AT_START, v ==  0.0 );
  ocp.subjectTo( AT_START, w ==  0.0 );
  
  ocp.subjectTo( -h_b <= h-hw <= 0.0 );
  ocp.subjectTo( 0.0 <= u <= 100.0 );
  
  

      // DEFINE A PLOT WINDOW:
    // ---------------------
  GnuplotWindow window;
  window.addSubplot( h,"Height of buoy" );
  window.addSubplot( v,"Velocity of buoy" );
  window.addSubplot( w,"Objective function " );
  window.addSubplot( u,"Resistance" );
  window.addSubplot( hw,"Wave height" );
  //  window.addSubplot( PLOT_KKT_TOLERANCE,"KKT Tolerance" );

  // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
  // ---------------------------------------------------
  OptimizationAlgorithm algorithm(ocp);
  algorithm.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN );
  algorithm.set( MAX_NUM_ITERATIONS, 100 );
  //  algorithm.set( KKT_TOLERANCE, 1e-10 );
  
  algorithm << window;
  algorithm.solve();

  return 0;
}



