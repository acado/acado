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
 *    \file examples/integration_algorithm/getting_started.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2010
 */

#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>

using namespace std;

USING_NAMESPACE_ACADO

/* >>> start tutorial code >>> */
int main( )
{
	// Define a Right-Hand-Side:
	// -------------------------
	DifferentialState x;
	DifferentialEquation f;
	TIME t;

	f << dot(x) == -x + sin(0.01 * t);

	// Define an initial value:
	// ------------------------

	DVector xStart(1);
	xStart(0) = 1.0;

	double tStart = 0.0;
	double tEnd = 1000.0;

	Grid timeHorizon(tStart, tEnd, 2);
	Grid timeGrid(tStart, tEnd, 20);

	// Define an integration algorithm:
	// --------------------------------

	IntegrationAlgorithm intAlg;

	intAlg.addStage(f, timeHorizon);

	intAlg.set(INTEGRATOR_TYPE, INT_BDF);
	intAlg.set(INTEGRATOR_PRINTLEVEL, MEDIUM);
	intAlg.set(INTEGRATOR_TOLERANCE, 1.0e-3);
	intAlg.set(PRINT_INTEGRATOR_PROFILE, YES);
	intAlg.set(PLOT_RESOLUTION, HIGH);

	GnuplotWindow window;
	window.addSubplot(x, "x");

	intAlg << window;

	// START THE INTEGRATION
	// ----------------------

	intAlg.integrate(timeHorizon, xStart);

	// GET THE RESULTS
	// ---------------

	VariablesGrid differentialStates;
	intAlg.getX(differentialStates);

	cout << "x = " << endl << differentialStates << endl;

	DVector xEnd;
	intAlg.getX(xEnd);

	cout << "xEnd = " << endl << xEnd << endl;

	return 0;
}
/* <<< end tutorial code <<< */
