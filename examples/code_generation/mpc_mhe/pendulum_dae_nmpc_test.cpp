/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2013 by Boris Houska, Hans Joachim Ferreau,
 *    Milan Vukov and Rien Quirynen, KU Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC) under
 *    supervision of Moritz Diehl. All rights reserved.
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

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
#include <string.h>
#include <cmath>

using namespace std;

#include "acado_common.h"
#include "acado_auxiliary_functions.h"

#define NX          ACADO_NX      /* number of differential states  */
#define NXA         ACADO_NXA      /* number of alg. states  */
#define NU          ACADO_NU      /* number of control inputs       */
#define N          	ACADO_N      /* number of control intervals    */
#define NY			ACADO_NY
#define NYN			ACADO_NYN
#define NUM_STEPS   100      /* number of real time iterations */
#define VERBOSE     1      /* show iterations: 1, silent: 0  */

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

int main()
{
	int    i, iter, j   , k     ;
	real_t measurement[ NX ];

	// Reset all memory, just for safety
	memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
	memset(&acadoVariables, 0, sizeof( acadoVariables ));

	//
	// Initialize the solver
	//
	initializeSolver();

	// 1.0 -5.0 1.0 0.1 -0.5 0.1
	// -1.5 -0.3 -0.3 -3.0 19.0

	for (i = 0; i < N + 1; ++i)
	{
		acadoVariables.x[i * NX + 0] = 1;
		acadoVariables.x[i * NX + 1] = sqrt(1.0 - 0.1 * 0.1);
		acadoVariables.x[i * NX + 2] = 0.9;
		acadoVariables.x[i * NX + 3] = 0;
		acadoVariables.x[i * NX + 4] = 0;
		acadoVariables.x[i * NX + 5] = 0;
	}

//	for (i = 0; i < N; ++i)
//	{
//		acadoVariables.z[i * NXA + 0] = -1.5;
//		acadoVariables.z[i * NXA + 1] = -0.3;
//		acadoVariables.z[i * NXA + 2] = -0.3;
//		acadoVariables.z[i * NXA + 3] = -3.0;
//		acadoVariables.z[i * NXA + 4] = 19.0;
//	}

	for (i = 0; i < N; ++i)
	{
		acadoVariables.y[i * NY + 0] = 0; // x
		acadoVariables.y[i * NY + 1] = 1.0; // y
		acadoVariables.y[i * NY + 2] = 0; // w
		acadoVariables.y[i * NY + 3] = 0;
		acadoVariables.y[i * NY + 4] = 0;
		acadoVariables.y[i * NY + 5] = 0;
		acadoVariables.y[i * NY + 6] = 0; // u
	}

	acadoVariables.yN[ 0 ] = 0; // x
	acadoVariables.yN[ 1 ] = 1.0; // y
	acadoVariables.yN[ 2 ] = 0; // w
	acadoVariables.yN[ 3 ] = 0;
	acadoVariables.yN[ 4 ] = 0;
	acadoVariables.yN[ 5 ] = 0;

	for (i = 0; i < NX; ++i)
		acadoVariables.x0[ i ] = acadoVariables.x[ i ];

	//
	// Logger initalization
	//
	vector< vector< double > > log;

	log.resize(NUM_STEPS);
	for (unsigned i = 0; i < log.size(); ++i)
		log[ i ].resize(NX + NXA + NU + 6, 0.0);

	//
	// Timing stuff
	//
	real_t t1, t2, t3, t4;
	real_t fdbSum = 0.0;
	real_t prepSum = 0.0;
	int status, nIt;

	t1 = 0; t2 = 0; t3 = 0; t4 = 0;

	timer t;

   // THE REAL-TIME ITERATION LOOP:
   // ----------------------------------------------
	for( iter = 0; iter < NUM_STEPS; iter++ )
	{
		tic( &t );
		preparationStep();
		t1 = toc( &t );

		tic( &t );
		status = feedbackStep( );
		t3 = toc( &t );

		printDifferentialVariables();
		printControlVariables();

		if ( status )
		{
			cout << "Iteration:" << iter << ", QP problem! QP status: " << status << endl;

			break;
		}

		for (unsigned ii = 0; ii < (N + 1) * NX; ++ii)
			if (acadoVariables.x[ ii ] != acadoVariables.x[ ii ])
			{
				cout << "Iteration:" << iter << ", NaN problems with diff. variables" << endl;

				exit( 1 );
			}

		for(i = 0; i < NX; i++)
			log[ iter ][ i ] = acadoVariables.x[ i ];
		for(j = 0; i < NX + NXA; i++, j++)
			log[ iter ][ i ] = acadoVariables.z[ j ];
		for(k = 0;  i < NX + NXA + NU; i++, k++)
			log[ iter ][ i ] = acadoVariables.u[ k ];

		log[ iter ][ i++ ] = t1;
		log[ iter ][ i++ ] = t2;
		log[ iter ][ i++ ] = t3;
		log[ iter ][ i++ ] = getObjective();
		log[ iter ][ i++ ] = getKKT();
		log[ iter ][ i++ ] = getNWSR();

		for (i = 0; i < NX; ++i)
			acadoVariables.x0[ i ] = acadoVariables.x[NX + i];

		shiftStates(2, 0, 0);
		shiftControls( 0 );

		prepSum += t1;
		fdbSum += t3;
	}

	cout << "Average fdbTime: " << scientific << fdbSum / NUM_STEPS * 1e6 << "usec" << endl;
	cout << "Average prepTime: " << scientific << prepSum / NUM_STEPS * 1e6 << "usec" << endl;

	ofstream dataLog( "./pendulum_dae_nmpc_test_sim_log.txt" );
	if ( dataLog.is_open() )
	{
		for (i = 0; i < log.size(); i++)
		{
			for (unsigned j = 0; j < log[ i ].size(); j++)
				dataLog << log[ i ][ j ] << " ";
			dataLog << endl;
		}

		dataLog.close();
	}
	else
	{
		cout << "Log file could not be opened" << endl;

		return 1;
	}

    return 0;
}


