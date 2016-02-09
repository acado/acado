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

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
#include <cstring>
#include <cmath>
#include <cstdlib>

using namespace std;

#include "acado_common.h"
#include "acado_auxiliary_functions.h"

#define NX          ACADO_NX	/* number of differential states */
#define NXA         ACADO_NXA	/* number of algebraic states */
#define NU          ACADO_NU	/* number of control inputs */
#define N          	ACADO_N		/* number of control intervals */
#define NY			ACADO_NY	/* number of measurements, nodes 0..N-1 */
#define NYN			ACADO_NYN	/* number of measurements, node N */
#define NUM_STEPS   100		/* number of simulation steps */
#define VERBOSE     1			/* show iterations: 1, silent: 0  */

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

bool readDataFromFile( const char* fileName, vector< vector< double > >& data )
{
	ifstream file( fileName );
	string line;

	if ( file.is_open() )
	{
		while( getline(file, line) )
		{
			istringstream linestream( line );
			vector< double > linedata;
			double number;

			while( linestream >> number )
			{
				linedata.push_back( number );
			}

			data.push_back( linedata );
		}

		file.close();
	}
	else
		return false;

	return true;
}

int main()
{
	unsigned i, j, iter;

	real_t t1, t2;
	real_t fdbSum = 0.0;
	real_t prepSum = 0.0;
	int status;

	t1 = t2 = 0;

	acado_timer t;

	// Reset all solver memory
	memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
	memset(&acadoVariables, 0, sizeof( acadoVariables ));

	vector< vector< double > > measurements;
	if (readDataFromFile("./crane_kul_mhe_data.txt", measurements) == false)
	{
		cout << "Cannot read measurements" << endl;
		return EXIT_FAILURE;
	}

	//
	// Initialize the solver
	//
	acado_initializeSolver();

	// Init states with measurements
	for (i = 0; i < N + 1; ++i)
	{
		acadoVariables.x[i * NX + 0] = measurements[ i ][ 0 ];
		acadoVariables.x[i * NX + 2] = measurements[ i ][ 1 ];
		acadoVariables.x[i * NX + 4] = measurements[ i ][ 2 ];

		acadoVariables.x[i * NX + 5] = measurements[ i ][ 3 ];
		acadoVariables.x[i * NX + 6] = measurements[ i ][ 4 ];
	}

	for (i = 0; i < N; ++i)
	{
		acadoVariables.u[i * NU + 0] = measurements[ i ][ 5 ];
		acadoVariables.u[i * NU + 1] = measurements[ i ][ 6 ];
	}

	//
	// Logger initalization
	//
	vector< vector< double > > log;

	log.resize(NUM_STEPS);
	for (i = 0; i < log.size(); ++i)
		log[ i ].resize(NX + NXA + NU + 5, 0.0);

	//
	// Warm-up the solver
	//
	acado_preparationStep();

	//
	// Main simulation loop
	//
	for( iter = 0; iter < NUM_STEPS; iter++ )
	{
		//
		// Read new measurements
		//
		for (i = 0; i < N; ++i)
			for (j = 0; j < NY; ++j)
				acadoVariables.y[i * NY + j] = measurements[iter + i][ j ];

		for (j = 0; j < NYN; ++j)
			acadoVariables.yN[ j ] = measurements[iter + i][ j ];

		//
		// Run the feedback step
		//
		acado_tic( &t );
		status = acado_feedbackStep( );
		t2 = acado_toc( &t );

#if VERBOSE
//		printDifferentialVariables();
//		printControlVariables();
#endif // VERBOSE

		if ( status )
		{
			cout << "Iteration:" << iter << ", QP problem! QP status: " << status << endl;

			break;
		}

		//
		// Logging
		//
		for(i = 0; i < NX; i++)
			log[ iter ][ i ] = acadoVariables.x[ i ];
		for(j = 0;  i < NX + NU; i++, j++)
			log[ iter ][ i ] = acadoVariables.u[ j ];

		log[ iter ][ i++ ] = t1;
		log[ iter ][ i++ ] = t2;
		log[ iter ][ i++ ] = acado_getObjective();
		log[ iter ][ i++ ] = acado_getKKT();
		log[ iter ][ i++ ] = acado_getNWSR();

#if VERBOSE
		cout	<< "Iteration #" << setw( 4 ) << iter
				<< ", KKT value: " << scientific << acado_getKKT()
				<< ", objective value: " << scientific << acado_getObjective()
				<< endl;
#endif // VERBOSE

		//
		// Prepare for the next simulation step
		//

		// Shift states and controls
		acado_shiftStates(2, 0, 0);
		acado_shiftControls( 0 );

		// Execute the preparation step of the RTI scheme
		acado_tic( &t );
		acado_preparationStep();
		t1 = acado_toc( &t );

		//
		// More logging
		//
		prepSum += t1;
		fdbSum += t2;
	}

#if VERBOSE
	cout << "Average feedback time:    " << scientific << fdbSum / NUM_STEPS * 1e6 << " microseconds" << endl;
	cout << "Average preparation time: " << scientific << prepSum / NUM_STEPS * 1e6 << " microseconds" << endl;
#endif // VERBOSE

	//
	// Save log to a file
	//
	ofstream dataLog( "./pendulum_dae_nmpc_test_sim_log.txt" );
	if ( dataLog.is_open() )
	{
		for (i = 0; i < log.size(); i++)
		{
			for (j = 0; j < log[ i ].size(); j++)
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

	// For debugging
	if ( status )
	{
		cout << "Solver failed!" << endl;
		return EXIT_FAILURE;
	}

    return EXIT_SUCCESS;
}


