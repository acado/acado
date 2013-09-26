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
#include <stdlib.h>

using namespace std;

#include "acado_common.h"
#include "acado_auxiliary_functions.h"

#define NX          ACADO_NX      /* number of differential states  */
#define NXA         ACADO_NXA      /* number of alg. states  */
#define NU          ACADO_NU      /* number of control inputs       */
#define N          	ACADO_N      /* number of control intervals    */
#define NY			ACADO_NY
#define NYN			ACADO_NYN
#define NUM_STEPS   1000      /* number of real time iterations */
#define VERBOSE     1      /* show iterations: 1, silent: 0  */

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
	int    i, iter, j   , k     ;

	// Reset all memory, just for safety
	memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
	memset(&acadoVariables, 0, sizeof( acadoVariables ));

	bool fileStatus;
	vector< vector< double > > measurements;
	fileStatus = readDataFromFile("../../crane_mhe_data.txt", measurements);
	if (fileStatus == false)
	{
		cout << "Cannot read the meas. file" << endl;
		exit( 1 );
	}
	cout << "Input array size is: " << measurements.size() << " x " << measurements[ 0 ].size() << endl;

	//
	// Initialize the solver
	//
	initializeSolver();

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

	// TODO Initialize

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
		for (i = 0; i < N; ++i)
			for (j = 0; j < NY; ++j)
				acadoVariables.y[i * NY + j] = measurements[iter + i][ j ];

		for (j = 0; j < NYN; ++j)
			acadoVariables.yN[ j ] = measurements[iter + i][ j ];

		tic( &t );
		preparationStep();
		t1 = toc( &t );

		tic( &t );
		status = feedbackStep( );
		t3 = toc( &t );

//		printDifferentialVariables();
//		printControlVariables();

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
		for(k = 0;  i < NX + NU; i++, k++)
			log[ iter ][ i ] = acadoVariables.u[ k ];

		log[ iter ][ i++ ] = t1;
		log[ iter ][ i++ ] = t2;
		log[ iter ][ i++ ] = t3;
		log[ iter ][ i++ ] = getObjective();
		log[ iter ][ i++ ] = getKKT();
		log[ iter ][ i++ ] = getNWSR();

		shiftStates(2, 0, 0);
		shiftControls( 0 );

		prepSum += t1;
		fdbSum += t3;
	}

	cout << "Average fdbTime: " << scientific << fdbSum / NUM_STEPS * 1e6 << "usec" << endl;
	cout << "Average prepTime: " << scientific << prepSum / NUM_STEPS * 1e6 << "usec" << endl;

	ofstream dataLog( "./crane_mhe_test_sim_log.txt" );
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


