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
 *    \file external_packages/acado_gnuplot/gnuplot_window.cpp
 *    \author Boris Houska, Hans Joachim Ferreau, Milan Vukov
 *    \date   2009 - 2013
 */


#include <acado/bindings/acado_gnuplot/gnuplot_window.hpp>

#ifdef WIN32
#include <windows.h>

#define round( value ) floor( value + 0.5 )
#else
#include <unistd.h>
#endif

#include <cstdlib>
#include <string>

using namespace std;

BEGIN_NAMESPACE_ACADO

//
// PUBLIC MEMBER FUNCTIONS:
//


int GnuplotWindow::counter = 0;


GnuplotWindow::GnuplotWindow( ) : PlotWindow()
{
	gnuPipe = 0;
	mouseEvent = BT_FALSE;

	++counter;
}


GnuplotWindow::GnuplotWindow(	PlotFrequency _frequency
								) : PlotWindow( _frequency )
{
	gnuPipe = 0;
	mouseEvent = BT_FALSE;

	++counter;
}


GnuplotWindow::GnuplotWindow( const GnuplotWindow& arg ) : PlotWindow( arg )
{
    gnuPipe   = 0;

    mouseEvent = arg.mouseEvent;
}


GnuplotWindow::~GnuplotWindow( ){

    if( gnuPipe != 0 )
		fclose(gnuPipe);
}


GnuplotWindow& GnuplotWindow::operator=( const GnuplotWindow& arg ){

    if( this != &arg ){

		PlotWindow::operator=( arg );

        if( gnuPipe != 0 )
			fclose(gnuPipe);

        gnuPipe   = 0;

        mouseEvent = arg.mouseEvent;
    }

    return *this;
}


PlotWindow* GnuplotWindow::clone( ) const
{
	return new GnuplotWindow( *this );
}


returnValue GnuplotWindow::init()
{
	if (gnuPipe != 0)
	{
		fclose(gnuPipe);
		gnuPipe = 0;
	}

#ifndef __NO_PIPES__
	gnuPipe = popen("gnuplot -persist -background white", "w");

	// TODO In principle, we should just print out a warning, plot is not going
	// to generated anyways.
	if ( !gnuPipe )
		ACADOWARNING( RET_PLOT_WINDOW_CAN_NOT_BE_OPEN );
#endif

	return SUCCESSFUL_RETURN;
}


returnValue GnuplotWindow::replot(	PlotFrequency _frequency
									)
{
	if ( ( _frequency == PLOT_IN_ANY_CASE ) || ( _frequency == getPlotFrequency( ) ) )
	{
		if( gnuPipe == 0 )
		{
			init();
		}

		return sendDataToGnuplot( );
	}

	return SUCCESSFUL_RETURN;
}


returnValue GnuplotWindow::waitForMouseEvents(){

    mouseEvent = BT_TRUE;
    return SUCCESSFUL_RETURN;
}


BooleanType GnuplotWindow::getMouseEvent( double &mouseX, double &mouseY )
{
    DVector tmp;
    if (tmp.read( "mouse.dat" ) != SUCCESSFUL_RETURN)
    	return BT_FALSE;

    mouseX = tmp( tmp.getDim()-2 );
    mouseY = tmp( tmp.getDim()-1 );

	if ( system("rm mouse.dat") )
		return BT_FALSE;

    return BT_TRUE;
}


returnValue GnuplotWindow::waitForMouseEvent( double &mouseX, double &mouseY ){

	if (gnuPipe == 0)
	{
		ACADOWARNING( RET_NOT_YET_IMPLEMENTED );
		return SUCCESSFUL_RETURN;
	}

    FILE *check;
    check = fopen( "mouse.dat", "r" );
	
    if( check != 0 )
    {
        fclose( check );

        if (system("rm mouse.dat") )
        	return RET_PLOTTING_FAILED;
    }

    fprintf(gnuPipe,"pause mouse\n");
    fflush(gnuPipe);
    fprintf(gnuPipe,"a = MOUSE_X \n");
    fflush(gnuPipe);
    fprintf(gnuPipe,"b = MOUSE_Y \n");
    fflush(gnuPipe);
    fprintf(gnuPipe,"save var 'mouse.dat'\n");
    fflush(gnuPipe);

    while(1){
        if( getMouseEvent(mouseX,mouseY) == BT_TRUE ){
            break;
        }
        usleep(20000);
    }
    return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//


returnValue GnuplotWindow::sendDataToGnuplot( )
{
#ifndef __NO_PLOTTING__

	// if gnuPipe is 0: open a temporary file
	BooleanType toFile = (BooleanType)( gnuPipe == 0 );
	if( toFile == BT_TRUE )
		gnuPipe = fopen( "acado2gnuplot_tmp.dat","w" );

	uint run1, run2, run3, run4;

	uint nRows;
	uint nCols;

	switch ( getNumSubplots( ) )
	{
		case 0:
			nRows = 0;
			nCols = 0;
			break;

		case 1:
			nRows = 1;
			nCols = 1;
			break;

		case 2:
			nRows = 2;
			nCols = 1;
			break;

		case 3:
		case 4:
			nRows = 2;
			nCols = 2;
			break;

		case 5:
		case 6:
			nRows = 3;
			nCols = 2;
			break;

		case 7:
		case 8:
		case 9:
			nRows = 3;
			nCols = 3;
			break;

		default:
			nRows = 4;
			nCols = 3;
			break;
    }

	VariableType myType;

	Grid discretizationGrid;
	VariablesGrid dataGrid, dataGridX, dataGridTmp;
	MatrixVariablesGrid dataMatrixGrid;

	string plotDataString;
	string userDataString;
	string plotModeString;
	string plotStyleString;

	returnValue returnvalue;

    fprintf( gnuPipe,"set multiplot;\n" );

	run1 = 0;
	for( run3 = 0; run3<nRows; run3++ )
	{
		for( run2 = 0; run2<nCols; run2++ )
		{
			if( run1 < getNumSubplots( ) )
			{
				discretizationGrid.init( );

				// plot symbolic variables or user-specified variables grids
				switch ( operator()(run1).getSubPlotType( ) )
				{
					// plot symbolic variable
					case SPT_VARIABLE:
						if ( getVariableDataGrids( operator()( run1 ).plotVariableY,myType,dataGrid,discretizationGrid ) != SUCCESSFUL_RETURN )
							return ACADOERROR( RET_PLOTTING_FAILED );
						break;

					case SPT_VARIABLE_VARIABLE:
						if ( getVariableDataGrids( operator()( run1 ).plotVariableX,myType,dataGridX,discretizationGrid ) != SUCCESSFUL_RETURN )
							return ACADOERROR( RET_PLOTTING_FAILED );

						if ( getVariableDataGrids( operator()( run1 ).plotVariableY,myType,dataGrid,discretizationGrid ) != SUCCESSFUL_RETURN )
							return ACADOERROR( RET_PLOTTING_FAILED );

						dataGridX.appendValues( dataGrid );
						dataGrid = dataGridX;
						break;

					case SPT_VARIABLE_EXPRESSION:
						if ( getVariableDataGrids( operator()( run1 ).plotVariableX,myType,dataGridX,discretizationGrid ) != SUCCESSFUL_RETURN )
							return ACADOERROR( RET_PLOTTING_FAILED );

						if ( getExpressionDataGrids( operator()( run1 ).plotExpressionY,myType,dataGrid,discretizationGrid ) != SUCCESSFUL_RETURN )
							return ACADOERROR( RET_PLOTTING_FAILED );

						dataGridX.appendValues( dataGrid );
						dataGrid = dataGridX;
						break;

					// plot user-specified variables grid
					case SPT_VARIABLES_GRID:
						if ( getDataGrids( operator()(run1).plotVariablesGrid,myType,dataGrid,discretizationGrid ) != SUCCESSFUL_RETURN )
							return ACADOERROR( RET_PLOTTING_FAILED );
						break;

					case SPT_EXPRESSION:
						if ( getExpressionDataGrids( operator()( run1 ).plotExpressionY,myType,dataGrid,discretizationGrid ) != SUCCESSFUL_RETURN )
							return ACADOERROR( RET_PLOTTING_FAILED );
						break;

					case SPT_EXPRESSION_EXPRESSION:
						if ( getExpressionDataGrids( operator()( run1 ).plotExpressionX,myType,dataGridX,discretizationGrid ) != SUCCESSFUL_RETURN )
							return ACADOERROR( RET_PLOTTING_FAILED );

						if ( getExpressionDataGrids( operator()( run1 ).plotExpressionY,myType,dataGrid,discretizationGrid ) != SUCCESSFUL_RETURN )
							return ACADOERROR( RET_PLOTTING_FAILED );

						dataGridX.appendValues( dataGrid );
						dataGrid = dataGridX;
						break;

					case SPT_EXPRESSION_VARIABLE:
						if ( getExpressionDataGrids( operator()( run1 ).plotExpressionX,myType,dataGridX,discretizationGrid ) != SUCCESSFUL_RETURN )
							return ACADOERROR( RET_PLOTTING_FAILED );

						if ( getVariableDataGrids( operator()( run1 ).plotVariableY,myType,dataGrid,discretizationGrid ) != SUCCESSFUL_RETURN )
							return ACADOERROR( RET_PLOTTING_FAILED );

						dataGridX.appendValues( dataGrid );
						dataGrid = dataGridX;
						break;

					case SPT_ENUM:
						myType = operator()( run1 ).getYVariableType( );

						returnvalue = plotDataRecord.getAll( convertPlotToLogName( operator()( run1 ).getPlotEnum( ) ),dataMatrixGrid );
						if( returnvalue != SUCCESSFUL_RETURN )
							return ACADOERROR( returnvalue );

						dataGrid = dataMatrixGrid;
						break;

					default:
						ACADOERROR( RET_UNKNOWN_BUG );
				}


				// define subplot position
				fprintf(gnuPipe,"      set size   %.16e,%.16e;\n", (1.0 /nCols), (1.0 /nRows) );
				fprintf(gnuPipe,"      set origin %.16e,%.16e;\n", (double) run2/((double) nCols),
															(double) (nRows-1-run3)/((double) nRows) );

				// define subplot title
				if ( operator()( run1 ).title.empty() == false )
					fprintf(gnuPipe,"      set title '%s'\n", operator()( run1 ).title.c_str() );
				else
					fprintf(gnuPipe,"      set title ' '\n"  );

				// define subplot axes labels
				if ( operator()( run1 ).xLabel.empty() == false )
					fprintf(gnuPipe,"      set xlabel '%s'\n", operator()( run1 ).xLabel.c_str() );
				else
					fprintf(gnuPipe,"      set xlabel ' '\n"  );

				if ( operator()( run1 ).yLabel.empty() == false )
					fprintf(gnuPipe,"      set ylabel '%s'\n", operator()( run1 ).yLabel.c_str() );
				else
					fprintf(gnuPipe,"      set ylabel ' '\n"  );

				if( ( acadoIsFinite( operator()( run1 ).xRangeLowerLimit ) == BT_TRUE ) &&
					( acadoIsFinite( operator()( run1 ).xRangeUpperLimit ) == BT_TRUE ) )
				{
					fprintf(gnuPipe,"      set xrange [%.16e:%.16e]\n", operator()( run1 ).xRangeLowerLimit, operator()( run1 ).xRangeUpperLimit );
				}
				else
				{
					fprintf(gnuPipe,"      set autoscale x\n" );
				}

				if( ( acadoIsFinite( operator()( run1 ).yRangeLowerLimit ) == BT_TRUE ) &&
					( acadoIsFinite( operator()( run1 ).yRangeUpperLimit ) == BT_TRUE ) )
				{
					fprintf(gnuPipe,"      set yrange [%.16e:%.16e]\n", operator()( run1 ).yRangeLowerLimit, operator()( run1 ).yRangeUpperLimit );
				}
				else
				{
					if ( ( operator()( run1 ).nLines == 0 ) && ( operator()( run1 ).nData == 0 ) )
					{
						double lowerLimit, upperLimit;
						getAutoScaleYLimits( dataGrid,operator()( run1 ).plotFormat,lowerLimit,upperLimit );
						fprintf(gnuPipe,"      set yrange [%.16e:%.16e]\n", lowerLimit, upperLimit );
					}
					else
					{
						fprintf(gnuPipe,"      set autoscale y\n" );
					}
				}

				if ( operator()( run1 ).getPlotFormat( ) == PF_LOG )
					fprintf(gnuPipe,"      set logscale y\n" );

				if ( operator()( run1 ).plot3D == BT_FALSE )
					fprintf(gnuPipe,"      plot ");
				else
					fprintf(gnuPipe,"      splot ");

				// plot lines
				if ( operator()( run1 ).nLines > 0 )
					for( run4 = 0; run4 < operator()( run1 ).nLines; run4++ )
						fprintf(gnuPipe,"%.16e title '' lt 2 lw 2,\\\n", operator()( run1 ).lineValues[run4] );


				getPlotModeString(operator()(run1).plotMode, plotModeString);
				getPlotStyleString(myType, plotStyleString);

				switch ( operator()(run1).getSubPlotType( ) )
				{
					case SPT_VARIABLE:
					case SPT_EXPRESSION:
						if ( discretizationGrid.getNumPoints() > 1 )
							for( run4=0; run4<discretizationGrid.getNumPoints()-2; ++run4 )
								fprintf(gnuPipe,"'-' using 1:2 title '' with %s %s,\\\n", plotModeString.c_str(),plotStyleString.c_str() );

						fprintf(gnuPipe,"'-' using 1:2 title '' with %s %s", plotModeString.c_str(),plotStyleString.c_str() );
						break;

					case SPT_VARIABLE_VARIABLE:
					case SPT_VARIABLE_EXPRESSION:
					case SPT_EXPRESSION_EXPRESSION:
					case SPT_EXPRESSION_VARIABLE:
						if ( discretizationGrid.getNumPoints() > 1 )
							for( run4=0; run4<discretizationGrid.getNumPoints()-2; ++run4 )
								fprintf(gnuPipe,"'-' using 2:3 title '' with %s %s,\\\n", plotModeString.c_str(),plotStyleString.c_str() );

						fprintf(gnuPipe,"'-' using 2:3 title '' with %s %s", plotModeString.c_str(),plotStyleString.c_str() );
						break;

					case SPT_VARIABLES_GRID:
						if ( dataGrid.getNumValues( ) == 0 )
							return ACADOERROR( RET_EMPTY_PLOT_DATA );

						if( operator()( run1 ).plot3D == BT_FALSE )
						{
// 							for( run4=0; run4<dataGrid.getNumValues( )-1; ++run4 )
// 								fprintf(gnuPipe,"'-' using 1:2 title '' with %s %s,\\\n", plotModeString,plotStyleString );
							if ( discretizationGrid.getNumPoints() > 1 )
								for( run4=0; run4<discretizationGrid.getNumPoints()-2; ++run4 )
									fprintf(gnuPipe,"'-' using 1:2 title '' with %s %s,\\\n", plotModeString.c_str(),plotStyleString.c_str() );

							fprintf(gnuPipe,"'-' using 1:2 title '' with %s %s", plotModeString.c_str(),plotStyleString.c_str() );
						}
						else
						{
							if ( dataGrid.getNumValues( ) != 2 )
								return ACADOERROR( RET_INVALID_ARGUMENTS );

							fprintf(gnuPipe,"'-' title '' with %s", plotModeString.c_str() );
						}
						break;

					default:
						fprintf(gnuPipe,"'-' using 1:2 title '' with %s %s", plotModeString.c_str(),plotStyleString.c_str() );
				}

				// plot user-specified data
				if ( operator()(run1).nData > 0 )
				{
					returnvalue = obtainPlotDataString( *(operator()(run1).data[0]),userDataString );
					if( returnvalue != SUCCESSFUL_RETURN )
						return ACADOERROR( returnvalue );
				}

				if( userDataString.empty() == false )
					fprintf(gnuPipe,",\\\n'-' using 1:2 title '' with points lt 3 lw 3\n" );
				else
					fprintf(gnuPipe,"\n");

				if ( discretizationGrid.getNumPoints( ) == 0 )
				{
					if ( dataGrid.getNumValues( ) == 1 )
					{
						returnvalue = obtainPlotDataString( dataGrid,plotDataString );
						if( returnvalue != SUCCESSFUL_RETURN )
							return ACADOERROR( returnvalue );

						fprintf( gnuPipe, "%s", plotDataString.c_str() );
						fprintf( gnuPipe, "e\n" );
					}
					else
					{
						for( run4=0; run4<dataGrid.getNumValues( ); ++run4 )
						{
							dataGridTmp = dataGrid(run4);
							returnvalue = obtainPlotDataString( dataGridTmp,plotDataString );
							if( returnvalue != SUCCESSFUL_RETURN )
								return ACADOERROR( returnvalue );

							fprintf( gnuPipe, "%s", plotDataString.c_str() );
							fprintf( gnuPipe, "e\n" );
						}
					}
				}
				else
				{
					uint startIdx,endIdx;
			
					for( run4=0; run4<discretizationGrid.getNumPoints()-1; ++run4 )
					{
						startIdx = (uint)round( discretizationGrid.getTime( run4 ) );
						endIdx   = (uint)round( discretizationGrid.getTime( run4+1 ) ) - 1;
						dataGridTmp = dataGrid.getTimeSubGrid( startIdx,endIdx );

						returnvalue = obtainPlotDataString( dataGridTmp, plotDataString );
						if( returnvalue != SUCCESSFUL_RETURN )
							return ACADOERROR( returnvalue );

						fprintf( gnuPipe, "%s", plotDataString.c_str() );
						fprintf( gnuPipe, "e\n" );
					}
				}

				if ( operator()( run1 ).getPlotFormat( ) == PF_LOG )
					fprintf(gnuPipe,"      unset logscale y\n" );
			}

			fprintf( gnuPipe,"\n" );
			fflush( gnuPipe );

			++run1;
		}
	}

	fprintf( gnuPipe,"unset multiplot\n" );

	fflush( gnuPipe );


	// if print to file: end here
	if ( toFile == BT_TRUE )
	{
		
		fclose( gnuPipe );
		gnuPipe = 0;

#if defined(GNUPLOT_EXECUTABLE) && defined(WIN32)
		string tmp = string( GNUPLOT_EXECUTABLE ) + string(" -p acado2gnuplot_tmp.dat");

		if ( system( tmp.c_str() ) )
			return RET_PLOTTING_FAILED;

		if ( system("del acado2gnuplot_tmp.dat") )
			return RET_PLOTTING_FAILED;
#else
		if ( system("gnuplot -persist -background white acado2gnuplot_tmp.dat") )
			return RET_PLOTTING_FAILED;
		if (system("rm -rf acado2gnuplot_tmp.dat") )
			return RET_PLOTTING_FAILED;
#endif

		return SUCCESSFUL_RETURN;
	}

	if( mouseEvent == BT_TRUE )
	{
		fprintf(gnuPipe,"pause mouse\n");
		fflush(gnuPipe);

		fprintf(gnuPipe,"a = MOUSE_X \n");
		fflush(gnuPipe);

		fprintf(gnuPipe,"b = MOUSE_Y \n");
		fflush(gnuPipe);

		fprintf(gnuPipe,"save var 'mouse.dat'\n");
		fflush(gnuPipe);
	}

#endif // __NO_PLOTTING__

	return SUCCESSFUL_RETURN;
}


returnValue GnuplotWindow::getPlotModeString(	PlotMode plotMode,
												std::string& plotModeString
												) const
{
    switch( plotMode )
	{
        case PM_LINES:
        	plotModeString = "lines lw 2.5";
        	break;

        case PM_POINTS:
        	plotModeString = "points pt 2.5";
        	break;

        default:
        	plotModeString = "lines lw 2.5";
    }

    return SUCCESSFUL_RETURN;
}


returnValue GnuplotWindow::getPlotStyleString(	VariableType _type,
												std::string& plotStyleString
												) const
{
	switch( _type )
	{
		case VT_DIFFERENTIAL_STATE:
			plotStyleString = "lt -1"; //black
			break;

		case VT_ALGEBRAIC_STATE:
			plotStyleString = "lt 4"; //magenta
			break;

		case VT_PARAMETER:
			plotStyleString = "lt 8"; //orange
			break;

		case VT_CONTROL:
			plotStyleString = "lt 3"; //blue
			break;

		case VT_DISTURBANCE:
			plotStyleString = "lt 5"; //light blue
			break;

		case VT_INTERMEDIATE_STATE:
			plotStyleString = "lt 2"; //green
			break;

		default:
		//case VT_OUTPUT:
			plotStyleString = "lt 1"; //red
	}

	return SUCCESSFUL_RETURN;
}


returnValue GnuplotWindow::obtainPlotDataString(	VariablesGrid& _dataGrid,
													std::string& _plotDataString
													) const
{
    stringstream ss;

    _dataGrid.sprint( ss );

	_plotDataString = ss.str();

	return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO


/*
 *   end of file
 */

