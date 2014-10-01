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
 *    \file src/user_interaction/plot_window_subplot.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 12.06.2008
 */


#include <acado/user_interaction/plot_window_subplot.hpp>
#include <acado/curve/curve.hpp>


BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//

PlotWindowSubplot::PlotWindowSubplot( )
{

	plotVariableX     = 0;
	plotVariableY     = 0;
	plotVariablesGrid = 0;
	plotExpressionX   = 0;
	plotExpressionY   = 0;
	plotEnum          = PLOT_NOTHING;

	plotMode   = PM_UNKNOWN;
	plotFormat = PF_PLAIN;

	xRangeLowerLimit = INFTY;
	xRangeUpperLimit = INFTY;
	yRangeLowerLimit = INFTY;
	yRangeUpperLimit = INFTY;

    plot3D = BT_FALSE;

	nLines     = 0;
	lineValues = 0;

	nData = 0;
	data  = 0;

	next = 0;
}


PlotWindowSubplot::PlotWindowSubplot(	const Expression& _expression,
										const char* const _title,
										const char* const _xLabel,
										const char* const _yLabel,
										PlotMode _plotMode,
										double _xRangeLowerLimit,
										double _xRangeUpperLimit,
										double _yRangeLowerLimit,
										double _yRangeUpperLimit
										)
{

    if( _expression.isVariable() == BT_TRUE ){

        plotVariableX     = 0;
        plotVariableY     = new Expression(_expression);
        plotVariablesGrid = 0;
        plotExpressionX   = 0;
        plotExpressionY   = 0;
    }
    else{

        plotVariableX     = 0;
        plotVariableY     = 0;
        plotVariablesGrid = 0;
        plotExpressionX   = 0;
        plotExpressionY   = new Expression(_expression);
    }

	plotEnum = PLOT_NOTHING;

	setTitle( _title );
	setXLabel( _xLabel );
	setYLabel( _yLabel );

	setPlotMode( _plotMode );
	setPlotFormat( PF_PLAIN );

	setRanges( _xRangeLowerLimit,_xRangeUpperLimit,_yRangeLowerLimit,_yRangeUpperLimit );

    plot3D = BT_FALSE;

	nLines     = 0;
	lineValues = 0;

	nData = 0;
	data  = 0;

	next = 0;
}


PlotWindowSubplot::PlotWindowSubplot(	const Expression& _expressionX,
										const Expression& _expressionY,
										const char* const _title,
										const char* const _xLabel,
										const char* const _yLabel,
										PlotMode _plotMode,
										double _xRangeLowerLimit,
										double _xRangeUpperLimit,
										double _yRangeLowerLimit,
										double _yRangeUpperLimit
										)
{

    if( _expressionX.isVariable() == BT_TRUE && _expressionY.isVariable() == BT_TRUE ){

        plotVariableX     = new Expression(_expressionX);
        plotVariableY     = new Expression(_expressionY);
        plotVariablesGrid = 0;
        plotExpressionX   = 0;
        plotExpressionY   = 0;
    }
    else{

        plotVariableX     = 0;
        plotVariableY     = 0;
        plotVariablesGrid = 0;
        plotExpressionX   = new Expression(_expressionX);
        plotExpressionY   = new Expression(_expressionY);
    }

	plotEnum          = PLOT_NOTHING;

	setTitle( _title );
	setXLabel( _xLabel );
	setYLabel( _yLabel );

	setPlotMode( _plotMode );
	setPlotFormat( PF_PLAIN );

	setRanges( _xRangeLowerLimit,_xRangeUpperLimit,_yRangeLowerLimit,_yRangeUpperLimit );

    plot3D = BT_FALSE;

	nLines     = 0;
	lineValues = 0;

	nData = 0;
	data  = 0;

	next = 0;
}



PlotWindowSubplot::PlotWindowSubplot(	PlotName _name,
										const char* const _title,
										const char* const _xLabel,
										const char* const _yLabel,
										PlotMode _plotMode,
										double _xRangeLowerLimit,
										double _xRangeUpperLimit,
										double _yRangeLowerLimit,
										double _yRangeUpperLimit
										)
{
	plotVariableX     = 0;
    plotVariableY     = 0;
    plotVariablesGrid = 0;
	plotExpressionX   = 0;
	plotExpressionY   = 0;
	plotEnum          = _name;

	setTitle( _title );
	setXLabel( _xLabel );
	setYLabel( _yLabel );

	setPlotMode( _plotMode );
	setPlotFormat( PF_PLAIN );

	setRanges( _xRangeLowerLimit,_xRangeUpperLimit,_yRangeLowerLimit,_yRangeUpperLimit );

    plot3D = BT_FALSE;

	nLines     = 0;
	lineValues = 0;

	nData = 0;
	data  = 0;

	next = 0;
}


PlotWindowSubplot::PlotWindowSubplot( const VariablesGrid& _plotVariable,
                                      const char* const _title,
                                      const char* const _xLabel,
                                      const char* const _yLabel,
                                      PlotMode _plotMode,
                                      double _xRangeLowerLimit,
                                      double _xRangeUpperLimit,
                                      double _yRangeLowerLimit,
                                      double _yRangeUpperLimit,
                                      BooleanType  _plot3D   )
{
	plotVariableX     = 0;
	plotVariableY     = 0;
	plotVariablesGrid = new VariablesGrid(_plotVariable);
	plotExpressionX   = 0;
	plotExpressionY   = 0;
	plotEnum          = PLOT_NOTHING;

    setTitle( _title );
    setXLabel( _xLabel );
    setYLabel( _yLabel );

    setPlotMode( _plotMode );
	setPlotFormat( PF_PLAIN );

    setRanges( _xRangeLowerLimit,_xRangeUpperLimit,_yRangeLowerLimit,_yRangeUpperLimit );

    plot3D = _plot3D;

    nLines     = 0;
    lineValues = 0;

    nData = 0;
    data  = 0;

    next = 0;
}


PlotWindowSubplot::PlotWindowSubplot( const Curve& _curve,
                                      double _xRangeLowerLimit,
                                      double _xRangeUpperLimit,
                                      const char* const _title,
                                      const char* const _xLabel,
                                      const char* const _yLabel,
                                      PlotMode _plotMode,
                                      double _yRangeLowerLimit,
                                      double _yRangeUpperLimit
									  )
{
	
	
	plotVariableX     = 0;
	plotVariableY     = 0;
	plotVariablesGrid = new VariablesGrid;
	plotExpressionX   = 0;
	plotExpressionY   = 0;
	plotEnum          = PLOT_NOTHING;

	// discretize function for plotting
    Grid grid( _xRangeLowerLimit,_xRangeUpperLimit,100 );
    _curve.discretize( grid,*plotVariablesGrid );

    setTitle( _title );
    setXLabel( _xLabel );
    setYLabel( _yLabel );

    setPlotMode( _plotMode );
	setPlotFormat( PF_PLAIN );

    setRanges( _xRangeLowerLimit,_xRangeUpperLimit,_yRangeLowerLimit,_yRangeUpperLimit );

    plot3D = BT_FALSE;

    nLines     = 0;
    lineValues = 0;

    nData = 0;
    data  = 0;

    next = 0;
}

PlotWindowSubplot::PlotWindowSubplot( const PlotWindowSubplot& rhs )
{
	if( rhs.plotVariableX != 0 )
		plotVariableX = new Expression(*rhs.plotVariableX);
	else
		plotVariableX = 0;

	if( rhs.plotVariableY != 0 )
		plotVariableY = new Expression(*rhs.plotVariableY);
	else
		plotVariableY = 0;

	if( rhs.plotVariablesGrid != 0 )
		plotVariablesGrid = new VariablesGrid(*rhs.plotVariablesGrid);
	else
		plotVariablesGrid = 0;

	if( rhs.plotExpressionX != 0 )
		plotExpressionX = new Expression(*rhs.plotExpressionX);
	else
		plotExpressionX = 0;

	if( rhs.plotExpressionY != 0 )
		plotExpressionY = new Expression(*rhs.plotExpressionY);
	else
		plotExpressionY = 0;

	plotEnum = rhs.plotEnum;

	setTitle( rhs.title );
	setXLabel( rhs.xLabel );
	setYLabel( rhs.yLabel );

	setPlotMode( rhs.plotMode );
	setPlotFormat( rhs.plotFormat );

	setRanges( rhs.xRangeLowerLimit,rhs.xRangeUpperLimit,rhs.yRangeLowerLimit,rhs.yRangeUpperLimit );

    plot3D = rhs.plot3D;

	nLines = rhs.nLines;
	if ( rhs.lineValues == 0 )
		lineValues = 0;
	else
	{
		lineValues = new double[nLines];
		for( uint i=0; i<nLines; ++i )
			lineValues[i] = rhs.lineValues[i];
	}

	nData = 0;
	data = 0;
	for( uint i=0; i<rhs.nData; ++i )
		if ( rhs.data[i] != 0 )
			addData( *(rhs.data[i]) );

	next = 0;
}


PlotWindowSubplot::~PlotWindowSubplot( )
{
	if ( plotVariableX != 0 )
		delete plotVariableX;

	if ( plotVariableY != 0 )
		delete plotVariableY;

    if ( plotVariablesGrid != 0 )
        delete plotVariablesGrid;

	if ( plotExpressionX != 0 )
		delete plotExpressionX;

	if ( plotExpressionY != 0 )
		delete plotExpressionY;

	if ( lineValues != 0 )
		free( lineValues );

	if ( data != 0 )
	{
		for( uint i=0; i<nData; ++i )
			delete data[i];
		free( data );
	}
}


PlotWindowSubplot& PlotWindowSubplot::operator=( const PlotWindowSubplot& rhs )
{
	if ( this != &rhs )
	{
		if ( plotVariableX != 0 )
			delete plotVariableX;
	
		if ( plotVariableY != 0 )
			delete plotVariableY;

		if ( plotVariablesGrid != 0 )
			delete plotVariablesGrid;

		if ( plotExpressionX != 0 )
			delete plotExpressionX;

		if ( plotExpressionY != 0 )
			delete plotExpressionY;

		if ( lineValues != 0 )
			free( lineValues );

		if ( data != 0 )
			delete[] data;


		if( rhs.plotVariableX != 0 )
			plotVariableX = new Expression(*rhs.plotVariableX);
		else
			plotVariableX = 0;

		if( rhs.plotVariableY != 0 )
			plotVariableY = new Expression(*rhs.plotVariableY);
		else
			plotVariableY = 0;

		if( rhs.plotVariablesGrid != 0 )
			plotVariablesGrid = new VariablesGrid(*rhs.plotVariablesGrid);
		else
			plotVariablesGrid = 0;

		if( rhs.plotExpressionX != 0 )
			plotExpressionX = new Expression(*rhs.plotExpressionX);
		else
			plotExpressionX = 0;

		if( rhs.plotExpressionY != 0 )
			plotExpressionY = new Expression(*rhs.plotExpressionY);
		else
			plotExpressionY = 0;

		plotEnum = rhs.plotEnum;

		setTitle( rhs.title );
		setXLabel( rhs.xLabel );
		setYLabel( rhs.yLabel );

		setPlotMode( rhs.plotMode );
		setPlotFormat( rhs.plotFormat );

		setRanges( rhs.xRangeLowerLimit,rhs.xRangeUpperLimit,rhs.yRangeLowerLimit,rhs.yRangeUpperLimit );

		plot3D = rhs.plot3D;

		nLines = rhs.nLines;
		if ( rhs.lineValues == 0 )
			lineValues = 0;
		else
		{
			lineValues = new double[nLines];
			for( uint i=0; i<nLines; ++i )
				lineValues[i] = rhs.lineValues[i];
		}

		nData = 0;
		data = 0;
		for( uint i=0; i<rhs.nData; ++i )
			if ( rhs.data[i] != 0 )
				addData( *(rhs.data[i]) );

		next = 0;
	}

	return *this;
}



returnValue PlotWindowSubplot::addLine(	double _lineValue
										)
{
    ++nLines;
    lineValues = (double*)realloc( lineValues,nLines*sizeof(double) );
    lineValues[nLines-1] = _lineValue;

	return SUCCESSFUL_RETURN;
}


returnValue PlotWindowSubplot::addData(	const VariablesGrid& _newData
										)
{
    ++nData;
    data = (VariablesGrid**)realloc( data,nData*sizeof(VariablesGrid*) );
    data[nData-1] = new VariablesGrid( _newData );

	return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//






CLOSE_NAMESPACE_ACADO


/*
 *	end of file
 */
