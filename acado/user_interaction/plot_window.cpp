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
 *    \file src/user_interaction/plot_window.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 12.06.2008
 */


#include <acado/user_interaction/plot_window_subplot.hpp>
#include <acado/user_interaction/plot_window.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//


PlotWindow::PlotWindow( )
{
	next  = 0;
	aliasIdx = -1;

	frequency = PLOT_AT_END;
	setupLogFrequency( frequency );

	first = 0;
	last  = 0;

	number = 0;

	shallPlotNominalControls = BT_FALSE;
	shallPlotNominalParameters = BT_FALSE;
	shallPlotNominalOutputs = BT_TRUE;
}


PlotWindow::PlotWindow(	PlotFrequency _frequency
						)
{
	next  = 0;
	aliasIdx = -1;

	frequency = _frequency;
	setupLogFrequency( frequency );

	first = 0;
	last  = 0;

	number = 0;

	shallPlotNominalControls = BT_FALSE;
	shallPlotNominalParameters = BT_FALSE;
	shallPlotNominalOutputs = BT_TRUE;
}


PlotWindow::PlotWindow( const PlotWindow& rhs )
{
	next =  rhs.next;
	aliasIdx = rhs.aliasIdx;//-1

	frequency = rhs.frequency;

	first = 0;
	last  = 0;

	number = 0;

	/* if rhs plot_window list is not empty, add all option items... */
	PlotWindowSubplot* current = rhs.first;

	while ( current != 0 )
	{
		addSubplot( *current );
		current = current->getNext( );
	}

	plotDataRecord = rhs.plotDataRecord;

	shallPlotNominalControls = rhs.shallPlotNominalControls;
	shallPlotNominalParameters = rhs.shallPlotNominalParameters;
	shallPlotNominalOutputs = rhs.shallPlotNominalOutputs;

}


PlotWindow::~PlotWindow( )
{
	clearAllSubplots( );
}


PlotWindow& PlotWindow::operator=( const PlotWindow& rhs )
{
	if ( this != &rhs )
	{
		clearAllSubplots( );


		next = rhs.next;
		aliasIdx = rhs.aliasIdx;//-1

		frequency = rhs.frequency;

		/* if rhs plot_window list is not empty, add all option items... */
		PlotWindowSubplot* current = rhs.first;

		while ( current != 0 )
		{
			addSubplot( *current );
			current = current->getNext( );
		}

		plotDataRecord = rhs.plotDataRecord;
		
		shallPlotNominalControls = rhs.shallPlotNominalControls;
		shallPlotNominalParameters = rhs.shallPlotNominalParameters;
		shallPlotNominalOutputs = rhs.shallPlotNominalOutputs;
	}

	return *this;
}


PlotWindow* PlotWindow::clone( ) const
{
	return new PlotWindow( *this );
}


returnValue PlotWindow::init( )
{
	return SUCCESSFUL_RETURN;
}


returnValue PlotWindow::plot(	PlotFrequency _frequency
									)
{
	init( );

	return replot( _frequency );
}


returnValue PlotWindow::replot(	PlotFrequency _frequency
								)
{
	return SUCCESSFUL_RETURN;
}


returnValue PlotWindow::setTitle( uint idx, const char* const title_ )
{
	if ( idx >= getNumSubplots( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	return operator()( idx ).setTitle( title_ );
}

returnValue PlotWindow::setLabelX( uint idx, const char* const xLabel_ )
{
	if ( idx >= getNumSubplots( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	return operator()( idx ).setXLabel( xLabel_ );
}


returnValue PlotWindow::setLabelY( uint idx, const char* const yLabel_ )
{
	if ( idx >= getNumSubplots( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	return operator()( idx ).setYLabel( yLabel_ );
}


returnValue PlotWindow::setPlotMode( uint idx, PlotMode plotMode_ )
{
	if ( idx >= getNumSubplots( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	return operator()( idx ).setPlotMode( plotMode_ );
}


returnValue PlotWindow::setRanges(	uint idx,
									double xRange1_,
									double xRange2_,
									double yRange1_,
									double yRange2_ 
									)
{
	if ( idx >= getNumSubplots( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	return operator()( idx ).setRanges( xRange1_,xRange2_,yRange1_,yRange2_ );
}



returnValue PlotWindow::addLine(	uint idx,
									double _lineValue
									)
{
	if ( idx >= getNumSubplots( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	return operator()( idx ).addLine( _lineValue );
}


returnValue PlotWindow::addData(	uint idx,
									const VariablesGrid& _newData//,
									//PlotMode plotMode
									)
{
	if ( idx >= getNumSubplots( ) )
		return ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	return operator()( idx ).addData( _newData );
}


returnValue PlotWindow::operator<<(	PlotWindowSubplot& _subplot
									)
{
	return addSubplot( _subplot );
}


returnValue PlotWindow::operator<<(	PlotName _name
									)
{
	return addSubplot( _name );
}


returnValue PlotWindow::operator<<(	const Expression& _name
									)
{
	return addSubplot( _name );
}


returnValue PlotWindow::addSubplot(	PlotWindowSubplot& _subplot
									)
{
	// create new item
	PlotWindowSubplot* newItem = new PlotWindowSubplot( _subplot );

	if ( number == 0 )
	{
		first = newItem;
		last = newItem;
	}
	else
	{
		if ( last->setNext( newItem ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_PLOT_WINDOW_CORRUPTED );
		last = newItem;
	}

	// add plot item for all possible plot objects
	if ( addPlotDataItem( _subplot.getXVariableType( ) ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_LOG_RECORD_CORRUPTED );

	if ( addPlotDataItem( _subplot.getYVariableType( ) ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_LOG_RECORD_CORRUPTED );

	if ( addPlotDataItem( _subplot.getXPlotExpression( ) ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_LOG_RECORD_CORRUPTED );

	if ( addPlotDataItem( _subplot.getYPlotExpression( ) ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_LOG_RECORD_CORRUPTED );

	if ( addPlotDataItem( _subplot.getPlotEnum( ) ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_LOG_RECORD_CORRUPTED );

	++number;

	return SUCCESSFUL_RETURN;
}


returnValue PlotWindow::addSubplot(	const Expression& _expression,
									const char* const _title,
									const char* const _xLabel,
									const char* const _yLabel,
									PlotMode _plotMode,
									double _xRangeLowerLimit,
									double _xRangeUpperLimit,
									double _yRangeLowerLimit,
									double _yRangeUpperLimit
									){

    uint run1;

    for( run1 = 0; run1 < _expression.getDim(); run1++ ){

    	PlotWindowSubplot* newSubplot = new PlotWindowSubplot(	_expression(run1),
																_title,_xLabel,_yLabel,_plotMode,
																_xRangeLowerLimit,_xRangeUpperLimit,
																_yRangeLowerLimit,_yRangeUpperLimit );

		if ( number == 0 )
		{
			first = newSubplot;
			last = newSubplot;
		}
		else
		{
			if ( last->setNext( newSubplot ) != SUCCESSFUL_RETURN )
				return ACADOERROR( RET_PLOT_WINDOW_CORRUPTED );
			last = newSubplot;
		}

        Expression tmp( _expression(run1) );

		if ( addPlotDataItem( &tmp ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_LOG_RECORD_CORRUPTED );

		++number;
	}

	return SUCCESSFUL_RETURN;
}


returnValue PlotWindow::addSubplot(	const Expression& _expressionX,
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

    ASSERT( _expressionX.getDim() == _expressionY.getDim() );

    uint run1;

    for( run1 = 0; run1 < _expressionX.getDim(); run1++ ){

		PlotWindowSubplot* newSubplot = new PlotWindowSubplot(	_expressionX(run1),_expressionY(run1),
																_title,_xLabel,_yLabel,_plotMode,
																_xRangeLowerLimit,_xRangeUpperLimit,
																_yRangeLowerLimit,_yRangeUpperLimit );

		if ( number == 0 )
		{
			first = newSubplot;
			last = newSubplot;
		}
		else
		{
			if ( last->setNext( newSubplot ) != SUCCESSFUL_RETURN )
				return ACADOERROR( RET_OPTIONS_LIST_CORRUPTED );
			last = newSubplot;
		}

        Expression tmpX( _expressionX(run1) );

		if ( addPlotDataItem( &tmpX ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_LOG_RECORD_CORRUPTED );

        Expression tmpY( _expressionY(run1) );

		if ( addPlotDataItem( &tmpY ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_LOG_RECORD_CORRUPTED );

		++number;
    }

	return SUCCESSFUL_RETURN;
}



returnValue PlotWindow::addSubplot(	PlotName _name,
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
	PlotWindowSubplot* newSubplot = new PlotWindowSubplot(	_name,
															_title,_xLabel,_yLabel,_plotMode,
															_xRangeLowerLimit,_xRangeUpperLimit,
															_yRangeLowerLimit,_yRangeUpperLimit );

	// for certain PlotName, switch to logarithmic plot
	if ( ( _name == PLOT_KKT_TOLERANCE ) ||
		 ( _name == PLOT_NORM_LAGRANGE_GRADIENT ) )
		newSubplot->setPlotFormat( PF_LOG );

	if ( number == 0 )
	{
		first = newSubplot;
		last = newSubplot;
	}
	else
	{
		if ( last->setNext( newSubplot ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_OPTIONS_LIST_CORRUPTED );
		last = newSubplot;
	}

	if ( addPlotDataItem( _name ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_LOG_RECORD_CORRUPTED );

	++number;

	return SUCCESSFUL_RETURN;
}


returnValue PlotWindow::addSubplot( const VariablesGrid& _variable,
                                    const char* const _title,
                                    const char* const _xLabel,
                                    const char* const _yLabel,
                                    PlotMode _plotMode,
                                    double _xRangeLowerLimit,
                                    double _xRangeUpperLimit,
                                    double _yRangeLowerLimit,
                                    double _yRangeUpperLimit  ){


    PlotWindowSubplot* newSubplot = new PlotWindowSubplot( _variable,
                                                           _title,_xLabel,_yLabel,_plotMode,
                                                           _xRangeLowerLimit,_xRangeUpperLimit,
                                                           _yRangeLowerLimit,_yRangeUpperLimit,
                                                           BT_FALSE );

	if ( number == 0 )
	{
		first = newSubplot;
		last = newSubplot;
	}
	else
	{
		if ( last->setNext( newSubplot ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_OPTIONS_LIST_CORRUPTED );
		last = newSubplot;
	}

	++number;

	return SUCCESSFUL_RETURN;
}


returnValue PlotWindow::addSubplot3D( const VariablesGrid& _variable,
                                      const char* const _title,
                                      const char* const _xLabel,
                                      const char* const _yLabel,
                                      PlotMode _plotMode,
                                      double _xRangeLowerLimit,
                                      double _xRangeUpperLimit,
                                      double _yRangeLowerLimit,
                                      double _yRangeUpperLimit  )
{

    PlotWindowSubplot* newSubplot = new PlotWindowSubplot( _variable,
                                                           _title,_xLabel,_yLabel,_plotMode,
                                                           _xRangeLowerLimit,_xRangeUpperLimit,
                                                           _yRangeLowerLimit,_yRangeUpperLimit,
                                                           BT_TRUE );

	if ( number == 0 )
	{
		first = newSubplot;
		last = newSubplot;
	}
	else
	{
		if ( last->setNext( newSubplot ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_OPTIONS_LIST_CORRUPTED );
		last = newSubplot;
	}

	++number;
	
    return SUCCESSFUL_RETURN;
}


returnValue PlotWindow::addSubplot( const Curve& _curve,
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
	PlotWindowSubplot* newSubplot = new PlotWindowSubplot(	_curve,
															_xRangeLowerLimit,_xRangeUpperLimit,
														   _title,_xLabel,_yLabel,_plotMode,
															_yRangeLowerLimit,_yRangeUpperLimit );

	if ( number == 0 )
	{
		first = newSubplot;
		last = newSubplot;
	}
	else
	{
		if ( last->setNext( newSubplot ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_OPTIONS_LIST_CORRUPTED );
		last = newSubplot;
	}

	++number;

	return SUCCESSFUL_RETURN;
}


returnValue PlotWindow::clearAllSubplots( )
{
	PlotWindowSubplot* current = first;
	PlotWindowSubplot* tmp;

	/* deallocate all PlotWindowSubplots within list */
	while ( current != 0 )
	{
		tmp = current->getNext( );
		delete current;
		current = tmp;
	}

	/* ... and initialise an empty list. */
	first = 0;
	last  = 0;

	number = 0;

	return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue PlotWindow::addPlotDataItem(	const Expression* const _expression
											)
{
	if ( _expression != 0 )
	{
		// use dedicated call for variables
		if ( _expression->isVariable( ) == BT_TRUE )
			return addPlotDataItem( _expression->getVariableType( ) );

		// otherwise add all log entries that might be needed
		plotDataRecord.addItem( LOG_DIFFERENTIAL_STATES );
		plotDataRecord.addItem( LOG_ALGEBRAIC_STATES );
		plotDataRecord.addItem( LOG_PARAMETERS );
		plotDataRecord.addItem( LOG_CONTROLS );
		plotDataRecord.addItem( LOG_DISTURBANCES );

		plotDataRecord.addItem( LOG_SIMULATED_OUTPUT );

		plotDataRecord.addItem( LOG_DISCRETIZATION_INTERVALS );
		plotDataRecord.addItem( LOG_STAGE_BREAK_POINTS );
	}

	return SUCCESSFUL_RETURN;
}


returnValue PlotWindow::addPlotDataItem(	VariableType _type
											)
{
	switch( _type )
	{
		case VT_DIFFERENTIAL_STATE:
			plotDataRecord.addItem( LOG_DIFFERENTIAL_STATES );
			break;

		case VT_ALGEBRAIC_STATE:
			plotDataRecord.addItem( LOG_ALGEBRAIC_STATES );
			break;

		case VT_PARAMETER:
			plotDataRecord.addItem( LOG_PARAMETERS );
			plotDataRecord.addItem( LOG_NOMINAL_PARAMETERS );
			//plotDataRecord.addItem( LOG_SIMULATED_PARAMETERS );
			break;

		case VT_CONTROL:
			plotDataRecord.addItem( LOG_CONTROLS );
 			plotDataRecord.addItem( LOG_NOMINAL_CONTROLS );
			//plotDataRecord.addItem( LOG_SIMULATED_CONTROLS );
			break;

		case VT_DISTURBANCE:
			plotDataRecord.addItem( LOG_DISTURBANCES );
			break;

		case VT_INTERMEDIATE_STATE:
			plotDataRecord.addItem( LOG_INTERMEDIATE_STATES );
			break;
			
		case VT_OUTPUT:
			plotDataRecord.addItem( LOG_SIMULATED_OUTPUT );
 			plotDataRecord.addItem( LOG_PROCESS_OUTPUT );
			break;

		default:
			break;
	}

	plotDataRecord.addItem( LOG_DISCRETIZATION_INTERVALS );
	plotDataRecord.addItem( LOG_STAGE_BREAK_POINTS );

	return SUCCESSFUL_RETURN;
}


returnValue PlotWindow::addPlotDataItem(	PlotName _name
											)
{
	LogName logName = convertPlotToLogName( _name );

	if ( logName != LOG_NOTHING )
		return plotDataRecord.addItem( logName );

	plotDataRecord.setLogFrequency( LOG_AT_EACH_ITERATION );

	return SUCCESSFUL_RETURN;
}



LogName PlotWindow::convertPlotToLogName(	PlotName _name
											) const
{
	switch( _name )
	{
		case PLOT_KKT_TOLERANCE:
			return LOG_KKT_TOLERANCE;
			break;

		case PLOT_OBJECTIVE_VALUE:
			return LOG_OBJECTIVE_VALUE;

		case PLOT_MERIT_FUNCTION_VALUE:
			return LOG_MERIT_FUNCTION_VALUE;

		case PLOT_LINESEARCH_STEPLENGTH:
			return LOG_LINESEARCH_STEPLENGTH;

		case PLOT_NORM_LAGRANGE_GRADIENT:
			return LOG_NORM_LAGRANGE_GRADIENT;

		default:
			return LOG_NOTHING;
	}
}


PlotName PlotWindow::convertLogToPlotName(	LogName _name
											) const
{
	switch( _name )
	{
		case LOG_KKT_TOLERANCE:
			return PLOT_KKT_TOLERANCE;
			break;

		case LOG_OBJECTIVE_VALUE:
			return PLOT_OBJECTIVE_VALUE;

		case LOG_MERIT_FUNCTION_VALUE:
			return PLOT_MERIT_FUNCTION_VALUE;

		case LOG_LINESEARCH_STEPLENGTH:
			return PLOT_LINESEARCH_STEPLENGTH;

		case LOG_NORM_LAGRANGE_GRADIENT:
			return PLOT_NORM_LAGRANGE_GRADIENT;

		default:
			return PLOT_NOTHING;
	}
}



returnValue PlotWindow::setupLogFrequency( PlotFrequency _frequency )
{
	LogFrequency logFrequency;

	switch ( _frequency )
	{
		case PLOT_AT_START:
			logFrequency = LOG_AT_START;
			break;

		case PLOT_AT_END:
			logFrequency = LOG_AT_END;
			break;

		default:
			logFrequency = LOG_AT_EACH_ITERATION;
			break;
	}

	return plotDataRecord.setLogFrequency( logFrequency );
}



returnValue PlotWindow::getVariableDataGrids(	const Expression* const variable,
												VariableType& _type,
												VariablesGrid& _dataGrid,
												Grid& _discretizationGrid
												)
{

    ASSERT( variable->isVariable() == BT_TRUE );

	VariablesGrid dataGridTmp, dataGridComponent;
	int component = variable->getComponent(0);

	_type = variable->getVariableType();
	
	switch( _type )
	{
		case VT_DIFFERENTIAL_STATE:
			plotDataRecord.getLast( LOG_DIFFERENTIAL_STATES,dataGridTmp );
			break;

		case VT_ALGEBRAIC_STATE:
			plotDataRecord.getLast( LOG_ALGEBRAIC_STATES,dataGridTmp );
			break;

		case VT_PARAMETER:
			if ( shallPlotNominalParameters == BT_TRUE )
				plotDataRecord.getLast( LOG_NOMINAL_PARAMETERS,dataGridTmp );
			else
				plotDataRecord.getLast( LOG_PARAMETERS,dataGridTmp );
			
			component = variable->getComponent(0);
			if ( dataGridTmp.getNumValues( ) <= (uint)component )
				return ACADOERROR( RET_EMPTY_PLOT_DATA );
			
			dataGridComponent = dataGridTmp(component);
			dataGridComponent.setType( VT_PARAMETER );

			return getDataGrids( &dataGridComponent,_type,_dataGrid,_discretizationGrid );

		case VT_CONTROL:
			if ( shallPlotNominalControls == BT_TRUE )
				plotDataRecord.getLast( LOG_NOMINAL_CONTROLS,dataGridTmp );
			else
				plotDataRecord.getLast( LOG_CONTROLS,dataGridTmp );
			
			component = variable->getComponent(0);
			if ( dataGridTmp.getNumValues( ) <= (uint)component )
				return ACADOERROR( RET_EMPTY_PLOT_DATA );
			
			dataGridComponent = dataGridTmp(component);
			dataGridComponent.setType( VT_CONTROL );

			return getDataGrids( &dataGridComponent,_type,_dataGrid,_discretizationGrid );

		case VT_DISTURBANCE:
			plotDataRecord.getLast( LOG_DISTURBANCES,dataGridTmp );
			break;

		case VT_INTERMEDIATE_STATE:
			plotDataRecord.getLast( LOG_INTERMEDIATE_STATES,dataGridTmp );
			break;

		case VT_OUTPUT:
			if ( shallPlotNominalOutputs == BT_TRUE )
				plotDataRecord.getLast( LOG_SIMULATED_OUTPUT,dataGridTmp );
			else
				plotDataRecord.getLast( LOG_PROCESS_OUTPUT,dataGridTmp );
			
			if ( dataGridTmp.getNumValues( ) > (uint)component )
				_dataGrid = dataGridTmp(component);
			else
				return ACADOERROR( RET_EMPTY_PLOT_DATA );
			
			_discretizationGrid.init();
			return SUCCESSFUL_RETURN;

		default:
			// empty plot
			return ACADOERROR( RET_EMPTY_PLOT_DATA );
	}

	if ( dataGridTmp.getNumValues( ) > (uint)component )
		_dataGrid = dataGridTmp(component);
	else
		return ACADOERROR( RET_EMPTY_PLOT_DATA );

	VariablesGrid tmp;
	plotDataRecord.getLast( LOG_DISCRETIZATION_INTERVALS,tmp );
	tmp.getGrid( _discretizationGrid );
	
	return SUCCESSFUL_RETURN;
}


returnValue PlotWindow::getExpressionDataGrids(	const Expression* const expression,
												VariableType& _type,
												VariablesGrid& _dataGrid,
												Grid& _discretizationGrid
												)
{
	OutputFcn f;
	VariablesGrid loggedX,loggedXA,loggedP,loggedU,loggedW;

	_type = expression->getVariableType( );

	f << *expression;

	plotDataRecord.getLast( LOG_DIFFERENTIAL_STATES,loggedX );
	plotDataRecord.getLast( LOG_ALGEBRAIC_STATES,loggedXA );
	plotDataRecord.getLast( LOG_PARAMETERS,loggedP );
	plotDataRecord.getLast( LOG_CONTROLS,loggedU );
	plotDataRecord.getLast( LOG_DISTURBANCES,loggedW );

	if ( loggedP.isEmpty() == BT_FALSE )
		loggedP.refineGrid( loggedX );

	if ( loggedU.isEmpty() == BT_FALSE )
		loggedU.refineGrid( loggedX );

	if ( loggedW.isEmpty() == BT_FALSE )
		loggedW.refineGrid( loggedX );

	returnValue returnvalue = f.evaluate( &loggedX,&loggedXA,&loggedP,&loggedU,&loggedW, &_dataGrid );
	if( returnvalue != SUCCESSFUL_RETURN )
		return ACADOERROR( returnvalue );

	VariablesGrid tmp;
	plotDataRecord.getLast( LOG_DISCRETIZATION_INTERVALS,tmp );
	tmp.getGrid( _discretizationGrid );
	
	return SUCCESSFUL_RETURN;
}


returnValue PlotWindow::getDataGrids(	const VariablesGrid* const variablesGrid,
										VariableType& _type,
										VariablesGrid& _dataGrid,
										Grid& _discretizationGrid
										)
{
	_dataGrid.init();
	_discretizationGrid.init( );

	_type = variablesGrid->getType( );

	InterpolationMode mode = IM_LINEAR;
	if ( ( _type == VT_CONTROL ) || ( _type == VT_PARAMETER ) )
		mode = IM_CONSTANT;
	
	switch ( mode )
	{
		case IM_CONSTANT:
			_discretizationGrid.addTime( 0.0 );
			
			for( uint i=0; i<variablesGrid->getNumPoints( )-1; ++i )
			{
				_dataGrid.addVector( variablesGrid->getVector(i),variablesGrid->getTime(i) );
				_dataGrid.addVector( variablesGrid->getVector(i),variablesGrid->getTime(i+1) );
				_discretizationGrid.addTime( (double)_dataGrid.getNumPoints() );
			}
 			_dataGrid.addVector( variablesGrid->getLastVector(),variablesGrid->getLastTime() );
			_discretizationGrid.addTime( (double)_dataGrid.getNumPoints() );
			break;

		case IM_LINEAR:
			_dataGrid = *variablesGrid;
			break;

		default:
			return ACADOERROR( RET_NOT_YET_IMPLEMENTED );
	}

	_dataGrid.setType( _type );

	return SUCCESSFUL_RETURN;
}



returnValue PlotWindow::getAutoScaleYLimits(	const VariablesGrid& dataGridY,
												PlotFormat plotFormat,
												double& lowerLimit,
												double& upperLimit
												) const
{
	double maxValue  = dataGridY.getMax( );
	double minValue  = dataGridY.getMin( );
	double meanValue = dataGridY.getMean( );

	if ( ( acadoIsZero( maxValue ) == BT_TRUE ) && ( acadoIsZero( minValue ) == BT_TRUE ) )
		meanValue = 0.1 / 0.025;

	lowerLimit = minValue - 0.1*( maxValue-minValue ) - 0.025*meanValue - 1.0e-8;
	upperLimit = maxValue + 0.1*( maxValue-minValue ) + 0.025*meanValue + 1.0e-8;

	
	if ( ( plotFormat == PF_LOG ) || ( plotFormat == PF_LOG_LOG ) )
	{
		if ( ( acadoIsStrictlyGreater( minValue,0.0, ZERO ) == BT_TRUE ) && ( acadoIsGreater( lowerLimit,0.0, ZERO ) == BT_FALSE ) )
		{
			lowerLimit = ( minValue + acadoMin( minValue,EPS ) ) / sqrt(10.0);
			upperLimit = upperLimit * sqrt(10.0);
		}
	}
	else
	{
		if ( ( acadoIsStrictlyGreater( minValue,0.0, ZERO ) == BT_TRUE ) && ( acadoIsGreater( lowerLimit,0.0, ZERO ) == BT_FALSE ) )
			lowerLimit = 0.0;

		if ( ( acadoIsStrictlySmaller( maxValue,0.0 ) == BT_TRUE ) && ( acadoIsSmaller( upperLimit,0.0 ) == BT_FALSE ) )
			upperLimit = 0.0;
	}
	
	return SUCCESSFUL_RETURN;
}




CLOSE_NAMESPACE_ACADO


/*
 *	end of file
 */
