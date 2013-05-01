/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
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



/**
 *    \file src/code_generation/integrator_export.cpp
 *    \author Hans Joachim Ferreau, Boris Houska, Rien Quirynen
 *    \date 2010-2011
 */

#include <acado/code_generation/integrators/integrator_export.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

IntegratorExport::IntegratorExport(	UserInteraction* _userInteraction,
									const String& _commonHeaderName
									) : ExportAlgorithm( _userInteraction,_commonHeaderName )
{
	NX1 = 0;
	NX2 = 0;
	NX3 = 0;

	NDX3 = 0;
	NXA3 = 0;

	exportRhs = BT_TRUE;
	equidistant = BT_TRUE;
	crsFormat = BT_FALSE;

	reset_int = ExportVariable( "resetIntegrator", 1, 1, INT, ACADO_LOCAL, BT_TRUE );
	error_code = ExportVariable( "error", 1, 1, INT, ACADO_LOCAL, BT_TRUE );
}


IntegratorExport::IntegratorExport(	const IntegratorExport& arg
									) : ExportAlgorithm( arg )
{
	NX1 = arg.NX1;
	NX2 = arg.NX2;
	NX3 = arg.NX3;

	NDX3 = arg.NDX3;
	NXA3 = arg.NXA3;

	exportRhs = BT_TRUE;
	equidistant = BT_TRUE;
	crsFormat = BT_FALSE;
}


IntegratorExport::~IntegratorExport( )
{
	clear( );
}


IntegratorExport& IntegratorExport::operator=(	const IntegratorExport& arg
												)
{
	if( this != &arg && &arg != 0 )
	{
		clear( );
		ExportAlgorithm::operator=( arg );
	}
    return *this;
}


returnValue IntegratorExport::setGrid(	const Grid& _grid )
{
	grid = _grid;

	return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::setLinearInput( const Matrix& M1, const Matrix& A1, const Matrix& B1 )
{
	if( !A1.isEmpty() ) {
		if( A1.getNumRows() != M1.getNumRows() || A1.getNumRows() != B1.getNumRows() || A1.getNumRows() != A1.getNumCols() || M1.getNumRows() != M1.getNumCols() || B1.getNumCols() != NU) {
			return RET_UNABLE_TO_EXPORT_CODE;
		}
		NX1 = A1.getNumRows();
		if( !equidistant ) {
			// TODO: WHAT IF NONEQUIDISTANT INTEGRATION GRID??
			return RET_UNABLE_TO_EXPORT_CODE;
		}
		M11 = M1;
		A11 = A1;
		B11 = B1;

		Parameter         dummy0;
		Control           dummy1;
		DifferentialState dummy2;
		dummy0.clearStaticCounters();
		dummy1.clearStaticCounters();
		dummy2.clearStaticCounters();
		x = DifferentialState(NX1);
		u = Control(NU);
		p = Parameter(NP);

		DifferentialEquation fun_input;
		fun_input << A11*x+B11*u;
		lin_input.init( fun_input,"acado_linear_input",NX,NXA,NU );
	}

	return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::setModel(	const String& _name_ODE, const String& _name_diffs_ODE ) {

	if( rhs.getFunctionDim() == 0 ) {
		name_rhs = String(_name_ODE);
		name_diffs_rhs = String(_name_diffs_ODE);

		exportRhs = BT_FALSE;
	}
	else {
		return ACADOERROR( RET_INVALID_OPTION );
	}

	Parameter         dummy0;
	Control           dummy1;
	DifferentialState dummy2;
	AlgebraicState 	  dummy3;
	DifferentialStateDerivative dummy4;
	dummy0.clearStaticCounters();
	dummy1.clearStaticCounters();
	dummy2.clearStaticCounters();
	dummy3.clearStaticCounters();
	dummy4.clearStaticCounters();

	NX2 = NX-NX1-NX3;

	x = DifferentialState(NX);
	dx = DifferentialStateDerivative(NDX);
	z = AlgebraicState(NXA);
	u = Control(NU);
	p = Parameter(NP);

	setup();

	return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::setLinearOutput( const Matrix& M3, const Matrix& A3, const String& _rhs3, const String& _diffs_rhs3 )
{
	return RET_INVALID_OPTION;
}


returnValue IntegratorExport::setModelData( const ModelData& data ) {

	setDimensions( data.getNX(),data.getNDX(),data.getNXA(),data.getNU(),data.getNP(),data.getN() );
	NX1 = data.getNX1();
	NX2 = data.getNX2();
	NX3 = data.getNX3();
	NDX3 = data.getNDX3();
	NXA3 = data.getNXA3();
	exportRhs = data.exportRhs();

	Matrix M1, A1, B1;
	data.getLinearInput( M1, A1, B1 );
	if ( M1.getNumRows() > 0 && setLinearInput( M1, A1, B1 ) != SUCCESSFUL_RETURN )
		return RET_UNABLE_TO_EXPORT_CODE;

	Matrix M3, A3;
	data.getLinearOutput( M3, A3 );

	if( exportRhs ) {
		DifferentialEquation f;
		data.getModel(f);
		OutputFcn f3;
		data.getLinearOutput( M3, A3, f3 );
		Matrix parms;
		uint delay;
		data.getNARXmodel( delay, parms );

		Expression rhs_, rhs3_;
		f.getExpression( rhs_ );
		f3.getExpression( rhs3_ );

		if ( f.getDim() > 0 && setDifferentialEquation( rhs_ ) != SUCCESSFUL_RETURN )
			return RET_UNABLE_TO_EXPORT_CODE;
		if ( M3.getNumRows() > 0 && setLinearOutput( M3, A3, rhs3_ ) != SUCCESSFUL_RETURN )
			return RET_UNABLE_TO_EXPORT_CODE;
		if ( !parms.isEmpty() && setNARXmodel( delay, parms ) != SUCCESSFUL_RETURN )
			return RET_UNABLE_TO_EXPORT_CODE;
	}
	else {
		if ( M3.getNumRows() > 0 && setLinearOutput( M3, A3, data.getNameOutput(), data.getNameDiffsOutput() ) != SUCCESSFUL_RETURN )
			return RET_UNABLE_TO_EXPORT_CODE;
		if ( setModel( data.getNameRhs(), data.getNameDiffsRhs() ) != SUCCESSFUL_RETURN )
			return RET_UNABLE_TO_EXPORT_CODE;
	}
	Grid integrationGrid;
	data.getIntegrationGrid(integrationGrid);
	grid = integrationGrid;
	data.getNumSteps( numSteps );

	equidistant = data.hasEquidistantIntegrationGrid();

	setup( );

	if( data.hasOutputs() ) {
		uint i;

		std::vector<Grid> newGrids_;
		std::vector<Grid> outputGrids_;
		data.getOutputGrids(outputGrids_);
		std::vector<Expression> outputExpressions_;
		data.getOutputExpressions(outputExpressions_);
		for( i = 0; i < outputGrids_.size(); i++ ) {
			uint numOuts = (int) ceil((double)outputGrids_[i].getNumIntervals()/((double) data.getN()) - 10.0*EPS);
			Grid nextGrid( 0.0, 1.0, numOuts + 1 );
			newGrids_.push_back( nextGrid );
		}

		if( outputExpressions_.size() > 0 ) {
			setupOutput( newGrids_, outputExpressions_ );
		}
		else {
			std::vector<String> outputNames;
			std::vector<String> diffs_outputNames;
			std::vector<uint> dim_outputs;
			std::vector<Matrix> outputDependencies_ = data.getOutputDependencies();
			data.getNameOutputs(outputNames);
			data.getNameDiffsOutputs(diffs_outputNames);
			data.getDimOutputs(dim_outputs);
			if( !data.hasCompressedStorage() ) {
				setupOutput( newGrids_, outputNames, diffs_outputNames, dim_outputs );
			}
			else {
				setupOutput( newGrids_, outputNames, diffs_outputNames, dim_outputs, outputDependencies_ );
			}
		}
	}

	setup( );

	return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::updateInputSystem(	ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& tmp_index )
{
	if( NX1 > 0 ) {
		ExportForLoop loop01( index1,0,NX1 );
		ExportForLoop loop02( index2,0,NX1 );
		loop02.addStatement( tmp_index == index2+index1*NX );
		loop02.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) == rk_diffsNew1.getSubMatrix( index1,index1+1,index2,index2+1 ) );
		loop01.addStatement( loop02 );

		if( NU > 0 ) {
			ExportForLoop loop03( index2,0,NU );
			loop03.addStatement( tmp_index == index2+index1*NU );
			loop03.addStatement( rk_eta.getCol( tmp_index+(NX+NXA)*(1+NX) ) == rk_diffsNew1.getSubMatrix( index1,index1+1,NX1+index2,NX1+index2+1 ) );
			loop01.addStatement( loop03 );
		}
		block->addStatement( loop01 );
	}

	return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::propagateInputSystem( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2,
															 const ExportIndex& index3, const ExportIndex& tmp_index )
{
	if( NX1 > 0 ) {
		ExportForLoop loop01( index1,0,NX1 );
		ExportForLoop loop02( index2,0,NX1 );
		loop02.addStatement( tmp_index == index2+index1*NX );
		loop02.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) == rk_diffsNew1.getSubMatrix( index1,index1+1,0,1 )*rk_diffsPrev1.getSubMatrix( 0,1,index2,index2+1 ) );
		ExportForLoop loop03( index3,1,NX1 );
		loop03.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) += rk_diffsNew1.getSubMatrix( index1,index1+1,index3,index3+1 )*rk_diffsPrev1.getSubMatrix( index3,index3+1,index2,index2+1 ) );
		loop02.addStatement( loop03 );
		loop01.addStatement( loop02 );

		if( NU > 0 ) {
			ExportForLoop loop04( index2,0,NU );
			loop04.addStatement( tmp_index == index2+index1*NU );
			loop04.addStatement( rk_eta.getCol( tmp_index+(NX+NXA)*(1+NX) ) == rk_diffsNew1.getSubMatrix( index1,index1+1,NX1+index2,NX1+index2+1 ) );
			ExportForLoop loop05( index3,0,NX1 );
			loop05.addStatement( rk_eta.getCol( tmp_index+(NX+NXA)*(1+NX) ) += rk_diffsNew1.getSubMatrix( index1,index1+1,index3,index3+1 )*rk_diffsPrev1.getSubMatrix( index3,index3+1,NX1+index2,NX1+index2+1 ) );
			loop04.addStatement( loop05 );
			loop01.addStatement( loop04 );
		}
		block->addStatement( loop01 );
	}
	return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::updateImplicitSystem(	ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& tmp_index )
{
	if( NX2 > 0 ) {
		ExportForLoop loop01( index1,NX1,NX1+NX2 );
		ExportForLoop loop02( index2,0,NX1+NX2 );
		loop02.addStatement( tmp_index == index2+index1*NX );
		loop02.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) == rk_diffsNew2.getSubMatrix( index1-NX1,index1-NX1+1,index2,index2+1 ) );
		loop01.addStatement( loop02 );

		if( NU > 0 ) {
			ExportForLoop loop03( index2,0,NU );
			loop03.addStatement( tmp_index == index2+index1*NU );
			loop03.addStatement( rk_eta.getCol( tmp_index+(NX+NXA)*(1+NX) ) == rk_diffsNew2.getSubMatrix( index1-NX1,index1-NX1+1,NX1+NX2+index2,NX1+NX2+index2+1 ) );
			loop01.addStatement( loop03 );
		}
		block->addStatement( loop01 );
	}
	// ALGEBRAIC STATES
	if( NXA > 0 ) {
		ExportForLoop loop01( index1,NX,NX+NXA );
		ExportForLoop loop02( index2,0,NX1+NX2 );
		loop02.addStatement( tmp_index == index2+index1*NX );
		loop02.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) == rk_diffsNew2.getSubMatrix( index1-NX1-NX3,index1-NX1-NX3+1,index2,index2+1 ) );
		loop01.addStatement( loop02 );

		if( NU > 0 ) {
			ExportForLoop loop03( index2,0,NU );
			loop03.addStatement( tmp_index == index2+index1*NU );
			loop03.addStatement( rk_eta.getCol( tmp_index+(NX+NXA)*(1+NX) ) == rk_diffsNew2.getSubMatrix( index1-NX1-NX3,index1-NX1-NX3+1,NX1+NX2+index2,NX1+NX2+index2+1 ) );
			loop01.addStatement( loop03 );
		}
		block->addStatement( loop01 );
	}

	return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::propagateImplicitSystem(	ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2,
																const ExportIndex& index3, const ExportIndex& tmp_index )
{
	if( NX2 > 0 ) {
		ExportForLoop loop01( index1,NX1,NX1+NX2 );
		ExportForLoop loop02( index2,0,NX1 );
		loop02.addStatement( tmp_index == index2+index1*NX );
		loop02.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) == 0.0 );
		ExportForLoop loop03( index3,0,NX1 );
		loop03.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) += rk_diffsNew2.getSubMatrix( index1-NX1,index1-NX1+1,index3,index3+1 )*rk_diffsPrev1.getSubMatrix( index3,index3+1,index2,index2+1 ) );
		loop02.addStatement( loop03 );
		ExportForLoop loop04( index3,0,NX2 );
		loop04.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) += rk_diffsNew2.getSubMatrix( index1-NX1,index1-NX1+1,NX1+index3,NX1+index3+1 )*rk_diffsPrev2.getSubMatrix( index3,index3+1,index2,index2+1 ) );
		loop02.addStatement( loop04 );
		loop01.addStatement( loop02 );

		ExportForLoop loop05( index2,NX1,NX1+NX2 );
		loop05.addStatement( tmp_index == index2+index1*NX );
		loop05.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) == 0.0 );
		ExportForLoop loop06( index3,0,NX2 );
		loop06.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) += rk_diffsNew2.getSubMatrix( index1-NX1,index1-NX1+1,NX1+index3,NX1+index3+1 )*rk_diffsPrev2.getSubMatrix( index3,index3+1,index2,index2+1 ) );
		loop05.addStatement( loop06 );
		loop01.addStatement( loop05 );

		if( NU > 0 ) {
			ExportForLoop loop07( index2,0,NU );
			loop07.addStatement( tmp_index == index2+index1*NU );
			loop07.addStatement( rk_eta.getCol( tmp_index+(NX+NXA)*(1+NX) ) == rk_diffsNew2.getSubMatrix( index1-NX1,index1-NX1+1,NX1+NX2+index2,NX1+NX2+index2+1 ) );
			ExportForLoop loop08( index3,0,NX1 );
			loop08.addStatement( rk_eta.getCol( tmp_index+(NX+NXA)*(1+NX) ) += rk_diffsNew2.getSubMatrix( index1-NX1,index1-NX1+1,index3,index3+1 )*rk_diffsPrev1.getSubMatrix( index3,index3+1,NX1+index2,NX1+index2+1 ) );
			loop07.addStatement( loop08 );
			ExportForLoop loop09( index3,0,NX2 );
			loop09.addStatement( rk_eta.getCol( tmp_index+(NX+NXA)*(1+NX) ) += rk_diffsNew2.getSubMatrix( index1-NX1,index1-NX1+1,NX1+index3,NX1+index3+1 )*rk_diffsPrev2.getSubMatrix( index3,index3+1,NX1+NX2+index2,NX1+NX2+index2+1 ) );
			loop07.addStatement( loop09 );
			loop01.addStatement( loop07 );
		}
		block->addStatement( loop01 );
	}
	// ALGEBRAIC STATES: NO PROPAGATION OF SENSITIVITIES NEEDED

	return SUCCESSFUL_RETURN;
}



// PROTECTED:


returnValue IntegratorExport::copy(	const IntegratorExport& arg
									)
{
	exportRhs = arg.exportRhs;
	equidistant = arg.equidistant;
	crsFormat = arg.crsFormat;
	grid = arg.grid;
	numSteps = arg.numSteps;

	// ExportFunctions
	integrate = arg.integrate;
	
	return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::clear( )
{
	return SUCCESSFUL_RETURN;
}


uint IntegratorExport::getIntegrationInterval( double time ) {
	uint index = 0;
	double scale = 1.0/(grid.getLastTime() - grid.getFirstTime());
	while( index < (grid.getNumIntervals()-1) && time > scale*grid.getTime( index+1 ) ) {
		index++;
	}
	return index;
}


returnValue IntegratorExport::getGrid( Grid& grid_ ) const{

    grid_ = grid;
    return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::getNumSteps( Vector& _numSteps ) const{

    _numSteps = numSteps;
    return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::getOutputExpressions( std::vector<Expression>& outputExpressions_ ) const{

    outputExpressions_ = outputExpressions;
    return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::getOutputGrids( std::vector<Grid>& outputGrids_ ) const{

    outputGrids_ = outputGrids;
    return SUCCESSFUL_RETURN;
}

BooleanType IntegratorExport::equidistantControlGrid( ) const{
	
	return numSteps.isEmpty();
}

const String IntegratorExport::getNameRHS() const{
	if( exportRhs ) {
		return rhs.getName();
	}
	else {
		return name_rhs;
	}
}

const String IntegratorExport::getNameFullRHS() const{
	return getNameRHS();
}

const String IntegratorExport::getNameOUTPUT( uint index ) const{
	if( exportRhs ) {
		return outputs[index].getName();
	}
	else {
		return name_outputs[index];
	}
}

uint IntegratorExport::getDimOUTPUT( uint index ) const{
	if( exportRhs ) {
		return outputExpressions[index].getDim();
	}
	else {
		return num_outputs[index];
	}
}


const String IntegratorExport::getNameDiffsRHS() const{
	if( exportRhs ) {
		return diffs_rhs.getName();
	}
	else {
		return name_diffs_rhs;
	}
}

const String IntegratorExport::getNameDiffsOUTPUT( uint index ) const{
	if( exportRhs ) {
		return diffs_outputs[index].getName();
	}
	else {
		return name_diffs_outputs[index];
	}
}


CLOSE_NAMESPACE_ACADO

// end of file.
