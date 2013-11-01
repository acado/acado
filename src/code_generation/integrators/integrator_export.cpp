/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2013 by Boris Houska, Hans Joachim Ferreau,
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

	timeDependant = BT_FALSE;

	exportRhs = BT_TRUE;
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

	timeDependant = BT_FALSE;

	exportRhs = BT_TRUE;
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

	return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::setLinearOutput( const Matrix& M3, const Matrix& A3, const Expression& _rhs )
{
	if( !A3.isEmpty() ) {
		if( A3.getNumRows() != M3.getNumRows() || M3.getNumRows() != M3.getNumCols() || A3.getNumRows() != A3.getNumCols() || A3.getNumRows() != _rhs.getDim() ) {
			return RET_UNABLE_TO_EXPORT_CODE;
		}
		NX3 = A3.getNumRows();
		M33 = M3;
		A33 = A3;

		OutputFcn f;
		f << _rhs;
		Parameter         dummy0;
		Control           dummy1;
		DifferentialState dummy2;
		AlgebraicState 	  dummy3;
		DifferentialStateDerivative dummy4;
		dummy0.clearStaticCounters();
		dummy1.clearStaticCounters();
		dummy2.clearStaticCounters();
		x = DifferentialState(NX1+NX2);
		u = Control(NU);
		p = Parameter(NP);

		if( (uint)f.getNDX() > (NX1+NX2) ) {
			return ACADOERROR( RET_INVALID_OPTION );
		}
		if( f.getNDX() > 0 ) NDX3 = NX1+NX2;
		else NDX3 = 0;
		dummy4.clearStaticCounters();
		dx = DifferentialStateDerivative(NDX3);

		if( f.getNXA() > 0 && NXA == 0 ) {
			return ACADOERROR( RET_INVALID_OPTION );
		}
		if( f.getNXA() > 0 ) NXA3 = NXA;
		else NXA3 = 0;
		dummy3.clearStaticCounters();
		z = AlgebraicState(NXA3);

		uint i;
		OutputFcn g;
		for( i = 0; i < _rhs.getDim(); i++ ) {
			g << forwardDerivative( _rhs(i), x );
			g << forwardDerivative( _rhs(i), z );
			g << forwardDerivative( _rhs(i), u );
			g << forwardDerivative( _rhs(i), dx );
		}

		dummy2.clearStaticCounters();
		x = DifferentialState(NX);

		Matrix dependencyMat = _rhs.getDependencyPattern( x );
		Vector dependency = sumRow( dependencyMat );
		for( i = NX1+NX2; i < NX; i++ ) {
			if( acadoRoundAway(dependency(i)) != 0 ) { // This expression should not depend on these differential states
				return RET_UNABLE_TO_EXPORT_CODE;
			}
		}

		OutputFcn f_large;
		Matrix A3_large = expandOutputMatrix(A3);
		f_large << _rhs + A3_large*x;

		return (rhs3.init( f_large,"acado_rhs3",NX,NXA,NU,NP ) & diffs_rhs3.init( g,"acado_diffs3",NX,NXA,NU,NP ) );
	}

	return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::setLinearOutput( const Matrix& M3, const Matrix& A3, const String& _rhs3, const String& _diffs_rhs3 )
{
	if( !A3.isEmpty() ) {
		if( A3.getNumRows() != M3.getNumRows() || M3.getNumRows() != M3.getNumCols() || A3.getNumRows() != A3.getNumCols() ) {
			return RET_UNABLE_TO_EXPORT_CODE;
		}
		NX3 = A3.getNumRows();
		M33 = M3;
		A33 = A3;

		name_rhs3 = String(_rhs3);
		name_diffs_rhs3 = String(_diffs_rhs3);
		exportRhs = BT_FALSE;

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

		x = DifferentialState(NX);
		dx = DifferentialStateDerivative(NDX);
		z = AlgebraicState(NXA);
		u = Control(NU);
		p = Parameter(NP);
	}

	return SUCCESSFUL_RETURN;
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
		if ( !parms.isEmpty() && setNARXmodel( delay, parms ) != SUCCESSFUL_RETURN )
			return RET_UNABLE_TO_EXPORT_CODE;
		if ( M3.getNumRows() > 0 && setLinearOutput( M3, A3, rhs3_ ) != SUCCESSFUL_RETURN )
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

	setup( );

	if( data.hasOutputs() ) {
		std::vector<Grid> outputGrids_;
		data.getOutputGrids(outputGrids_);
		std::vector<Expression> outputExpressions_;
		data.getOutputExpressions(outputExpressions_);

		if( outputExpressions_.size() > 0 ) {
			setupOutput( outputGrids_, outputExpressions_ );
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
				setupOutput( outputGrids_, outputNames, diffs_outputNames, dim_outputs );
			}
			else {
				setupOutput( outputGrids_, outputNames, diffs_outputNames, dim_outputs, outputDependencies_ );
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


returnValue IntegratorExport::updateOutputSystem(	ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& tmp_index )
{
	if( NX3 > 0 ) {
		ExportForLoop loop01( index1,NX1+NX2,NX );
		ExportForLoop loop02( index2,0,NX );
		loop02.addStatement( tmp_index == index2+index1*NX );
		loop02.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) == rk_diffsNew3.getSubMatrix( index1-NX1-NX2,index1-NX1-NX2+1,index2,index2+1 ) );
		loop01.addStatement( loop02 );

		if( NU > 0 ) {
			ExportForLoop loop03( index2,0,NU );
			loop03.addStatement( tmp_index == index2+index1*NU );
			loop03.addStatement( rk_eta.getCol( tmp_index+(NX+NXA)*(1+NX) ) == rk_diffsNew3.getSubMatrix( index1-NX1-NX2,index1-NX1-NX2+1,NX+index2,NX+index2+1 ) );
			loop01.addStatement( loop03 );
		}
		block->addStatement( loop01 );
	}

	return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::propagateOutputSystem(	ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2,
																const ExportIndex& index3, const ExportIndex& tmp_index )
{
	if( NX3 > 0 ) {
		ExportForLoop loop01( index1,NX1+NX2,NX );
		ExportForLoop loop02( index2,0,NX1 );
		loop02.addStatement( tmp_index == index2+index1*NX );
		loop02.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) == 0.0 );
		ExportForLoop loop03( index3,0,NX1 );
		loop03.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) += rk_diffsNew3.getSubMatrix( index1-NX1-NX2,index1-NX1-NX2+1,index3,index3+1 )*rk_diffsPrev1.getSubMatrix( index3,index3+1,index2,index2+1 ) );
		loop02.addStatement( loop03 );
		ExportForLoop loop04( index3,0,NX2 );
		loop04.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) += rk_diffsNew3.getSubMatrix( index1-NX1-NX2,index1-NX1-NX2+1,NX1+index3,NX1+index3+1 )*rk_diffsPrev2.getSubMatrix( index3,index3+1,index2,index2+1 ) );
		loop02.addStatement( loop04 );
		ExportForLoop loop042( index3,0,NX3 );
		loop042.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) += rk_diffsNew3.getSubMatrix( index1-NX1-NX2,index1-NX1-NX2+1,NX1+NX2+index3,NX1+NX2+index3+1 )*rk_diffsPrev3.getSubMatrix( index3,index3+1,index2,index2+1 ) );
		loop02.addStatement( loop042 );
		loop01.addStatement( loop02 );

		ExportForLoop loop05( index2,NX1,NX1+NX2 );
		loop05.addStatement( tmp_index == index2+index1*NX );
		loop05.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) == 0.0 );
		ExportForLoop loop06( index3,0,NX2 );
		loop06.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) += rk_diffsNew3.getSubMatrix( index1-NX1-NX2,index1-NX1-NX2+1,NX1+index3,NX1+index3+1 )*rk_diffsPrev2.getSubMatrix( index3,index3+1,index2,index2+1 ) );
		loop05.addStatement( loop06 );
		ExportForLoop loop062( index3,0,NX3 );
		loop062.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) += rk_diffsNew3.getSubMatrix( index1-NX1-NX2,index1-NX1-NX2+1,NX1+NX2+index3,NX1+NX2+index3+1 )*rk_diffsPrev3.getSubMatrix( index3,index3+1,index2,index2+1 ) );
		loop05.addStatement( loop062 );
		loop01.addStatement( loop05 );

		ExportForLoop loop07( index2,NX1+NX2,NX );
		loop07.addStatement( tmp_index == index2+index1*NX );
		loop07.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) == 0.0 );
		ExportForLoop loop08( index3,0,NX3 );
		loop08.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) += rk_diffsNew3.getSubMatrix( index1-NX1-NX2,index1-NX1-NX2+1,NX1+NX2+index3,NX1+NX2+index3+1 )*rk_diffsPrev3.getSubMatrix( index3,index3+1,index2,index2+1 ) );
		loop07.addStatement( loop08 );
		loop01.addStatement( loop07 );

		if( NU > 0 ) {
			ExportForLoop loop09( index2,0,NU );
			loop09.addStatement( tmp_index == index2+index1*NU );
			loop09.addStatement( rk_eta.getCol( tmp_index+(NX+NXA)*(1+NX) ) == rk_diffsNew3.getSubMatrix( index1-NX1-NX2,index1-NX1-NX2+1,NX+index2,NX+index2+1 ) );
			ExportForLoop loop10( index3,0,NX1 );
			loop10.addStatement( rk_eta.getCol( tmp_index+(NX+NXA)*(1+NX) ) += rk_diffsNew3.getSubMatrix( index1-NX1-NX2,index1-NX1-NX2+1,index3,index3+1 )*rk_diffsPrev1.getSubMatrix( index3,index3+1,NX1+index2,NX1+index2+1 ) );
			loop09.addStatement( loop10 );
			ExportForLoop loop11( index3,0,NX2 );
			loop11.addStatement( rk_eta.getCol( tmp_index+(NX+NXA)*(1+NX) ) += rk_diffsNew3.getSubMatrix( index1-NX1-NX2,index1-NX1-NX2+1,NX1+index3,NX1+index3+1 )*rk_diffsPrev2.getSubMatrix( index3,index3+1,NX1+NX2+index2,NX1+NX2+index2+1 ) );
			loop09.addStatement( loop11 );
			ExportForLoop loop112( index3,0,NX3 );
			loop112.addStatement( rk_eta.getCol( tmp_index+(NX+NXA)*(1+NX) ) += rk_diffsNew3.getSubMatrix( index1-NX1-NX2,index1-NX1-NX2+1,NX1+NX2+index3,NX1+NX2+index3+1 )*rk_diffsPrev3.getSubMatrix( index3,index3+1,NX+index2,NX+index2+1 ) );
			loop09.addStatement( loop112 );
			loop01.addStatement( loop09 );
		}
		block->addStatement( loop01 );
	}

	return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::prepareFullRhs( ) {

	uint i, j;
	// PART 1:
	for( i = 0; i < NX1; i++ ) {
		fullRhs.addStatement( rhs_out.getRow(i) == A11(i,0)*rhs_in.getRow(0) );
		for( j = 1; j < NX1; j++ ) {
			if( acadoRoundAway(A11(i,j)) != 0 ) {
				fullRhs.addStatement( rhs_out.getRow(i) += A11(i,j)*rhs_in.getRow(j) );
			}
		}
		for( j = 0; j < NU; j++ ) {
			if( acadoRoundAway(B11(i,j)) != 0 ) {
				fullRhs.addStatement( rhs_out.getRow(i) += B11(i,j)*rhs_in.getRow(NX+NXA+j) );
			}
		}
	}

	// PART 2:
	if( NX2 > 0 ) {
		fullRhs.addFunctionCall( getNameRHS(), rhs_in, rhs_out.getAddress(NX1,0) );
	}

	// PART 3:
	if( NX3 > 0 ) {
		fullRhs.addFunctionCall( getNameOutputRHS(), rhs_in, rhs_out.getAddress(NX1+NX2,0) );
	}


	return SUCCESSFUL_RETURN;
}



// PROTECTED:


Matrix IntegratorExport::expandOutputMatrix( const Matrix& A3 ) {
	Matrix result = zeros(NX3,NX);
	uint i,j;
	for( i = 0; i < NX3; i++ ) {
		for( j = 0; j < NX3; j++ ) {
			result(i,NX-NX3+j) = A3(i,j);
		}
	}

	return result;
}


returnValue IntegratorExport::copy(	const IntegratorExport& arg
									)
{
	exportRhs = arg.exportRhs;
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
	if( NX2 == NX ) {
		return getNameRHS();
	}
	else {
		return fullRhs.getName();
	}
}

const String IntegratorExport::getNameOutputRHS() const{
	if( exportRhs ) {
		return rhs3.getName();
	}
	else {
		return name_rhs3;
	}
}

const String IntegratorExport::getNameOutputDiffs() const{
	if( exportRhs ) {
		return diffs_rhs3.getName();
	}
	else {
		return name_diffs_rhs3;
	}
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
