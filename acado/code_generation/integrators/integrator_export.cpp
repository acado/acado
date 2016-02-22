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
 *    \file src/code_generation/integrator_export.cpp
 *    \author Hans Joachim Ferreau, Boris Houska, Rien Quirynen
 *    \date 2010-2011
 */

#include <acado/code_generation/integrators/integrator_export.hpp>


using namespace std;

BEGIN_NAMESPACE_ACADO

//
// PUBLIC MEMBER FUNCTIONS:
//

IntegratorExport::IntegratorExport(	UserInteraction* _userInteraction,
									const std::string& _commonHeaderName
									) : ExportAlgorithm( _userInteraction,_commonHeaderName )
{
	NX1 = 0;
	NX2 = 0;
	NX3 = 0;

	NDX3 = 0;
	NXA3 = 0;

	timeDependant = false;

	exportRhs = true;
	crsFormat = false;

	reset_int = ExportVariable( "resetIntegrator", 1, 1, INT, ACADO_LOCAL, true );
	error_code = ExportVariable( "error", 1, 1, INT, ACADO_LOCAL, true );
}


IntegratorExport::IntegratorExport(	const IntegratorExport& arg
									) : ExportAlgorithm( arg )
{
	NX1 = arg.NX1;
	NX2 = arg.NX2;
	NX3 = arg.NX3;

	NDX3 = arg.NDX3;
	NXA3 = arg.NXA3;

	timeDependant = false;

	exportRhs = true;
	crsFormat = false;
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


returnValue IntegratorExport::setLinearInput( const DMatrix& M1, const DMatrix& A1, const DMatrix& B1 )
{
	if( !A1.isEmpty() ) {
		LOG( LVL_DEBUG ) << "Integrator: setLinearInput... " << endl;
		if( A1.getNumRows() != M1.getNumRows() || A1.getNumRows() != B1.getNumRows() || A1.getNumRows() != A1.getNumCols() || M1.getNumRows() != M1.getNumCols() || B1.getNumCols() != NU) {
			return RET_UNABLE_TO_EXPORT_CODE;
		}
		NX1 = A1.getNumRows();
		M11 = M1;
		A11 = A1;
		B11 = B1;

		OnlineData         dummy0;
		Control           dummy1;
		DifferentialState dummy2;
		dummy0.clearStaticCounters();
		dummy1.clearStaticCounters();
		dummy2.clearStaticCounters();
		x = DifferentialState("", NX1, 1);
		u = Control("", NU, 1);
		od = OnlineData("", NOD, 1);

		DifferentialEquation fun_input;
		fun_input << A11*x+B11*u;
		lin_input.init(fun_input, "acado_linear_input", NX, NXA, NU);
		LOG( LVL_DEBUG ) << "done!" << endl;
	}

	return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::setModel(	const std::string& _name_ODE, const std::string& _name_diffs_ODE ) {

	if( rhs.getFunctionDim() == 0 ) {
		rhs = ExportAcadoFunction(_name_ODE);
		diffs_rhs = ExportAcadoFunction(_name_diffs_ODE);

		exportRhs = false;
	}
	else {
		return ACADOERROR( RET_INVALID_OPTION );
	}

	OnlineData        dummy0;
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

	x = DifferentialState("", NX, 1);
	dx = DifferentialStateDerivative("", NDX, 1);
	z = AlgebraicState("", NXA, 1);
	u = Control("", NU, 1);
	od = OnlineData("", NOD, 1);

	return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::setLinearOutput( const DMatrix& M3, const DMatrix& A3, const Expression& _rhs )
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
		OnlineData        dummy0;
		Control           dummy1;
		DifferentialState dummy2;
		AlgebraicState 	  dummy3;
		DifferentialStateDerivative dummy4;
		dummy0.clearStaticCounters();
		dummy1.clearStaticCounters();
		dummy2.clearStaticCounters();
		x = DifferentialState("", NX1+NX2, 1);
		u = Control("", NU, 1);
		od = OnlineData("", NOD, 1);

		if( (uint)f.getNX() > (NX1+NX2) ) {
			return ACADOERROR( RET_INVALID_LINEAR_OUTPUT_FUNCTION );
		}
		if( (uint)f.getNDX() > (NX1+NX2) ) {
			return ACADOERROR( RET_INVALID_LINEAR_OUTPUT_FUNCTION );
		}
		if( f.getNDX() > 0 ) {
			NDX3 = NX1+NX2;
			NDX = NX;
		}
		else NDX3 = 0;

		dummy4.clearStaticCounters();
		dx = DifferentialStateDerivative("", NDX3, 1);

		if( f.getNXA() > 0 && NXA == 0 ) {
			return ACADOERROR( RET_INVALID_LINEAR_OUTPUT_FUNCTION );
		}
		if( f.getNXA() > 0 ) NXA3 = NXA;
		else NXA3 = 0;
		dummy3.clearStaticCounters();
		z = AlgebraicState("", NXA3, 1);

		uint i;
		OutputFcn g;
		for( i = 0; i < _rhs.getDim(); i++ ) {
			g << forwardDerivative( _rhs(i), x );
			g << forwardDerivative( _rhs(i), z );
			g << forwardDerivative( _rhs(i), u );
			g << forwardDerivative( _rhs(i), dx );
		}

		dummy2.clearStaticCounters();
		x = DifferentialState("", NX, 1);

		DMatrix dependencyMat = _rhs.getDependencyPattern( x );
		DVector dependency = dependencyMat.sumRow(  );
		for( i = NX1+NX2; i < NX; i++ ) {
			if( acadoRoundAway(dependency(i)) != 0 ) { // This expression should not depend on these differential states
				return RET_UNABLE_TO_EXPORT_CODE;
			}
		}

		OutputFcn f_large;
		DMatrix A3_large = expandOutputMatrix(A3);
		f_large << _rhs + A3_large*x;

		return (rhs3.init(f_large, "rhs3", NX, NXA, NU, NP, NDX, NOD) &
				diffs_rhs3.init(g, "diffs3", NX, NXA, NU, NP, NDX, NOD));
	}

	return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::setNonlinearFeedback( const DMatrix& C, const Expression& feedb )
{
	if( !C.isEmpty() ) {
		return RET_NOT_IMPLEMENTED_YET;
	}
	else {
		return SUCCESSFUL_RETURN;
	}
}


returnValue IntegratorExport::setLinearOutput( const DMatrix& M3, const DMatrix& A3, const std::string& _rhs3, const std::string& _diffs_rhs3 )
{
	if( !A3.isEmpty() ) {
		if( A3.getNumRows() != M3.getNumRows() || M3.getNumRows() != M3.getNumCols() || A3.getNumRows() != A3.getNumCols() ) {
			return RET_UNABLE_TO_EXPORT_CODE;
		}
		NX3 = A3.getNumRows();
		M33 = M3;
		A33 = A3;

		rhs3 = ExportAcadoFunction(_rhs3);
		diffs_rhs3 = ExportAcadoFunction(_diffs_rhs3);
		exportRhs = false;

		OnlineData        dummy0;
		Control           dummy1;
		DifferentialState dummy2;
		AlgebraicState 	  dummy3;
		DifferentialStateDerivative dummy4;
		dummy0.clearStaticCounters();
		dummy1.clearStaticCounters();
		dummy2.clearStaticCounters();
		dummy3.clearStaticCounters();
		dummy4.clearStaticCounters();

		x = DifferentialState("", NX, 1);
		dx = DifferentialStateDerivative("", NDX, 1);
		z = AlgebraicState("", NXA, 1);
		u = Control("", NU, 1);
		od = OnlineData("", NOD, 1);
	}

	return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::setModelData( const ModelData& data ) {

	LOG( LVL_DEBUG ) << "Integrator: setup model data... " << endl;
	setDimensions( data.getNX(),data.getNDX(),data.getNXA(),data.getNU(),data.getNP(),data.getN(), data.getNOD() );
	NX1 = data.getNX1();
	NX2 = data.getNX2();
	NX3 = data.getNX3();
	NDX3 = data.getNDX3();
	NXA3 = data.getNXA3();
	exportRhs = data.exportRhs();

	DMatrix M1, A1, B1;
	data.getLinearInput( M1, A1, B1 );
	if ( M1.getNumRows() > 0 && setLinearInput( M1, A1, B1 ) != SUCCESSFUL_RETURN )
		return RET_UNABLE_TO_EXPORT_CODE;

	DMatrix M3, A3;
	data.getLinearOutput( M3, A3 );

	if( exportRhs ) {
		DifferentialEquation f;
		data.getModel(f);
		if(f.getDim() > 0 && f.getNDX() == 0) {
			DVector order = f.getDifferentialStateComponents();
			for( uint i = 0; i < order.getDim(); i++ ) {
//				std::cout << "NX1+i: " << NX1+i << ", order(i): " << order(i) << std::endl;
				if( (NX1+i) != (uint)order(i) ) {
					return ACADOERRORTEXT(RET_NOT_IMPLEMENTED_YET, "The order of defined state variables should correspond to the order of equations in case of an explicit system!");
				}
			}
		}

		OutputFcn f3;
		data.getLinearOutput( M3, A3, f3 );
		DMatrix parms;
		uint delay;
		data.getNARXmodel( delay, parms );

		Expression rhs_, rhs3_;
		f.getExpression( rhs_ );
		f3.getExpression( rhs3_ );

		// Nonlinear feedback function:
		DMatrix C;
		OutputFcn feedb;
		data.getNonlinearFeedback( C, feedb );
		Expression feedb_;
		feedb.getExpression( feedb_ );

		if ( f.getDim() > 0 && setDifferentialEquation( rhs_ ) != SUCCESSFUL_RETURN )
			return RET_UNABLE_TO_EXPORT_CODE;
		if ( !parms.isEmpty() && setNARXmodel( delay, parms ) != SUCCESSFUL_RETURN )
			return RET_UNABLE_TO_EXPORT_CODE;
		if ( M3.getNumRows() > 0 && setLinearOutput( M3, A3, rhs3_ ) != SUCCESSFUL_RETURN )
			return RET_UNABLE_TO_EXPORT_CODE;
		if ( C.getNumCols() > 0 && setNonlinearFeedback( C, feedb_ ) != SUCCESSFUL_RETURN )
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
			std::vector<std::string> outputNames;
			std::vector<std::string> diffs_outputNames;
			std::vector<uint> dim_outputs;
			std::vector<DMatrix> outputDependencies_ = data.getOutputDependencies();
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
	LOG( LVL_DEBUG ) << "done!" << endl;

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
																const ExportIndex& _index3, const ExportIndex& tmp_index )
{
	uint index3; // index3 instead of _index3 to unroll loops
	if( NX2 > 0 ) {
		ExportForLoop loop01( index1,NX1,NX1+NX2 );
		if( NX1 > 0 ) {
			ExportForLoop loop02( index2,0,NX1 );
			loop02.addStatement( tmp_index == index2+index1*NX );
			loop02.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) == rk_diffsNew2.getSubMatrix( index1-NX1,index1-NX1+1,0,1 )*rk_diffsPrev1.getSubMatrix( 0,1,index2,index2+1 ) );
			for( index3 = 1; index3 < NX1; index3++ ) {
				loop02.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) += rk_diffsNew2.getSubMatrix( index1-NX1,index1-NX1+1,index3,index3+1 )*rk_diffsPrev1.getSubMatrix( index3,index3+1,index2,index2+1 ) );
			}
			for( index3 = 0; index3 < NX2; index3++ ) {
				loop02.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) += rk_diffsNew2.getSubMatrix( index1-NX1,index1-NX1+1,NX1+index3,NX1+index3+1 )*rk_diffsPrev2.getSubMatrix( index3,index3+1,index2,index2+1 ) );
			}
			loop01.addStatement( loop02 );
		}

		ExportForLoop loop05( index2,NX1,NX1+NX2 );
		loop05.addStatement( tmp_index == index2+index1*NX );
		loop05.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) == rk_diffsNew2.getSubMatrix( index1-NX1,index1-NX1+1,NX1,NX1+1 )*rk_diffsPrev2.getSubMatrix( 0,1,index2,index2+1 ) );
		for( index3 = 1; index3 < NX2; index3++ ) {
			loop05.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) += rk_diffsNew2.getSubMatrix( index1-NX1,index1-NX1+1,NX1+index3,NX1+index3+1 )*rk_diffsPrev2.getSubMatrix( index3,index3+1,index2,index2+1 ) );
		}
		loop01.addStatement( loop05 );

		if( NU > 0 ) {
			ExportForLoop loop07( index2,0,NU );
			loop07.addStatement( tmp_index == index2+index1*NU );
			loop07.addStatement( rk_eta.getCol( tmp_index+(NX+NXA)*(1+NX) ) == rk_diffsNew2.getSubMatrix( index1-NX1,index1-NX1+1,NX1+NX2+index2,NX1+NX2+index2+1 ) );
			for( index3 = 0; index3 < NX1; index3++ ) {
				loop07.addStatement( rk_eta.getCol( tmp_index+(NX+NXA)*(1+NX) ) += rk_diffsNew2.getSubMatrix( index1-NX1,index1-NX1+1,index3,index3+1 )*rk_diffsPrev1.getSubMatrix( index3,index3+1,NX1+index2,NX1+index2+1 ) );
			}
			for( index3 = 0; index3 < NX2; index3++ ) {
				loop07.addStatement( rk_eta.getCol( tmp_index+(NX+NXA)*(1+NX) ) += rk_diffsNew2.getSubMatrix( index1-NX1,index1-NX1+1,NX1+index3,NX1+index3+1 )*rk_diffsPrev2.getSubMatrix( index3,index3+1,NX1+NX2+index2,NX1+NX2+index2+1 ) );
			}
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



// PROTECTED:


DMatrix IntegratorExport::expandOutputMatrix( const DMatrix& A3 ) {
	DMatrix result = zeros<double>(NX3,NX);
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


returnValue IntegratorExport::getNumSteps( DVector& _numSteps ) const{

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

bool IntegratorExport::equidistantControlGrid( ) const{
	
	return numSteps.isEmpty();
}

const std::string IntegratorExport::getNameRHS() const{
	return rhs.getName();
}

const std::string IntegratorExport::getNameFullRHS() const{
	if( NX2 == NX ) {
		return getNameRHS();
	}
	else {
		return fullRhs.getName();
	}
}

const std::string IntegratorExport::getNameOutputRHS() const{
	return rhs3.getName();
}

const std::string IntegratorExport::getNameOutputDiffs() const{
	return diffs_rhs3.getName();
}

const std::string IntegratorExport::getNameOUTPUT( uint index ) const{
	return outputs[index].getName();
}

uint IntegratorExport::getDimOUTPUT( uint index ) const{
	if( exportRhs ) {
		return outputExpressions[index].getDim();
	}
	else {
		return num_outputs[index];
	}
}


const std::string IntegratorExport::getNameDiffsRHS() const{
	return diffs_rhs.getName();
}

const std::string IntegratorExport::getNameDiffsOUTPUT( uint index ) const{
	return diffs_outputs[index].getName();
}


CLOSE_NAMESPACE_ACADO

// end of file.
