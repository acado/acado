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
 *    \file src/code_generation/erk_export.cpp
 *    \author Rien Quirynen, Milan Vukov
 *    \date 2012
 */

#include <acado/code_generation/integrators/erk_export.hpp>

#include <sstream>
using namespace std;

BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

ExplicitRungeKuttaExport::ExplicitRungeKuttaExport(	UserInteraction* _userInteraction,
									const String& _commonHeaderName
									) : RungeKuttaExport( _userInteraction,_commonHeaderName )
{
}


ExplicitRungeKuttaExport::ExplicitRungeKuttaExport(	const ExplicitRungeKuttaExport& arg
									) : RungeKuttaExport( arg )
{
	copy( arg );
}


ExplicitRungeKuttaExport::~ExplicitRungeKuttaExport( )
{
	clear( );
}


returnValue ExplicitRungeKuttaExport::setup( )
{
	int sensGen;
	get( DYNAMIC_SENSITIVITY,sensGen );
	if ( (ExportSensitivityType)sensGen != FORWARD && (ExportSensitivityType)sensGen != NO_SENSITIVITY ) ACADOERROR( RET_INVALID_OPTION );

	bool DERIVATIVES = ((ExportSensitivityType)sensGen != NO_SENSITIVITY);

	LOG( LVL_DEBUG ) << "Preparing to export ExplicitRungeKuttaExport... " << endl;

	// export RK scheme
	uint rhsDim   = NX*(NX+NU+1);
	if( !DERIVATIVES ) rhsDim = NX;
	inputDim = NX*(NX+NU+1) + NU + NP;
	if( !DERIVATIVES ) inputDim = NX + NU + NP;
	const uint rkOrder  = getNumStages();

	double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();    

	ExportVariable Ah ( "A*h",  Matrix( AA )*=h );
	ExportVariable b4h( "b4*h", Matrix( bb )*=h );

	rk_index = ExportVariable( "rk_index", 1, 1, INT, ACADO_LOCAL, BT_TRUE );
	rk_eta = ExportVariable( "rk_eta", 1, inputDim );

	int useOMP;
	get(CG_USE_OPENMP, useOMP);
	ExportStruct structWspace;
	structWspace = useOMP ? ACADO_LOCAL : ACADO_WORKSPACE;

	rk_ttt.setup( "rk_ttt", 1, 1, REAL, structWspace, BT_TRUE );
	uint timeDep = 0;
	if( timeDependant ) timeDep = 1;
	
	rk_xxx.setup("rk_xxx", 1, inputDim+timeDep, REAL, structWspace);
	rk_kkk.setup("rk_kkk", rkOrder, rhsDim, REAL, structWspace);

	if ( useOMP )
	{
		ExportVariable auxVar;

		auxVar = diffs_rhs.getGlobalExportVariable();
		auxVar.setName( "odeAuxVar" );
		auxVar.setDataStruct( ACADO_LOCAL );
		diffs_rhs.setGlobalExportVariable( auxVar );
	}

	ExportIndex run( "run1" );

	// setup INTEGRATE function
	if( equidistantControlGrid() ) {
		integrate = ExportFunction( "integrate", rk_eta, reset_int );
	}
	else {
		integrate = ExportFunction( "integrate", rk_eta, reset_int, rk_index );
	}
	integrate.setReturnValue( error_code );
	rk_eta.setDoc( "Working array to pass the input values and return the results." );
	reset_int.setDoc( "The internal memory of the integrator can be reset." );
	rk_index.setDoc( "Number of the shooting interval." );
	error_code.setDoc( "Status code of the integrator." );
	integrate.doc( "Performs the integration and sensitivity propagation for one shooting interval." );
	integrate.addIndex( run );

	ExportVariable numInt( "numInts", 1, 1, INT );
	if( !equidistantControlGrid() ) {
		integrate.addStatement( String( "int numSteps[" ) << String( numSteps.getDim() ) << "] = {" << String( numSteps(0) ) );
		uint i;
		for( i = 1; i < numSteps.getDim(); i++ ) {
			integrate.addStatement( String( ", " ) << String( numSteps(i) ) );
		}
		integrate.addStatement( String( "};\n" ) );
		integrate.addStatement( String( "int " ) << numInt.getName() << " = numSteps[" << rk_index.getName() << "];\n" );
	}
	
	integrate.addStatement( rk_ttt == Matrix(grid.getFirstTime()) );

	if( DERIVATIVES ) {
		// initialize sensitivities:
		Matrix idX    = eye( NX );
		Matrix zeroXU = zeros( NX,NU );
		integrate.addStatement( rk_eta.getCols( NX,NX*(1+NX) ) == idX.makeVector().transpose() );
		integrate.addStatement( rk_eta.getCols( NX*(1+NX),NX*(1+NX+NU) ) == zeroXU.makeVector().transpose() );
	}

	if( inputDim > rhsDim ) {
		integrate.addStatement( rk_xxx.getCols( rhsDim,inputDim ) == rk_eta.getCols( rhsDim,inputDim ) );
	}
	integrate.addLinebreak( );

    // integrator loop
	ExportForLoop loop;
	if( equidistantControlGrid() ) {
		loop = ExportForLoop( run, 0, grid.getNumIntervals() );
	}
	else {
		loop = ExportForLoop( run, 0, 1 );
		loop.addStatement( String("for(") << run.getName() << " = 0; " << run.getName() << " < " << numInt.getName() << "; " << run.getName() << "++ ) {\n" );
	}

	for( uint run1 = 0; run1 < rkOrder; run1++ )
	{
		loop.addStatement( rk_xxx.getCols( 0,rhsDim ) == rk_eta.getCols( 0,rhsDim ) + Ah.getRow(run1)*rk_kkk );
		if( timeDependant ) loop.addStatement( rk_xxx.getCol( inputDim ) == rk_ttt + ((double)cc(run1))/grid.getNumIntervals() );
		loop.addFunctionCall( getNameDiffsRHS(),rk_xxx,rk_kkk.getAddress(run1,0) );
	}
	loop.addStatement( rk_eta.getCols( 0,rhsDim ) += b4h^rk_kkk );
	loop.addStatement( rk_ttt += Matrix(1.0/grid.getNumIntervals()) );
    // end of integrator loop

	if( !equidistantControlGrid() ) {
		loop.addStatement( "}\n" );
//		loop.unrollLoop();
	}
	integrate.addStatement( loop );
	
	integrate.addStatement( error_code == 0 );

	LOG( LVL_DEBUG ) << "done" << endl;

	return SUCCESSFUL_RETURN;
}



returnValue ExplicitRungeKuttaExport::setDifferentialEquation(	const Expression& rhs_ )
{
	int sensGen;
	get( DYNAMIC_SENSITIVITY,sensGen );
	bool DERIVATIVES = ((ExportSensitivityType)sensGen != NO_SENSITIVITY);

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
	
	if( NDX > 0 && NDX != NX ) {
		return ACADOERROR( RET_INVALID_OPTION );
	}
	if( rhs_.getNumRows() != (NX+NXA) ) {
		return ACADOERROR( RET_INVALID_OPTION );
	}
	
	DifferentialEquation f, f_ODE;
	// add usual ODE
	f_ODE << rhs_;
	if( f_ODE.getNDX() > 0 ) {
		return ACADOERROR( RET_INVALID_OPTION );
	}

	if( DERIVATIVES ) {
		DifferentialState Gx(NX,NX), Gu(NX,NU);
		// no free parameters yet!
		// DifferentialState Gp(NX,NP);

		f << rhs_;
		/*	if ( f.getDim() != f.getNX() )
		return ACADOERROR( RET_ILLFORMED_ODE );*/

		// add VDE for differential states
		f << forwardDerivative( rhs_, x ) * Gx;
		/*	if ( f.getDim() != f.getNX() )
		return ACADOERROR( RET_ILLFORMED_ODE );*/

		// add VDE for control inputs
		f << forwardDerivative( rhs_, x ) * Gu + forwardDerivative( rhs_, u );
		// 	if ( f.getDim() != f.getNX() )
		// 		return ACADOERROR( RET_ILLFORMED_ODE );

		// no free parameters yet!
		// f << forwardDerivative( rhs_, x ) * Gp + forwardDerivative( rhs_, p );

		if( f.getNT() > 0 ) timeDependant = BT_TRUE;
	}

	int matlabInterface;
	userInteraction->get(GENERATE_MATLAB_INTERFACE, matlabInterface);
	if( matlabInterface && DERIVATIVES ) {
		return rhs.init(f_ODE, "acado_rhs", NX, 0, NU, NP)
				& diffs_rhs.init(f, "acado_rhs_ext", NX * (1 + NX + NU), 0, NU, NP);
	} else if( DERIVATIVES ) {
		return diffs_rhs.init(f, "acado_rhs_ext", NX * (1 + NX + NU), 0, NU, NP);
	} else {
		return diffs_rhs.init(f_ODE, "acado_rhs", NX, 0, NU, NP);
	}

	return SUCCESSFUL_RETURN;
}


returnValue ExplicitRungeKuttaExport::setLinearInput( const Matrix& M1, const Matrix& A1, const Matrix& B1 ) {

	return ACADOERROR( RET_INVALID_OPTION );
}


returnValue ExplicitRungeKuttaExport::setLinearOutput( const Matrix& M3, const Matrix& A3, const Expression& _rhs ) {

	return ACADOERROR( RET_INVALID_OPTION );
}


returnValue ExplicitRungeKuttaExport::setLinearOutput( const Matrix& M3, const Matrix& A3, const String& _rhs3, const String& _diffs_rhs3 )
{
	return RET_INVALID_OPTION;
}


returnValue ExplicitRungeKuttaExport::getDataDeclarations(	ExportStatementBlock& declarations,
													ExportStruct dataStruct
													) const
{
	if( exportRhs ) {
		declarations.addDeclaration( getAuxVariable(),dataStruct );
	}
	declarations.addDeclaration( rk_ttt,dataStruct );
	declarations.addDeclaration( rk_xxx,dataStruct );
	declarations.addDeclaration( rk_kkk,dataStruct );

//	declarations.addDeclaration( reset_int,dataStruct );

	return SUCCESSFUL_RETURN;
}


returnValue ExplicitRungeKuttaExport::getFunctionDeclarations(	ExportStatementBlock& declarations
														) const
{
	declarations.addDeclaration( integrate );
	if( exportRhs ) {
//		declarations.addDeclaration( diffs_rhs );
	}
	else {
		Function tmpFun;
		tmpFun << zeros(1,1);
		ExportAcadoFunction tmpExport(tmpFun, getNameDiffsRHS());
		declarations.addDeclaration( tmpExport );
	}

	int matlabInterface;
	userInteraction->get( GENERATE_MATLAB_INTERFACE, matlabInterface );
	if (matlabInterface) {
		if( exportRhs ) {
			declarations.addDeclaration( rhs );
		}
		else {
			Function tmpFun;
			tmpFun << zeros(1,1);
			ExportAcadoFunction tmpExport(tmpFun, getNameRHS());
			declarations.addDeclaration( tmpExport );
		}
	}

	return SUCCESSFUL_RETURN;
}


returnValue ExplicitRungeKuttaExport::getCode(	ExportStatementBlock& code
										)
{
// 	int printLevel;
// 	get( PRINTLEVEL,printLevel );
// 
// 	if ( (PrintLevel)printLevel >= HIGH ) 
// 		acadoPrintf( "--> Exporting %s... ",fileName.getName() );

	int useOMP;
	get(CG_USE_OPENMP, useOMP);
	if ( useOMP )
	{
		getDataDeclarations( code, ACADO_LOCAL );

		stringstream s;
		s << "#pragma omp threadprivate( "
				<< diffs_rhs.getGlobalExportVariable().getFullName().getName()  << ", "
				<< rk_xxx.getFullName().getName() << ", "
				<< rk_ttt.getFullName().getName() << ", "
				<< rk_kkk.getFullName().getName()
				<< " )" << endl << endl;

		code.addStatement( s.str().c_str() );
	}

	if( exportRhs ) code.addFunction( diffs_rhs );

	double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();
	code.addComment(String("Fixed step size:") << String(h));
	code.addFunction( integrate );

	int matlabInterface;
	userInteraction->get( GENERATE_MATLAB_INTERFACE, matlabInterface );
	if (matlabInterface) {
		if( exportRhs ) code.addFunction( rhs );
	}

// 	if ( (PrintLevel)printLevel >= HIGH ) 
// 		acadoPrintf( "done.\n" );

	return SUCCESSFUL_RETURN;
}


returnValue ExplicitRungeKuttaExport::setupOutput( const std::vector<Grid> outputGrids_, const std::vector<Expression> _rhs ) {
	
	return ACADOERROR( RET_INVALID_OPTION );
}


returnValue ExplicitRungeKuttaExport::setupOutput(  const std::vector<Grid> outputGrids_,
									  	  	  	  	const std::vector<String> _outputNames,
									  	  	  	  	const std::vector<String> _diffs_outputNames,
									  	  	  	  	const std::vector<uint> _dims_output ) {

	return ACADOERROR( RET_INVALID_OPTION );
}


returnValue ExplicitRungeKuttaExport::setupOutput(  const std::vector<Grid> outputGrids_,
									  	  	  	  	const std::vector<String> _outputNames,
									  	  	  	  	const std::vector<String> _diffs_outputNames,
									  	  	  	  	const std::vector<uint> _dims_output,
									  	  	  	  	const std::vector<Matrix> _outputDependencies ) {

	return ACADOERROR( RET_INVALID_OPTION );
}


ExportVariable ExplicitRungeKuttaExport::getAuxVariable() const
{

	return diffs_rhs.getGlobalExportVariable();
}



// PROTECTED:



CLOSE_NAMESPACE_ACADO

// end of file.
