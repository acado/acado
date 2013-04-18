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
	// non equidistant integration grids not yet implemented for explicit integrators
	if( !equidistant ) return ACADOERROR( RET_INVALID_OPTION );

	String fileName( "integrator.c" );

	int printLevel;
	get( PRINTLEVEL,printLevel );

	if ( (PrintLevel)printLevel >= HIGH ) 
		acadoPrintf( "--> Preparing to export %s... ",fileName.getName() );

	// export RK scheme
	const uint rhsDim   = NX*(NX+NU+1);
	const uint inputDim = NX*(NX+NU+1) + NU + NP;
	const uint rkOrder  = getNumStages();
	
	initializeButcherTableau();
	//grid.print();
	   
	double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();    

	ExportVariable Ah ( "A*h",  AA*=h );
	ExportVariable b4h( "b4*h", Matrix( bb )*=h );

	rk_index = ExportVariable( "rk_index", 1, 1, INT, ACADO_LOCAL, BT_TRUE );
	rk_eta = ExportVariable( "rk_eta", 1, inputDim );


//	rk_ttt.setup( "rk_ttt", 1,1,            REAL,ACADO_WORKSPACE,BT_TRUE );
	
	int useOMP;
	get(CG_USE_OPENMP, useOMP);
	ExportStruct structWspace;
	structWspace = useOMP ? ACADO_LOCAL : ACADO_WORKSPACE;
	rk_xxx.setup("rk_xxx", 1, inputDim, REAL, structWspace);
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
	
//	integrate.addStatement( rk_ttt == Matrix(grid.getFirstTime()) );

	// initialize sensitivities:
	Matrix idX    = eye( NX );
	Matrix zeroXU = zeros( NX,NU );
	integrate.addStatement( rk_eta.getCols( NX,NX*(1+NX) ) == idX.makeVector().transpose() );
	integrate.addStatement( rk_eta.getCols( NX*(1+NX),NX*(1+NX+NU) ) == zeroXU.makeVector().transpose() );

	integrate.addStatement( rk_xxx.getCols( rhsDim,inputDim ) == rk_eta.getCols( rhsDim,inputDim ) );
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
		loop.addFunctionCall( diffs_rhs.getName(), rk_xxx,rk_kkk.getAddress(run1,0) );
	}
	loop.addStatement( rk_eta.getCols( 0,rhsDim ) += b4h^rk_kkk );
//	loop.addStatement( rk_ttt += Matrix(h) );
    // end of integrator loop

	if( !equidistantControlGrid() ) {
		loop.addStatement( "}\n" );
//		loop.unrollLoop();
	}
	integrate.addStatement( loop );
	
	if ( (PrintLevel)printLevel >= HIGH ) 
		acadoPrintf( "done.\n" );	

	return SUCCESSFUL_RETURN;
}



returnValue ExplicitRungeKuttaExport::setDifferentialEquation(	const Expression& rhs_ )
{
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
	
	DifferentialState Gx(NX,NX), Gu(NX,NU);
	// no free parameters yet!
	// DifferentialState Gp(NX,NP);

	DifferentialEquation f, f_ODE;

	// add usual ODE
	f_ODE << rhs_;
	if( f_ODE.getNDX() > 0 ) {
		return ACADOERROR( RET_INVALID_OPTION );
	}

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

	int matlabInterface;
	userInteraction->get(GENERATE_MATLAB_INTERFACE, matlabInterface);
	if (matlabInterface) {
		return rhs.init(f_ODE, "acado_rhs", NX, 0, NU)
				& diffs_rhs.init(f, "acado_rhs_ext", NX * (1 + NX + NU), 0, NU);
	} else {
		return diffs_rhs.init(f, "acado_rhs_ext", NX * (1 + NX + NU), 0, NU);
	}

	return SUCCESSFUL_RETURN;
}


returnValue ExplicitRungeKuttaExport::setLinearInput( const Matrix& M1, const Matrix& A1, const Matrix& B1 ) {

	return ACADOERROR( RET_INVALID_OPTION );
}


returnValue ExplicitRungeKuttaExport::setLinearOutput( const Matrix& M3, const Matrix& A3, const Expression& _rhs ) {

	return ACADOERROR( RET_INVALID_OPTION );
}


returnValue ExplicitRungeKuttaExport::setModel(	const String& _rhs, const String& _diffs_rhs ) {

	// You can't use this feature yet with explicit integrators, because they need the Variational Differential Equations !
	return ACADOERROR( RET_INVALID_OPTION );
}


returnValue ExplicitRungeKuttaExport::getDataDeclarations(	ExportStatementBlock& declarations,
													ExportStruct dataStruct
													) const
{
	declarations.addDeclaration( diffs_rhs.getGlobalExportVariable(),dataStruct );
//	declarations.addDeclaration( rk_ttt,dataStruct );
	declarations.addDeclaration( rk_xxx,dataStruct );
	declarations.addDeclaration( rk_kkk,dataStruct );

//	declarations.addDeclaration( reset_int,dataStruct );

	return SUCCESSFUL_RETURN;
}


returnValue ExplicitRungeKuttaExport::getFunctionDeclarations(	ExportStatementBlock& declarations
														) const
{
	declarations.addDeclaration( integrate );
	declarations.addDeclaration( diffs_rhs );

	int matlabInterface;
	userInteraction->get( GENERATE_MATLAB_INTERFACE, matlabInterface );
	if (matlabInterface) {
		declarations.addDeclaration( rhs );
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
				<< rk_kkk.getFullName().getName()
				<< " )" << endl << endl;

		code.addStatement( s.str().c_str() );
	}

	code.addFunction( diffs_rhs );
	code.addFunction( integrate );

	int matlabInterface;
	userInteraction->get( GENERATE_MATLAB_INTERFACE, matlabInterface );
	if (matlabInterface) {
		code.addFunction( rhs );
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
