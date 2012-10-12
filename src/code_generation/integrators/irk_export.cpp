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
 *    \file src/code_generation/irk_export.cpp
 *    \author Rien Quirynen
 *    \date 2012
 */

#include <acado/code_generation/integrators/irk_export.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

ImplicitRungeKuttaExport::ImplicitRungeKuttaExport(	UserInteraction* _userInteraction,
									const String& _commonHeaderName
									) : RungeKuttaExport( _userInteraction,_commonHeaderName )
{

	numIts = 2; 		// DEFAULT value
	numItsInit = 0; 	// DEFAULT value
	numAlgIts = 1;		// DEFAULT value
	numAlgItsInit = 2; 	// DEFAULT value
	REUSE = BT_TRUE;
	CONTINUOUS_OUTPUT = BT_FALSE;
	UNROLL_OUTPUT = BT_TRUE;

	solver = 0;
	daeSolver = 0;
}

ImplicitRungeKuttaExport::ImplicitRungeKuttaExport( const ImplicitRungeKuttaExport& arg ) : RungeKuttaExport( arg )
{

	numIts = arg.numIts ;
	numItsInit = arg.numItsInit ;
	numAlgIts = arg.numAlgIts;
	numAlgItsInit = arg.numAlgItsInit;
    diffs_OUTPUTS = arg.diffs_OUTPUTS;
    OUTPUTS = arg.OUTPUTS;
	outputGrids = arg.outputGrids;
    solver = arg.solver;
    daeSolver = arg.daeSolver;
	REUSE = arg.REUSE;;
	CONTINUOUS_OUTPUT = arg.CONTINUOUS_OUTPUT;
	UNROLL_OUTPUT = arg.UNROLL_OUTPUT;
}


ImplicitRungeKuttaExport::~ImplicitRungeKuttaExport( )
{
	if ( solver )
		delete solver;
	solver = 0;

	if ( daeSolver )
		delete daeSolver;
	daeSolver = 0;

	clear( );
}


ImplicitRungeKuttaExport& ImplicitRungeKuttaExport::operator=( const ImplicitRungeKuttaExport& arg ){

    if( this != &arg ){

		RungeKuttaExport::operator=( arg );
		copy( arg );
    }
    return *this;
}


returnValue ImplicitRungeKuttaExport::setDifferentialEquation(	const Expression& rhs )
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
	
	uint i;
	
	if( NDX > 0 && NDX != NX ) {
		return ACADOERROR( RET_INVALID_OPTION );
	}
	if( rhs.getNumRows() != (NX+NXA) ) {
		return ACADOERROR( RET_INVALID_OPTION );
	}
	
	// ODE equations:
	Expression rhs_ODE; 
	for( i = 0; i < NX; i++ ) {
		rhs_ODE << rhs.getRow( i );
	}
	DifferentialEquation f_ODE;
	f_ODE << rhs_ODE;
	
	DifferentialEquation g_ODE;
	g_ODE << forwardDerivative( rhs_ODE, x );
	g_ODE << forwardDerivative( rhs_ODE, z );
	g_ODE << forwardDerivative( rhs_ODE, u );
	g_ODE << forwardDerivative( rhs_ODE, dx );
	
	// DAE equations:
	Expression rhs_DAE; 
	for( i = 0; i < NXA; i++ ) {
		rhs_DAE << rhs.getRow( NX+i );
	}
	if( rhs_DAE.isDependingOn( VT_DDIFFERENTIAL_STATE ) ) {
		return ACADOERROR( RET_INVALID_OPTION );
	}
	DifferentialEquation f_DAE;
	if( NXA > 0 ) {
		f_DAE << rhs_DAE;
	}
	
	DifferentialEquation g_DAE;
	if( NXA > 0 ) {
		g_DAE << forwardDerivative( rhs_DAE, x );
		g_DAE << forwardDerivative( rhs_DAE, z );
		g_DAE << forwardDerivative( rhs_DAE, u );
	}

	setup();
	if( NXA > 0 ) {
		return (ODE.init( f_ODE,"acado_rhs",NX,NXA,NU ) & diffs_ODE.init( g_ODE,"acado_diffs",NX,NXA,NU ) & DAE.init( f_DAE,"acado_alg_rhs",NX,NXA,NU ) & diffs_DAE.init( g_DAE,"acado_alg_diffs",NX,NXA,NU ) );
	}
	else {
		return (ODE.init( f_ODE,"acado_rhs",NX,NXA,NU ) & diffs_ODE.init( g_ODE,"acado_diffs",NX,NXA,NU ));
	}
}


returnValue ImplicitRungeKuttaExport::setModel(	const String& _rhs, const String& _diffs_rhs ) {

	IntegratorExport::setModel( _rhs, _diffs_rhs );

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

	setup();

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::getDataDeclarations(	ExportStatementBlock& declarations,
												ExportStruct dataStruct
												) const
{
	solver->getDataDeclarations( declarations,dataStruct );
	if( NXA > 0 ) daeSolver->getDataDeclarations( declarations,dataStruct );
	
	if( EXPORT_RHS ) {
		ExportVariable max = ODE.getGlobalExportVariable();
		if( DAE.getGlobalExportVariable().getDim() >= max.getDim() ) {
			max = DAE.getGlobalExportVariable();
		}
		if( diffs_ODE.getGlobalExportVariable().getDim() >= max.getDim() ) {
			max = diffs_ODE.getGlobalExportVariable();
		}
		if( diffs_DAE.getGlobalExportVariable().getDim() >= max.getDim() ) {
			max = diffs_DAE.getGlobalExportVariable();
		}
		uint i;
		for( i = 0; i < OUTPUTS.size(); i++ ) {
			if( OUTPUTS[i].getGlobalExportVariable().getDim() >= max.getDim() ) {
				max = OUTPUTS[i].getGlobalExportVariable();
			}
			if( diffs_OUTPUTS[i].getGlobalExportVariable().getDim() >= max.getDim() ) {
				max = diffs_OUTPUTS[i].getGlobalExportVariable();
			}
		}
		declarations.addDeclaration( max,dataStruct );
	}

	declarations.addDeclaration( rk_ttt,dataStruct );
	declarations.addDeclaration( rk_xxx,dataStruct );
	declarations.addDeclaration( rk_kkk,dataStruct );
	declarations.addDeclaration( rk_sol,dataStruct );
	declarations.addDeclaration( rk_A,dataStruct );
	declarations.addDeclaration( rk_b,dataStruct );
	declarations.addDeclaration( rk_alg_A,dataStruct );
	declarations.addDeclaration( rk_alg_b,dataStruct );
	declarations.addDeclaration( rk_rhsTemp,dataStruct );
	declarations.addDeclaration( rk_diffsTemp,dataStruct );
	declarations.addDeclaration( rk_alg_diffsTemp,dataStruct );
	
	declarations.addDeclaration( rk_num,dataStruct );
	
	if( grid.getNumIntervals() > 1 || !hasEquidistantGrid() ) {
		declarations.addDeclaration( rk_diffsPrev,dataStruct );
		declarations.addDeclaration( rk_diffsNew,dataStruct );
	}
	
	if( CONTINUOUS_OUTPUT ) {
		declarations.addDeclaration( rk_xPrev,dataStruct );
		declarations.addDeclaration( rk_rhsOutputTemp,dataStruct );
		declarations.addDeclaration( rk_diffsOutputTemp,dataStruct );
		if( !UNROLL_OUTPUT ) declarations.addDeclaration( rk_outH,dataStruct );
	}

    return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::getFunctionDeclarations(	ExportStatementBlock& declarations
													) const
{
	declarations.addDeclaration( integrate );
	solver->getFunctionDeclarations( declarations );

	if( EXPORT_RHS ) {
		declarations.addDeclaration( ODE );
		declarations.addDeclaration( diffs_ODE );
	}
	else {
		Function tmpFun;
		ExportODEfunction tmpExport(tmpFun, getNameODE());
		declarations.addDeclaration( tmpExport );
		tmpExport = ExportODEfunction(tmpFun, getNameDiffsODE());
		declarations.addDeclaration( tmpExport );
	}
	if( NXA > 0 ) {
		if( EXPORT_RHS ) {
			declarations.addDeclaration( DAE );
			declarations.addDeclaration( diffs_DAE );
		}
		else {
			Function tmpFun;
			ExportODEfunction tmpExport(tmpFun, getNameDAE());
			declarations.addDeclaration( tmpExport );
			tmpExport = ExportODEfunction(tmpFun, getNameDiffsDAE());
			declarations.addDeclaration( tmpExport );
		}
		daeSolver->getFunctionDeclarations( declarations );
		declarations.addDeclaration( makeStatesConsistent );
		declarations.addDeclaration( getNormConsistency );
	}
	uint i;
	if( EXPORT_RHS && CONTINUOUS_OUTPUT ) {
		for( i = 0; i < OUTPUTS.size(); i++ ) {
			declarations.addDeclaration( OUTPUTS[i] );
			declarations.addDeclaration( diffs_OUTPUTS[i] );
		}
	}
	else {
		for( i = 0; i < name_OUTPUTS.size(); i++ ) {
			Function tmpFun;
			ExportODEfunction tmpExport(tmpFun, getNameOUTPUT(i));
			declarations.addDeclaration( tmpExport );
			tmpExport = ExportODEfunction(tmpFun, getNameDiffsOUTPUT(i));
			declarations.addDeclaration( tmpExport );
		}
	}

    return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::getCode(	ExportStatementBlock& code
									)
{
	if( EXPORT_RHS ) {
		code.addFunction( ODE );
		code.addStatement( "\n\n" );
		code.addFunction( diffs_ODE );
		code.addStatement( "\n\n" );

		if( NXA > 0 ) {
			code.addFunction( DAE );
			code.addStatement( "\n\n" );
			code.addFunction( diffs_DAE );
			code.addStatement( "\n\n" );
		}

		if( CONTINUOUS_OUTPUT ) {
			uint i;
			for( i = 0; i < OUTPUTS.size(); i++ ) {
				code.addFunction( OUTPUTS[i] );
				code.addStatement( "\n\n" );
				code.addFunction( diffs_OUTPUTS[i] );
				code.addStatement( "\n\n" );
			}
		}
	}

	// export RK scheme
	uint run1, run2, run3, run4, run5;
	String tempString;
	
	Matrix id = eye( (NX+NXA)*numStages );
	Matrix nul = zeros( 1,NX+NXA );
	initializeButcherTableau();
	initializeDDMatrix();
	initializeCoefficients();
	   
	double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();    

	ExportVariable Ah ( "AA*h",  AA*=h );
	ExportVariable Bh( "bb*h", Matrix( bb, BT_TRUE )*=h );
	ExportVariable DM( "DD", DD );
	ExportVariable eyeM( "id", id );
	ExportVariable zeroM( "nul", nul );
	
	ExportIndex i( "i" );
	ExportIndex j( "j" );
	ExportIndex k( "k" );
	ExportIndex run( "run" );
	

	ExportVariable numInt( "numInts", 1, 1, INT );
	if( !hasEquidistantGrid() ) {
		integrate.addStatement( String( "int " ) << run.getName() << ";\n" );
		integrate.addStatement( String( "static const int numSteps[" ) << String( numSteps.getDim() ) << "] = {" << String( (int)numSteps(0) ) );
		uint index;
		for( index = 1; index < numSteps.getDim(); index++ ) {
			integrate.addStatement( String( ", " ) << String( (int)numSteps(index) ) );
		}
		integrate.addStatement( String( "};\n" ) );
		integrate.addStatement( String( "int " ) << numInt.getName() << " = numSteps[" << rk_index.getName() << "];\n" );
	}
	
	std::vector<ExportVariable> gridVariables;
	if( !UNROLL_OUTPUT ) {
		for( run1 = 0; run1 < outputGrids.size(); run1++ ) {
			ExportVariable gridVariable( (String)"gridOutput" << run1, 1, outputGrids[run1].getNumIntervals(), REAL );
			integrate.addStatement( (String)"static const real_t " << gridVariable.getName() << "[" << outputGrids[run1].getNumIntervals() << "] = {" << outputGrids[run1].getTime(1) );
			uint index;
			for( index = 1; index < outputGrids[run1].getNumIntervals(); index++ ) {
				integrate.addStatement( (String)", " << outputGrids[run1].getTime( index+1 ) );
			}
			integrate.addStatement( String( "};\n" ) );
			gridVariables.push_back( gridVariable );
		}
	}

	integrate.addIndex( i );
	integrate.addIndex( j );
	integrate.addIndex( k );
	integrate.addIndex( run );
	integrate.addStatement( rk_ttt == Matrix(grid.getFirstTime()) );
	integrate.addStatement( rk_xxx.getCols( NX+NXA,inputDim-diffsDim ) == rk_eta.getCols( NX+NXA+diffsDim,inputDim ) );
	if( CONTINUOUS_OUTPUT ) integrate.addStatement( rk_xPrev.getCols( NX+NXA,inputDim-diffsDim ) == rk_xxx.getCols( NX+NXA,inputDim-diffsDim ) );
	
	integrate.addLinebreak( );

    // integrator loop:
    ExportForLoop loop;
	if( hasEquidistantGrid() ) {
		loop = ExportForLoop( run, 0, grid.getNumIntervals() );
	}
	else {
		loop = ExportForLoop( run, 0, 1 );
		loop.addStatement( String("for(") << run.getName() << " = 0; " << run.getName() << " < " << numInt.getName() << "; " << run.getName() << "++ ) {\n" );
	}
	
	if( grid.getNumIntervals() > 1 || !hasEquidistantGrid() ) {
		if( hasEquidistantGrid() ) loop.addStatement( String( "if( run > 1 ) {\n" ) ); // greater than one, so only when the multiplication of the sensitivity matrices is done (see end of loop)
		for( run1 = 0; run1 < (NX+NXA); run1++ ) {
			loop.addStatement( rk_diffsPrev.getSubMatrix( run1,run1+1,0,NX ) == rk_eta.getCols( NX+NXA+run1*NX,NX+NXA+run1*NX+NX ) );
		}
		for( run1 = 0; run1 < (NX+NXA); run1++ ) {
			loop.addStatement( rk_diffsPrev.getSubMatrix( run1,run1+1,NX,NX+NU ) == rk_eta.getCols( (NX+NXA)*(NX+1)+run1*NU,(NX+NXA)*(NX+1)+run1*NU+NU ) );
		}
		if( hasEquidistantGrid() ) loop.addStatement( String( "}\n" ) );
	}
	
	if( REUSE ) loop.addStatement( String( "if( " ) << rk_num.get(0,0) << " == 0 ) {\n" );
	// Initialization iterations:
	ExportForLoop loop1( i,0,numItsInit+1 ); // NOTE: +1 because 0 will lead to NaNs, so the minimum number of iterations is 1 at the initialization
	for( run1 = 0; run1 < numStages; run1++ ) {
		loop1.addStatement( rk_xxx.getCols( 0,NX+NXA ) == rk_eta.getCols( 0,NX+NXA ) + Ah.getRow(run1)*rk_kkk );
		if( NDX > 0 ) {
			loop1.addStatement( rk_xxx.getCols( inputDim-diffsDim,inputDim-diffsDim+NDX ) == rk_kkk.getSubMatrix( run1,run1+1,0,NX ) );
		}
		loop1.addFunctionCall( getNameODE(), rk_xxx, rk_rhsTemp.getAddress(0,0) );
		loop1.addFunctionCall( getNameDiffsODE(), rk_xxx, rk_diffsTemp.getAddress(0,0) );
		if( NXA > 0 ) {
			loop1.addFunctionCall( getNameDAE(), rk_xxx, rk_rhsTemp.getAddress(0,NX) );
			loop1.addFunctionCall( getNameDiffsDAE(), rk_xxx, rk_diffsTemp.getAddress(0,NX*(NX+NXA+NU+NDX)) );
		}
		// matrix rk_A:
		for( run2 = 0; run2 < NX; run2++ ) {
			for( run3 = 0; run3 < numStages; run3++ ) {
				if( NDX == 0 ) {
					loop1.addStatement( rk_A.getSubMatrix( run1*(NX+NXA)+run2,run1*(NX+NXA)+run2+1,run3*(NX+NXA),run3*(NX+NXA)+NX ) == eyeM.getSubMatrix( run1*(NX+NXA)+run2,run1*(NX+NXA)+run2+1,run3*(NX+NXA),run3*(NX+NXA)+NX ) - Ah.getSubMatrix( run1,run1+1,run3,run3+1 )*rk_diffsTemp.getSubMatrix( 0,1,run2*NX,run2*NX+NX ) );
				}
				else {
					if( run3 == run1 ) {
						loop1.addStatement( rk_A.getSubMatrix( run1*(NX+NXA)+run2,run1*(NX+NXA)+run2+1,run3*(NX+NXA),run3*(NX+NXA)+NX ) == rk_diffsTemp.getSubMatrix( 0,1,NX*(NX+NXA+NU)+run2*NDX,NX*(NX+NXA+NU)+run2*NDX+NDX ) + Ah.getSubMatrix( run1,run1+1,run3,run3+1 )*rk_diffsTemp.getSubMatrix( 0,1,run2*NX,run2*NX+NX ) );
					}
					else {
						loop1.addStatement( rk_A.getSubMatrix( run1*(NX+NXA)+run2,run1*(NX+NXA)+run2+1,run3*(NX+NXA),run3*(NX+NXA)+NX ) == Ah.getSubMatrix( run1,run1+1,run3,run3+1 )*rk_diffsTemp.getSubMatrix( 0,1,run2*NX,run2*NX+NX ) );
					}
				}
				if( NXA > 0 ) {
					loop1.addStatement( rk_A.getSubMatrix( run1*(NX+NXA)+run2,run1*(NX+NXA)+run2+1,run3*(NX+NXA)+NX,run3*(NX+NXA)+NX+NXA ) == zeroM.getCols( 0,NXA ) - Ah.getSubMatrix( run1,run1+1,run3,run3+1 )*rk_diffsTemp.getSubMatrix( 0,1,NX*NX+run2*NXA,NX*NX+run2*NXA+NXA ) );
				}
			}
		}
		for( run2 = 0; run2 < NXA; run2++ ) {
			for( run3 = 0; run3 < numStages; run3++ ) {
				loop1.addStatement( rk_A.getSubMatrix( run1*(NX+NXA)+NX+run2,run1*(NX+NXA)+NX+run2+1,run3*(NX+NXA),run3*(NX+NXA)+NX ) == Ah.getSubMatrix( run1,run1+1,run3,run3+1 )*rk_diffsTemp.getSubMatrix( 0,1,NX*(NX+NXA+NU+NDX)+run2*NX,NX*(NX+NXA+NU+NDX)+run2*NX+NX ) );
				loop1.addStatement( rk_A.getSubMatrix( run1*(NX+NXA)+NX+run2,run1*(NX+NXA)+NX+run2+1,run3*(NX+NXA)+NX,run3*(NX+NXA)+NX+NXA ) == Ah.getSubMatrix( run1,run1+1,run3,run3+1 )*rk_diffsTemp.getSubMatrix( 0,1,NX*(NX+NXA+NU+NDX)+NXA*NX+run2*NXA,NX*(NX+NXA+NU+NDX)+NXA*NX+run2*NXA+NXA ) );
			}
		}
		// matrix rk_b:
		if( NDX == 0 ) {
			loop1.addStatement( rk_b.getCols( run1*(NX+NXA),run1*(NX+NXA)+NX ) == rk_rhsTemp.getCols( 0,NX ) - rk_kkk.getSubMatrix( run1,run1+1,0,NX ) );
		}
		else {
			loop1.addStatement( rk_b.getCols( run1*(NX+NXA),run1*(NX+NXA)+NX ) == zeroM.getCols( 0,NX ) - rk_rhsTemp.getCols( 0,NX ) );
		}
		if( NXA > 0 ) {
			loop1.addStatement( rk_b.getCols( run1*(NX+NXA)+NX,run1*(NX+NXA)+NX+NXA ) == zeroM.getCols( 0,NXA ) - rk_rhsTemp.getCols( NX,NX+NXA ) );
		}
	}
	loop1.addFunctionCall( solver->getNameSolveFunction(),rk_A.getAddress(0,0),rk_b.getAddress(0,0),rk_sol.getAddress(0,0) );
	loop1.addStatement( rk_kkk += rk_sol );
	loop.addStatement( loop1 );
	if( REUSE ) loop.addStatement( String( "}\n" ) );
	

	// the rest (numIts) of the Newton iterations with reuse of the Jacobian (no evaluation or factorization needed)
	ExportForLoop loop2( i,0,numIts );
	for( run1 = 0; run1 < numStages; run1++ ) {
		loop2.addStatement( rk_xxx.getCols( 0,NX+NXA ) == rk_eta.getCols( 0,NX+NXA ) + Ah.getRow(run1)*rk_kkk );
		if( NDX > 0 ) {
			loop2.addStatement( rk_xxx.getCols( inputDim-diffsDim,inputDim-diffsDim+NDX ) == rk_kkk.getSubMatrix( run1,run1+1,0,NX ) );
		}
		loop2.addFunctionCall( getNameODE(), rk_xxx, rk_rhsTemp.getAddress(0,0) );
		if( NXA > 0 ) {
			loop2.addFunctionCall( getNameDAE(), rk_xxx, rk_rhsTemp.getAddress(0,NX) );
		}
		
		// matrix rk_b:
		if( NDX == 0 ) {
			loop2.addStatement( rk_b.getCols( run1*(NX+NXA),run1*(NX+NXA)+NX ) == rk_rhsTemp.getCols( 0,NX ) - rk_kkk.getSubMatrix( run1,run1+1,0,NX ) );
		}
		else {
			loop2.addStatement( rk_b.getCols( run1*(NX+NXA),run1*(NX+NXA)+NX ) == zeroM.getCols( 0,NX ) - rk_rhsTemp.getCols( 0,NX ) );
		}
		if( NXA > 0 ) {
			loop2.addStatement( rk_b.getCols( run1*(NX+NXA)+NX,run1*(NX+NXA)+NX+NXA ) == zeroM.getCols( 0,NXA ) - rk_rhsTemp.getCols( NX,NX+NXA ) );
		}
	}
	loop2.addFunctionCall( solver->getNameSolveReuseFunction(),rk_A.getAddress(0,0),rk_b.getAddress(0,0),rk_sol.getAddress(0,0) );
	loop2.addStatement( rk_kkk += rk_sol );
	loop.addStatement( loop2 );
	
	// generate continuous output (except for the last grid point):
	for( run1 = 0; run1 < rk_outputs.size(); run1++ ) {
		if( UNROLL_OUTPUT ) {
			for( run2 = 0; run2 < outputGrids[run1].getNumIntervals()-1; run2++ ) {
				Vector out = evaluatePolynomial( outputGrids[run1].getTime( run2+1 ) );
				rk_outH = ExportVariable( "out*h", Matrix( out, BT_TRUE )*=h );
				Vector dependency;
				if( EXPORT_RHS ) dependency = sumRow( outputExpressions[run1].getDependencyPattern( x ) );
				for( run3 = 0; run3 < NX; run3++ ) {
					if( !EXPORT_RHS || (int)dependency(run3) != 0 ) {
						loop.addStatement( rk_xxx.getCols( run3,run3+1 ) == rk_eta.getCols( run3,run3+1 ) + rk_outH*rk_kkk.getCol( run3 ) );
					}
				}
				for( run3 = 0; run3 < NXA; run3++ ) { // getDependencyPattern doesn't work for algebraic states:
					loop.addStatement( rk_xxx.getCols( NX+run3,NX+run3+1 ) == rk_eta.getCols( NX+run3,NX+run3+1 ) + rk_outH*rk_kkk.getCol( NX+run3 ) );
				}
				loop.addFunctionCall( getNameOUTPUT( run1 ), rk_xxx, rk_rhsOutputTemp.getAddress(0,0) );
				uint numOutputs = getDimOUTPUT( run1 );
				uint outputDim = outputGrids[run1].getNumIntervals( )*numOutputs*(NX+NU+1);
				for( run3 = 0; run3 < numOutputs; run3++ ) {
					loop.addStatement( String( rk_outputs[run1].getName() ) << "[run*" << String( outputDim ) << "+" << String( run2*numOutputs*(NX+NU+1)+run3 ) << "] = " << rk_rhsOutputTemp.get( 0,run3 ) << ";\n" );
				}
			}
		}
		else {
			loop2 = ExportForLoop( i,0,outputGrids[run1].getNumIntervals()-1 );
			evaluatePolynomial( loop2, rk_outH, gridVariables[run1], i, h );
			Vector dependency;
			if( EXPORT_RHS ) dependency = sumRow( outputExpressions[run1].getDependencyPattern( x ) );
			for( run3 = 0; run3 < NX; run3++ ) {
				if( !EXPORT_RHS || (int)dependency(run3) != 0 ) {
					loop2.addStatement( rk_xxx.getCols( run3,run3+1 ) == rk_eta.getCols( run3,run3+1 ) + rk_outH*rk_kkk.getCol( run3 ) );
				}
			}
			for( run3 = 0; run3 < NXA; run3++ ) { // getDependencyPattern doesn't work for algebraic states:
				loop2.addStatement( rk_xxx.getCols( NX+run3,NX+run3+1 ) == rk_eta.getCols( NX+run3,NX+run3+1 ) + rk_outH*rk_kkk.getCol( NX+run3 ) );
			}
			loop2.addFunctionCall( getNameOUTPUT( run1 ), rk_xxx, rk_rhsOutputTemp.getAddress(0,0) );
			uint numOutputs = getDimOUTPUT( run1 );
			uint outputDim = outputGrids[run1].getNumIntervals( )*numOutputs*(NX+NU+1);
			for( run3 = 0; run3 < numOutputs; run3++ ) {
				loop2.addStatement( String( rk_outputs[run1].getName() ) << "[run*" << String( outputDim ) << "+i*" << String( numOutputs*(NX+NU+1) ) << "+" << run3 << "] = " << rk_rhsOutputTemp.get( 0,run3 ) << ";\n" );
			}
			loop.addStatement( loop2 );
		}
	}
	
	// solution calculated --> evaluate and save the necessary derivatives in rk_diffsTemp and update the matrix rk_A:
	for( run1 = 0; run1 < numStages; run1++ ) {
		loop.addStatement( rk_xxx.getCols( 0,NX+NXA ) == rk_eta.getCols( 0,NX+NXA ) + Ah.getRow(run1)*rk_kkk );
		if( NDX > 0 ) {
			loop.addStatement( rk_xxx.getCols( inputDim-diffsDim,inputDim-diffsDim+NDX ) == rk_kkk.getSubMatrix( run1,run1+1,0,NX ) );
		}
		loop.addFunctionCall( getNameDiffsODE(), rk_xxx, rk_diffsTemp.getAddress(run1,0) );
		if( NXA > 0 ) {
			loop.addFunctionCall( getNameDiffsDAE(), rk_xxx, rk_diffsTemp.getAddress(run1,NX*(NX+NXA+NU+NDX)) );
		}
		
		// matrix rk_A:
		for( run2 = 0; run2 < NX; run2++ ) {
			for( run3 = 0; run3 < numStages; run3++ ) {
				if( NDX == 0 ) {
					loop.addStatement( rk_A.getSubMatrix( run1*(NX+NXA)+run2,run1*(NX+NXA)+run2+1,run3*(NX+NXA),run3*(NX+NXA)+NX ) == eyeM.getSubMatrix( run1*(NX+NXA)+run2,run1*(NX+NXA)+run2+1,run3*(NX+NXA),run3*(NX+NXA)+NX ) - Ah.getSubMatrix( run1,run1+1,run3,run3+1 )*rk_diffsTemp.getSubMatrix( 0,1,run2*NX,run2*NX+NX ) );
				}
				else {
					if( run3 == run1 ) {
						loop.addStatement( rk_A.getSubMatrix( run1*(NX+NXA)+run2,run1*(NX+NXA)+run2+1,run3*(NX+NXA),run3*(NX+NXA)+NX ) == rk_diffsTemp.getSubMatrix( 0,1,NX*(NX+NXA+NU)+run2*NDX,NX*(NX+NXA+NU)+run2*NDX+NDX ) + Ah.getSubMatrix( run1,run1+1,run3,run3+1 )*rk_diffsTemp.getSubMatrix( 0,1,run2*NX,run2*NX+NX ) );
					}
					else {
						loop.addStatement( rk_A.getSubMatrix( run1*(NX+NXA)+run2,run1*(NX+NXA)+run2+1,run3*(NX+NXA),run3*(NX+NXA)+NX ) == Ah.getSubMatrix( run1,run1+1,run3,run3+1 )*rk_diffsTemp.getSubMatrix( 0,1,run2*NX,run2*NX+NX ) );
					}
				}
				if( NXA > 0 ) {
					loop.addStatement( rk_A.getSubMatrix( run1*(NX+NXA)+run2,run1*(NX+NXA)+run2+1,run3*(NX+NXA)+NX,run3*(NX+NXA)+NX+NXA ) == zeroM.getCols( 0,NXA ) - Ah.getSubMatrix( run1,run1+1,run3,run3+1 )*rk_diffsTemp.getSubMatrix( 0,1,NX*NX+run2*NXA,NX*NX+run2*NXA+NXA ) );
				}
			} 
		}
		for( run2 = 0; run2 < NXA; run2++ ) {
			for( run3 = 0; run3 < numStages; run3++ ) {
				loop.addStatement( rk_A.getSubMatrix( run1*(NX+NXA)+NX+run2,run1*(NX+NXA)+NX+run2+1,run3*(NX+NXA),run3*(NX+NXA)+NX ) == Ah.getSubMatrix( run1,run1+1,run3,run3+1 )*rk_diffsTemp.getSubMatrix( 0,1,NX*(NX+NXA+NU+NDX)+run2*NX,NX*(NX+NXA+NU+NDX)+run2*NX+NX ) );
				loop.addStatement( rk_A.getSubMatrix( run1*(NX+NXA)+NX+run2,run1*(NX+NXA)+NX+run2+1,run3*(NX+NXA)+NX,run3*(NX+NXA)+NX+NXA ) == Ah.getSubMatrix( run1,run1+1,run3,run3+1 )*rk_diffsTemp.getSubMatrix( 0,1,NX*(NX+NXA+NU+NDX)+NXA*NX+run2*NXA,NX*(NX+NXA+NU+NDX)+NXA*NX+run2*NXA+NXA ) );
			} 
		}
	}
	
	// update rk_eta:
	if( CONTINUOUS_OUTPUT ) loop.addStatement( rk_xPrev.getCols( 0,NX+NXA ) == rk_eta.getCols( 0,NX+NXA ) );
	loop.addStatement( rk_eta.getCols( 0,NX+NXA ) += Bh*rk_kkk );
	
	// make algebraic states consistent:
	if( NXA > 0 ) {
		loop.addStatement( rk_xxx.getCols( 0,NX+NXA ) == rk_eta.getCols( 0,NX+NXA ) );
		
		if( numAlgIts > 0 ) {
			if( REUSE ) loop.addStatement( String( "if( " ) << rk_num.get(0,0) << " == 0 ) {\n" );
			// matrix rk_alg_A (first step):
			loop.addFunctionCall( getNameDiffsDAE(), rk_xxx, rk_alg_diffsTemp.getAddress(0,0) );
			for( run1 = 0; run1 < NXA; run1++ ) {
				loop.addStatement( rk_alg_A.getRow( run1 ) == rk_alg_diffsTemp.getCols( NXA*NX+run1*NXA,NXA*NX+run1*NXA+NXA ) );
			}
			// update rk_alg_b:
			loop.addFunctionCall( getNameDAE(), rk_xxx, rk_rhsTemp.getAddress(0,NX) );
			loop.addStatement( rk_alg_b == zeroM.getCols( 0,NXA ) - rk_rhsTemp.getCols( NX,NX+NXA ) );
			loop.addFunctionCall( daeSolver->getNameSolveFunction(),rk_alg_A.getAddress(0,0),rk_alg_b.getAddress(0,0),rk_sol.getAddress(0,0) );
			loop.addStatement( rk_xxx.getCols( NX,NX+NXA ) += rk_sol.getSubMatrix( 0,1,0,NXA ) );
			if( REUSE ) loop.addStatement( String( "}\n" ) );
			
			
			ExportForLoop loop3( i,0,numAlgIts );
			// update rk_alg_b:
			loop3.addFunctionCall( getNameDAE(), rk_xxx, rk_rhsTemp.getAddress(0,NX) );
			loop3.addStatement( rk_alg_b == zeroM.getCols( 0,NXA ) - rk_rhsTemp.getCols( NX,NX+NXA ) );
			loop3.addFunctionCall( daeSolver->getNameSolveReuseFunction(),rk_alg_A.getAddress(0,0),rk_alg_b.getAddress(0,0),rk_sol.getAddress(0,0) );
			loop3.addStatement( rk_xxx.getCols( NX,NX+NXA ) += rk_sol.getSubMatrix( 0,1,0,NXA ) );
			loop.addStatement( loop3 );
		}
		
		// algebraic states consistent --> evaluate and save the necessary derivatives in rk_alg_diffsTemp and update the matrix rk_alg_A:
		loop.addFunctionCall( getNameDiffsDAE(), rk_xxx, rk_alg_diffsTemp.getAddress(0,0) );
		for( run1 = 0; run1 < NXA; run1++ ) {
			loop.addStatement( rk_alg_A.getRow( run1 ) == rk_alg_diffsTemp.getCols( NXA*NX+run1*NXA,NXA*NX+run1*NXA+NXA ) );
		}
		
		// update rk_eta:
		loop.addStatement( rk_eta.getCols( NX,NX+NXA ) == rk_xxx.getCols( NX,NX+NXA ) );
	}
	
	// generate continuous output (only the last grid point):
	if( CONTINUOUS_OUTPUT ) loop.addStatement( rk_xxx.getCols( 0,NX+NXA ) == rk_eta.getCols( 0,NX+NXA ) );
	for( run1 = 0; run1 < rk_outputs.size(); run1++ ) {
		run2 = outputGrids[run1].getNumIntervals()-1;
		loop.addFunctionCall( getNameOUTPUT( run1 ), rk_xxx, rk_rhsOutputTemp.getAddress(0,0) );
		uint numOutputs = getDimOUTPUT( run1 );
		uint outputDim = outputGrids[run1].getNumIntervals( )*numOutputs*(NX+NU+1);
		for( run3 = 0; run3 < numOutputs; run3++ ) {
			loop.addStatement( String( rk_outputs[run1].getName() ) << "[run*" << String( outputDim ) << "+" << String( run2*numOutputs*(NX+NU+1)+run3 ) << "] = " << rk_rhsOutputTemp.get( 0,run3 ) << ";\n" );
		}
	}
	
	// derivatives wrt the states (IFT):
	for( run1 = 0; run1 < NX; run1++ ) {
		for( run2 = 0; run2 < numStages; run2++ ) {
			for( run3 = 0; run3 < NX; run3++ ) {
				if( NDX == 0 ) {
					loop.addStatement( rk_b.getCol( run2*(NX+NXA)+run3 ) == rk_diffsTemp.getSubMatrix( run2,run2+1,run3*NX+run1,run3*NX+run1+1 ) );
				}
				else {
					loop.addStatement( rk_b.getCol( run2*(NX+NXA)+run3 ) == zeroM.getCols( 0,1 ) - rk_diffsTemp.getSubMatrix( run2,run2+1,run3*NX+run1,run3*NX+run1+1 ) );
				}
			}
			for( run3 = 0; run3 < NXA; run3++ ) {
				loop.addStatement( rk_b.getCol( run2*(NX+NXA)+NX+run3 ) == zeroM.getCols( 0,1 ) - rk_diffsTemp.getSubMatrix( run2,run2+1,NX*(NX+NXA+NU+NDX)+run3*NX+run1,NX*(NX+NXA+NU+NDX)+run3*NX+run1+1 ) );
			}
		}
		if( run1 == 0 ) { // factorization of the new matrix rk_A not yet calculated!
			loop.addFunctionCall( solver->getNameSolveFunction(),rk_A.getAddress(0,0),rk_b.getAddress(0,0),rk_sol.getAddress(0,0) );
		} else {
			loop.addFunctionCall( solver->getNameSolveReuseFunction(),rk_A.getAddress(0,0),rk_b.getAddress(0,0),rk_sol.getAddress(0,0) );
		}
		
		// generate sensitivities wrt states for continuous output (except for the last grid point):
		for( run2 = 0; run2 < rk_outputs.size(); run2++ ) {
			if(UNROLL_OUTPUT) {
				for( run3 = 0; run3 < outputGrids[run2].getNumIntervals()-1; run3++ ) {
					Vector out = evaluatePolynomial( outputGrids[run2].getTime( run3+1 ) );
					rk_outH = ExportVariable( "out*h", Matrix( out, BT_TRUE )*=h );
					Vector dependency;
					if( EXPORT_RHS ) dependency = sumRow( outputExpressions[run2].getDependencyPattern( x ) );
					for( run4 = 0; run4 < NX; run4++ ) {
						if( !EXPORT_RHS || (int)dependency(run4) != 0 ) {
							if( run1 == run4 ) {
								loop.addStatement( rk_rhsTemp.getCol( run4 ) == 1 );
							}
							else {
								loop.addStatement( rk_rhsTemp.getCol( run4 ) == 0 );
							}
							loop.addStatement( rk_rhsTemp.getCol( run4 ) += rk_outH*rk_sol.getCol( run4 ) );
							loop.addStatement( rk_xxx.getCol( run4 ) == rk_xPrev.getCol( run4 ) + rk_outH*rk_kkk.getCol( run4 ) );
						}
					}
					for( run4 = 0; run4 < NXA; run4++ ) { // getDependencyPattern doesn't work for algebraic states:
						loop.addStatement( rk_rhsTemp.getCol( NX+run4 ) == rk_outH*rk_sol.getCol( NX+run4 ) );
						loop.addStatement( rk_xxx.getCol( NX+run4 ) == rk_xPrev.getCol( NX+run4 ) + rk_outH*rk_kkk.getCol( NX+run4 ) );
					}
					
					loop.addFunctionCall( getNameDiffsOUTPUT( run2 ), rk_xxx, rk_diffsOutputTemp.getAddress(0,0) );
					uint numOutputs = getDimOUTPUT( run2 );
					uint outputDim = outputGrids[run2].getNumIntervals( )*numOutputs*(NX+NU+1);
					for( run4 = 0; run4 < numOutputs; run4++ ) {
						if( EXPORT_RHS ) dependency = outputExpressions[run2].getRow(run4).getDependencyPattern( x );
						loop.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+" << String( run3*numOutputs*(NX+NU+1)+numOutputs+run4*NX+run1 ) << "] = 0.0;\n" );
						for( run5 = 0; run5 < NX; run5++ ) {	
							if( !EXPORT_RHS || (int)dependency(run5) != 0 ) {
								tempString = rk_diffsOutputTemp.get( 0,run4*NX+run5 );
								loop.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+" << String( run3*numOutputs*(NX+NU+1)+numOutputs+run4*NX+run1 ) << "] += " << rk_rhsTemp.get( 0,run5 ) << "*" << tempString << ";\n" );
							}
						}
						for( run5 = 0; run5 < NXA; run5++ ) {	
							tempString = rk_diffsOutputTemp.get( 0,numOutputs*NX+run4*NXA+run5 );
							loop.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+" << String( run3*numOutputs*(NX+NU+1)+numOutputs+run4*NX+run1 ) << "] += " << rk_rhsTemp.get( 0,NX+run5 ) << "*" << tempString << ";\n" );
						}
					}
				}
			}
			else {
				loop2 = ExportForLoop( i,0,outputGrids[run2].getNumIntervals()-1 );
				evaluatePolynomial( loop2, rk_outH, gridVariables[run2], i, h );
				Vector dependency;
				if( EXPORT_RHS ) dependency = sumRow( outputExpressions[run2].getDependencyPattern( x ) );
				for( run4 = 0; run4 < NX; run4++ ) {
					if( !EXPORT_RHS || (int)dependency(run4) != 0 ) {
						if( run1 == run4 ) {
							loop2.addStatement( rk_rhsTemp.getCol( run4 ) == 1 );
						}
						else {
							loop2.addStatement( rk_rhsTemp.getCol( run4 ) == 0 );
						}
						loop2.addStatement( rk_rhsTemp.getCol( run4 ) += rk_outH*rk_sol.getCol( run4 ) );
						loop2.addStatement( rk_xxx.getCol( run4 ) == rk_xPrev.getCol( run4 ) + rk_outH*rk_kkk.getCol( run4 ) );
					}
				}
				for( run4 = 0; run4 < NXA; run4++ ) { // getDependencyPattern doesn't work for algebraic states:
					loop2.addStatement( rk_rhsTemp.getCol( NX+run4 ) == rk_outH*rk_sol.getCol( NX+run4 ) );
					loop2.addStatement( rk_xxx.getCol( NX+run4 ) == rk_xPrev.getCol( NX+run4 ) + rk_outH*rk_kkk.getCol( NX+run4 ) );
				}
				
				loop2.addFunctionCall( getNameDiffsOUTPUT( run2 ), rk_xxx, rk_diffsOutputTemp.getAddress(0,0) );
				uint numOutputs = getDimOUTPUT( run2 );
				uint outputDim = outputGrids[run2].getNumIntervals( )*numOutputs*(NX+NU+1);
				for( run4 = 0; run4 < numOutputs; run4++ ) {
					if( EXPORT_RHS ) dependency = outputExpressions[run2].getRow(run4).getDependencyPattern( x );
					loop2.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+i*" << String( numOutputs*(NX+NU+1) ) << "+" << numOutputs+run4*NX+run1<< "] = 0.0;\n" );
					for( run5 = 0; run5 < NX; run5++ ) {	
						if( !EXPORT_RHS || (int)dependency(run5) != 0 ) {
							tempString = rk_diffsOutputTemp.get( 0,run4*NX+run5 );
							loop2.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+i*" << String( numOutputs*(NX+NU+1) ) << "+" << numOutputs+run4*NX+run1 << "] += " << rk_rhsTemp.get( 0,run5 ) << "*" << tempString << ";\n" );
						}
					}
					for( run5 = 0; run5 < NXA; run5++ ) {	
						tempString = rk_diffsOutputTemp.get( 0,numOutputs*NX+run4*NXA+run5 );
						loop2.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+i*" << String( numOutputs*(NX+NU+1) ) << "+" << numOutputs+run4*NX+run1 << "] += " << rk_rhsTemp.get( 0,NX+run5 ) << "*" << tempString << ";\n" );
					}
				}
				loop.addStatement( loop2 );
			}
		}
		
		// update rk_eta with the new sensitivities:
		if( grid.getNumIntervals() == 1 && hasEquidistantGrid() ) {
			for( run2 = 0; run2 < NX; run2++ ) {
				if( run1 == run2 ) {
					loop.addStatement( rk_eta.getCol( NX+NXA+run2*NX+run1 ) == 1 );
				}
				else {
					loop.addStatement( rk_eta.getCol( NX+NXA+run2*NX+run1 ) == 0 );
				}
				for (run3 = 0; run3 < numStages; run3++ ) {
					loop.addStatement( rk_eta.getCol( NX+NXA+run2*NX+run1 ) += Bh.getCol( run3 )*rk_sol.getSubMatrix( run3,run3+1,run2,run2+1 ) );
				}
			}
			for( run2 = 0; run2 < NXA; run2++ ) {
				loop.addStatement( rk_alg_b.getCol( run2 ) == zeroM.getCols( 0,1 ) - rk_alg_diffsTemp.getCol( run2*NX ) * rk_eta.getCol( NX+NXA+run1 ) );
				for( run3 = 1; run3 < NX; run3++ ) {
					loop.addStatement( rk_alg_b.getCol( run2 ) -= rk_alg_diffsTemp.getCol( run2*NX+run3 ) * rk_eta.getCol( NX+NXA+run3*NX+run1 ) );
				}
			}
			if( NXA > 0 && run1 == 0 ) { // factorization of the new matrix rk_alg_A not yet calculated!
				loop.addFunctionCall( daeSolver->getNameSolveFunction(),rk_alg_A.getAddress(0,0),rk_alg_b.getAddress(0,0),rk_sol.getAddress(0,0) );
			} else if( NXA > 0 ) {
				loop.addFunctionCall( daeSolver->getNameSolveReuseFunction(),rk_alg_A.getAddress(0,0),rk_alg_b.getAddress(0,0),rk_sol.getAddress(0,0) );
			}
			for( run2 = 0; run2 < NXA; run2++ ) {
				loop.addStatement( rk_eta.getCol( NX+NXA+NX*NX+run2*NX+run1 ) == rk_sol.getSubMatrix( 0,1,run2,run2+1 ) );
			}
			
			// generate sensitivities wrt states for continuous output (only the last grid point):
			if( CONTINUOUS_OUTPUT ) loop.addStatement( rk_xxx.getCols( 0,NX+NXA ) == rk_eta.getCols( 0,NX+NXA ) );
			for( run2 = 0; run2 < rk_outputs.size(); run2++ ) {
				run3 = outputGrids[run2].getNumIntervals()-1;
				loop.addFunctionCall( getNameDiffsOUTPUT( run2 ), rk_xxx, rk_diffsOutputTemp.getAddress(0,0) );
				uint numOutputs = getDimOUTPUT( run2 );
				uint outputDim = outputGrids[run2].getNumIntervals( )*numOutputs*(NX+NU+1);
				for( run4 = 0; run4 < numOutputs; run4++ ) {
					Vector dependency;
					if( EXPORT_RHS ) dependency = outputExpressions[run2].getRow(run4).getDependencyPattern( x );
					loop.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+" << String( run3*numOutputs*(NX+NU+1)+numOutputs+run4*NX+run1 ) << "] = 0.0;\n" );
					for( run5 = 0; run5 < NX; run5++ ) {	
						if( !EXPORT_RHS || (int)dependency(run5) != 0 ) {
							tempString = rk_diffsOutputTemp.get( 0,run4*NX+run5 );
							loop.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+" << String( run3*numOutputs*(NX+NU+1)+numOutputs+run4*NX+run1 ) << "] += " << rk_eta.get( 0,NX+NXA+run5*NX+run1 ) << "*" << tempString << ";\n" );
						}
					}
					for( run5 = 0; run5 < NXA; run5++ ) {	
						tempString = rk_diffsOutputTemp.get( 0,numOutputs*NX+run4*NXA+run5 );
						loop.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+" << String( run3*numOutputs*(NX+NU+1)+numOutputs+run4*NX+run1 ) << "] += " << rk_eta.get( 0,NX+NXA+NX*NX+run5*NX+run1 ) << "*" << tempString << ";\n" );
					}
				}
			}
		}
		else { // update immediately rk_diffsNew:
			for( run2 = 0; run2 < NX; run2++ ) {
				if( run1 == run2 ) {
					loop.addStatement( rk_diffsNew.getSubMatrix( run2,run2+1,run1,run1+1 ) == 1 );
				}
				else {
					loop.addStatement( rk_diffsNew.getSubMatrix( run2,run2+1,run1,run1+1 ) == 0 );
				}
				for (run3 = 0; run3 < numStages; run3++ ) {
					loop.addStatement( rk_diffsNew.getSubMatrix( run2,run2+1,run1,run1+1 ) += Bh.getCol( run3 )*rk_sol.getSubMatrix( run3,run3+1,run2,run2+1 ) );
				}
			}
			for( run2 = 0; run2 < NXA; run2++ ) {
				loop.addStatement( rk_alg_b.getCol( run2 ) == zeroM.getCols( 0,1 ) - rk_alg_diffsTemp.getCol( run2*NX ) * rk_diffsNew.getSubMatrix( 0,1,run1,run1+1 ) );
				for( run3 = 1; run3 < NX; run3++ ) {
					loop.addStatement( rk_alg_b.getCol( run2 ) -= rk_alg_diffsTemp.getCol( run2*NX+run3 ) * rk_diffsNew.getSubMatrix( run3,run3+1,run1,run1+1 ) );
				}
			}
			if( NXA > 0 && run1 == 0 ) { // factorization of the new matrix rk_alg_A not yet calculated!
				loop.addFunctionCall( daeSolver->getNameSolveFunction(),rk_alg_A.getAddress(0,0),rk_alg_b.getAddress(0,0),rk_sol.getAddress(0,0) );
			} else if( NXA > 0 ) {
				loop.addFunctionCall( daeSolver->getNameSolveReuseFunction(),rk_alg_A.getAddress(0,0),rk_alg_b.getAddress(0,0),rk_sol.getAddress(0,0) );
			}
			for( run2 = 0; run2 < NXA; run2++ ) {
				loop.addStatement( rk_diffsNew.getSubMatrix( NX+run2,NX+run2+1,run1,run1+1 ) == rk_sol.getSubMatrix( 0,1,run2,run2+1 ) );
			}
			
			// generate sensitivities wrt states for continuous output (only the last grid point):
			if( CONTINUOUS_OUTPUT ) loop.addStatement( rk_xxx.getCols( 0,NX+NXA ) == rk_eta.getCols( 0,NX+NXA ) );
			for( run2 = 0; run2 < rk_outputs.size(); run2++ ) {
				run3 = outputGrids[run2].getNumIntervals()-1;
				loop.addFunctionCall( getNameDiffsOUTPUT( run2 ), rk_xxx, rk_diffsOutputTemp.getAddress(0,0) );
				uint numOutputs = getDimOUTPUT( run2 );
				uint outputDim = outputGrids[run2].getNumIntervals( )*numOutputs*(NX+NU+1);
				for( run4 = 0; run4 < numOutputs; run4++ ) {
					Vector dependency;
					if( EXPORT_RHS ) dependency = outputExpressions[run2].getRow(run4).getDependencyPattern( x );
					loop.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+" << String( run3*numOutputs*(NX+NU+1)+numOutputs+run4*NX+run1 ) << "] = 0.0;\n" );
					for( run5 = 0; run5 < NX; run5++ ) {	
						if( !EXPORT_RHS || (int)dependency(run5) != 0 ) {
							tempString = rk_diffsOutputTemp.get( 0,run4*NX+run5 );
							loop.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+" << String( run3*numOutputs*(NX+NU+1)+numOutputs+run4*NX+run1 ) << "] += " << rk_diffsNew.get( run5,run1 ) << "*" << tempString << ";\n" );
						}
					}
					for( run5 = 0; run5 < NXA; run5++ ) {	
						tempString = rk_diffsOutputTemp.get( 0,numOutputs*NX+run4*NXA+run5 );
						loop.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+" << String( run3*numOutputs*(NX+NU+1)+numOutputs+run4*NX+run1 ) << "] += " << rk_diffsNew.get( NX+run5,run1 ) << "*" << tempString << ";\n" );
					}
				}
			}
		}
	}
	
	// derivatives wrt the control inputs (IFT):
	for( run1 = 0; run1 < NU; run1++ ) {
		for( run2 = 0; run2 < numStages; run2++ ) {
			for( run3 = 0; run3 < NX; run3++ ) {
				if( NDX == 0 ) {
					loop.addStatement( rk_b.getCol( run2*(NX+NXA)+run3 ) == rk_diffsTemp.getSubMatrix( run2,run2+1,NX*(NX+NXA)+run3*NU+run1,NX*(NX+NXA)+run3*NU+run1+1 ) );
				}
				else {
					loop.addStatement( rk_b.getCol( run2*(NX+NXA)+run3 ) == zeroM.getCols( 0,1 ) - rk_diffsTemp.getSubMatrix( run2,run2+1,NX*(NX+NXA)+run3*NU+run1,NX*(NX+NXA)+run3*NU+run1+1 ) );
				}
			}
			for( run3 = 0; run3 < NXA; run3++ ) {
				loop.addStatement( rk_b.getCol( run2*(NX+NXA)+NX+run3 ) == zeroM.getCols( 0,1 ) - rk_diffsTemp.getSubMatrix( run2,run2+1,NX*(NX+NXA+NU+NDX)+NXA*(NX+NXA)+run3*NU+run1,NX*(NX+NXA+NU+NDX)+NXA*(NX+NXA)+run3*NU+run1+1 ) );
			}
		}
		loop.addFunctionCall( solver->getNameSolveReuseFunction(),rk_A.getAddress(0,0),rk_b.getAddress(0,0),rk_sol.getAddress(0,0) );
		
		// generate sensitivities wrt controls for continuous output (except for the last grid point):
		for( run2 = 0; run2 < rk_outputs.size(); run2++ ) {
			if( UNROLL_OUTPUT ) {
				for( run3 = 0; run3 < outputGrids[run2].getNumIntervals()-1; run3++ ) {
					Vector out = evaluatePolynomial( outputGrids[run2].getTime( run3+1 ) );
					rk_outH = ExportVariable( "out*h", Matrix( out, BT_TRUE )*=h );
					Vector dependency;
					if( EXPORT_RHS ) dependency = sumRow( outputExpressions[run2].getDependencyPattern( x ) );
					for( run4 = 0; run4 < NX; run4++ ) {
						if( !EXPORT_RHS || (int)dependency(run4) != 0 ) {
							loop.addStatement( rk_rhsTemp.getCol( run4 ) == rk_outH*rk_sol.getCol( run4 ) );
							loop.addStatement( rk_xxx.getCol( run4 ) == rk_xPrev.getCol( run4 ) + rk_outH*rk_kkk.getCol( run4 ) );
						}
					}
					for( run4 = 0; run4 < NXA; run4++ ) { // getDependencyPattern doesn't work for algebraic states:
						loop.addStatement( rk_rhsTemp.getCol( NX+run4 ) == rk_outH*rk_sol.getCol( NX+run4 ) );
						loop.addStatement( rk_xxx.getCol( NX+run4 ) == rk_xPrev.getCol( NX+run4 ) + rk_outH*rk_kkk.getCol( NX+run4 ) );
					}
					
					loop.addFunctionCall( getNameDiffsOUTPUT( run2 ), rk_xxx, rk_diffsOutputTemp.getAddress(0,0) );
					uint numOutputs = getDimOUTPUT( run2 );
					uint outputDim = outputGrids[run2].getNumIntervals( )*numOutputs*(NX+NU+1);
					for( run4 = 0; run4 < numOutputs; run4++ ) {
						if( EXPORT_RHS ) dependency = outputExpressions[run2].getRow(run4).getDependencyPattern( x );
						loop.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+" << String( run3*numOutputs*(NX+NU+1)+numOutputs*(1+NX)+run4*NU+run1 ) << "] = " <<  rk_diffsOutputTemp.get( 0,numOutputs*(NX+NXA)+run4*NU+run1 ) << ";\n" );
						for( run5 = 0; run5 < NX; run5++ ) {	
							if( !EXPORT_RHS || (int)dependency(run5) != 0 ) {
								tempString = rk_diffsOutputTemp.get( 0,run4*NX+run5 );
								loop.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+" << String( run3*numOutputs*(NX+NU+1)+numOutputs*(1+NX)+run4*NU+run1 ) << "] += " << rk_rhsTemp.get( 0,run5 ) << "*" << tempString << ";\n" );
							}
						}
						for( run5 = 0; run5 < NXA; run5++ ) {	
							tempString = rk_diffsOutputTemp.get( 0,numOutputs*NX+run4*NXA+run5 );
							loop.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+" << String( run3*numOutputs*(NX+NU+1)+numOutputs*(1+NX)+run4*NU+run1 ) << "] += " << rk_rhsTemp.get( 0,NX+run5 ) << "*" << tempString << ";\n" );
						}
					}
				}
			}
			else {
				loop2 = ExportForLoop( i,0,outputGrids[run2].getNumIntervals()-1 );
				evaluatePolynomial( loop2, rk_outH, gridVariables[run2], i, h );
				Vector dependency;
				if( EXPORT_RHS ) dependency = sumRow( outputExpressions[run2].getDependencyPattern( x ) );
				for( run4 = 0; run4 < NX; run4++ ) {
					if( !EXPORT_RHS || (int)dependency(run4) != 0 ) {
						loop2.addStatement( rk_rhsTemp.getCol( run4 ) == rk_outH*rk_sol.getCol( run4 ) );
						loop2.addStatement( rk_xxx.getCol( run4 ) == rk_xPrev.getCol( run4 ) + rk_outH*rk_kkk.getCol( run4 ) );
					}
				}
				for( run4 = 0; run4 < NXA; run4++ ) { // getDependencyPattern doesn't work for algebraic states:
					loop2.addStatement( rk_rhsTemp.getCol( NX+run4 ) == rk_outH*rk_sol.getCol( NX+run4 ) );
					loop2.addStatement( rk_xxx.getCol( NX+run4 ) == rk_xPrev.getCol( NX+run4 ) + rk_outH*rk_kkk.getCol( NX+run4 ) );
				}
				
				loop2.addFunctionCall( getNameDiffsOUTPUT( run2 ), rk_xxx, rk_diffsOutputTemp.getAddress(0,0) );
				uint numOutputs = getDimOUTPUT( run2 );
				uint outputDim = outputGrids[run2].getNumIntervals( )*numOutputs*(NX+NU+1);
				for( run4 = 0; run4 < numOutputs; run4++ ) {
					if( EXPORT_RHS ) dependency = outputExpressions[run2].getRow(run4).getDependencyPattern( x );
					loop2.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+i*" << String( numOutputs*(NX+NU+1) ) << "+" << numOutputs*(1+NX)+run4*NU+run1 << "] = " <<  rk_diffsOutputTemp.get( 0,numOutputs*(NX+NXA)+run4*NU+run1 ) << ";\n" );
					for( run5 = 0; run5 < NX; run5++ ) {	
						if( !EXPORT_RHS || (int)dependency(run5) != 0 ) {
							tempString = rk_diffsOutputTemp.get( 0,run4*NX+run5 );
							loop2.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+i*" << String( numOutputs*(NX+NU+1) ) << "+" << numOutputs*(1+NX)+run4*NU+run1 << "] += " << rk_rhsTemp.get( 0,run5 ) << "*" << tempString << ";\n" );
						}
					}
					for( run5 = 0; run5 < NXA; run5++ ) {	
						tempString = rk_diffsOutputTemp.get( 0,numOutputs*NX+run4*NXA+run5 );
						loop2.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+i*" << String( numOutputs*(NX+NU+1) ) << "+" << numOutputs*(1+NX)+run4*NU+run1 << "] += " << rk_rhsTemp.get( 0,NX+run5 ) << "*" << tempString << ";\n" );
					}
				}
				loop.addStatement( loop2 );
			}
		}
		
		// update rk_eta with the new sensitivities:
		if( grid.getNumIntervals() == 1 && hasEquidistantGrid() ) {
			for( run2 = 0; run2 < NX; run2++ ) {
				loop.addStatement( rk_eta.getCol( (NX+NXA)*(NX+1)+run2*NU+run1 ) == 0 );
				for (run3 = 0; run3 < numStages; run3++ ) {
					loop.addStatement( rk_eta.getCol( (NX+NXA)*(NX+1)+run2*NU+run1 ) += Bh.getCol( run3 )*rk_sol.getSubMatrix( run3,run3+1,run2,run2+1 ) );
				}
			}
			for( run2 = 0; run2 < NXA; run2++ ) {
				loop.addStatement( rk_alg_b.getCol( run2 ) == zeroM.getCols( 0,1 ) - rk_alg_diffsTemp.getCol( run2*NX ) * rk_eta.getCol( (NX+NXA)*(NX+1)+run1 ) );
				for( run3 = 1; run3 < NX; run3++ ) {
					loop.addStatement( rk_alg_b.getCol( run2 ) -= rk_alg_diffsTemp.getCol( run2*NX+run3 ) * rk_eta.getCol( (NX+NXA)*(NX+1)+run3*NU+run1 ) );
				}
				loop.addStatement( rk_alg_b.getCol( run2 ) -= rk_alg_diffsTemp.getCol( NXA*(NX+NXA)+run2*NU+run1 ) );
			}
			if( NXA > 0 ) {
				loop.addFunctionCall( daeSolver->getNameSolveReuseFunction(),rk_alg_A.getAddress(0,0),rk_alg_b.getAddress(0,0),rk_sol.getAddress(0,0) );
			}
			
			for( run2 = 0; run2 < NXA; run2++ ) {
				loop.addStatement( rk_eta.getCol( (NX+NXA)*(NX+1)+NX*NU+run2*NU+run1 ) == rk_sol.getSubMatrix( 0,1,run2,run2+1 ) );
			}
			
			// generate sensitivities wrt controls for continuous output (only the last grid point):
			if( CONTINUOUS_OUTPUT ) loop.addStatement( rk_xxx.getCols( 0,NX+NXA ) == rk_eta.getCols( 0,NX+NXA ) );
			for( run2 = 0; run2 < rk_outputs.size(); run2++ ) {
				run3 = outputGrids[run2].getNumIntervals()-1;
				loop.addFunctionCall( getNameDiffsOUTPUT( run2 ), rk_xxx, rk_diffsOutputTemp.getAddress(0,0) );
				uint numOutputs = getDimOUTPUT( run2 );
				uint outputDim = outputGrids[run2].getNumIntervals( )*numOutputs*(NX+NU+1);
				for( run4 = 0; run4 < numOutputs; run4++ ) {
					Vector dependency;
					if( EXPORT_RHS ) dependency = outputExpressions[run2].getRow(run4).getDependencyPattern( x );
					loop.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+" << String( run3*numOutputs*(NX+NU+1)+numOutputs*(1+NX)+run4*NU+run1 ) << "] = " <<  rk_diffsOutputTemp.get( 0,numOutputs*(NX+NXA)+run4*NU+run1 ) << ";\n" );
					for( run5 = 0; run5 < NX; run5++ ) {	
						if( !EXPORT_RHS || (int)dependency(run5) != 0 ) {
							tempString = rk_diffsOutputTemp.get( 0,run4*NX+run5 );
							loop.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+" << String( run3*numOutputs*(NX+NU+1)+numOutputs*(1+NX)+run4*NU+run1 ) << "] += " << rk_eta.get( 0,(NX+NXA)*(NX+1)+run5*NU+run1 ) << "*" << tempString << ";\n" );
						}
					}
					for( run5 = 0; run5 < NXA; run5++ ) {	
						tempString = rk_diffsOutputTemp.get( 0,numOutputs*NX+run4*NXA+run5 );
						loop.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+" << String( run3*numOutputs*(NX+NU+1)+numOutputs*(1+NX)+run4*NU+run1 ) << "] += " << rk_eta.get( 0,(NX+NXA)*(NX+1)+NX*NU+run5*NU+run1 ) << "*" << tempString << ";\n" );
					}
				}
			}
		}
		else { // update immediately rk_diffsNew:
			for( run2 = 0; run2 < NX; run2++ ) {
				loop.addStatement( rk_diffsNew.getSubMatrix( run2,run2+1,NX+run1,NX+run1+1 ) == 0 );
				for (run3 = 0; run3 < numStages; run3++ ) {
					loop.addStatement( rk_diffsNew.getSubMatrix( run2,run2+1,NX+run1,NX+run1+1 ) += Bh.getCol( run3 )*rk_sol.getSubMatrix( run3,run3+1,run2,run2+1 ) );
				}
			}
			for( run2 = 0; run2 < NXA; run2++ ) {
				loop.addStatement( rk_alg_b.getCol( run2 ) == zeroM.getCols( 0,1 ) - rk_alg_diffsTemp.getCol( run2*NX ) * rk_diffsNew.getSubMatrix( 0,1,NX+run1,NX+run1+1 ) );
				for( run3 = 1; run3 < NX; run3++ ) {
					loop.addStatement( rk_alg_b.getCol( run2 ) -= rk_alg_diffsTemp.getCol( run2*NX+run3 ) * rk_diffsNew.getSubMatrix( run3,run3+1,NX+run1,NX+run1+1 ) );
				}
				loop.addStatement( rk_alg_b.getCol( run2 ) -= rk_alg_diffsTemp.getCol( NXA*(NX+NXA)+run2*NU+run1 ) );
			}
			if( NXA > 0 ) {
				loop.addFunctionCall( daeSolver->getNameSolveReuseFunction(),rk_alg_A.getAddress(0,0),rk_alg_b.getAddress(0,0),rk_sol.getAddress(0,0) );
			}
			
			for( run2 = 0; run2 < NXA; run2++ ) {
				loop.addStatement( rk_diffsNew.getSubMatrix( NX+run2,NX+run2+1,NX+run1,NX+run1+1 ) == rk_sol.getSubMatrix( 0,1,run2,run2+1 ) );
			}
			
			// generate sensitivities wrt controls for continuous output (only the last grid point):
			if( CONTINUOUS_OUTPUT ) loop.addStatement( rk_xxx.getCols( 0,NX+NXA ) == rk_eta.getCols( 0,NX+NXA ) );
			for( run2 = 0; run2 < rk_outputs.size(); run2++ ) {
				run3 = outputGrids[run2].getNumIntervals()-1;
				loop.addFunctionCall( getNameDiffsOUTPUT( run2 ), rk_xxx, rk_diffsOutputTemp.getAddress(0,0) );
				uint numOutputs = getDimOUTPUT( run2 );
				uint outputDim = outputGrids[run2].getNumIntervals( )*numOutputs*(NX+NU+1);
				for( run4 = 0; run4 < numOutputs; run4++ ) {
					Vector dependency;
					if( EXPORT_RHS ) dependency = outputExpressions[run2].getRow(run4).getDependencyPattern( x );
					loop.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+" << String( run3*numOutputs*(NX+NU+1)+numOutputs*(1+NX)+run4*NU+run1 ) << "] = " <<  rk_diffsOutputTemp.get( 0,numOutputs*(NX+NXA)+run4*NU+run1 ) << ";\n" );
					for( run5 = 0; run5 < NX; run5++ ) {	
						if( !EXPORT_RHS || (int)dependency(run5) != 0 ) {
							tempString = rk_diffsOutputTemp.get( 0,run4*NX+run5 );
							loop.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+" << String( run3*numOutputs*(NX+NU+1)+numOutputs*(1+NX)+run4*NU+run1 ) << "] += " << rk_diffsNew.get( run5,NX+run1 ) << "*" << tempString << ";\n" );
						}
					}
					for( run5 = 0; run5 < NXA; run5++ ) {	
						tempString = rk_diffsOutputTemp.get( 0,numOutputs*NX+run4*NXA+run5 );
						loop.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+" << String( run3*numOutputs*(NX+NU+1)+numOutputs*(1+NX)+run4*NU+run1 ) << "] += " << rk_diffsNew.get( NX+run5,NX+run1 ) << "*" << tempString << ";\n" );
					}
				}
			}
		}
	}
	
	
	if( grid.getNumIntervals() > 1 || !hasEquidistantGrid() ) {
		if( hasEquidistantGrid() ) loop.addStatement( String( "if( run > 0 ) {\n" ) );
		// calculation of the sensitivities using the chain rule:
		
		ExportForLoop loop01( i,0,NX+NXA );
		ExportForLoop loop02( j,0,NX );
		loop02.addStatement( String( "rk_eta[" ) << String( NX+NXA ) << "+i*" << String( NX ) << "+j] = acadoWorkspace.rk_diffsNew[i*" << String( NX+NU ) << "]*acadoWorkspace.rk_diffsPrev[j];\n" );
		for( run1 = 1; run1 < NX; run1++ ) {
			loop02.addStatement( String( "rk_eta[" ) << String( NX+NXA ) << "+i*" << String( NX ) << "+j] += acadoWorkspace.rk_diffsNew[i*" << String( NX+NU ) << "+" << String( run1 ) << "]*acadoWorkspace.rk_diffsPrev[" << String( run1*(NX+NU) ) << "+j];\n" );
		}
		loop01.addStatement( loop02 );
		loop.addStatement( loop01 );
		
		ExportForLoop loop03( i,0,NX+NXA );
		ExportForLoop loop04( j,0,NU );
		loop04.addStatement( String( "rk_eta[" ) << String( (NX+NXA)*(1+NX) ) << "+i*" << String( NU ) << "+j] = acadoWorkspace.rk_diffsNew[i*" << String( NX+NU ) << "+" << String( NX ) << "+j] + acadoWorkspace.rk_diffsNew[i*" << String( NX+NU ) << "]*acadoWorkspace.rk_diffsPrev[" << String( NX ) << "+j];\n" );
		for( run1 = 1; run1 < NX; run1++ ) {
			loop04.addStatement( String( "rk_eta[" ) << String( (NX+NXA)*(1+NX) ) << "+i*" << String( NU ) << "+j] += acadoWorkspace.rk_diffsNew[i*" << String( NX+NU ) << "+" << String( run1 ) << "]*acadoWorkspace.rk_diffsPrev[" << String( run1*(NX+NU) ) << "+" << String( NX ) << "+j];\n" );
		}
		loop03.addStatement( loop04 );
		loop.addStatement( loop03 );
		
		// chain rule for the sensitivities of the continuous output:
		for( run1 = 0; run1 < rk_outputs.size(); run1++ ) {
			ExportForLoop loop05( k,0,outputGrids[run1].getNumIntervals() );
			uint numOutputs = getDimOUTPUT( run1 );
			uint outputDim = outputGrids[run1].getNumIntervals( )*numOutputs*(NX+NU+1);
			for( run2 = 0; run2 < numOutputs*(NX+NU); run2++ ) {
				loop05.addStatement( String( rk_diffsOutputTemp.get( 0,run2 ) ) << " = " << rk_outputs[run1].getName() << "[run*" << String( outputDim ) << "+(k*" << String( numOutputs*(NX+NU+1) ) << ")+" << String( numOutputs+run2 ) << "];\n" );
			}
			
			ExportForLoop loop06( i,0,numOutputs );
			ExportForLoop loop07( j,0,NX );
			loop07.addStatement( String( rk_outputs[run1].getName() ) << "[run*" << String( outputDim ) << "+(k*" << String( numOutputs*(NX+NU+1) ) << ")+" << String( numOutputs ) << "+i*" << String( NX ) << "+j] = acadoWorkspace.rk_diffsOutputTemp[i*" << String( NX ) << "]*acadoWorkspace.rk_diffsPrev[j];\n" );
			for( run2 = 1; run2 < NX; run2++ ) {
				loop07.addStatement( String( rk_outputs[run1].getName() ) << "[run*" << String( outputDim ) << "+(k*" << String( numOutputs*(NX+NU+1) ) << ")+" << String( numOutputs ) << "+i*" << String( NX ) << "+j] += acadoWorkspace.rk_diffsOutputTemp[i*" << String( NX ) << "+" << String( run2 ) << "]*acadoWorkspace.rk_diffsPrev[" << String( run2*(NX+NU) ) << "+j];\n" );
			}
			loop06.addStatement( loop07 );
			loop05.addStatement( loop06 );
			
			ExportForLoop loop08( i,0,numOutputs );
			ExportForLoop loop09( j,0,NU );
			loop09.addStatement( String( rk_outputs[run1].getName() ) << "[run*" << String( outputDim ) << "+(k*" << String( numOutputs*(NX+NU+1) ) << ")+" << String( numOutputs*(1+NX) ) << "+i*" << String( NU ) << "+j] = acadoWorkspace.rk_diffsOutputTemp[" << String( numOutputs*NX ) << "+i*" << String( NU ) << "+j] + acadoWorkspace.rk_diffsOutputTemp[i*" << String( NX ) << "]*acadoWorkspace.rk_diffsPrev[" << String( NX ) << "+j];\n" );
			for( run2 = 1; run2 < NX; run2++ ) {
				loop09.addStatement( String( rk_outputs[run1].getName() ) << "[run*" << String( outputDim ) << "+(k*" << String( numOutputs*(NX+NU+1) ) << ")+" << String( numOutputs*(1+NX) ) << "+i*" << String( NU ) << "+j] += acadoWorkspace.rk_diffsOutputTemp[i*" << String( NX ) << "+" << String( run2 ) << "]*acadoWorkspace.rk_diffsPrev[" << String( run2*(NX+NU)+NX ) << "+j];\n" );
			}
			loop08.addStatement( loop09 );
			loop05.addStatement( loop08 );
			loop.addStatement( loop05 );
		}
		
		if( hasEquidistantGrid() ) {
			loop.addStatement( String( "}\n" ) );
			loop.addStatement( String( "else {\n" ) );
			loop.addStatement( rk_diffsPrev == rk_diffsNew );
			loop.addStatement( String( "}\n" ) );
		}
	}
	
	loop.addStatement( rk_ttt += Matrix(h) );
	
	// Initialization for the NEXT step:
	loop.addStatement( rk_sol == rk_kkk );
	loop.addStatement( rk_kkk == DM*rk_sol );
	
	loop.addStatement( String( rk_num.get(0,0) ) << " += 1;\n" );
	
    // end of the integrator loop.
    if( !hasEquidistantGrid() ) {
		loop.addStatement( "}\n" );
		loop.unrollLoop();
	}
	integrate.addStatement( loop );
	code.addFunction( integrate );
	
    code.addLinebreak( 2 );
	
	solver->getCode( code );
	if( NXA > 0 ) {
		daeSolver->getCode( code );
		addDAEFunctions( code );
	}

    return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::addDAEFunctions( ExportStatementBlock& code )
{
	if( NXA > 0 ) {
		
	Matrix nul = zeros( 1,NX+NXA );
	ExportVariable zeroM( "nul", nul );
	uint run1;
	ExportIndex i( "i" );
	
	// The function makeStatesConsistent:
	
	// Reset the number of the integration step:
	makeStatesConsistent.addStatement( String( rk_num.get(0,0) ) << " = 0;\n" );
	
	makeStatesConsistent.addStatement( rk_xxx.getCols( 0,NX+NXA ) == rk_eta.getCols( 0,NX+NXA ) );
	makeStatesConsistent.addStatement( rk_xxx.getCols( NX+NXA,inputDim-diffsDim ) == rk_eta.getCols( NX+NXA+diffsDim,inputDim ) );
	
	// matrix rk_alg_A (first step):
	makeStatesConsistent.addFunctionCall( getNameDiffsDAE(), rk_xxx, rk_alg_diffsTemp.getAddress(0,0) );
	for( run1 = 0; run1 < NXA; run1++ ) {
		makeStatesConsistent.addStatement( rk_alg_A.getRow( run1 ) == rk_alg_diffsTemp.getCols( NXA*NX+run1*NXA,NXA*NX+run1*NXA+NXA ) );
	}
	// update rk_alg_b:
	makeStatesConsistent.addFunctionCall( getNameDAE(), rk_xxx, rk_rhsTemp.getAddress(0,NX) );
	makeStatesConsistent.addStatement( rk_alg_b == zeroM.getCols( 0,NXA ) - rk_rhsTemp.getCols( NX,NX+NXA ) );
	makeStatesConsistent.addFunctionCall( daeSolver->getNameSolveFunction(),rk_alg_A.getAddress(0,0),rk_alg_b.getAddress(0,0),rk_sol.getAddress(0,0) );
	makeStatesConsistent.addStatement( rk_xxx.getCols( NX,NX+NXA ) += rk_sol.getSubMatrix( 0,1,0,NXA ) );
	
	
	ExportForLoop loop0( i,0,numAlgItsInit-1 );
	// update rk_alg_b:
	loop0.addFunctionCall( getNameDAE(), rk_xxx, rk_rhsTemp.getAddress(0,NX) );
	loop0.addStatement( rk_alg_b == zeroM.getCols( 0,NXA ) - rk_rhsTemp.getCols( NX,NX+NXA ) );
	loop0.addFunctionCall( daeSolver->getNameSolveReuseFunction(),rk_alg_A.getAddress(0,0),rk_alg_b.getAddress(0,0),rk_sol.getAddress(0,0) );
	loop0.addStatement( rk_xxx.getCols( NX,NX+NXA ) += rk_sol.getSubMatrix( 0,1,0,NXA ) );
	makeStatesConsistent.addStatement( loop0 );
	
	makeStatesConsistent.addStatement( rk_eta.getCols( NX,NX+NXA ) == rk_xxx.getCols( NX,NX+NXA ) );
	
	// add function "makeStatesConsistent"
	code.addFunction( makeStatesConsistent );
	
	
	// The function getNormConsistency:
	getNormConsistency.addStatement( String( "real_t norm;\n" ) );
	getNormConsistency.addStatement( rk_xxx.getCols( 0,NX+NXA ) == rk_eta.getCols( 0,NX+NXA ) );
	getNormConsistency.addStatement( rk_xxx.getCols( NX+NXA,inputDim-diffsDim ) == rk_eta.getCols( NX+NXA+diffsDim,inputDim ) );
	
	getNormConsistency.addFunctionCall( getNameDAE(), rk_xxx, rk_rhsTemp.getAddress( 0,NX ) );
	getNormConsistency.addStatement( norm == rk_rhsTemp.getCol( NX )*rk_rhsTemp.getCol( NX ) );
	for( run1 = 1; run1 < NXA; run1++ ) {
		getNormConsistency.addStatement( norm += rk_rhsTemp.getCol( NX+run1 )*rk_rhsTemp.getCol( NX+run1 ) );
	}
	getNormConsistency.addStatement( String( "norm = sqrt( norm );\n" ) );
	
	// add function "getNormConsistency"
	code.addFunction( getNormConsistency );

	}
    return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::initializeDDMatrix( )
{
	uint i, j, k;
	DD = Matrix( numStages, numStages );
	
	for( i = 0; i < numStages; i++ ) {
		for( j = 0; j < numStages; j++ ) {
			DD( i, j ) = 1;
			for( k = 0; k < numStages; k++ ) {
				if( k != j ) {
					DD( i, j ) *= ((1+cc(i))-cc(k))/(cc(j)-cc(k));
				}
			}
		}
	}
    return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::initializeCoefficients( )
{
	uint i, j, k, index;
	double sum;
	Vector cVec( numStages-1 );
	Vector products;
	coeffs = Matrix( numStages, numStages );
	
	for( i = 0; i < numStages; i++ ) {
		for( j = 0; j < numStages; j++ ) {
			coeffs( i, j ) = 1/((double) numStages-i);
			index = 0;
			for( k = 0; k < numStages; k++ ) {
				if( k != j ) {
					coeffs( i, j ) *= 1/((double) cc(j)-cc(k));
					cVec(index) = cc(k);
					index++;
				}
			}
			
			if( i > 0 ) {
				products = computeCombinations( cVec, 0, i );
				sum = 0.0;
				for( k = 0; k < products.getDim(); k++ ) {
					sum += products(k);
				}
				if( i%2 == 0 ) {
					coeffs( i, j ) *= sum;
				}
				else {
					coeffs( i, j ) *= (-1.0*sum);
				}
			}
		}
	}
	
    return SUCCESSFUL_RETURN;
}


Vector ImplicitRungeKuttaExport::computeCombinations( const Vector& cVec, uint index, uint numEls ) {
	uint k, l;
	Vector products;
	
	if( numEls == 0 ) {
		products = Vector(1);
		products(0) = 1;
		return products;
	}
	products = Vector();
	for( k = index; k < cVec.getDim()-numEls+1; k++ ) {
		Vector temp = computeCombinations( cVec, k+1, numEls-1 );
		for( l = 0; l < temp.getDim(); l++ ) {
			temp(l) *= cVec(k);
		}
		products.append(temp);
	}
	
	return products;
}


Vector ImplicitRungeKuttaExport::evaluatePolynomial( double time )
{
	uint i, j;
	Vector coeffsPolyn( numStages );
	
	for( j = 0; j < numStages; j++ ) {
		coeffsPolyn( j ) = 0.0;
		for( i = 0; i < numStages; i++ ) {
			coeffsPolyn( j ) += pow( time, static_cast<int>(numStages-i) )*coeffs( i,j );
		}
	} 
	
    return coeffsPolyn;
}


returnValue ImplicitRungeKuttaExport::evaluatePolynomial( ExportStatementBlock& block, const ExportVariable& variable, const ExportVariable& gridVariable, const ExportIndex& indexTime, double h )
{
	uint i, j;
	
	for( j = 0; j < numStages; j++ ) {
		block.addStatement( (String)variable.getFullName() << "[" << j << "] = pow( " << gridVariable.getName() << "[" << indexTime.getName() << "], " << numStages << " )*" << coeffs( 0,j ) << ";\n" );
		for( i = 1; i < numStages; i++ ) {
			block.addStatement( (String)variable.getFullName() << "[" << j << "] += pow( " << gridVariable.getName() << "[" << indexTime.getName() << "], " << numStages-i << " )*" << coeffs( i,j ) << ";\n" );
		}
		block.addStatement( (String)variable.getFullName() << "[" << j << "] *= " << h << ";\n" );
	}

    return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::setup( )
{
	int intMode;
	userInteraction->get( IMPLICIT_INTEGRATOR_MODE,intMode ); 
	switch( (ImplicitIntegratorMode) intMode ) {
		case IFTR:
			REUSE = BT_TRUE;
			break;
		case IFT:
			REUSE = BT_FALSE;
			break;
		default:
			return ACADOERROR( RET_INVALID_OPTION );
	}
	
	int newNumIts;
	userInteraction->get( IMPLICIT_INTEGRATOR_NUM_ITS,newNumIts ); 
	if (newNumIts >= 0) {
		numIts = newNumIts;
	}
	
	int newNumItsInit;
	userInteraction->get( IMPLICIT_INTEGRATOR_NUM_ITS_INIT,newNumItsInit );
	if (newNumItsInit >= 0) {
		numItsInit = newNumItsInit;
	}
	
	int newNumAlgIts;
	userInteraction->get( IMPLICIT_INTEGRATOR_NUM_ALG_ITS,newNumAlgIts ); 
	if (newNumAlgIts >= 0) {
		numAlgIts = newNumAlgIts;
	}
	
	int newNumAlgItsInit;
	userInteraction->get( IMPLICIT_INTEGRATOR_NUM_ALG_ITS_INIT,newNumAlgItsInit );
	if (newNumAlgItsInit > 0) {
		numAlgItsInit = newNumAlgItsInit;
	}
	
	diffsDim = (NX+NXA)*(NX+NU);
	inputDim = (NX+NXA)*(NX+NU+1) + NU + NP;
	
	rk_num = ExportVariable( "rk_num", 1, 1, INT, ACADO_WORKSPACE, BT_TRUE );
	rk_ttt = ExportVariable( "rk_ttt", 1, 1, REAL, ACADO_WORKSPACE, BT_TRUE );
	rk_xxx = ExportVariable( "rk_xxx", 1, inputDim-diffsDim+NDX, REAL, ACADO_WORKSPACE );
	if( CONTINUOUS_OUTPUT ) rk_xPrev = ExportVariable( "rk_xPrev", 1, inputDim-diffsDim, REAL, ACADO_WORKSPACE );
	rk_kkk = ExportVariable( "rk_kkk", numStages, NX+NXA, REAL, ACADO_WORKSPACE );
	rk_sol = ExportVariable( "rk_sol", numStages, NX+NXA, REAL, ACADO_WORKSPACE );
	rk_A = ExportVariable( "rk_A", numStages*(NX+NXA), numStages*(NX+NXA), REAL, ACADO_WORKSPACE );
	rk_b = ExportVariable( "rk_b", 1, numStages*(NX+NXA), REAL, ACADO_WORKSPACE );
	rk_alg_A = ExportVariable( "rk_alg_A", NXA, NXA, REAL, ACADO_WORKSPACE );
	rk_alg_b = ExportVariable( "rk_alg_b", 1, NXA, REAL, ACADO_WORKSPACE );
	rk_rhsTemp = ExportVariable( "rk_rhsTemp", 1, NX+NXA, REAL, ACADO_WORKSPACE );
	rk_diffsTemp = ExportVariable( "rk_diffsTemp", numStages, (NX+NXA)*(NX+NXA+NU)+NX*NDX, REAL, ACADO_WORKSPACE );
	rk_alg_diffsTemp = ExportVariable( "rk_alg_diffsTemp", 1, NXA*(NX+NXA+NU), REAL, ACADO_WORKSPACE );
	rk_diffsPrev = ExportVariable( "rk_diffsPrev", NX+NXA, NX+NU, REAL, ACADO_WORKSPACE );
	rk_diffsNew = ExportVariable( "rk_diffsNew", NX+NXA, NX+NU, REAL, ACADO_WORKSPACE );
	rk_index = ExportVariable( "rk_index", 1, 1, INT, ACADO_LOCAL, BT_TRUE );
	rk_eta = ExportVariable( "rk_eta", 1, inputDim, REAL );
	if( hasEquidistantGrid() ) {
		integrate = ExportFunction( "integrate", rk_eta );
	}
	else {
		integrate = ExportFunction( "integrate", rk_index, rk_eta );
	}
	uint i;
	for( i = 0; i < rk_outputs.size(); i++ ) {
		integrate.addArgument( rk_outputs[i] );
	}
	integrate.addLinebreak( );	// TO MAKE SURE IT GETS EXPORTED
	
	// setup linear solver:
	int solverType;
	userInteraction->get( LINEAR_ALGEBRA_SOLVER,solverType );

	if ( solver )
		delete solver;
	solver = 0;

	if ( daeSolver )
		delete daeSolver;
	daeSolver = 0;

	switch( (LinearAlgebraSolver) solverType ) {
		case GAUSS_LU:
			solver = new ExportGaussElim( userInteraction,commonHeaderName );
			daeSolver = new ExportGaussElim( userInteraction,commonHeaderName );
			break;
		case HOUSEHOLDER_QR:
			solver = new ExportHouseholderQR( userInteraction,commonHeaderName );
			daeSolver = new ExportHouseholderQR( userInteraction,commonHeaderName );
			break;
		default:
			return ACADOERROR( RET_INVALID_OPTION );
	}
	solver->setReuse( BT_TRUE ); 	// IFTR method
	solver->init( (NX+NXA)*numStages );
	solver->setup();
	
	if( NXA > 0 ) {
		daeSolver->setReuse( BT_TRUE ); 	// IFTR method
		daeSolver->init( NXA );
		daeSolver->setup();
		
		norm = ExportVariable( "norm", 1, 1, REAL, ACADO_LOCAL, BT_TRUE );
		makeStatesConsistent = ExportFunction( "makeStatesConsistent", rk_eta );
		makeStatesConsistent.addLinebreak( );	// TO MAKE SURE IT GETS EXPORTED
		getNormConsistency = ExportFunction( "getNormConsistency", rk_eta );
		getNormConsistency.setReturnValue( norm, BT_FALSE );
	}

    return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::setupOutput( const std::vector<Grid> outputGrids_, const std::vector<Expression> outputExpressions_ ) {
	returnValue val = SUCCESSFUL_RETURN;
	CONTINUOUS_OUTPUT = BT_TRUE;
	if( outputGrids_.size() != outputExpressions_.size() ) return ACADOERROR( RET_INVALID_ARGUMENTS ); 
	outputGrids = outputGrids_;
	outputExpressions = outputExpressions_;
	
	uint i;
	uint maxOutputs = 0;
	rk_outputs.clear();
	OUTPUTS.clear();
	diffs_OUTPUTS.clear();
	for( i = 0; i < outputGrids.size(); i++ ) {
		uint numOutputs = outputExpressions_[i].getDim();
		uint outputDim = outputGrids[i].getNumIntervals( )*grid.getNumIntervals()*numOutputs*(NX+NU+1);
		if( outputGrids[i].getNumIntervals( ) > 5 ) UNROLL_OUTPUT = BT_FALSE;
		
		if( numOutputs > maxOutputs ) maxOutputs = numOutputs;
		
		DifferentialEquation f_Output;
		f_Output << outputExpressions_[i];
	
		DifferentialEquation g_Output;
		g_Output << forwardDerivative( outputExpressions_[i], x );
		g_Output << forwardDerivative( outputExpressions_[i], z );
		g_Output << forwardDerivative( outputExpressions_[i], u );
	
		ExportODEfunction OUTPUT, diffs_OUTPUT;
		val = val & OUTPUT.init( f_Output,String("acado_output")<<String(i)<<"_rhs",NX,NXA,NU ) & diffs_OUTPUT.init( g_Output,String("acado_output")<<String(i)<<"_diffs",NX,NXA,NU );
		
		ExportVariable rk_output( String("rk_output")<<String(i), 1, outputDim, REAL );
		rk_outputs.push_back( rk_output );
		
		OUTPUTS.push_back( OUTPUT );
		diffs_OUTPUTS.push_back( diffs_OUTPUT );
	}
	
	setup();
	rk_rhsOutputTemp = ExportVariable( "rk_rhsOutputTemp", 1, maxOutputs, REAL, ACADO_WORKSPACE );
	rk_diffsOutputTemp = ExportVariable( "rk_diffsOutputTemp", 1, maxOutputs*(NX+NXA+NU), REAL, ACADO_WORKSPACE );
	if( !UNROLL_OUTPUT ) rk_outH = ExportVariable( "rk_outH", 1, numStages, REAL, ACADO_WORKSPACE );
	
	return ( val );
}


returnValue ImplicitRungeKuttaExport::setupOutput(  const std::vector<Grid> outputGrids_,
									  	  const std::vector<String> _outputNames,
									  	  const std::vector<String> _diffs_outputNames,
										  const std::vector<uint> _dims_output ) {
	if( (ODE.getFunctionDim() + DAE.getFunctionDim()) == 0 && (rk_outputs.size() + OUTPUTS.size() + diffs_OUTPUTS.size()) == 0) {
		CONTINUOUS_OUTPUT = BT_TRUE;
		if( outputGrids_.size() != _outputNames.size() || outputGrids_.size() != _diffs_outputNames.size() || outputGrids_.size() != _dims_output.size() ) {
			return ACADOERROR( RET_INVALID_ARGUMENTS );
		}
		outputGrids = outputGrids_;
		name_OUTPUTS = _outputNames;
		name_diffs_OUTPUTS = _diffs_outputNames;
		num_OUTPUTS = _dims_output;

		uint i;
		uint maxOutputs = 0;
		rk_outputs.clear();
		OUTPUTS.clear();
		diffs_OUTPUTS.clear();
		for( i = 0; i < outputGrids.size(); i++ ) {
			uint numOutputs = num_OUTPUTS[i];
			uint outputDim = outputGrids[i].getNumIntervals( )*grid.getNumIntervals()*numOutputs*(NX+NU+1);
			if( outputGrids[i].getNumIntervals( ) > 5 ) UNROLL_OUTPUT = BT_FALSE;

			if( numOutputs > maxOutputs ) maxOutputs = numOutputs;

			ExportVariable rk_output( String("rk_output")<<String(i), 1, outputDim, REAL );
			rk_outputs.push_back( rk_output );
		}

		setup();
		rk_rhsOutputTemp = ExportVariable( "rk_rhsOutputTemp", 1, maxOutputs, REAL, ACADO_WORKSPACE );
		rk_diffsOutputTemp = ExportVariable( "rk_diffsOutputTemp", 1, maxOutputs*(NX+NXA+NU), REAL, ACADO_WORKSPACE );
		if( !UNROLL_OUTPUT ) rk_outH = ExportVariable( "rk_outH", 1, numStages, REAL, ACADO_WORKSPACE );

		EXPORT_RHS = BT_FALSE;
	}
	else {
		return ACADOERROR( RET_INVALID_OPTION );
	}


	return SUCCESSFUL_RETURN;
}



// PROTECTED:


returnValue ImplicitRungeKuttaExport::copy(	const ImplicitRungeKuttaExport& arg
									)
{
	numStages = arg.numStages;
	numIts = arg.numIts;
	numItsInit = arg.numItsInit;
	numAlgIts = arg.numAlgIts;
	numAlgItsInit = arg.numAlgItsInit;
	diffsDim = arg.diffsDim;
	inputDim = arg.inputDim;
	
	ODE = arg.ODE;
	DAE = arg.DAE;
	OUTPUTS = arg.OUTPUTS;
	diffs_ODE = arg.diffs_ODE;
	diffs_DAE = arg.diffs_DAE;
	diffs_OUTPUTS = arg.diffs_OUTPUTS;
	name_ODE = arg.name_ODE;
	name_DAE = arg.name_DAE;
	name_OUTPUTS = arg.name_OUTPUTS;
	num_OUTPUTS = arg.num_OUTPUTS;
	name_diffs_ODE = arg.name_diffs_ODE;
	name_diffs_DAE = arg.name_diffs_DAE;
	name_diffs_OUTPUTS = arg.name_diffs_OUTPUTS;
	grid = arg.grid;
	outputGrids = arg.outputGrids;
	solver = arg.solver;
	daeSolver = arg.daeSolver;

	// ExportVariables
	norm = arg.norm;
	makeStatesConsistent = arg.makeStatesConsistent;
	getNormConsistency = arg.getNormConsistency;
	rk_ttt = arg.rk_ttt;
	rk_xxx = arg.rk_xxx;
	rk_xPrev = arg.rk_xPrev;
	rk_kkk = arg.rk_kkk;
	rk_sol = arg.rk_sol;
	rk_A = arg.rk_A;
	rk_b = arg.rk_b;
	rk_alg_A = arg.rk_alg_A;
	rk_alg_b = arg.rk_alg_b;
	rk_rhsTemp = arg.rk_rhsTemp;
	rk_diffsTemp = arg.rk_diffsTemp;
	rk_alg_diffsTemp = arg.rk_alg_diffsTemp;
	rk_diffsPrev = arg.rk_diffsPrev;
	rk_diffsNew = arg.rk_diffsNew;
	rk_eta = arg.rk_eta;
	rk_xPrev = arg.rk_xPrev;
	rk_rhsOutputTemp = arg.rk_rhsOutputTemp;
	rk_diffsOutputTemp = arg.rk_diffsOutputTemp;
	rk_outH = arg.rk_outH;
	rk_outputs = arg.rk_outputs;
	
	// ExportFunctions
	integrate = arg.integrate;
	
	REUSE = arg.REUSE;
	CONTINUOUS_OUTPUT = arg.CONTINUOUS_OUTPUT;
	UNROLL_OUTPUT = arg.UNROLL_OUTPUT;
	
	DD = arg.DD;
	coeffs = arg.coeffs;
	
	return SUCCESSFUL_RETURN;
}


uint ImplicitRungeKuttaExport::getNumIts() const
{
	return numIts;
}


uint ImplicitRungeKuttaExport::getNumItsInit() const
{
	return numItsInit;
}


uint ImplicitRungeKuttaExport::getNumAlgIts() const
{
	return numAlgIts;
}


uint ImplicitRungeKuttaExport::getNumAlgItsInit() const
{
	return numAlgItsInit;
}


CLOSE_NAMESPACE_ACADO

// end of file.
