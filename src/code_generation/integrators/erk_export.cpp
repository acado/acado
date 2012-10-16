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
 *    \author Rien Quirynen
 *    \date 2012
 */

#include <acado/code_generation/integrators/erk_export.hpp>



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
	rk_num = ExportVariable( "rk_num", 1, 1, INT, ACADO_WORKSPACE, BT_TRUE );
	rk_ttt.setup( "rk_ttt", 1,1,            REAL,ACADO_WORKSPACE,BT_TRUE );
	rk_xxx.setup( "rk_xxx", 1,inputDim,     REAL,ACADO_WORKSPACE );
	rk_kkk.setup( "rk_kkk", rkOrder,rhsDim, REAL,ACADO_WORKSPACE );
	
	ExportIndex run( "run1" );

	// setup INTEGRATE function
	if( hasEquidistantGrid() ) {
		integrate = ExportFunction( "integrate", rk_eta );
	}
	else {
		integrate = ExportFunction( "integrate", rk_index, rk_eta );
	}

	integrate.addIndex( run );

	ExportVariable numInt( "numInts", 1, 1, INT );
	if( !hasEquidistantGrid() ) {
		integrate.addStatement( String( "int " ) << run.getName() << ";\n" );
		integrate.addStatement( String( "int numSteps[" ) << String( numSteps.getDim() ) << "] = {" << String( numSteps(0) ) );
		uint i;
		for( i = 1; i < numSteps.getDim(); i++ ) {
			integrate.addStatement( String( ", " ) << String( numSteps(i) ) );
		}
		integrate.addStatement( String( "};\n" ) );
		integrate.addStatement( String( "int " ) << numInt.getName() << " = numSteps[" << rk_index.getName() << "];\n" );
	}
	
	integrate.addStatement( rk_ttt == Matrix(grid.getFirstTime()) );
	integrate.addStatement( rk_xxx.getCols( rhsDim,inputDim ) == rk_eta.getCols( rhsDim,inputDim ) );
	integrate.addLinebreak( );

    // integrator loop
	ExportForLoop loop;
	if( hasEquidistantGrid() ) {
		loop = ExportForLoop( run, 0, grid.getNumIntervals() );
	}
	else {
		loop = ExportForLoop( run, 0, 1 );
		loop.addStatement( String("for(") << run.getName() << " = 0; " << run.getName() << " < " << numInt.getName() << "; " << run.getName() << "++ ) {\n" );
	}

	for( uint run1 = 0; run1 < rkOrder; run1++ )
	{
		loop.addStatement( rk_xxx.getCols( 0,rhsDim ) == rk_eta.getCols( 0,rhsDim ) + Ah.getRow(run1)*rk_kkk );
		loop.addFunctionCall( diffs_ODE.getName(), rk_xxx,rk_kkk.getAddress(run1,0) );
	}
	loop.addStatement( rk_eta.getCols( 0,rhsDim ) += b4h^rk_kkk );
	loop.addStatement( rk_ttt += Matrix(h) );
    // end of integrator loop

	if( !hasEquidistantGrid() ) {
		loop.addStatement( "}\n" );
		loop.unrollLoop();
	}
	integrate.addStatement( loop );
	
	if ( (PrintLevel)printLevel >= HIGH ) 
		acadoPrintf( "done.\n" );	

	return SUCCESSFUL_RETURN;
}



returnValue ExplicitRungeKuttaExport::setDifferentialEquation(	const Expression& rhs )
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
	if( rhs.getNumRows() != (NX+NXA) ) {
		return ACADOERROR( RET_INVALID_OPTION );
	}
	
	DifferentialState Gx(NX,NX), Gu(NX,NU);
	// no free parameters yet!
	// DifferentialState Gp(NX,NP);

	DifferentialEquation f, f_ODE;

	// add usual ODE
	f_ODE << rhs;
	f << rhs;
/*	if ( f.getDim() != f.getNX() )
		return ACADOERROR( RET_ILLFORMED_ODE );*/
	
	// add VDE for differential states
	f << forwardDerivative( rhs, x ) * Gx;
/*	if ( f.getDim() != f.getNX() )
		return ACADOERROR( RET_ILLFORMED_ODE );*/
	
	// add VDE for control inputs
	f << forwardDerivative( rhs, x ) * Gu + forwardDerivative( rhs, u );
// 	if ( f.getDim() != f.getNX() )
// 		return ACADOERROR( RET_ILLFORMED_ODE );

	// no free parameters yet!
	// f << forwardDerivative( rhs, x ) * Gp + forwardDerivative( rhs, p );

	int matlabInterface;
	userInteraction->get( GENERATE_MATLAB_INTERFACE, matlabInterface );
	if (matlabInterface) {
		return ODE.init( f_ODE,"acado_rhs",NX,0,NU ) & diffs_ODE.init( f,"acado_rhs_ext",NX*(1+NX+NU),0,NU );
	}
	else {
		return diffs_ODE.init( f,"acado_rhs_ext",NX*(1+NX+NU),0,NU );
	}


}



returnValue ExplicitRungeKuttaExport::setModel(	const String& _rhs, const String& _diffs_rhs ) {

	// You can't use this feature yet with explicit integrators, because they need the Variational Differential Equations !
	return ACADOERROR( RET_INVALID_OPTION );
}


returnValue ExplicitRungeKuttaExport::getDataDeclarations(	ExportStatementBlock& declarations,
													ExportStruct dataStruct
													) const
{
	declarations.addDeclaration( diffs_ODE.getGlobalExportVariable(),dataStruct );
	declarations.addDeclaration( rk_ttt,dataStruct );
	declarations.addDeclaration( rk_xxx,dataStruct );
	declarations.addDeclaration( rk_kkk,dataStruct );
	
	declarations.addDeclaration( rk_num,dataStruct );

	return SUCCESSFUL_RETURN;
}


returnValue ExplicitRungeKuttaExport::getFunctionDeclarations(	ExportStatementBlock& declarations
														) const
{
	declarations.addDeclaration( integrate );
	declarations.addDeclaration( diffs_ODE );

	int matlabInterface;
	userInteraction->get( GENERATE_MATLAB_INTERFACE, matlabInterface );
	if (matlabInterface) {
		declarations.addDeclaration( ODE );
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

	code.addFunction( diffs_ODE );
	code.addFunction( integrate );

	int matlabInterface;
	userInteraction->get( GENERATE_MATLAB_INTERFACE, matlabInterface );
	if (matlabInterface) {
		code.addFunction( ODE );
	}

// 	if ( (PrintLevel)printLevel >= HIGH ) 
// 		acadoPrintf( "done.\n" );

	return SUCCESSFUL_RETURN;
}


returnValue ExplicitRungeKuttaExport::setupOutput( const std::vector<Grid> outputGrids_, const std::vector<Expression> rhs ) {
	
	return ACADOERROR( RET_INVALID_OPTION );
}


returnValue ExplicitRungeKuttaExport::setupOutput(  const std::vector<Grid> outputGrids_,
									  	  	  	  	const std::vector<String> _outputNames,
									  	  	  	  	const std::vector<String> _diffs_outputNames,
									  	  	  	  	const std::vector<uint> _dims_output ) {

	return ACADOERROR( RET_INVALID_OPTION );
}



// PROTECTED:



CLOSE_NAMESPACE_ACADO

// end of file.
