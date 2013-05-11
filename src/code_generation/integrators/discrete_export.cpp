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
 *    \file src/code_generation/integrators/discrete_export.cpp
 *    \author Rien Quirynen
 *    \date 2013
 */

#include <acado/code_generation/integrators/discrete_export.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

DiscreteTimeExport::DiscreteTimeExport(	UserInteraction* _userInteraction,
									const String& _commonHeaderName
									) : IntegratorExport( _userInteraction,_commonHeaderName )
{
}


DiscreteTimeExport::DiscreteTimeExport(	const DiscreteTimeExport& arg
									) : IntegratorExport( arg )
{
	copy( arg );
}


DiscreteTimeExport::~DiscreteTimeExport( )
{
	clear( );
}


returnValue DiscreteTimeExport::setDifferentialEquation(	const Expression& rhs_ )
{
	if( rhs_.getDim() > 0 ) {
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

		NX2 = rhs_.getDim() - NXA;
		x = DifferentialState(NX1+NX2);
		z = AlgebraicState(NXA);
		dx = DifferentialStateDerivative(NDX);
		u = Control(NU);
		p = Parameter(NP);

		DifferentialEquation f;
		f << rhs_;

		DifferentialEquation g;
		for( uint i = 0; i < rhs_.getDim(); i++ ) {
			g << forwardDerivative( rhs_(i), x );
			g << forwardDerivative( rhs_(i), u );
			// There are not supposed to be algebraic states or differential state derivatives !
		}

		return (rhs.init( f,"acado_rhs",NX,NXA,NU ) & diffs_rhs.init( g,"acado_diffs",NX,NXA,NU ) );
	}
	return SUCCESSFUL_RETURN;
}


returnValue DiscreteTimeExport::getDataDeclarations(	ExportStatementBlock& declarations,
														ExportStruct dataStruct
													) const
{
	ExportVariable max = getAuxVariable();
	declarations.addDeclaration( max,dataStruct );
	declarations.addDeclaration( rk_xxx,dataStruct );
	declarations.addDeclaration( reset_int,dataStruct );

	declarations.addDeclaration( rk_diffsPrev1,dataStruct );
	declarations.addDeclaration( rk_diffsPrev2,dataStruct );

	declarations.addDeclaration( rk_diffsNew1,dataStruct );
	declarations.addDeclaration( rk_diffsNew2,dataStruct );

	return SUCCESSFUL_RETURN;
}


returnValue DiscreteTimeExport::getFunctionDeclarations(	ExportStatementBlock& declarations
														) const
{
	declarations.addDeclaration( integrate );
	if( NX2 != NX ) declarations.addDeclaration( fullRhs );

	if( exportRhs ) {
		if( NX1 > 0 ) {
			declarations.addDeclaration( lin_input );
		}
		if( NX2 > 0 ) {
			declarations.addDeclaration( rhs );
			declarations.addDeclaration( diffs_rhs );
		}
	}
	else {
		Function tmpFun;
		tmpFun << zeros(1,1);
		ExportODEfunction tmpExport(tmpFun, getNameRHS());
		declarations.addDeclaration( tmpExport );
		tmpExport = ExportODEfunction(tmpFun, getNameDiffsRHS());
		declarations.addDeclaration( tmpExport );
	}

	return SUCCESSFUL_RETURN;
}


returnValue DiscreteTimeExport::setup( )
{
	int useOMP;
	get(CG_USE_OPENMP, useOMP);
	ExportStruct structWspace;
	structWspace = useOMP ? ACADO_LOCAL : ACADO_WORKSPACE;

	// non equidistant integration grids not implemented for NARX integrators
	if( !equidistant ) return ACADOERROR( RET_INVALID_OPTION );

	String fileName( "integrator.c" );

	int printLevel;
	get( PRINTLEVEL,printLevel );

	if ( (PrintLevel)printLevel >= HIGH )
		acadoPrintf( "--> Preparing to export %s... ",fileName.getName() );

	ExportIndex run( "run" );
	ExportIndex i( "i" );
	ExportIndex j( "j" );
	ExportIndex k( "k" );
	ExportIndex tmp_index("tmp_index");
	uint diffsDim = NX*(NX+NU);
	uint inputDim = NX*(NX+NU+1) + NU + NP;
	// setup INTEGRATE function
	rk_index = ExportVariable( "rk_index", 1, 1, INT, ACADO_LOCAL, BT_TRUE );
	rk_eta = ExportVariable( "rk_eta", 1, inputDim, REAL );
	if( equidistantControlGrid() ) {
		integrate = ExportFunction( "integrate", rk_eta, reset_int );
	}
	else {
		integrate = ExportFunction( "integrate", rk_eta, reset_int, rk_index );
	}
	integrate.setReturnValue( error_code );
	integrate.addIndex( run );
	integrate.addIndex( i );
	integrate.addIndex( j );
	integrate.addIndex( k );
	integrate.addIndex( tmp_index );
	rhs_in = ExportVariable( "x", inputDim-diffsDim, 1, REAL, ACADO_LOCAL );
	rhs_out = ExportVariable( "f", NX, 1, REAL, ACADO_LOCAL );
	fullRhs = ExportFunction( "full_rhs", rhs_in, rhs_out );
	rk_xxx = ExportVariable( "rk_xxx", 1, inputDim-diffsDim, REAL, structWspace );
	if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) {
		rk_diffsPrev1 = ExportVariable( "rk_diffsPrev1", NX1, NX1+NU, REAL, structWspace );
		rk_diffsPrev2 = ExportVariable( "rk_diffsPrev2", NX2, NX1+NX2+NU, REAL, structWspace );
	}
	rk_diffsNew1 = ExportVariable( "rk_diffsNew1", NX1, NX1+NU, REAL, structWspace );
	rk_diffsNew2 = ExportVariable( "rk_diffsNew2", NX2, NX1+NX2+NU, REAL, structWspace );

	ExportVariable numInt( "numInts", 1, 1, INT );
	if( !equidistantControlGrid() ) {
		ExportVariable numStepsV( "numSteps", numSteps, STATIC_CONST_INT );
		integrate.addStatement( String( "int " ) << numInt.getName() << " = " << numStepsV.getName() << "[" << rk_index.getName() << "];\n" );
	}

	integrate.addStatement( rk_xxx.getCols( NX,inputDim-diffsDim ) == rk_eta.getCols( NX+diffsDim,inputDim ) );
	integrate.addLinebreak( );

	// integrator loop:
	ExportForLoop tmpLoop( run, 0, grid.getNumIntervals() );
	ExportStatementBlock *loop;
	if( equidistantControlGrid() ) {
		loop = &tmpLoop;
	}
	else {
		loop = &integrate;
		loop->addStatement( String("for(") << run.getName() << " = 0; " << run.getName() << " < " << numInt.getName() << "; " << run.getName() << "++ ) {\n" );
	}

	loop->addStatement( rk_xxx.getCols( 0,NX ) == rk_eta.getCols( 0,NX ) );

	if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) {
		// Set rk_diffsPrev:
		loop->addStatement( String("if( run > 0 ) {\n") );
		if( NX1 > 0 ) {
			ExportForLoop loopTemp1( i,0,NX1 );
			loopTemp1.addStatement( rk_diffsPrev1.getSubMatrix( i,i+1,0,NX1 ) == rk_eta.getCols( i*NX+NX+NXA,i*NX+NX+NXA+NX1 ) );
			if( NU > 0 ) loopTemp1.addStatement( rk_diffsPrev1.getSubMatrix( i,i+1,NX1,NX1+NU ) == rk_eta.getCols( i*NU+(NX+NXA)*(NX+1),i*NU+(NX+NXA)*(NX+1)+NU ) );
			loop->addStatement( loopTemp1 );
		}
		if( NX2 > 0 ) {
			ExportForLoop loopTemp2( i,0,NX2 );
			loopTemp2.addStatement( rk_diffsPrev2.getSubMatrix( i,i+1,0,NX1+NX2 ) == rk_eta.getCols( i*NX+NX+NXA+NX1*NX,i*NX+NX+NXA+NX1*NX+NX1+NX2 ) );
			if( NU > 0 ) loopTemp2.addStatement( rk_diffsPrev2.getSubMatrix( i,i+1,NX1+NX2,NX1+NX2+NU ) == rk_eta.getCols( i*NU+(NX+NXA)*(NX+1)+NX1*NU,i*NU+(NX+NXA)*(NX+1)+NX1*NU+NU ) );
			loop->addStatement( loopTemp2 );
		}
		loop->addStatement( String("}\n") );
	}

	// evaluate states:
	if( NX1 > 0 ) {
		loop->addFunctionCall( lin_input.getName(), rk_xxx, rk_eta.getAddress(0,0) );
	}
	if( NX2 > 0 ) {
		loop->addFunctionCall( getNameRHS(), rk_xxx, rk_eta.getAddress(0,NX1) );
	}

	// evaluate sensitivities:
	if( NX1 > 0 ) {
		for( uint i1 = 0; i1 < NX1; i1++ ) {
			for( uint i2 = 0; i2 < NX1; i2++ ) {
				loop->addStatement( rk_diffsNew1.getSubMatrix(i1,i1+1,i2,i2+1) == A11(i1,i2) );
			}
			for( uint i2 = 0; i2 < NU; i2++ ) {
				loop->addStatement( rk_diffsNew1.getSubMatrix(i1,i1+1,NX1+i2,NX1+i2+1) == B11(i1,i2) );
			}
		}
	}
	if( NX2 > 0 ) {
		loop->addFunctionCall( getNameDiffsRHS(), rk_xxx, rk_diffsNew2.getAddress(0,0) );
	}

	// computation of the sensitivities using chain rule:
	if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) {
		loop->addStatement( String( "if( run == 0 ) {\n" ) );
	}
	// PART 1
	updateInputSystem(loop, i, j, tmp_index);
	// PART 2
	updateImplicitSystem(loop, i, j, tmp_index);

	if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) {
		loop->addStatement( String( "}\n" ) );
		loop->addStatement( String( "else {\n" ) );
		// PART 1
		propagateInputSystem(loop, i, j, k, tmp_index);
		// PART 2
		propagateImplicitSystem(loop, i, j, k, tmp_index);
		loop->addStatement( String( "}\n" ) );
	}

	// end of the integrator loop.
	if( !equidistantControlGrid() ) {
		loop->addStatement( "}\n" );
	}
	else {
		integrate.addStatement( *loop );
	}
	// PART 1
	if( NX1 > 0 ) {
		Matrix zeroR = zeros(1, NX2);
		ExportForLoop loop1( i,0,NX1 );
		loop1.addStatement( rk_eta.getCols( i*NX+NX+NXA+NX1,i*NX+NX+NXA+NX ) == zeroR );
		integrate.addStatement( loop1 );
	}

	if ( (PrintLevel)printLevel >= HIGH )
		acadoPrintf( "done.\n" );

	return SUCCESSFUL_RETURN;
}


returnValue DiscreteTimeExport::getCode(	ExportStatementBlock& code
										)
{
	if( NX1 > 0 ) {
		code.addFunction( lin_input );
		code.addStatement( "\n\n" );
	}

	if( NX2 > 0 ) {
		code.addFunction( rhs );
		code.addStatement( "\n\n" );
		code.addFunction( diffs_rhs );
		code.addStatement( "\n\n" );
	}

	if( !equidistantControlGrid() ) {
		ExportVariable numStepsV( "numSteps", numSteps, STATIC_CONST_INT );
		code.addDeclaration( numStepsV );
		code.addLinebreak( 2 );
	}
	double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();
	code.addComment(String("Fixed step size:") << String(h));

	code.addFunction( integrate );

	if( NX2 != NX ) {
		prepareFullRhs();
		code.addFunction( fullRhs );
	}

	return SUCCESSFUL_RETURN;
}


returnValue DiscreteTimeExport::setLinearOutput( const Matrix& M3, const Matrix& A3, const Expression& _rhs ) {

	return ACADOERROR( RET_INVALID_OPTION );
}


returnValue DiscreteTimeExport::setNARXmodel( const uint delay, const Matrix& parms ) {

	return RET_INVALID_OPTION;
}


returnValue DiscreteTimeExport::setupOutput( const std::vector<Grid> outputGrids_, const std::vector<Expression> _rhs ) {

	return ACADOERROR( RET_INVALID_OPTION );
}


returnValue DiscreteTimeExport::setupOutput(  const std::vector<Grid> outputGrids_,
									  	  	  	  	const std::vector<String> _outputNames,
									  	  	  	  	const std::vector<String> _diffs_outputNames,
									  	  	  	  	const std::vector<uint> _dims_output ) {

	return ACADOERROR( RET_INVALID_OPTION );
}


returnValue DiscreteTimeExport::setupOutput(  const std::vector<Grid> outputGrids_,
									  	  	  	  	const std::vector<String> _outputNames,
									  	  	  	  	const std::vector<String> _diffs_outputNames,
									  	  	  	  	const std::vector<uint> _dims_output,
									  	  	  	  	const std::vector<Matrix> _outputDependencies ) {

	return ACADOERROR( RET_INVALID_OPTION );
}


DiscreteTimeExport& DiscreteTimeExport::operator=( const DiscreteTimeExport& arg
												)
{
	if( this != &arg )
	{
		clear( );
		IntegratorExport::operator=( arg );
		copy( arg );
	}
    return *this;
}



// PROTECTED:

//
// Register the integrator
//

IntegratorExport* createDiscreteTimeExport(	UserInteraction* _userInteraction,
											const String &_commonHeaderName )
{
	return new DiscreteTimeExport(_userInteraction, _commonHeaderName);
}

RegisterDiscreteTimeExport::RegisterDiscreteTimeExport()
{
	IntegratorExportFactory::instance().registerAlgorithm(INT_DT, createDiscreteTimeExport);
}


ExportVariable DiscreteTimeExport::getAuxVariable() const
{
	ExportVariable max;
	if( NX1 > 0 ) {
		max = lin_input.getGlobalExportVariable();
	}
	if( NX2 > 0 ) {
		if( rhs.getGlobalExportVariable().getDim() >= max.getDim() ) {
			max = rhs.getGlobalExportVariable();
		}
		if( diffs_rhs.getGlobalExportVariable().getDim() >= max.getDim() ) {
			max = diffs_rhs.getGlobalExportVariable();
		}
	}

	return max;
}


returnValue DiscreteTimeExport::copy(	const DiscreteTimeExport& arg
									)
{
	rhs = arg.rhs;
	diffs_rhs = arg.diffs_rhs;

	// ExportVariables
	rk_ttt = arg.rk_ttt;
	rk_xxx = arg.rk_xxx;
	
	// ExportFunctions
	integrate = arg.integrate;
	
	return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO

// end of file.
