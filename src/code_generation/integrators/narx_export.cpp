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
 *    \file src/code_generation/integrators/narx_export.cpp
 *    \author Rien Quirynen
 *    \date 2013
 */

#include <acado/code_generation/integrators/narx_export.hpp>

#include <sstream>
using namespace std;

BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

NARXExport::NARXExport(	UserInteraction* _userInteraction,
									const String& _commonHeaderName
									) : DiscreteTimeExport( _userInteraction,_commonHeaderName )
{
	delay = 1;
}


NARXExport::NARXExport(	const NARXExport& arg
									) : DiscreteTimeExport( arg )
{
	delay = arg.delay;
	parms = arg.parms;

	copy( arg );
}


NARXExport::~NARXExport( )
{
	clear( );
}


returnValue NARXExport::setup( )
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
	rk_xxx = ExportVariable( "rk_xxx", 1, inputDim-diffsDim, REAL, structWspace );
	mem_narx = ExportVariable( "mem_narx", delay, NX, REAL, structWspace );
	if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) {
		rk_diffsPrev1 = ExportVariable( "rk_diffsPrev1", delay*NX1, NX1+NU, REAL, structWspace );
		rk_diffsPrev2 = ExportVariable( "rk_diffsPrev2", delay*NX2, NX1+NX2+NU, REAL, structWspace );
	}
	rk_diffsNew1 = ExportVariable( "rk_diffsNew1", NX1, NX1+NU, REAL, structWspace );
	rk_diffsNew2 = ExportVariable( "rk_diffsNew2", NX2, delay*NX, REAL, structWspace );

	ExportVariable numInt( "numInts", 1, 1, INT );
	if( !equidistantControlGrid() ) {
		ExportVariable numStepsV( "numSteps", numSteps, STATIC_CONST_INT );
		integrate.addStatement( String( "int " ) << numInt.getName() << " = " << numStepsV.getName() << "[" << rk_index.getName() << "];\n" );
	}

	integrate.addStatement( rk_xxx.getCols( NX,inputDim-diffsDim ) == rk_eta.getCols( NX+diffsDim,inputDim ) );
	integrate.addLinebreak( );
	if( NX2 > 0 ) {
		// TODO: is there a correct way of resetting this?
//		integrate.addStatement( String("if( ") << reset_int.getName() << " ) { \n" );
		Matrix zeroM = zeros(delay, NX);
//		integrate.addStatement( mem_narx == zeroM );
//		integrate.addStatement( String("} \n") );

		if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) {
			// Linear input:
			Matrix eyeM = eye(NX1);
			eyeM.appendCols( zeros(NX1,NU) );
			if( NX1 > 0 ) {
				integrate.addStatement( rk_diffsPrev1.getRows(0,NX1) == eyeM );
				zeroM = zeros((delay-1)*NX1,NX1+NU);
				integrate.addStatement( rk_diffsPrev1.getRows(NX1,delay*NX1) == zeroM );
			}

			// Nonlinear part:
			eyeM = zeros(NX2,NX1);
			eyeM.appendCols( eye(NX2) );
			eyeM.appendCols( zeros(NX2,NU) );
			integrate.addStatement( rk_diffsPrev2.getRows(0,NX2) == eyeM );
			zeroM = zeros((delay-1)*NX2,NX1+NX2+NU);
			integrate.addStatement( rk_diffsPrev2.getRows(NX2,delay*NX2) == zeroM );
		}
	}

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
		loop->addStatement( String("if( run > 0 ) {\n") );
		// SHIFT rk_diffsPrev:
		// TODO: write using exportforloop
		if( NX1 > 0 ) {
			for( uint s = 1; s < delay; s++ ) {
				loop->addStatement( rk_diffsPrev1.getRows(s*NX1,s*NX1+NX1) == rk_diffsPrev1.getRows((s-1)*NX1,s*NX1) );
			}
		}
		if( NX2 > 0 ) {
			for( uint s = 1; s < delay; s++ ) {
				loop->addStatement( rk_diffsPrev2.getRows(s*NX2,s*NX2+NX2) == rk_diffsPrev2.getRows((s-1)*NX2,s*NX2) );
			}
		}

		// Add to rk_diffsPrev:
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

	// shifting memory of NARX model:
	for( uint s = 1; s < delay; s++ ) {
		loop->addStatement( mem_narx.getRow(s) == mem_narx.getRow(s-1) );
	}
	loop->addStatement( mem_narx.getRow(0) == rk_xxx.getCols( 0,NX ) );

	// evaluate states:
	if( NX1 > 0 ) {
		loop->addFunctionCall( lin_input.getName(), rk_xxx, rk_eta.getAddress(0,0) );
	}
	if( NX2 > 0 ) {
		loop->addFunctionCall( rhs.getName(), mem_narx, rk_eta.getAddress(0,NX1) );
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
		loop->addFunctionCall( diffs_rhs.getName(), mem_narx, rk_diffsNew2.getAddress(0,0) );
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


returnValue NARXExport::updateImplicitSystem(	ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& tmp_index )
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
			loop03.addStatement( rk_eta.getCol( tmp_index+(NX+NXA)*(1+NX) ) == 0.0 ); // the control inputs do not appear directly in the NARX model !!
			loop01.addStatement( loop03 );
		}
		block->addStatement( loop01 );
	}
	// ALGEBRAIC STATES: NONE

	return SUCCESSFUL_RETURN;
}


returnValue NARXExport::propagateImplicitSystem(	ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2,
																const ExportIndex& index3, const ExportIndex& tmp_index )
{
	uint i;
	if( NX2 > 0 ) {
		ExportForLoop loop01( index1,NX1,NX1+NX2 );
		ExportForLoop loop02( index2,0,NX1 );
		loop02.addStatement( tmp_index == index2+index1*NX );
		loop02.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) == 0.0 );
		for( i = 0; i < delay; i++ ) {
			ExportForLoop loop03( index3,0,NX1 );
			loop03.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) += rk_diffsNew2.getSubMatrix( index1-NX1,index1-NX1+1,i*NX+index3,i*NX+index3+1 )*rk_diffsPrev1.getSubMatrix( i*NX1+index3,i*NX1+index3+1,index2,index2+1 ) );
			loop02.addStatement( loop03 );
			ExportForLoop loop04( index3,0,NX2 );
			loop04.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) += rk_diffsNew2.getSubMatrix( index1-NX1,index1-NX1+1,i*NX+NX1+index3,i*NX+NX1+index3+1 )*rk_diffsPrev2.getSubMatrix( i*NX2+index3,i*NX2+index3+1,index2,index2+1 ) );
			loop02.addStatement( loop04 );
		}
		loop01.addStatement( loop02 );

		ExportForLoop loop05( index2,NX1,NX1+NX2 );
		loop05.addStatement( tmp_index == index2+index1*NX );
		loop05.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) == 0.0 );
		for( i = 0; i < delay; i++ ) {
			ExportForLoop loop06( index3,0,NX2 );
			loop06.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) += rk_diffsNew2.getSubMatrix( index1-NX1,index1-NX1+1,i*NX+NX1+index3,i*NX+NX1+index3+1 )*rk_diffsPrev2.getSubMatrix( i*NX2+index3,i*NX2+index3+1,index2,index2+1 ) );
			loop05.addStatement( loop06 );
		}
		loop01.addStatement( loop05 );

		if( NU > 0 ) {
			ExportForLoop loop07( index2,0,NU );
			loop07.addStatement( tmp_index == index2+index1*NU );
			loop07.addStatement( rk_eta.getCol( tmp_index+(NX+NXA)*(1+NX) ) == 0.0 );
			for( i = 0; i < delay; i++ ) {
				ExportForLoop loop08( index3,0,NX1 );
				loop08.addStatement( rk_eta.getCol( tmp_index+(NX+NXA)*(1+NX) ) += rk_diffsNew2.getSubMatrix( index1-NX1,index1-NX1+1,i*NX+index3,i*NX+index3+1 )*rk_diffsPrev1.getSubMatrix( i*NX1+index3,i*NX1+index3+1,NX1+index2,NX1+index2+1 ) );
				loop07.addStatement( loop08 );
				ExportForLoop loop09( index3,0,NX2 );
				loop09.addStatement( rk_eta.getCol( tmp_index+(NX+NXA)*(1+NX) ) += rk_diffsNew2.getSubMatrix( index1-NX1,index1-NX1+1,i*NX+NX1+index3,i*NX+NX1+index3+1 )*rk_diffsPrev2.getSubMatrix( i*NX2+index3,i*NX2+index3+1,NX1+NX2+index2,NX1+NX2+index2+1 ) );
				loop07.addStatement( loop09 );
			}
			loop01.addStatement( loop07 );
		}
		block->addStatement( loop01 );
	}
	// ALGEBRAIC STATES: NO PROPAGATION OF SENSITIVITIES NEEDED

	return SUCCESSFUL_RETURN;
}


returnValue NARXExport::setDifferentialEquation(	const Expression& rhs_ )
{

	return ACADOERROR( RET_INVALID_OPTION );
}


returnValue NARXExport::setModel(	const String& _rhs, const String& _diffs_rhs ) {

	// You can't use this feature yet with NARX integrators !
	return ACADOERROR( RET_INVALID_OPTION );
}


returnValue NARXExport::getDataDeclarations(	ExportStatementBlock& declarations,
												ExportStruct dataStruct
													) const
{
	declarations.addDeclaration( mem_narx,dataStruct );

	return DiscreteTimeExport::getDataDeclarations( declarations, dataStruct );
}


returnValue NARXExport::setNARXmodel( const uint _delay, const Matrix& _parms ) {

	NX2 = _parms.getNumRows();
	delay = _delay;
	parms = _parms;

	DifferentialState dummy;
	dummy.clearStaticCounters();
	uint n = _delay*NX;
	x = DifferentialState(n);

	OutputFcn narxFun;
	OutputFcn narxDiff;
	for( uint i = 0; i < NX2; i++ ) {
		Expression expr;
		expr = _parms(i,0);
		if( _delay >= 1 ) {
			IntermediateState sum;
			for( uint j = 0; j < n; j++ ) {
				sum = sum + _parms(i,j+1)*x(j);
			}
			expr = expr + sum;
			uint indexParms = n+1;
			for( uint j = 1; j < _delay; j++ ) {
				IntermediateState tmp;
				formNARXpolynomial(i, j, indexParms, 0, tmp);
				expr = expr + tmp;
			}
		}

		narxFun << expr;
		narxDiff << forwardDerivative( expr, x );
	}
	rhs.init( narxFun,"acado_NARX_fun",NX,NXA,NU );
	diffs_rhs.init( narxDiff,"acado_NARX_diff",NX,NXA,NU );

	dummy.clearStaticCounters();
	x = DifferentialState(NX);

	return SUCCESSFUL_RETURN;
}


returnValue NARXExport::formNARXpolynomial( const uint num, const uint order, uint& base, const uint index, IntermediateState& result ) {

	uint n = delay*NX;
	for( uint i = index; i < n; i++ ) {
		if( order == 0 ) { 	// compute sum
			result = result + parms(num,base)*x(i);
			base = base+1;
		}
		else {				// recursive call
			IntermediateState tmp;
			formNARXpolynomial( num, order-1, base, i, tmp );
			result = result + tmp*x(i);
		}
	}

	return SUCCESSFUL_RETURN;
}


// PROTECTED:

//
// Register the integrator
//

IntegratorExport* createNARXExport(	UserInteraction* _userInteraction,
													const String &_commonHeaderName)
{
	return new NARXExport(_userInteraction, _commonHeaderName);
}

RegisterNARXExport::RegisterNARXExport()
{
	IntegratorExportFactory::instance().registerAlgorithm(INT_NARX, createNARXExport);
}



CLOSE_NAMESPACE_ACADO

// end of file.
