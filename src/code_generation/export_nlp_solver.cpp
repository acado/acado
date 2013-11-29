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
 *    \file src/code_generation/export_nlp_solver.cpp
 *    \author Milan Vukov
 *    \date 2012 - 2013
 */

#include <acado/code_generation/export_nlp_solver.hpp>

#include <sstream>

BEGIN_NAMESPACE_ACADO

using namespace std;

ExportNLPSolver::ExportNLPSolver(	UserInteraction* _userInteraction,
									const String& _commonHeaderName
									) : ExportAlgorithm(_userInteraction, _commonHeaderName),
											cholObjS(_userInteraction, _commonHeaderName),
											cholSAC(_userInteraction, _commonHeaderName),
											acSolver(userInteraction, _commonHeaderName)

{
	levenbergMarquardt    = 0.0;

	dimPacH = 0;
	dimPocH = 0;

	externObjective = BT_FALSE;
}

returnValue ExportNLPSolver::setIntegratorExport(	IntegratorExportPtr const _integrator
													)
{
	integrator = _integrator;
	return SUCCESSFUL_RETURN;
}

returnValue ExportNLPSolver::setLevenbergMarquardt(	double _levenbergMarquardt
														)
{
	if ( _levenbergMarquardt < 0.0 )
	{
		ACADOWARNINGTEXT(RET_INVALID_ARGUMENTS, "Levenberg-Marquardt regularization factor must be positive!");
		levenbergMarquardt = 0.0;
	}
	else
	{
		levenbergMarquardt = _levenbergMarquardt;
	}

	return SUCCESSFUL_RETURN;
}

BooleanType ExportNLPSolver::performsSingleShooting( ) const
{
	int discretizationType;
	get( DISCRETIZATION_TYPE,discretizationType );

	if ( discretizationType == SINGLE_SHOOTING )
		return BT_TRUE;
	else
		return BT_FALSE;
}

returnValue ExportNLPSolver::getDataDeclarations(	ExportStatementBlock& declarations,
													ExportStruct dataStruct
													) const
{
	declarations.addDeclaration(state, dataStruct);
	declarations.addDeclaration(x, dataStruct);
	declarations.addDeclaration(z, dataStruct);
	declarations.addDeclaration(u, dataStruct);
	declarations.addDeclaration(p, dataStruct);
	declarations.addDeclaration(d, dataStruct);

	declarations.addDeclaration(y, dataStruct);
	declarations.addDeclaration(yN, dataStruct);
	declarations.addDeclaration(Dy, dataStruct);
	declarations.addDeclaration(DyN, dataStruct);

	declarations.addDeclaration(evGx, dataStruct);
	declarations.addDeclaration(evGu, dataStruct);

	declarations.addDeclaration(objS, dataStruct);

	declarations.addDeclaration(objAuxVar, dataStruct);
	declarations.addDeclaration(objValueIn, dataStruct);
	declarations.addDeclaration(objValueOut, dataStruct);

	declarations.addDeclaration(Q1, dataStruct);
	declarations.addDeclaration(Q2, dataStruct);

	declarations.addDeclaration(R1, dataStruct);
	declarations.addDeclaration(R2, dataStruct);

	declarations.addDeclaration(objSEndTerm, dataStruct);

	declarations.addDeclaration(QN1, dataStruct);
	declarations.addDeclaration(QN2, dataStruct);

	declarations.addDeclaration(SAC, dataStruct);
	declarations.addDeclaration(xAC, dataStruct);
	declarations.addDeclaration(DxAC, dataStruct);

	declarations.addDeclaration(conAuxVar, dataStruct);
	declarations.addDeclaration(conValueIn, dataStruct);
	declarations.addDeclaration(conValueOut, dataStruct);

	declarations.addDeclaration(pacEvH, dataStruct);
	declarations.addDeclaration(pacEvHx, dataStruct);
	declarations.addDeclaration(pacEvHu, dataStruct);
	declarations.addDeclaration(pacEvHxd, dataStruct);

	declarations.addDeclaration(pocEvH, dataStruct);
	declarations.addDeclaration(pocEvHx, dataStruct);
	declarations.addDeclaration(pocEvHu, dataStruct);
	declarations.addDeclaration(pocEvHxd, dataStruct);

	// Arrival cost stuff
	declarations.addDeclaration(acA, dataStruct);
	declarations.addDeclaration(acb, dataStruct);
	declarations.addDeclaration(acP, dataStruct);
	declarations.addDeclaration(acTmp, dataStruct);
	declarations.addDeclaration(acWL, dataStruct);
	declarations.addDeclaration(acVL, dataStruct);
	declarations.addDeclaration(acHx, dataStruct);
	declarations.addDeclaration(acHu, dataStruct);
	declarations.addDeclaration(acXx, dataStruct);
	declarations.addDeclaration(acXu, dataStruct);
	declarations.addDeclaration(acXTilde, dataStruct);
	declarations.addDeclaration(acHTilde, dataStruct);

	return SUCCESSFUL_RETURN;
}

returnValue ExportNLPSolver::setupSimulation( void )
{
	// \todo Implement free parameters and support for DAEs

	//
	// By default, here will be defined model simulation suitable for sparse QP solver.
	// Condensing based QP solvers should redefine/extend model simulation
	//

	// \todo Move to something like: setupInitialization
	ExportVariable retInit("ret", 1, 1, INT, ACADO_LOCAL);
	retInit.setDoc("=0: OK, otherwise an error code of a QP solver.");
	initialize.setup( "initializeSolver" );
	initialize.doc( "Solver initialization. Must be called once before any other function call." );
	initialize.setReturnValue(retInit);
	initialize << retInit.getFullName() << String(" = 0;\n");

	initialize.addComment( "This is a function which must be called once before any other function call!" );
	initialize.addLinebreak( 2 );

	modelSimulation.setup( "modelSimulation" );
	ExportIndex run;
	modelSimulation.acquire( run );
	ExportForLoop loop(run, 0, getN());

	int useOMP;
	get(CG_USE_OPENMP, useOMP);

	x.setup("x", (getN() + 1), getNX(), REAL, ACADO_VARIABLES);
	x.setDoc( (String)"Matrix containing " << (getN() + 1) << " differential variable vectors." );
	z.setup("z", getN(), getNXA(), REAL, ACADO_VARIABLES);
	z.setDoc( (String)"Matrix containing " << N << " algebraic variable vectors." );
	u.setup("u", getN(), getNU(), REAL, ACADO_VARIABLES);
	u.setDoc( (String)"Matrix containing " << N << " control variable vectors." );
	p.setup("p", 1, getNP(), REAL, ACADO_VARIABLES);
	p.setDoc( (String)"Vector of parameters." );

	if (performsSingleShooting() == BT_FALSE)
	{
		d.setup("d", getN() * getNX(), 1, REAL, ACADO_WORKSPACE);
	}

	evGx.setup("evGx", N * NX, NX, REAL, ACADO_WORKSPACE);
	evGu.setup("evGu", N * NX, NU, REAL, ACADO_WORKSPACE);

	ExportStruct dataStructWspace;
	dataStructWspace = (useOMP && performsSingleShooting() == BT_FALSE) ? ACADO_LOCAL : ACADO_WORKSPACE;
	state.setup("state", 1, (getNX() + getNXA()) * (getNX() + getNU() + 1) + getNU() + getNP(), REAL, dataStructWspace);

	unsigned indexZ   = NX + NXA;
	unsigned indexGxx = indexZ + NX * NX;
	unsigned indexGzx = indexGxx + NXA * NX;
	unsigned indexGxu = indexGzx + NX * NU;
	unsigned indexGzu = indexGxu + NXA * NU;
	unsigned indexU   = indexGzu + NU;
	unsigned indexP   = indexU + NP;

	////////////////////////////////////////////////////////////////////////////
	//
	// Code for model simulation
	//
	////////////////////////////////////////////////////////////////////////////
	if (performsSingleShooting() == BT_TRUE)
	{
		modelSimulation.addStatement( state.getCols(0, NX)				== x.getRow( 0 ) );
		modelSimulation.addStatement( state.getCols(NX, NX + NXA)		== z.getRow( 0 ) );
		modelSimulation.addStatement( state.getCols(indexGzu, indexU)	== u.getRow( 0 ) );
		modelSimulation.addStatement( state.getCols(indexU, indexP)		== p );
		modelSimulation.addLinebreak( );
	}

	if ( useOMP )
	{
		stringstream s;
		s << "#pragma omp parallel for private(" << run.getName().getName() << ", " << state.getFullName().getName()
				<< ") shared("
				<< evGx.getDataStructString().getName() << ", "
				<< x.getDataStructString().getName()
				<< ")" << endl;

		modelSimulation.addStatement( s.str().c_str() );
	}

	if (performsSingleShooting() == BT_FALSE)
	{
		loop.addStatement( state.getCols(0, NX)			== x.getRow( run ) );
		loop.addStatement( state.getCols(NX, NX + NXA)	== z.getRow( run ) );
	}
	loop.addLinebreak( );

	// Fill in the input vector
	loop.addStatement( state.getCols(indexGzu, indexU)	== u.getRow( run ) );
	loop.addStatement( state.getCols(indexU, indexP)	== p );
	loop.addLinebreak( );

	// Integrate the model
	// TODO make that function calls can accept constant defined scalars
	if ( integrator->equidistantControlGrid() )
	{
		if (performsSingleShooting() == BT_FALSE)
			loop.addStatement( (String)"integrate"
					<< "(" << state.getFullName() << ", 1);\n"  );
		else
			loop.addStatement( (String)"integrate"
					<< "(" << state.getFullName() << ", "
					<< run.getFullName() << " == 0"
					<< ");\n"  );
	}
	else
	{
		if (performsSingleShooting() == BT_FALSE)
			loop.addStatement( (String)"integrate"
					<< "(" << state.getFullName() << ", 1, " << run.getFullName() << ");\n" );
		else
			loop.addStatement( (String)"integrate"
					<< "(" << state.getFullName() << ", "
					<< run.getFullName() << " == 0"
					<< ", " << run.getFullName() << ");\n" );
	}
	loop.addLinebreak( );

	if ( performsSingleShooting() == BT_TRUE )
	{
		// Single shooting case: prepare for the next iteration
		loop.addStatement( x.getRow(run + 1) == state.getCols(0, NX) );
		loop.addLinebreak( );
	}
	else
	{
		// Multiple shootin', compute residuum
		loop.addStatement( d.getTranspose().getCols(run * NX, (run  + 1) * NX) == state.getCols( 0,getNX() ) - x.getRow( run+1 ) );
		loop.addLinebreak( );
	}

	loop.addStatement( z.getRow( run ) == state.getCols(NX, NX + NXA) );

	// Stack sensitivities
	// \todo Upgrade this code later to stack Z sens
	loop.addStatement(
			evGx.makeRowVector().getCols(run * NX * NX, (run + 1) * NX * NX) == state.getCols(indexZ, indexGxx)
	);
	loop.addLinebreak();

	loop.addStatement(
			evGu.makeRowVector().getCols(run * NX * NU, (run + 1) * NX * NU) == state.getCols(indexGzx, indexGxu)
	);

	modelSimulation.addStatement( loop );

	// XXX This should be revisited at some point
//	modelSimulation.release( run );

	return SUCCESSFUL_RETURN;
}

returnValue ExportNLPSolver::setObjective(const Objective& _objective)
{
	int variableObjS;
	get(CG_USE_VARIABLE_WEIGHTING_MATRIX, variableObjS);

	// Temporary variables
	ExportVariable objSTemp, objSEndTermTemp;

	////////////////////////////////////////////////////////////////////////////
	//
	// Setup arrival cost calc variables
	//
	////////////////////////////////////////////////////////////////////////////

	int useArrivalCost;
	get(CG_USE_ARRIVAL_COST, useArrivalCost);

	if ( useArrivalCost )
	{
		SAC.setup("SAC", NX, NX, REAL, ACADO_VARIABLES);
		SAC.setDoc("Arrival cost term: inverse of the covariance matrix.");
		xAC.setup("xAC", NX, 1, REAL, ACADO_VARIABLES);
		xAC.setDoc("Arrival cost term: a priori state estimate.");
		DxAC.setup("DxAC", NX, 1, REAL, ACADO_WORKSPACE);
	}

	////////////////////////////////////////////////////////////////////////////
	//
	// Check first if we are dealing with external functions
	//
	////////////////////////////////////////////////////////////////////////////

	vector<String> lsqExternFunctions;
	vector<ExportVariable> lsqExternMatrices;

	_objective.getLSQTerms(lsqExternMatrices, lsqExternFunctions);

	vector<String> lsqExternEndTermFunctions;
	vector<ExportVariable> lsqExternEndTermMatrices;

	_objective.getLSQEndTerms(lsqExternEndTermMatrices, lsqExternEndTermFunctions);

	if (lsqExternMatrices.size() > 0 || lsqExternEndTermMatrices.size() > 0)
	{
		if (lsqExternMatrices.size() != lsqExternEndTermMatrices.size())
			return ACADOERROR( RET_INVALID_ARGUMENTS );
		if (lsqExternMatrices.size() > 1 || lsqExternEndTermMatrices.size() > 1)
			return ACADOERROR( RET_INVALID_ARGUMENTS );

		objSTemp = lsqExternMatrices[ 0 ];
		objSTemp.setDataStruct( ACADO_VARIABLES );
		objSEndTermTemp = lsqExternEndTermMatrices[ 0 ];
		objSEndTermTemp.setDataStruct( ACADO_VARIABLES );

		if (objSTemp.getGivenMatrix().isSquare() == BT_FALSE ||
				objSEndTermTemp.getGivenMatrix().isSquare() == BT_FALSE)
			return ACADOERROR( RET_INVALID_ARGUMENTS );

		setNY( objSTemp.getNumRows() );
		setNYN( objSEndTermTemp.getNumRows() );

		if (variableObjS == YES)
		{
			objS.setup("W", N * NY, NY, REAL, ACADO_VARIABLES);
		}
		else
		{
			objS.setup("W", NY, NY, REAL, ACADO_VARIABLES);
		}
		objSEndTerm = CasADi::deepcopy( objSEndTermTemp );
		objSEndTerm.setName( "WN" );

		evaluateExternLSQ = lsqExternFunctions[ 0 ];
		evaluateExternLSQEndTerm = lsqExternEndTermFunctions[ 0 ];

		// ExportVariable objEvFx, objEvFu, objEvFxEnd; // aliasing

		objEvFx.setup("evFx", NY, NX, REAL, ACADO_WORKSPACE);
		objEvFu.setup("evFx", NY, NU, REAL, ACADO_WORKSPACE);
		objEvFxEnd.setup("evFx", NYN, NX, REAL, ACADO_WORKSPACE);

		Q1.setup("Q1", NX * N, NX, REAL, ACADO_WORKSPACE);
		Q2.setup("Q2", NX * N, NY, REAL, ACADO_WORKSPACE);

		R1.setup("R1", NU * N, NU, REAL, ACADO_WORKSPACE);
		R2.setup("R2", NU * N, NY, REAL, ACADO_WORKSPACE);

		QN1.setup("QN1", NX, NX, REAL, ACADO_WORKSPACE);
		QN2.setup("QN2", NX, NYN, REAL, ACADO_WORKSPACE);

		objValueIn.setup("objValueIn", 1, NX + 0 + NU + NP, REAL, ACADO_WORKSPACE);
		objValueOut.setup("objValueOut", 1,
				NY < NYN ? NYN * (1 + NX + NU): NY * (1 + NX + NU), REAL, ACADO_WORKSPACE);

		externObjective = BT_TRUE;

		setupResidualVariables();

		return SUCCESSFUL_RETURN;
	}

	////////////////////////////////////////////////////////////////////////////
	//
	// ... or we use ACADO AD
	//
	////////////////////////////////////////////////////////////////////////////

	vector<Function> lsqFunctions;
	vector<ExportVariable> lsqMatrices;

	vector<Function> lsqEndTermFunctions;
	vector<ExportVariable> lsqEndTermMatrices;

	Function objF, objFEndTerm;

	_objective.getLSQTerms(lsqMatrices, lsqFunctions);

	if(	lsqFunctions.size() == 0 || lsqMatrices.size() == 0 ||
			lsqFunctions.size() != lsqMatrices.size() )
		return ACADOERRORTEXT(RET_INITIALIZE_FIRST, "Objective function is not initialized.");

	else if (lsqFunctions.size() > 1 || lsqEndTermFunctions.size() > 1)
		return ACADOERRORTEXT(RET_INITIALIZE_FIRST,
				"Current implementation of code generation module\n"
				"supports only one LSQ term definition per one OCP." );

	_objective.getLSQEndTerms(lsqEndTermMatrices, lsqEndTermFunctions);

	objF = lsqFunctions[ 0 ];
	setNY( objF.getDim() );

	objSTemp = lsqMatrices[ 0 ];

	if (objSTemp.getNumCols() != objSTemp.getNumRows())
		return ACADOERRORTEXT(RET_INVALID_ARGUMENTS, "Weighting matrices must be square.");
	if (objSTemp.getNumRows() != (unsigned)objF.getDim())
		return ACADOERRORTEXT(RET_INVALID_ARGUMENTS, "Wrong dimensions of the weighting matrix.");

	DifferentialState dummy0;
	Control dummy1;
	dummy0.clearStaticCounters();
	dummy1.clearStaticCounters();

	DifferentialState vX( NX );
	Control vU( NU );

	////////////////////////////////////////////////////////////////////////////
	//
	// Setup the Lagrange LSQ terms
	//
	////////////////////////////////////////////////////////////////////////////

	// TODO FunctionEvaluationTree: add isConstant()

	// Setup the S matrix
	if ( objSTemp.isGiven() == BT_FALSE || (objSTemp.getDim() != (NY * NY)) )
	{
		if ( variableObjS == YES )
		{
			// We allow user to define different w. matrix on every node

			objS.setup("W", N * NY, NY, REAL, ACADO_VARIABLES);
		}
		else
		{
			objS.setup("W", NY, NY, REAL, ACADO_VARIABLES);
		}
	}
	else
	{
		Matrix mObjS = objSTemp.getGivenMatrix();

		if (mObjS.isPositiveDefinite() == BT_FALSE)
			return ACADOERROR( RET_NONPOSITIVE_WEIGHT );

		objS.setup("W", mObjS, REAL, ACADO_VARIABLES);
	}

	Expression expF;
	objF.getExpression( expF );

	Expression expFx;
	Expression expFu;

	expFx = forwardDerivative(expF, vX);
	expFu = forwardDerivative(expF, vU);

	Function Fx, Fu;
	Fx << expFx;
	Fu << expFu;

	if ( Fx.getNX() == 0 && Fx.getNU() == 0 )
	{
		EvaluationPoint epFx( Fx );

		Matrix mFx(NY, NX);

		Vector vFx = Fx.evaluate( epFx );

		for (unsigned i = 0; i < NY; ++i)
			for(unsigned j = 0; j < NX; ++j)
				mFx(i, j) = vFx(i * NX + j);

		objEvFx.setup("evFx", mFx, REAL, ACADO_WORKSPACE);
	}
	else
	{
		objF << expFx;

		objEvFx.setup("evFx", NY, NX, REAL, ACADO_WORKSPACE);
	}

//	objF << expFx;
//	evFx.setup("evFx", NY, NX, REAL, ACADO_WORKSPACE);

	if ( Fu.getNX() == 0 && Fu.getNU() == 0 )
	{
		EvaluationPoint epFu( Fu );

		Matrix mFu(NY, NU);

		Vector vFu = Fu.evaluate( epFu );

		for (unsigned i = 0; i < NY; ++i)
			for(unsigned j = 0; j < NU; ++j)
				mFu(i, j) = vFu(i * NU + j);

		objEvFu.setup("evFu", mFu, REAL, ACADO_WORKSPACE);
	}
	else
	{
		objF << expFu;

		objEvFu.setup("evFu", NY, NU, REAL, ACADO_WORKSPACE);
	}

//	objF << expFu;
//	evFu.setup("evFu", NY, NU, REAL, ACADO_WORKSPACE);

	//
	// Initialize the export of the LSQ function which evaluates the
	// objective and (possibly) its derivatives.
	//

	// Set the separate aux variable for the evaluation of the objective.

	objAuxVar.setup("objAuxVar", objF.getGlobalExportVariableSize(), 1, REAL, ACADO_WORKSPACE);
	evaluateLSQ.init(objF, "evaluateLSQ", NX, 0, NU);
	evaluateLSQ.setGlobalExportVariable( objAuxVar );

	objValueIn.setup("objValueIn", 1, NX + 0 + NU + NP, REAL, ACADO_WORKSPACE);
	objValueOut.setup("objValueOut", 1, objF.getDim(), REAL, ACADO_WORKSPACE);

	//
	// Optional pre-computing of Q1, Q2, R1, R2 matrices
	//

	if (objS.isGiven() == BT_TRUE && objEvFx.isGiven() == BT_TRUE)
	{
		if (useArrivalCost)
			return ACADOERROR( RET_NOT_IMPLEMENTED_YET );

		// Precompute Q1 and Q2;

		Matrix m1(NX,NX), m2(NX, NY);

		m2 = objEvFx.getGivenMatrix().transpose() * objS.getGivenMatrix();
		m1 = m2 * objEvFx.getGivenMatrix();

		Q1 = m1;
		if ( m1 == m2 )
		{
			Q2 = Q1;
		}
		else
		{
			Q2 = m2;
		}
	}
	else if (Fx.isOneOrZero() == NE_ZERO)
	{
		if (useArrivalCost)
			return ACADOERROR( RET_NOT_IMPLEMENTED_YET );

		Q1 = zeros(NX, NX);
		Q2 = zeros(NX, NY);
	}
	else
	{
		Q1.setup("Q1", NX * N, NX, REAL, ACADO_WORKSPACE);
		Q2.setup("Q2", NX * N, NY, REAL, ACADO_WORKSPACE);
	}

	// TODO ExportVariable, add function isZero()
	// TODO This if-then-else part should be done in more elegant way
	if (objS.isGiven() == BT_TRUE && objEvFu.isGiven() == BT_TRUE)
	{
		// Precompute R1 and R2

		Matrix m2 = objEvFu.getGivenMatrix().transpose() * objS.getGivenMatrix();
		Matrix m1 = m2 * objEvFu.getGivenMatrix();

		R1 = m1;
		if (m1 == m2)
		{
			R2 = R1;
		}
		else
		{
			R2 = m2;
		}
	}
	else if (Fu.isOneOrZero() == NE_ZERO)
	{
		R1 = zeros(NU, NU);
		R2 = zeros(NU, NY);
	}
	else
	{
		R1.setup("R1", NU * N, NU, REAL, ACADO_WORKSPACE);
		R2.setup("R2", NU * N, NY, REAL, ACADO_WORKSPACE);
	}

	////////////////////////////////////////////////////////////////////////////
	//
	// Setup the quadratic Mayer term stuff
	//
	////////////////////////////////////////////////////////////////////////////

	if ( lsqEndTermFunctions.size() == 0 )
		return ACADOERRORTEXT(RET_INVALID_OBJECTIVE_FOR_CODE_EXPORT, "The terminal cost must be defined for MHE export");
	if (lsqEndTermFunctions.size() != lsqEndTermMatrices.size())
		return ACADOERRORTEXT(RET_INVALID_OBJECTIVE_FOR_CODE_EXPORT, "Dimensions of the terminal cost function and the terminal cost weighting matrix must be the same");

	objSEndTermTemp = lsqEndTermMatrices[ 0 ];
	objFEndTerm = lsqEndTermFunctions[ 0 ];

	if (objFEndTerm.getNU() > 0)
		return ACADOERRORTEXT(RET_INVALID_OBJECTIVE_FOR_CODE_EXPORT, "The terminal cost function must not depend on controls.");

	setNYN( objFEndTerm.getDim() );

	if (objSEndTermTemp.getNumCols() != objSEndTermTemp.getNumRows())
		return ACADOERRORTEXT(RET_INVALID_ARGUMENTS, "Weighting matrices must be square.");
	if (objSEndTermTemp.getNumRows() != (unsigned)objFEndTerm.getDim())
		return ACADOERRORTEXT(RET_INVALID_ARGUMENTS, "Wrong dimensions of the weighting matrix.");

	// Setup the SN matrix
	if (objSEndTermTemp.isGiven() == BT_FALSE || objSEndTermTemp.getDim() != (NYN * NYN))
	{
		objSEndTerm.setup("WN", NYN, NYN, REAL, ACADO_VARIABLES);
	}
	else
	{
		Matrix mWN = objSEndTermTemp.getGivenMatrix();

		if (mWN.isPositiveDefinite() == BT_FALSE)
			return ACADOERROR( RET_NONPOSITIVE_WEIGHT );

		objSEndTerm.setup("WN", mWN, REAL, ACADO_VARIABLES);
	}

	Expression expFEndTerm;
	objFEndTerm.getExpression( expFEndTerm );

	Expression expFEndTermX;

	expFEndTermX = forwardDerivative(expFEndTerm, vX);

	Function FEndTermX;
	FEndTermX << expFEndTermX;

	if ( FEndTermX.getNX() == 0 )
	{
		EvaluationPoint epFEndTermX( FEndTermX );

		Matrix mFEndX(NYN, NX);

		Vector vFx = FEndTermX.evaluate( epFEndTermX );

		// And now reshape vFx and vFu to export variables
		for (unsigned i = 0; i < NYN; ++i)
			for(unsigned j = 0; j < NX; ++j)
				mFEndX(i, j) = vFx(i * NX + j);

		objEvFxEnd.setup("evFxEnd", mFEndX, REAL, ACADO_WORKSPACE);
	}
	else
	{
		objFEndTerm << expFEndTermX;

		objEvFxEnd.setup("evFxEnd", NYN, NX, REAL, ACADO_WORKSPACE);
	}

//	objFEndTerm << expFEndTermX;
//	objEvFxEnd.setup("evFxEnd", NYN, NX, REAL, ACADO_WORKSPACE);

	unsigned objFEndTermSize = objFEndTerm.getGlobalExportVariableSize();
	if ( objFEndTermSize > objAuxVar.getDim() )
	{
		objAuxVar.setup(objAuxVar.getName(), objFEndTermSize, 1, REAL, objAuxVar.getDataStruct());
	}

	evaluateLSQEndTerm.init(objFEndTerm, "evaluateLSQEndTerm", NX, 0, 0);
	evaluateLSQEndTerm.setGlobalExportVariable( objAuxVar );

	if (objFEndTerm.getDim() > objF.getDim())
	{
		objValueOut.setup("objValueOut", 1, objFEndTerm.getDim(), REAL, ACADO_WORKSPACE);
	}

	if (objSEndTerm.isGiven() == BT_TRUE && objEvFxEnd.isGiven() == BT_TRUE)
	{
		// Precompute

		Matrix m2, m1;

		m2 = objEvFxEnd.getTranspose().getGivenMatrix() * objSEndTerm.getGivenMatrix();
		m1 = m2 * objEvFxEnd.getGivenMatrix();

		QN1 = m1;
		if (m1 ==  m2)
			QN2 = m1;
		else
			QN2 = m2;
	}
	else if (FEndTermX.isOneOrZero() == NE_ZERO)
	{
		QN1 = zeros(NX, NX);
		QN2 = zeros(NX, NYN);
	}
	else
	{
		QN1.setup("QN1", NX, NX, REAL, ACADO_WORKSPACE);
		QN2.setup("QN2", NX, NYN, REAL, ACADO_WORKSPACE);
	}

	// Both are given or none is given; otherwise give an error.
	if (getNYN() && (objS.isGiven() ^ objSEndTerm.isGiven()))
		return ACADOERRORTEXT(RET_INVALID_OBJECTIVE_FOR_CODE_EXPORT, "All weighting matrices have to be defined (or all undefined)");

	////////////////////////////////////////////////////////////////////////////
	//
	// Setup the linear terms
	//
	////////////////////////////////////////////////////////////////////////////

	std::vector<ExportVariable> vSlx, vSlu;

	_objective.getLSQLinearTerms(vSlx, vSlu);

	if (vSlx.size() > 0 && vSlu.size() > 0)
	{
		if (vSlx.size() != 1 || vSlu.size() != 1)
			return ACADOERROR(RET_INVALID_ARGUMENTS);

		if (vSlx[ 0 ].getDim() > 0)
		{
			if (vSlx[ 0 ].isGiven() && vSlx[ 0 ].getDim() != NX && vSlx[ 0 ].getNumCols() == 1)
				return ACADOERROR(RET_INVALID_ARGUMENTS);

			if (vSlx[ 0 ].isGiven() == BT_TRUE)
				objSlx.setup("Wlx", vSlx[ 0 ].getGivenMatrix(), REAL, ACADO_VARIABLES);
			else
				objSlx.setup("Wlx", NX, 1, REAL, ACADO_VARIABLES);
		}

		if (vSlu[ 0 ].getDim() > 0)
		{
			if (vSlu[ 0 ].isGiven() && vSlu[ 0 ].getDim() != NU && vSlu[ 0 ].getNumCols() == 1)
				return ACADOERROR(RET_INVALID_ARGUMENTS);

			if (vSlu[ 0 ].isGiven() == BT_TRUE)
				objSlu.setup("Wlu", vSlu[ 0 ].getGivenMatrix(), REAL, ACADO_VARIABLES);
			else
				objSlu.setup("Wlu", NU, 1, REAL, ACADO_VARIABLES);
		}
	}

	////////////////////////////////////////////////////////////////////////////
	//
	// Setup the residual variables
	//
	////////////////////////////////////////////////////////////////////////////

	setupResidualVariables();

	return SUCCESSFUL_RETURN;
}

returnValue ExportNLPSolver::setupResidualVariables()
{
	y.setup("y",  getN() * getNY(), 1, REAL, ACADO_VARIABLES);
	y.setDoc( (String)"Matrix containing " << getNY() <<
			" reference/measurement vectors for first " << N <<" nodes." );
	yN.setup("yN", getNYN(), 1, REAL, ACADO_VARIABLES);
	yN.setDoc( (String)"Reference/measurement vector for the " << N + 1 << ". node." );
	Dy.setup("Dy", getN() * getNY(), 1, REAL,ACADO_WORKSPACE);
	DyN.setup("DyN", getNYN(), 1, REAL, ACADO_WORKSPACE);

	return SUCCESSFUL_RETURN;
}

returnValue ExportNLPSolver::setConstraints(const OCP& _ocp)
{
	////////////////////////////////////////////////////////////////////////////
	//
	// Extract box constraints
	//
	////////////////////////////////////////////////////////////////////////////

	Grid grid;
	Constraint constraints;

	_ocp.getGrid( grid );
	_ocp.getConstraint( constraints );

	VariablesGrid ugrid(NU, grid);
	VariablesGrid xgrid(NX, grid);

	OCPiterate tmp;
	tmp.init(&xgrid, 0, 0, &ugrid, 0);

	constraints.getBounds( tmp );

	BooleanType isFinite = BT_FALSE;
	Vector lbTmp;
	Vector ubTmp;

	//
	// Extract box constraints on inputs
	//
	for (unsigned i = 0; i < tmp.u->getNumPoints(); ++i)
	{
		lbTmp = tmp.u->getLowerBounds( i );
		ubTmp = tmp.u->getUpperBounds( i );

		if ( (ubTmp - lbTmp).isPositive() == BT_FALSE )
			return ACADOERRORTEXT(RET_INVALID_ARGUMENTS, "Some lower bounds are bigger than upper bounds?");

		if ( (lbTmp.isFinite( ) == BT_TRUE) || (ubTmp.isFinite( ) == BT_TRUE) )
			isFinite = BT_TRUE;
	}

	if (isFinite == BT_TRUE)
		uBounds = *(tmp.u);
	else
		uBounds.init();

	//
	// Extract box constraints on states
	//
	isFinite = BT_FALSE;
	xBoundsIdx.clear();

	for (unsigned i = 0; i < tmp.x->getNumPoints(); ++i)
	{
		lbTmp = tmp.x->getLowerBounds( i );
		ubTmp = tmp.x->getUpperBounds( i );

		if ( (ubTmp - lbTmp).isPositive() == BT_FALSE )
			return ACADOERRORTEXT(RET_INVALID_ARGUMENTS, "Some lower bounds are bigger than upper bounds?");

		if ( (lbTmp.isFinite( ) == BT_TRUE) || (ubTmp.isFinite( ) == BT_TRUE) )
			isFinite = BT_TRUE;

		// This is maybe not necessary
		if (isFinite == BT_FALSE || i == 0)
			continue;

		for (unsigned j = 0; j < lbTmp.getDim(); ++j)
		{
			if ( ( acadoIsFinite( ubTmp( j ) ) == BT_TRUE ) || ( acadoIsFinite( lbTmp( j ) ) == BT_TRUE ) )
			{
				xBoundsIdx.push_back(i * lbTmp.getDim() + j);
			}
		}
	}

	if ( isFinite == BT_TRUE )
		xBounds = *(tmp.x);
	else
		xBounds.init();

	////////////////////////////////////////////////////////////////////////////
	//
	// Intermezzo - reset static counters
	//
	////////////////////////////////////////////////////////////////////////////

	DifferentialState dummy0;
	Control dummy1;
	dummy0.clearStaticCounters();
	dummy1.clearStaticCounters();

	DifferentialState vX( NX );
	Control vU( NU );

	////////////////////////////////////////////////////////////////////////////
	//
	// Extract path constraints; pac prefix
	//
	////////////////////////////////////////////////////////////////////////////

	conAuxVar.setName( "conAuxVar" );
	conAuxVar.setDataStruct( ACADO_WORKSPACE );

	Function pacH;

	constraints.getPathConstraints(pacH, pacLBMatrix, pacUBMatrix);

	dimPacH = pacH.getDim();

	if (dimPacH != 0)
	{
		Expression expPacH, expPacHx, expPacHu;
		pacH.getExpression( expPacH );

		expPacHx = forwardDerivative(expPacH, vX);
		expPacHu = forwardDerivative(expPacH, vU);

		Function pacHx, pacHu;
		pacHx << expPacHx;
		pacHu << expPacHu;

		// Set dimension of residual
		pacEvH.setup("evH", N * dimPacH, 1, REAL, ACADO_WORKSPACE);

		// Check derivative of path constraints w.r.t. x
		if (pacHx.getNX() == 0 && pacHx.getNU() == 0 && pacHx.getNP() == 0)
		{
			EvaluationPoint epPacHx( pacHx );
			Vector v = pacHx.evaluate( epPacHx );

			if (v.isZero() == BT_FALSE)
			{
				// Hard-code derivative evaluation

				Matrix m;
				m.init(dimPacH, NX);

				for (unsigned j = 0; j < dimPacH; ++j)
					for (unsigned k = 0; k < NX; ++k)
						m(j, k) = v(j * NX + k);

				pacEvHx.setup("evHx", m, REAL, ACADO_WORKSPACE);
			}
		}
		else
		{
			pacH << expPacHx;

			pacEvHx.setup("evHx", N * dimPacH, NX, REAL, ACADO_WORKSPACE);
		}

		// Check derivative of path constraints w.r.t. u
		if (pacHu.getNX() == 0 && pacHu.getNU() == 0 && pacHu.getNP() == 0)
		{
			EvaluationPoint epPacHu( pacHu );
			Vector v = pacHu.evaluate( epPacHu );

			if (v.isZero() == BT_FALSE)
			{
				// Hard-code derivative evaluation

				Matrix m;
				m.init(dimPacH, NU);

				for (unsigned j = 0; j < dimPacH; ++j)
					for (unsigned k = 0; k < NU; ++k)
						m(j, k) = v(j * NU + k);

				pacEvHu.setup("evHu", m, REAL, ACADO_WORKSPACE);
			}
		}
		else
		{
			pacH << expPacHu;

			pacEvHu.setup("evHu", N * dimPacH, NU, REAL, ACADO_WORKSPACE);
		}

		if (performsSingleShooting() == BT_FALSE)
		{
			pacEvHxd.setup("evHxd", dimPacH, 1, REAL, ACADO_WORKSPACE);
		}

		conAuxVar.setup("conAuxVar", pacH.getGlobalExportVariableSize(), 1, REAL, ACADO_WORKSPACE);
		conValueIn.setup("conValueIn", 1, NX + 0 + NU + NP, REAL, ACADO_WORKSPACE);
		conValueOut.setup("conValueOut", 1, pacH.getDim(), REAL, ACADO_WORKSPACE);

		evaluatePathConstraints.init(pacH, "evaluatePathConstraints", NX, 0, NU);
		evaluatePathConstraints.setGlobalExportVariable( conAuxVar );
	}

	////////////////////////////////////////////////////////////////////////////
	//
	// Extract point constraints; poc prefix
	//
	////////////////////////////////////////////////////////////////////////////

	Function pocH;
	Expression expPocH, expPocHx, expPocHu;
	Matrix pocLBMatrix, pocUBMatrix;

	evaluatePointConstraints.resize(N + 1);

	unsigned dimPocHMax = 0;

	pocLbStack.resize(N + 1);
	pocUbStack.resize(N + 1);

	// Setup the point constraints
	for (unsigned i = 0; i < N + 1; ++i)
	{
		// Get the point constraint
		constraints.getPointConstraint(i, pocH, pocLBMatrix, pocUBMatrix);

		// Extract and stack the point constraint if it exists
		if ( pocH.getDim() )
		{
			if (pocH.getNU() > 0 && i == N)
			{
				return ACADOERRORTEXT(RET_INVALID_ARGUMENTS, "The terminal (point) constraint must not depend on controls.");
			}

			// Extract the function expression and stack its Jacobians w.r.t.
			// x and u
			pocH.getExpression( expPocH );

			// XXX AFAIK, this is not bullet-proof!
//			if (expPocH.getVariableType() != VT_INTERMEDIATE_STATE)
			if (expPocH.getVariableType() != VT_UNKNOWN && expPocH.getVariableType() != VT_INTERMEDIATE_STATE)
				continue;

			expPocHx = forwardDerivative(expPocH, vX);
			pocH << expPocHx;

			if (i < N)
			{
				expPocHu = forwardDerivative(expPocH, vU);
				pocH << expPocHu;
			}

			// Stack the new function
			evaluatePointConstraints[ i ] = std::tr1::shared_ptr< ExportAcadoFunction >(new ExportAcadoFunction);

			String pocFName;

			pocFName = "evaluatePointConstraint";
			pocFName << String( i );

			if (i < N)
			{
				evaluatePointConstraints[ i ]->init(pocH, pocFName, NX, 0, NU);
			}
			else
			{
				evaluatePointConstraints[ i ]->init(pocH, pocFName, NX, 0, 0);
			}

			// Determine the maximum function dimension
			if ( dimPocHMax < (unsigned)pocH.getDim() )
			{
				dimPocHMax =  pocH.getDim();
			}

			// TODO This is too specific for condensing, thus should be moved to condensing class.
			// Stack the lower and upper bounds
			pocLB.append( pocLBMatrix.getRow( 0 ) );
			pocUB.append( pocUBMatrix.getRow( 0 ) );

			pocLbStack[ i ] = pocLBMatrix.getRow( 0 );
			pocUbStack[ i ] = pocUBMatrix.getRow( 0 );
		}
	}

//	std::cout << "lb dim: " << pocLB.getDim() << std::endl;
//	std::cout << "ub dim: " << pocUB.getDim() << std::endl;

	dimPocH = pocLB.getDim();

	if ( dimPocH != 0 )
	{
		unsigned pocAuxVarDim = 0;

		ExportVariable pocAuxVarTemp;

		for (unsigned i = 0; i < evaluatePointConstraints.size(); ++i)
		{
			if ( !evaluatePointConstraints[ i ] )
				continue;

			pocAuxVarTemp = evaluatePointConstraints[ i ]->getGlobalExportVariable();

			pocAuxVarDim = pocAuxVarDim < pocAuxVarTemp.getDim() ? pocAuxVarTemp.getDim() : pocAuxVarDim;

			evaluatePointConstraints[ i ]->setGlobalExportVariable( conAuxVar );
		}

		int conAuxVarDim =
				(conAuxVar.getDim() < pocAuxVarDim) ? pocAuxVarDim : conAuxVar.getDim();
		conAuxVar.setup("conAuxVar", conAuxVarDim, 1, REAL, ACADO_WORKSPACE);

		conValueIn.setup("conValueIn", 1, NX + 0 + NU + NP, REAL, ACADO_WORKSPACE);

		unsigned conValueOutDim =
				(dimPocHMax < conValueOut.getDim()) ? conValueOut.getDim() : dimPocHMax;
		conValueOut.setup("conValueOut", 1, conValueOutDim, REAL, ACADO_WORKSPACE);

		pocEvH.setup("pocEvH", dimPocH, 1, REAL, ACADO_WORKSPACE);
		pocEvHx.setup("pocEvHx", dimPocH, NX, REAL, ACADO_WORKSPACE);

		// For this guy we actually need less... but no worry for now
		pocEvHu.setup("pocEvHu", dimPocH, NU, REAL, ACADO_WORKSPACE);

		// Setup one more variable for MS:
		if (performsSingleShooting() == BT_FALSE)
		{
			pocEvHxd.setup("pocEvHxd", dimPocH, 1, REAL, ACADO_WORKSPACE);
		}
	}

	return SUCCESSFUL_RETURN;
}

unsigned ExportNLPSolver::getNumComplexConstraints( void )
{
	return N * dimPacH + dimPocH;
}

BooleanType ExportNLPSolver::initialStateFixed() const
{
	int fixInitialState;
	get(FIX_INITIAL_STATE, fixInitialState);

	return (BooleanType)fixInitialState;
}

returnValue ExportNLPSolver::setupAuxiliaryFunctions()
{
	////////////////////////////////////////////////////////////////////////////
	//
	// Shift controls
	//
	////////////////////////////////////////////////////////////////////////////
	ExportVariable uEnd("uEnd", NU, 1, REAL, ACADO_LOCAL);
	uEnd.setDoc( "Value for the u vector on the second to last node. If =0 the old value is used." );
	ExportIndex index( "index" );
	shiftControls.setup("shiftControls", uEnd);
	shiftControls.addIndex( index );
	shiftControls.doc( "Shift controls vector by one interval." );

	ExportForLoop uLoop(index, 0, N - 1);
	uLoop.addStatement( u.getRow( index ) == u.getRow(index + 1) );

	shiftControls.addStatement( uLoop );
	shiftControls.addLinebreak( );
	shiftControls.addStatement( "if (uEnd != 0)\n{\n" );
	shiftControls.addStatement(u.getRow(N - 1) == uEnd.getTranspose());
	shiftControls.addStatement( "}\n" );

	////////////////////////////////////////////////////////////////////////////
	//
	// Shift states
	//
	////////////////////////////////////////////////////////////////////////////
	ExportVariable xEnd("xEnd", NX, 1, REAL, ACADO_LOCAL);
	xEnd.setDoc( "Value for the x vector on the last node. If =0 the old value is used." );
	ExportIndex strategy( "strategy" );
	strategy.setDoc( (String)"Shifting strategy: 1. Initialize node "<< N + 1 << " with xEnd." \
			" 2. Initialize node " << N + 1 << " by forward simulation." );
	// TODO Think about adding zEnd here at some point...
	shiftStates.setup("shiftStates", strategy.makeArgument(), xEnd, uEnd);
	shiftStates.addIndex( index );
	if (NXA == 0)
		shiftStates.doc( "Shift differential variables vector by one interval." );
	else
		shiftStates.doc( "Shift differential variables vector and algebraic variables vector by one interval." );

	ExportForLoop xLoop(index, 0, N);
	xLoop.addStatement( x.getRow( index ) == x.getRow(index + 1) );
	shiftStates.addStatement( xLoop );

	if (NXA > 0)
	{
		ExportForLoop zLoop(index, 0, N - 1);
		zLoop.addStatement( z.getRow( index ) == z.getRow(index + 1) );
		shiftStates.addStatement( zLoop );
	}


	shiftStates.addLinebreak( );
	shiftStates.addStatement( "if (strategy == 1 && xEnd != 0)\n{\n" );
	shiftStates.addStatement( x.getRow( N ) == xEnd.getTranspose() );
	shiftStates.addStatement( "}\n" );
	shiftStates.addStatement( "else if (strategy == 2) \n{\n" );

	unsigned indexZ   = NX + NXA;
	unsigned indexGxx = indexZ + NX * NX;
	unsigned indexGzx = indexGxx + NXA * NX;
	unsigned indexGxu = indexGzx + NX * NU;
	unsigned indexGzu = indexGxu + NXA * NU;
	unsigned indexU   = indexGzu + NU;
	unsigned indexP   = indexU + NP;

	shiftStates.addStatement( state.getCols(0, NX) == x.getRow( N ) );
	shiftStates.addStatement( state.getCols(NX, NX + NXA) == z.getRow(N - 1) );
	shiftStates.addStatement( "if (uEnd != 0)\n{\n" );
	shiftStates.addStatement( state.getCols(indexGzu, indexU) == uEnd.getTranspose() );
	shiftStates.addStatement( "}\n" );
	shiftStates.addStatement( "else\n{\n" );
	shiftStates.addStatement( state.getCols(indexGzu, indexU) == u.getRow(N - 1) );
	shiftStates.addStatement( "}\n" );
	shiftStates.addStatement( state.getCols(indexU, indexP) == p );
	shiftStates.addLinebreak( );

	if ( integrator->equidistantControlGrid() )
	{
		shiftStates.addStatement( (String)"integrate"
				<< "(" << state.getFullName() << ", 1);\n"  );
	}
	else
	{
		shiftStates.addStatement( (String)"integrate"
				<< "(" << state.getFullName() << ", 1, " << N - 1 << ");\n" );
	}

	shiftStates.addLinebreak( );
	shiftStates.addStatement( x.getRow( N ) == state.getCols(0, NX) );
	if ( NXA )
	{
		shiftStates.addLinebreak();
		shiftStates.addStatement(z.getRow(N - 1) == state.getCols(NX, NX + NXA));
	}


	shiftStates.addStatement( "}\n" );

	////////////////////////////////////////////////////////////////////////////
	//
	// Initialize nodes by a forward simulation
	//
	////////////////////////////////////////////////////////////////////////////
	initializeNodes.setup("initializeNodesByForwardSimulation");
	initializeNodes.addIndex( index );
	initializeNodes.doc( "Initialize shooting nodes by a forward simulation starting from the first node." );

	ExportForLoop iLoop(index, 0, N);


	iLoop.addStatement( state.getCols(0, NX)		== x.getRow( index ) );
	if ( NXA )
	{
		iLoop << String("if (") << index.getFullName() << String(" > 0){");
		iLoop.addStatement( state.getCols(NX, NX + NXA)	== z.getRow(index - 1) );
		iLoop << String("}\n");
	}
	iLoop.addStatement( state.getCols(indexGzu, indexU)	== u.getRow( index ) );
	iLoop.addStatement( state.getCols(indexU, indexP)	== p );
	iLoop.addLinebreak( );

	if ( integrator->equidistantControlGrid() )
	{
		iLoop.addStatement( (String)"integrate"
				<< "(" << state.getFullName() << ", "
				<< index.getFullName() << " == 0"
				<< ");\n"  );
	}
	else
	{
		iLoop.addStatement( (String)"integrate"
				<< "(" << state.getFullName() << ", "
				<< index.getFullName() << " == 0"
				<< ", " << index.getFullName() << ");\n" );
	}

	iLoop.addLinebreak();
	iLoop.addStatement( x.getRow(index + 1) == state.getCols(0, NX) );

	// Store improved initial guess from the integrator
	iLoop.addStatement( z.getRow(index) == state.getCols(NX, NX + NXA) );

	initializeNodes.addStatement( iLoop );

	////////////////////////////////////////////////////////////////////////////
	//
	// Objective value calculation
	//
	////////////////////////////////////////////////////////////////////////////

	getObjective.setup( "getObjective" );
	getObjective.doc( "Calculate the objective value." );
	ExportVariable objVal("objVal", 1, 1, REAL, ACADO_LOCAL, BT_TRUE);
	objVal.setDoc( "Value of the objective function." );
	getObjective.setReturnValue( objVal );

	ExportVariable tmpDx("tmpDx", 1, NX, REAL, ACADO_LOCAL );
	ExportVariable tmpDy("tmpDy", 1, getNY(), REAL, ACADO_LOCAL );
	ExportVariable tmpDyN("tmpDyN", 1, getNYN(), REAL, ACADO_LOCAL );

	getObjective.addVariable( tmpDy );
	getObjective.addVariable( tmpDyN );

	ExportIndex oInd;
	getObjective.acquire( oInd );

	// Recalculate objective

	ExportForLoop loopObjective(oInd, 0, N);

	loopObjective.addStatement( objValueIn.getCols(0, getNX()) == x.getRow( oInd ) );
	loopObjective.addStatement( objValueIn.getCols(NX, NX + NU) == u.getRow( oInd ) );
	loopObjective.addStatement( objValueIn.getCols(NX + NU, NX + NU + NP) == p );
	loopObjective.addLinebreak( );

	// Evaluate the objective function
	if (externObjective == BT_FALSE)
		loopObjective.addFunctionCall(evaluateLSQ, objValueIn, objValueOut);
	else
		loopObjective.addFunctionCall( evaluateExternLSQ, objValueIn, objValueOut );

	// Stack the measurement function value
	loopObjective.addStatement(
			Dy.getRows(oInd * NY, (oInd + 1) * NY) ==
					objValueOut.getTranspose().getRows(0, getNY()) - y.getRows(oInd * NY, (oInd + 1) * NY)
	);

	getObjective.addStatement( loopObjective );

	getObjective.addStatement( objValueIn.getCols(0, NX) == x.getRow( N ) );
	getObjective.addStatement( objValueIn.getCols(NX, NX + NP) == p );

	// Evaluate the objective function
	if (externObjective == BT_FALSE)
		getObjective.addFunctionCall(evaluateLSQEndTerm, objValueIn, objValueOut);
	else
		getObjective.addFunctionCall( evaluateExternLSQEndTerm, objValueIn, objValueOut );
	getObjective.addStatement( DyN.getTranspose() == objValueOut.getCols(0, NYN) - yN.getTranspose() );

	getObjective.addStatement( objVal == 0 );

	ExportForLoop oLoop(oInd, 0, N);

	int variableObjS;
	get(CG_USE_VARIABLE_WEIGHTING_MATRIX, variableObjS);

	if (variableObjS == NO)
	{
		oLoop.addStatement( tmpDy == Dy.getTranspose().getCols(oInd * NY, (oInd + 1) * NY) * objS );
		oLoop.addStatement( objVal += Dy.getTranspose().getCols(oInd * NY, (oInd + 1) * NY) * tmpDy.getTranspose() );
	}
	else
	{
		oLoop.addStatement( tmpDy == Dy.getTranspose().getCols(oInd * NY, (oInd + 1) * NY) * objS.getSubMatrix(oInd * NY, (oInd + 1) * NY, 0, NY) );
		oLoop.addStatement( objVal += Dy.getTranspose().getCols(oInd * NY, (oInd + 1) * NY) * tmpDy.getTranspose() );
	}

	getObjective.addStatement( oLoop );
	getObjective.addLinebreak( );

	getObjective.addStatement( tmpDyN == DyN.getTranspose() * objSEndTerm );
	getObjective.addStatement( objVal += DyN.getTranspose() * tmpDyN.getTranspose() );

	if ( SAC.getDim() > 0 )
	{
		getObjective.addVariable( tmpDx );
		getObjective.addStatement( tmpDx == DxAC.getTranspose() * SAC );
		getObjective.addStatement( objVal +=  tmpDx * DxAC );
	}
	getObjective.addLinebreak( );

	getObjective.addStatement( "objVal *= 0.5;\n" );

	return SUCCESSFUL_RETURN;
}

unsigned ExportNLPSolver::weightingMatricesType( void ) const
{
	if (objS.isGiven() == BT_TRUE && objSEndTerm.isGiven() == BT_TRUE)
		return 0;

	// get the option for variable objS matrix.
	int variableObjS;
	get(CG_USE_VARIABLE_WEIGHTING_MATRIX, variableObjS);
	if ( variableObjS )
		return 2;

	return 1;
}

returnValue ExportNLPSolver::setupArrivalCostCalculation()
{
	int useArrivalCost;
	get(CG_USE_ARRIVAL_COST, useArrivalCost);
	if (useArrivalCost == NO)
		return SUCCESSFUL_RETURN;

	ExportVariable evRet("ret", 1, 1, INT, ACADO_LOCAL, BT_TRUE);

	ExportVariable evReset("reset", 1, 1, INT, ACADO_LOCAL, BT_TRUE);
	evReset.setDoc("Reset S_{AC}. Set it to 1 to initialize arrival cost calculation, "
				   "and later should set it to 0.");

	updateArrivalCost.init("updateArrivalCost", evReset);
	updateArrivalCost.doc("Use this function to update the arrival cost.");
	updateArrivalCost.setReturnValue( evRet );
	updateArrivalCost << (evRet == 0);

	const unsigned AM = 2 * NX + NY;
	const unsigned AN = 2 * NX + NU; // A bit different from my implementation

	acA.setup("acA", AM, AN, REAL, ACADO_WORKSPACE);
	acb.setup("acb", AM, 1,  REAL, ACADO_WORKSPACE);
	acP.setup("acP", NX, NX, REAL, ACADO_WORKSPACE);

	acWL.setup("WL", NX, NX, REAL, ACADO_VARIABLES);
	acWL.setDoc("Arrival cost term: Cholesky decomposition, lower triangular, "
				" of the inverse of the state noise covariance matrix.");
	acVL.setup("acVL", NY, NY, REAL, ACADO_WORKSPACE);

	acHx.setup("acHx", NY, NX, REAL, ACADO_WORKSPACE);
	acHu.setup("acHu", NY, NU, REAL, ACADO_WORKSPACE);
	acXx.setup("acXx", NX, NX, REAL, ACADO_WORKSPACE);
	acXu.setup("acXu", NX, NU, REAL, ACADO_WORKSPACE);

	acXTilde.setup("acXTilde", NX, 1, REAL, ACADO_WORKSPACE);
	acHTilde.setup("acHTilde", NY, 1, REAL, ACADO_WORKSPACE);

	//
	// Perform a hard reset if necessary
	// This is a bit messy because the update code needs upper triangular P
	// matrix, but Cholesky function returns lower triangular. One way to
	// avoid this is to be add an option to ExportCholeskyDecomposition to
	// ---> export lower or upper triangular matrix.
	//
	cholSAC.init("cholSAC", NX);
	cholSAC.setup();

	updateArrivalCost.addStatement(
			String( "\nif ( " ) << evReset.getName() << " )\n{\n"
	);
	updateArrivalCost.addStatement( acXx == SAC );
	updateArrivalCost.addFunctionCall(cholSAC.getName(), acXx);
	updateArrivalCost << (acP == acXx.getTranspose());
	updateArrivalCost << String( "return 0;\n}\n\n" );

	//
	// Evaluate model @ the first node
	//

	unsigned indexZ   = NX + NXA;
	unsigned indexGxx = indexZ + NX * NX;
	unsigned indexGzx = indexGxx + NXA * NX;
	unsigned indexGxu = indexGzx + NX * NU;
	unsigned indexGzu = indexGxu + NXA * NU;
	unsigned indexU   = indexGzu + NU;
	unsigned indexP   = indexU + NP;

	updateArrivalCost.addStatement( state.getCols(0, NX) == x.getRow( 0 ) );
	updateArrivalCost.addStatement( state.getCols(NX, NX + NXA) == z.getRow( 0 ) );
	updateArrivalCost.addStatement( state.getCols(indexGzu, indexU) == u.getRow( 0 ) );
	updateArrivalCost.addStatement( state.getCols(indexU, indexP) == p );

	if (integrator->equidistantControlGrid())
		updateArrivalCost.addStatement(
				(String) "integrate" << "(" << state.getFullName() << ", 1);\n");
	else
		updateArrivalCost.addStatement(
				(String) "integrate" << "(" << state.getFullName() << ", 1, " << 0 << ");\n");
	updateArrivalCost.addLinebreak( );

	//
	// Evaluate objective function @ the first node
	//

	updateArrivalCost.addStatement( objValueIn.getCols(0, getNX()) == x.getRow( 0 ) );
	updateArrivalCost.addStatement( objValueIn.getCols(NX, NX + NU) == u.getRow( 0 ) );
	updateArrivalCost.addStatement( objValueIn.getCols(NX + NU, NX + NU + NP) == p );

	if (externObjective == BT_FALSE)
		updateArrivalCost.addFunctionCall(evaluateLSQ, objValueIn, objValueOut);
	else
		updateArrivalCost.addFunctionCall(evaluateExternLSQ, objValueIn, objValueOut);
	updateArrivalCost.addLinebreak( );

	//
	// Cholesky decomposition of the term objS
	//
	if (objS.isGiven() == BT_TRUE)
	{
		Matrix m = objS.getGivenMatrix();
		Matrix mChol = m.getCholeskyDecomposition();

		initialize << (acVL == mChol);
	}
	else
	{
		cholObjS.init("cholObjS", NY);
		cholObjS.setup();

		int variableObjS;
		get(CG_USE_VARIABLE_WEIGHTING_MATRIX, variableObjS);

		if ( variableObjS )
		{
			updateArrivalCost << (acVL == objS.getSubMatrix(0, NY, 0, NY));
			updateArrivalCost.addFunctionCall(cholObjS.getName(), acVL);
		}
		else
		{
			updateArrivalCost << (acVL == objS);
			updateArrivalCost.addFunctionCall(cholObjS.getName(), acVL);
		}
	}

	//
	// Create acA and acb
	//

	/*
		PL is square root form already.

	    	A = ||PL         zeros(nx, nu)  zeros(nx, nx) ||
                ||-VL * Hx   -VL * Hu       zeros(ny, nx) ||
                ||-WL * Xx   -WL * Xu       WL            ||

        Since A is initialized to 0, we should use use -= operator
	 */

	// Clear A and b
	updateArrivalCost
		<< (acA == zeros(AM, AN))
		<< (acb == zeros(AM, 1))
		<< String( "\n" );

	// Copy products to the matrices
	updateArrivalCost
		<< (acXx.makeRowVector() == state.getCols(indexZ, indexGxx))
		<< (acXu.makeRowVector() == state.getCols(indexGzx, indexGxu));

	unsigned ind = NY;
	if (objEvFx.isGiven() == BT_TRUE)
	{
		initialize << (acHx == objEvFx);
	}
	else
	{
		updateArrivalCost << (acHx.makeRowVector() == objValueOut.getCols(ind, ind + NY * NX));
		ind += NY * NX;
	}

	if (objEvFu.isGiven() == BT_TRUE)
	{
		initialize << (acHu == objEvFu);
	}
	else
	{
		updateArrivalCost << (acHu.makeRowVector() == objValueOut.getCols(ind, ind + NY * NU));
	}

	// acVL and acWL are lower triangular matrices
	// acP is ALWAYS upper triangular!

	updateArrivalCost
		<< (acA.getSubMatrix(0, NX, 0, NX) == acP)

		<< (acA.getSubMatrix(NX, NX + NY, 0, NX) -= (acVL ^ acHx))
		<< (acA.getSubMatrix(NX, NX + NY, NX, NX + NU) -= (acVL ^ acHu))

		<< (acA.getSubMatrix(NX + NY, NX + NY + NX, 0, NX) -= (acWL ^ acXx))
		<< (acA.getSubMatrix(NX + NY, NX + NY + NX, NX, NX + NU) -= (acWL ^ acXu))
		<< (acA.getSubMatrix(NX + NY, NX + NY + NX, NX + NU, NX + NU + NX) == acWL.getTranspose());

	/*

	x1 is output from the integrator
	h  is evaluated obj @ node 0
	x and u are current solutions
	xL and uL are previous arrival cost values.

	x_tilde = x1 - np.dot(Xx,x) - np.dot(Xu,u)
    h_tilde =  h - np.dot(Hx,x) - np.dot(Hu,u)

	res = np.bmat([ -np.dot(PL, xL),
                     np.dot(VL, yL - h_tilde),
                    -np.dot(WL, x_tilde) ])

	 */

	updateArrivalCost
		<< (acXTilde == state.getTranspose().getRows(0, NX) - acXx * x.getRow( 0 ).getTranspose())
		<< (acXTilde -= acXu * u.getRow( 0 ).getTranspose());

	updateArrivalCost
		<< (acHTilde == y.getRows(0, NY) - objValueOut.getTranspose().getRows(0, NY))
		<< (acHTilde += acHx * x.getRow( 0 ).getTranspose())
		<< (acHTilde += acHu * u.getRow( 0 ).getTranspose());

	// Inverted signs from Mario's implementation
	updateArrivalCost
		<< (acb.getRows(0, NX) == (acP * xAC))
		<< (acb.getRows(NX, NX + NY) -= (acVL ^ acHTilde))
		<< (acb.getRows(NX + NY, NX + NY + NX) == (acWL ^ acXTilde));

	//
	// Solver the linear system
	// We need first NX back-solves to get solution of this linear system...
	//
	acSolver.init(AM, AN, NX, BT_FALSE, BT_FALSE, String("ac"));
	acTmp = acSolver.getGlobalExportVariable( 1 );
	updateArrivalCost.addFunctionCall(acSolver.getNameSolveFunction(), acA, acb, acTmp);

	//
	// Get the solution of the linear system
	//

	updateArrivalCost << (xAC == acb.getRows(NX + NU, 2 * NX + NU));

	// Get the update acP, upper triangular part
	for (unsigned row = 0; row < NX; ++row)
		for (unsigned col = row; col < NX; ++col)
			updateArrivalCost << (acP.getElement(row, col) == acA.getElement(NX + NU + row, NX + NU + col));

	// Calculate the weighting matrix which is used outside.
	updateArrivalCost << (SAC == (acP ^ acP));

	return SUCCESSFUL_RETURN;
}

CLOSE_NAMESPACE_ACADO
