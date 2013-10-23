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
 *    \file include/acado/code_generation/export_gauss_newton_cn2.hpp
 *    \authors Milan Vukov, Joel Andersson
 *    \date 2013
 */

#ifndef ACADO_TOOLKIT_EXPORT_GAUSS_NEWTON_CN2_HPP
#define ACADO_TOOLKIT_EXPORT_GAUSS_NEWTON_CN2_HPP

#include <acado/code_generation/export_nlp_solver.hpp>

BEGIN_NAMESPACE_ACADO

/**
 *	\brief TBD
 *
 *	\ingroup NumericalAlgorithms
 *
 *	\authors Milan Vukov, Joel Andersson
 *
 *	\note Early experimental implementation
 */
class ExportGaussNewtonCN2 : public ExportNLPSolver
{
public:

	/** Default constructor.
	 *
	 *	@param[in] _userInteraction		Pointer to corresponding user interface.
	 *	@param[in] _commonHeaderName	Name of common header file to be included.
	 */
	ExportGaussNewtonCN2(	UserInteraction* _userInteraction = 0,
								const String& _commonHeaderName = ""
								);

	/** Destructor. */
	virtual ~ExportGaussNewtonCN2( )
	{}

	/** Initializes export of an algorithm.
	 *
	 *	\return SUCCESSFUL_RETURN
	 */
	virtual returnValue setup( );

	/** Adds all data declarations of the auto-generated condensing algorithm
	 *	to given list of declarations.
	 *
	 *	@param[in] declarations		List of declarations.
	 *
	 *	\return SUCCESSFUL_RETURN
	 */
	virtual returnValue getDataDeclarations(	ExportStatementBlock& declarations,
												ExportStruct dataStruct = ACADO_ANY
												) const;

	/** Adds all function (forward) declarations of the auto-generated condensing algorithm
	 *	to given list of declarations.
	 *
	 *	@param[in] declarations		List of declarations.
	 *
	 *	\return SUCCESSFUL_RETURN
	 */
	virtual returnValue getFunctionDeclarations(	ExportStatementBlock& declarations
													) const;


	/** Exports source code of the auto-generated condensing algorithm
	 *  into the given directory.
	 *
	 *	@param[in] code				Code block containing the auto-generated condensing algorithm.
	 *
	 *	\return SUCCESSFUL_RETURN
	 */
	virtual returnValue getCode(	ExportStatementBlock& code
									);


	/** Returns number of variables in underlying QP.
	 *
	 *  \return Number of variables in underlying QP
	 */
	unsigned getNumQPvars( ) const;

	/** Returns number of bounds on differential states.
	 *
	 *  \return Number of bounds on differential states
	 */
	virtual unsigned getNumStateBounds( ) const;

protected:

	/** Setting up of an objective evaluation:
	 *   - functions and derivatives evaulation
	 *   - creating Hessians and gradients
	 *
	 *   \return SUCCESSFUL_RETURN
	 */
	virtual returnValue setupObjectiveEvaluation( void );

	/** Set-up evaluation of constraints
	 *   - box constraints on states and controls
	 *
	 *  \return SUCCESSFUL_RETURN
	 */
	virtual returnValue setupConstraintsEvaluation( void );

	/** Initialization of all member variables.
	 *
	 *	\return SUCCESSFUL_RETURN
	 */
	virtual returnValue setupVariables( );

	/** Exports source code containing the multiplication routines of the algorithm.
	 *
	 *	\return SUCCESSFUL_RETURN
	 */
	virtual returnValue setupMultiplicationRoutines( );

	/** Exports source code containing the evaluation routines of the algorithm.
	 *
	 *	\return SUCCESSFUL_RETURN
	 */
	virtual returnValue setupEvaluation( );

	virtual returnValue setupQPInterface( );

	virtual returnValue setupCondensing( );

	BooleanType performFullCondensing( ) const;

protected:

	ExportFunction evaluateObjective;

	ExportVariable x0, Dx0;

	ExportFunction setObjQ1Q2;
	ExportFunction setObjR1R2;
	ExportFunction setObjQN1QN2;

	/** Variable containing the QP Hessian matrix. */
	ExportVariable H, H00, H10, H11;
	/** Variable containing the QP constraint matrix. */
	ExportVariable A;

	/** Variable containing the QP gradient. */
	ExportVariable g, g0, g1;

	/** Variable containing the lower limits on QP variables. */
	ExportVariable lb;

	/** Variable containing the upper limits on QP variables. */
	ExportVariable ub;

	/** Variable containing lower limits on QP constraints. */
	ExportVariable lbA;

	/** Variable containing upper limits on QP constraints. */
	ExportVariable ubA;

	/** Variable containing the primal QP variables. */
	ExportVariable xVars;

	/** Variable containing the dual QP variables. */
	ExportVariable yVars;

	ExportVariable lbValues, ubValues;
	ExportVariable lbAValues, ubAValues;

	ExportVariable Qd;

	ExportFunction condensePrep;
	ExportFunction condenseFdb;
	ExportFunction expand;

	ExportVariable T, E, QE, QGx, QDy;

	ExportFunction multGxd;
	ExportFunction moveGxT;
	ExportFunction multGxGx;
	ExportFunction multGxGu;
	ExportFunction moveGuE;
	ExportFunction setBlockH11;
	ExportFunction zeroBlockH11;
	ExportFunction copyHTH;
	ExportFunction multQ1d;
	ExportFunction multQN1d;
	ExportFunction multRDy;
	ExportFunction multQDy;
	ExportFunction multEQDy;
	ExportFunction multQETGx;
	ExportFunction zeroBlockH10;
	ExportFunction multEDu;
	ExportFunction multQ1Gx;
	ExportFunction multQN1Gx;
	ExportFunction multQ1Gu;
	ExportFunction multQN1Gu;
	ExportFunction zeroBlockH00;
	ExportFunction multCTQC;

	ExportVariable A10;
	ExportVariable A20;
	ExportVariable pacA01Dx0;
	ExportVariable pocA02Dx0;
	ExportFunction multHxC;
	ExportFunction multHxE;
	ExportFunction macHxd;

	ExportFunction macCTSlx;
	ExportFunction macETSlu;

	ExportFunction preparation;
	ExportFunction feedback;

	ExportFunction getKKT;

	// N2 condensing related
	ExportVariable W1, W2;
	ExportVariable sbar, w1, w2;

	ExportFunction multBTW1, multGxTGu, macQEW2;
	ExportFunction macATw1QDy, macBTw1, macQSbarW2, macASbar, macASbarD2;
	ExportFunction expansionStep;

};

/** \internal */
static struct RegisterGaussNewtonCN2
{
	RegisterGaussNewtonCN2();
} registerGaussNewtonCN2;

CLOSE_NAMESPACE_ACADO

#endif  // ACADO_TOOLKIT_EXPORT_GAUSS_NEWTON_CN2_HPP
