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
 *    \file include/acado/code_generation/export_exact_hessian_cn2_new.hpp
 *    \authors Rien Quirynen
 *    \date 2014
 */

#ifndef ACADO_TOOLKIT_EXPORT_EXACT_HESSIAN_CN2_NEW_HPP
#define ACADO_TOOLKIT_EXPORT_EXACT_HESSIAN_CN2_NEW_HPP

#include <acado/code_generation/export_nlp_solver.hpp>
#include <acado/code_generation/export_gauss_newton_cn2_new.hpp>

BEGIN_NAMESPACE_ACADO

/**
 *	\brief TBD
 *
 *	\ingroup NumericalAlgorithms
 *
 *	\authors Rien Quirynen
 *
 *	\note Early experimental implementation
 */
class ExportExactHessianCN2New : public ExportGaussNewtonCN2New
{
public:

	/** Default constructor.
	 *
	 *	@param[in] _userInteraction		Pointer to corresponding user interface.
	 *	@param[in] _commonHeaderName	Name of common header file to be included.
	 */
	ExportExactHessianCN2New(	UserInteraction* _userInteraction = 0,
								const std::string& _commonHeaderName = ""
								);

	/** Destructor. */
	virtual ~ExportExactHessianCN2New( )
	{}

	/** Initializes export of an algorithm.
	 *
	 *	\return SUCCESSFUL_RETURN
	 */
	virtual returnValue setup( );


	/** Exports source code of the auto-generated condensing algorithm
	 *  into the given directory.
	 *
	 *	@param[in] code				Code block containing the auto-generated condensing algorithm.
	 *
	 *	\return SUCCESSFUL_RETURN
	 */
	virtual returnValue getCode(	ExportStatementBlock& code
									);

	/** Adds all function (forward) declarations of the auto-generated condensing algorithm
	 *	to given list of declarations.
	 *
	 *	@param[in] declarations		List of declarations.
	 *
	 *	\return SUCCESSFUL_RETURN
	 */
	virtual returnValue getFunctionDeclarations(	ExportStatementBlock& declarations
													) const;

protected:

	/** Setting up of an objective evaluation:
	 *   - functions and derivatives evaulation
	 *   - creating Hessians and gradients
	 *
	 *   \return SUCCESSFUL_RETURN
	 */
	virtual returnValue setupObjectiveEvaluation( void );

	/** Setting up of a hessian regularization:
	 *
	 *   \return SUCCESSFUL_RETURN
	 */
	virtual returnValue setupHessianRegularization( void );

	/** Setup the function for evaluating the actual objective value. */
	virtual returnValue setupGetObjective();

	/** Exports source code containing the evaluation routines of the algorithm.
	 *
	 *	\return SUCCESSFUL_RETURN
	 */
	virtual returnValue setupEvaluation( );

protected:

	ExportFunction regularizeHessian;
	ExportFunction regularization;
	ExportFunction evaluateObjective;

};

CLOSE_NAMESPACE_ACADO

#endif  // ACADO_TOOLKIT_EXPORT_EXACT_HESSIAN_CN2_NEW_HPP
