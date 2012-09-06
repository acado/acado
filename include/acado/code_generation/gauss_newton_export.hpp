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
 *    \file include/acado/code_generation/gauss_newton_export.hpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010-2011
 */


#ifndef ACADO_TOOLKIT_GAUSS_NEWTON_EXPORT_HPP
#define ACADO_TOOLKIT_GAUSS_NEWTON_EXPORT_HPP

#include <acado/utils/acado_utils.hpp>
#include <acado/user_interaction/options.hpp>
#include <acado/matrix_vector/matrix_vector.hpp>
#include <acado/function/function.hpp>
#include <acado/code_generation/export_algorithm.hpp>
#include <acado/code_generation/condensing_export.hpp>

BEGIN_NAMESPACE_ACADO


/** 
 *	\brief Allows to export a tailored Gauss-Newton algorithm for fast model predictive control.
 *
 *	\ingroup NumericalAlgorithms
 *
 *  The class GaussNewtonExport allows to export a tailored Gauss-Newton 
 *  algorithm for fast model predictive control.
 *
 *	\author Hans Joachim Ferreau, Boris Houska
 */
class GaussNewtonExport : public ExportAlgorithm
{
    //
    // PUBLIC MEMBER FUNCTIONS:
    //
    public:

		/** Default constructor. 
		 *
		 *	@param[in] _userInteraction		Pointer to corresponding user interface.
		 *	@param[in] _commonHeaderName	Name of common header file to be included.
		 */
        GaussNewtonExport(	UserInteraction* _userInteraction = 0,
							const String& _commonHeaderName = ""
							);

		/** Copy constructor (deep copy).
		 *
		 *	@param[in] arg		Right-hand side object.
		 */
        GaussNewtonExport(	const GaussNewtonExport& arg
							);

        /** Destructor. 
		 */
        virtual ~GaussNewtonExport( );

		/** Assignment operator (deep copy).
		 *
		 *	@param[in] arg		Right-hand side object.
		 */
        GaussNewtonExport& operator=(	const GaussNewtonExport& arg
										);


		/** Initializes export of a tailored Gauss-Newton algorithm.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		virtual returnValue setup( );


		/** Assigns module for exporting a tailored condesing algorithm.
		 *
		 *	@param[in] _condenser	Condensing module.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
        returnValue setCondensingExport(	CondensingExport* const _condenser
											);

		/** Assigns lower/upper bounds on control inputs.
		 *
		 *	@param[in] _uBounds		Bounds on control inputs.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue setControlBounds(	const VariablesGrid& _uBounds
										);


		/** Adds all data declarations of the auto-generated Gauss-Newton algorithm 
		 *	to given list of declarations.
		 *
		 *	@param[in] declarations		List of declarations.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		virtual returnValue getDataDeclarations(	ExportStatementBlock& declarations,
													ExportStruct dataStruct = ACADO_ANY
													) const;

		/** Adds all function (forward) declarations of the auto-generated Gauss-Newton algorithm 
		 *	to given list of declarations.
		 *
		 *	@param[in] declarations		List of declarations.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		virtual returnValue getFunctionDeclarations(	ExportStatementBlock& declarations
														) const;


		/** Exports source code of the auto-generated Gauss-Newton algorithm 
		 *  into the given directory.
		 *
		 *	@param[in] code				Code block containing the auto-generated Gauss-Newton algorithm.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		virtual returnValue getCode(	ExportStatementBlock& code
										);


		/** Returns whether initial value of the differential states is fixed.
		 *
		 *	\return BT_TRUE  iff initial value is fixed, \n
		 *	        BT_FALSE otherwise
		 */
		BooleanType isInitialStateFixed( ) const;


		/** Returns number of variables in underlying QP.
		 *
		 *  \return Number of variables in underlying QP
		 */
		uint getNumQPvars( ) const;

		/** Returns number of bounds on differential states.
		 *
		 *  \return Number of bounds on differential states
		 */
		uint getNumStateBounds( ) const;


	protected:

		/** Copies all class members from given object.
		 *
		 *	@param[in] arg		Right-hand side object.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue copy(	const GaussNewtonExport& arg
							);

		/** Frees internal dynamic memory to yield an empty function.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue clear( );


    protected:

		CondensingExport* condenser;				/**< Module for exporting a tailored condensing algorithm. */
		VariablesGrid uBounds;						/**< Stores lower/upper bounds on control inputs. */

		ExportVariable x;							/**< Variable containing the exported differential states. */
		ExportVariable u;							/**< Variable containing the exported control inputs. */

		ExportVariable deltaX0;						/**< Variable containing. */
		ExportVariable H00;							/**< Variable containing Hessian approximation of Lagrange function w.r.t. to initial state. */
		ExportVariable H01;							/**< Variable containing Hessian approximation of Lagrange function w.r.t. to initial state/control inputs. */
		ExportVariable H11;							/**< Variable containing Hessian approximation of Lagrange function w.r.t. to control inputs. */
		ExportVariable d;							/**< Variable containing condensed multi-shooting residuum. */
		ExportVariable C;							/**< Variable containing sensitivities w.r.t. to initial state. */
		ExportVariable E;							/**< Variable containing sensitivities w.r.t. to control inputs. */
		ExportVariable g0;							/**< Variable containing gradient of Lagrange function w.r.t. initial state. */
		ExportVariable g1;							/**< Variable containing gradient of Lagrange function w.r.t. control inputs. */
		ExportVariable lbAValues;					/**< Variable containing user-defined lower limits on differential states. */
		ExportVariable ubAValues;					/**< Variable containing user-defined upper limits on differential states. */

		ExportVariable H;							/**< Variable containing the QP Hessian matrix. */
		ExportVariable A;							/**< Variable containing the QP constraint matrix. */
		ExportVariable g;							/**< Variable containing the QP gradient. */
		ExportVariable lb;							/**< Variable containing the lower limits on QP variables. */
		ExportVariable ub;							/**< Variable containing the upper limits on QP variables. */
		ExportVariable lbA;							/**< Variable containing lower limits on QP constraints. */
		ExportVariable ubA;							/**< Variable containing upper limits on QP constraints. */

		ExportVariable xVars;						/**< Variable containing the primal QP variables. */
		ExportVariable yVars;						/**< Variable containing the dual QP variables. */

		ExportFunction preparationStep;				/**< Function implementing the RTI preparation step. */
		ExportFunction feedbackStep;				/**< Function implementing the RTI feedback step. */
		ExportFunction initialValueEmbedding;		/**< Function implementing the RTI initial value embedding step. */
		ExportFunction shiftControls;				/**< Function implementing a shift of the control variables. */
		ExportFunction shiftStates;					/**< Function implementing a shift of the state variables. */
		ExportFunction getKKT;						/**< Function returning the KKT tolerance of current iterate. */
};


CLOSE_NAMESPACE_ACADO


#endif  // ACADO_TOOLKIT_GAUSS_NEWTON_EXPORT_HPP

// end of file.
