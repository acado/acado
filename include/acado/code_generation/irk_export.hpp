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
 *    \file include/acado/integrator/irk_export.hpp
 *    \author Rien Quirynen
 *    \date 2012
 */


#ifndef ACADO_TOOLKIT_IRK_EXPORT_HPP
#define ACADO_TOOLKIT_IRK_EXPORT_HPP

#include <acado/code_generation/rk_export.hpp>
#include <acado/code_generation/gaussian_elimination_export.hpp>
#include <acado/code_generation/householder_qr_export.hpp>


BEGIN_NAMESPACE_ACADO


/** 
 *	\brief Allows to export a tailored implicit Runge-Kutta integrator for fast model predictive control.
 *
 *	\ingroup NumericalAlgorithms
 *
 *	The class ImplicitRungeKuttaExport allows to export a tailored implicit Runge-Kutta integrator
 *	for fast model predictive control.
 *
 *	\author Rien Quirynen
 */
class ImplicitRungeKuttaExport : public RungeKuttaExport
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
        ImplicitRungeKuttaExport(	UserInteraction* _userInteraction = 0,
							const String& _commonHeaderName = ""
							);

		/** Copy constructor (deep copy).
		 *
		 *	@param[in] arg		Right-hand side object.
		 */
        ImplicitRungeKuttaExport(	const ImplicitRungeKuttaExport& arg
							);

        /** Destructor. 
		 */
        virtual ~ImplicitRungeKuttaExport( );


		/** Assignment operator (deep copy).
		 *
		 *	@param[in] arg		Right-hand side object.
		 */
		ImplicitRungeKuttaExport& operator=(	const ImplicitRungeKuttaExport& arg
										);


		/** Initializes export of a tailored integrator.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		virtual returnValue setup( );


		/** Assigns Differential Equation to be used by the integrator.
		 *
		 *	@param[in] rhs		Right-hand side expression.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		
		virtual returnValue setDifferentialEquation( const Expression& rhs );
							
        
        /** Sets up the output with the grids for the different output functions.									\n
		*                                                                      										\n
		*  \param outputGrids_	  	The vector containing a grid for each output function.			  				\n
		*  \param rhs 	  	  		The expressions corresponding the output functions.								\n
		*                                                                      										\n
		*  \return SUCCESSFUL_RETURN
		*/
		virtual returnValue setupOutput( const std::vector<Grid> outputGrids_,
									  const std::vector<Expression> rhs );
        

		/** Adds all data declarations of the auto-generated integrator to given list of declarations.
		 *
		 *	@param[in] declarations		List of declarations.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		virtual returnValue getDataDeclarations(	ExportStatementBlock& declarations,
													ExportStruct dataStruct = ACADO_ANY
													) const;


		/** Adds all function (forward) declarations of the auto-generated integrator to given list of declarations.
		 *
		 *	@param[in] declarations		List of declarations.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		virtual returnValue getFunctionDeclarations(	ExportStatementBlock& declarations
														) const;



		/** Exports source code of the auto-generated integrator into the given directory.
		 *
		 *	@param[in] code				Code block containing the auto-generated integrator.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		virtual returnValue getCode(	ExportStatementBlock& code
										);


	protected:
	
		/** Exports source code for some extra functions to deal with algebraic states.
		 *
		 *	@param[in] code				Code block containing the extra functions.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		virtual returnValue addDAEFunctions( ExportStatementBlock& code );
		
		
		/** Initializes the matrix DD, which is used to extrapolate the variables of the IRK method to the next step.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue initializeDDMatrix( );
		
		
		/** Initializes the matrix coeffs, containing coefficients of polynomials that are used to evaluate the 
		 * 	continuous output (see evaluatePolynomial).
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue initializeCoefficients( );
		
		
		/** Recursive function that helps with the computation of the coefficients of polynomials that are used to evaluate the 
		 * 	continuous output (see initializeCoefficients), by computing the correct combinations of elements of the vector
		 * 	cc from the Butcher table.
		 *
		 * 	@param[in] cVec			The vector with all the elements of the vector cc from the Butcher table, of which combinations
		 * 							are computed in a recursive way.
		 * 	@param[in] index		An index of the vector cVec which denotes the relevant part for this invocation.
		 * 	@param[in] numEls		The number of elements in the combination.
		 * 
		 *	\return SUCCESSFUL_RETURN
		 */
		Vector computeCombinations( const Vector& cVec, uint index, uint numEls );
		
		
		/** Returns the coefficients of the polynomial, representing the continuous output of the integrator.
		 *
		 *	@param[in] time				The point in the interval (0,1] for which the coefficients are returned.
		 *
		 *	\return Coefficients of the polynomial, corresponding the given grid point
		 */
		Vector evaluatePolynomial( double time );
		
		
		/** Exports the evaluation of the coefficients of the polynomial, representing the continuous output of the integrator.
		 *
		 *	@param[in] block			The block to which the code will be exported.
		 *	@param[in] variable			The variable containing the coefficients of the polynomial.
		 *	@param[in] grid				The variable containing the grid points for the specific output.
		 *	@param[in] indexTime		The index of the specific grid point.
		 *	@param[in] h				The integration step size.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue evaluatePolynomial( ExportStatementBlock& block, 
										const ExportVariable& variable, 
										const ExportVariable& grid, 
										const ExportIndex& indexTime, 
										double h );
		
		
		/** Copies all class members from given object.
		 *
		 *	@param[in] arg		Right-hand side object.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		virtual returnValue copy(	const ImplicitRungeKuttaExport& arg
							);

		
		/** This routine initializes the matrices AA, bb and cc which
		 * 	form the Butcher Tableau. */
		virtual returnValue initializeButcherTableau() = 0;
		
		
		/** Returns the performed number of Newton iterations.
		 * 
		 * 	\return The performed number of Newton iterations.
		 */
		uint getNumIts() const;
		
		
		/** Returns the performed number of Newton iterations for the initialization of the first step.
		 * 
		 * 	\return The performed number of Newton iterations for the initialization of the first step.
		 */
        uint getNumItsInit() const;
		
		
		/** Returns the performed number of Newton iterations to make the algebraic states consistent.
		 * 
		 * 	\return The performed number of Newton iterations to make the algebraic states consistent.
		 */
		uint getNumAlgIts() const;
		
		
		/** Returns the performed number of Newton iterations for the initialization of the algebraic states in the first step.
		 * 
		 * 	\return The performed number of Newton iterations for the initialization of the algebraic states in the first step.
		 */
        uint getNumAlgItsInit() const;


    protected:
    
		BooleanType REUSE;						/**< This boolean is true when the IFTR method is used instead of the IFT method. */
		BooleanType CONTINUOUS_OUTPUT;			/**< This boolean is true when continuous output needs to be provided. */
		BooleanType UNROLL_OUTPUT;				/**< This boolean is true when the evaluations for the continuous output should be unrolled. */

		uint diffsDim;							/**< This is the total number of sensitivities needed. */
		uint inputDim;							/**< This is the dimension of the input to the integrator. */
		uint numIts;							/**< This is the performed number of Newton iterations. */
		uint numItsInit;						/**< This is the performed number of Newton iterations for the initialization of the first step. */
		uint numAlgIts;							/**< This is the performed number of Newton iterations to make the algebraic states consistent. */
		uint numAlgItsInit;						/**< This is the performed number of Newton iterations for the initialization of the algebraic states in the first step. */

		ExportLinearSolver* solver;				/**< This is the exported linear solver that is used by the implicit Runge-Kutta method. */
		ExportLinearSolver* daeSolver;			/**< This is the other exported linear solver that is used by the implicit Runge-Kutta method in the case of differential algebraic equations. */
        ExportODEfunction diffs_ODE;			/**< Module to export the evaluation of the derivatives of the ordinary differential equations. */
        ExportODEfunction diffs_DAE;			/**< Module to export the evaluation of the derivatives of the differential algebraic equations. */
        
        std::vector<ExportODEfunction> diffs_OUTPUTS;			/**< Module to export the evaluation of the derivatives of the output functions. */
        
        // DEFINITION OF THE EXPORTVARIABLES
		ExportVariable  rk_sol;					/**< Variable containing the solution of the linear system. */
		ExportVariable	rk_A;					/**< Variable containing the matrix of the linear system. */
		ExportVariable	rk_b;					/**< Variable containing the right-hand side of the linear system. */
		ExportVariable	rk_alg_A;				/**< Variable containing the matrix of the linear system for the algebraic states. */
		ExportVariable	rk_alg_b;				/**< Variable containing the right-hand side of the linear system for the algebraic states. */
		ExportVariable 	rk_rhsTemp;				/**< Variable containing intermediate results of evaluations of the right-hand side expression. */
		ExportVariable  rk_diffsTemp;			/**< Variable containing intermediate results of evaluations of the derivatives of the differential equations (ordinary and algebraic). */
		ExportVariable	rk_alg_diffsTemp;		/**< Variable containing intermediate results of evaluations of the derivatives of the differential algebraic equations. */
		ExportVariable  rk_diffsPrev;			/**< Variable containing the sensitivities from the previous integration step. */
		ExportVariable  rk_diffsNew;			/**< Variable containing the derivatives wrt the previous values. */
		ExportVariable 	rk_xPrev;				/**< Variable containing the previous integrator state. */
		
		ExportVariable 	rk_rhsOutputTemp;		/**< Variable containing intermediate results of evaluations of the right-hand side expression of an output function. */
		ExportVariable  rk_diffsOutputTemp;		/**< Variable containing intermediate results of evaluations of the derivatives of an output function. */
		std::vector<ExportVariable> rk_outputs;	/**< Variables containing the evaluations of the continuous output from the integrator. */
		ExportVariable 	rk_outH;				/**< Variable that is used for the evaluations of the continuous output. */
		
		Matrix DD;								/**< This matrix is used for the initialization of the variables for the next integration step. */
		Matrix coeffs;							/**< This matrix contains coefficients of polynomials that are used to evaluate the continuous output (see evaluatePolynomial). */
		
		ExportVariable	norm;					/**< The variable that is used to return a measure for the consistency of the differential and algebraic states. */
		ExportFunction	makeStatesConsistent;	/**< This exported function can be used to make the differential and algebraic states more consistent when needed. */
		ExportFunction	getNormConsistency;		/**< This exported function returns a measure for the consistency of the differential and algebraic states. */
};


CLOSE_NAMESPACE_ACADO


#endif  // ACADO_TOOLKIT_IRK_EXPORT_HPP

// end of file.
