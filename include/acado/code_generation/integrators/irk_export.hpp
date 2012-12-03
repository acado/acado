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

#include <acado/code_generation/integrators/rk_export.hpp>
#include <acado/code_generation/linear_solvers/linear_solver_generation.hpp>


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


		/** Assigns the model to be used by the integrator.
		 *
		 *	@param[in] _rhs				Name of the function, evaluating the right-hand side.
		 *	@param[in] _diffs_rhs		Name of the function, evaluating the derivatives of the right-hand side.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */

		returnValue setModel( const String& _rhs, const String& _diffs_rhs );
							
        
        /** Sets up the output with the grids for the different output functions.									\n
		*                                                                      										\n
		*  \param outputGrids_	  	The vector containing a grid for each output function.			  				\n
		*  \param rhs 	  	  		The expressions corresponding the output functions.								\n
		*                                                                      										\n
		*  \return SUCCESSFUL_RETURN
		*/
		virtual returnValue setupOutput( const std::vector<Grid> outputGrids_,
									  const std::vector<Expression> rhs );


		/** Sets up the output with the grids for the different output functions.									\n
		 *                                                                      										\n
		 *  \param outputGrids_	  		The vector containing a grid for each output function.			  			\n
		 *  \param _outputNames 	  		The names of the output functions.									  		\n
		 *  \param _diffs_outputNames 	The names of the functions, evaluating the derivatives of the outputs.		\n
		 *  \param _dims_output 			The dimensions of the output functions.										\n
		 *                                                                      										\n
		 *  \return SUCCESSFUL_RETURN
		 */
		virtual returnValue setupOutput(  const std::vector<Grid> outputGrids_,
									  	  const std::vector<String> _outputNames,
									  	  const std::vector<String> _diffs_outputNames,
										  const std::vector<uint> _dims_output );


		/** Sets up the output with the grids for the different output functions.										\n
		 *                                                                      										\n
		 *  \param outputGrids_	  			The vector containing a grid for each output function.			  			\n
		 *  \param _outputNames 	  		The names of the output functions.									  		\n
		 *  \param _diffs_outputNames 		The names of the functions, evaluating the derivatives of the outputs.		\n
		 *  \param _dims_output 			The dimensions of the output functions.										\n
		 *  \param _outputDependencies		A separate dependency matrix for each output.								\n
		 *                                                                      										\n
		 *  \return SUCCESSFUL_RETURN
		 */
		virtual returnValue setupOutput(  const std::vector<Grid> outputGrids_,
									  	  const std::vector<String> _outputNames,
									  	  const std::vector<String> _diffs_outputNames,
										  const std::vector<uint> _dims_output,
										  const std::vector<Matrix> _outputDependencies );
        

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
		
		
		/** Returns the index corresponding Compressed Row Storage (CRS) for the dependency matrix of a specific output function.
		 *
		 * @param[in] output	The number of the output function.
		 * @param[in] row		The number of the row, corresponding the element of interest.
		 * @param[in] col		The number of the column, corresponding the element of interest.
		 *
		 *	\return The CRS index.
		 */
		returnValue getCRSIndex( uint output, ExportIndex row, ExportIndex col );


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
		
		
		/** Returns the coefficients of the derived polynomial, representing the derivative of the continuous output with respect to time.
		 *
		 *	@param[in] time				The point in the interval (0,1] for which the coefficients are returned.
		 *
		 *	\return Coefficients of the polynomial, corresponding the given grid point
		 */
		Vector evaluateDerivedPolynomial( double time );


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


		/** Exports the evaluation of the coefficients of the derived polynomial, representing the derivative of the continuous output with respect to time.
		 *
		 *	@param[in] block			The block to which the code will be exported.
		 *	@param[in] variable			The variable containing the coefficients of the polynomial.
		 *	@param[in] grid				The variable containing the grid points for the specific output.
		 *	@param[in] indexTime		The index of the specific grid point.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue evaluateDerivedPolynomial( ExportStatementBlock& block,
										const ExportVariable& variable,
										const ExportVariable& grid,
										const ExportIndex& indexTime );
		
		
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


    protected:
    
		BooleanType REUSE;						/**< This boolean is true when the IFTR method is used instead of the IFT method. */
		BooleanType CONTINUOUS_OUTPUT;			/**< This boolean is true when continuous output needs to be provided. */
//		BooleanType UNROLL_OUTPUT;				/**< This boolean is true when the evaluations for the continuous output should be unrolled. */

		uint NVARS;								/**< This is the total number of variables (=NX+NXA+NU+NDX). */
		uint diffsDim;							/**< This is the total number of sensitivities needed. */
		uint inputDim;							/**< This is the dimension of the input to the integrator. */
		uint numIts;							/**< This is the performed number of Newton iterations. */
		uint numItsInit;						/**< This is the performed number of Newton iterations for the initialization of the first step. */

		ExportLinearSolver* solver;				/**< This is the exported linear solver that is used by the implicit Runge-Kutta method. */
        
        // DEFINITION OF THE EXPORTVARIABLES
		ExportVariable  rk_sol;					/**< Variable containing the solution of the linear system. */
		ExportVariable	rk_A;					/**< Variable containing the matrix of the linear system. */
		ExportVariable	rk_b;					/**< Variable containing the right-hand side of the linear system. */
		ExportVariable 	rk_rhsTemp;				/**< Variable containing intermediate results of evaluations of the right-hand side expression. */
		ExportVariable  rk_diffsTemp;			/**< Variable containing intermediate results of evaluations of the derivatives of the differential equations (ordinary and algebraic). */
		ExportVariable  rk_diffsPrev;			/**< Variable containing the sensitivities from the previous integration step. */
		ExportVariable  rk_diffsNew;			/**< Variable containing the derivatives wrt the previous values. */
		ExportVariable 	rk_xPrev;				/**< Variable containing the previous integrator state. */
		
		ExportVariable 	rk_rhsOutputTemp;		/**< Variable containing intermediate results of evaluations of the right-hand side expression of an output function. */
		ExportVariable  rk_diffsOutputTemp;		/**< Variable containing intermediate results of evaluations of the derivatives of an output function. */
		std::vector<ExportVariable> rk_outputs;	/**< Variables containing the evaluations of the continuous output from the integrator. */
		ExportVariable 	rk_outH;				/**< Variable that is used for the evaluations of the continuous output. */
		ExportVariable 	rk_out2;				/**< Variable that is used for the evaluations of the continuous output. */
		ExportVariable 	polynEvalVar;			/**< Local variable that is used for the evaluations of the continuous output. */
		
		Matrix DD;								/**< This matrix is used for the initialization of the variables for the next integration step. */
		Matrix coeffs;							/**< This matrix contains coefficients of polynomials that are used to evaluate the continuous output (see evaluatePolynomial). */
};


CLOSE_NAMESPACE_ACADO


#endif  // ACADO_TOOLKIT_IRK_EXPORT_HPP

// end of file.
