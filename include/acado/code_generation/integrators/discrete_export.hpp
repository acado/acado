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
 *    \file include/acado/code_generation/integrators/discrete_export.hpp
 *    \author Rien Quirynen
 *    \date 2013
 */


#ifndef ACADO_TOOLKIT_DT_EXPORT_HPP
#define ACADO_TOOLKIT_DT_EXPORT_HPP

#include <acado/code_generation/integrators/integrator_export.hpp>

#include <acado/code_generation/export_algorithm_factory.hpp>


BEGIN_NAMESPACE_ACADO


/** 
 *	\brief Allows to export a tailored discrete-time 'integrator' for fast model predictive control.
 *
 *	\ingroup NumericalAlgorithms
 *
 *	The class DiscreteTimeExport allows to export a tailored discrete-time 'integrator'
 *	for fast model predictive control.
 *
 *	\author Rien Quirynen
 */
class DiscreteTimeExport : public IntegratorExport
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
        DiscreteTimeExport(	UserInteraction* _userInteraction = 0,
							const String& _commonHeaderName = ""
							);

		/** Copy constructor (deep copy).
		 *
		 *	@param[in] arg		Right-hand side object.
		 */
        DiscreteTimeExport(	const DiscreteTimeExport& arg
							);

        /** Destructor. 
		 */
        virtual ~DiscreteTimeExport( );

		/** Assignment operator (deep copy).
		 *
		 *	@param[in] arg		Right-hand side object.
		 */
		DiscreteTimeExport& operator=(	const DiscreteTimeExport& arg
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

		virtual returnValue setModel( const String& _rhs, const String& _diffs_rhs );


		/** .
		 *
		 *	@param[in] 		.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		virtual returnValue setLinearInput( const Matrix& M1, const Matrix& A1, const Matrix& B1 );


		/** .
		 *
		 *	@param[in] 		.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		virtual returnValue setLinearOutput( const Matrix& M3, const Matrix& A3, const Expression& rhs );


		/** Sets a polynomial NARX model to be used by the integrator.
		 *
		 *	@param[in] delay		The delay for the states in the NARX model.
		 *	@param[in] parms		The parameters defining the polynomial NARX model.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		virtual returnValue setNARXmodel( const uint delay, const Matrix& parms );


        /** Sets up the output with the grids for the different output functions.									\n
		*                                                                      										\n
		*  \param outputGrids_	  	The vector containing a grid for each output function.			  				\n
		*  \param rhs 	  	  		The expressions corresponding the output functions.								\n
		*                                                                      										\n
		*  \return SUCCESSFUL_RETURN
		*/
		virtual returnValue setupOutput(  const std::vector<Grid> outputGrids_,
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


		/** Returns the largest global export variable.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		ExportVariable getAuxVariable() const;


		/** Copies all class members from given object.
		 *
		 *	@param[in] arg		Right-hand side object.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		virtual returnValue copy(	const DiscreteTimeExport& arg
							);


    protected:
        

};

static struct RegisterDiscreteTimeExport
{
	RegisterDiscreteTimeExport();
} registerDiscreteTimeExport;


CLOSE_NAMESPACE_ACADO


#endif  // ACADO_TOOLKIT_DT_EXPORT_HPP

// end of file.
