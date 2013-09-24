/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2013 by Boris Houska, Hans Joachim Ferreau,
 *    Milan Vukov, Rien Quirynen, KU Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC)
 * 	  under supervision of Moritz Diehl. All rights reserved.
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
 *    \file include/acado/code_generation/auxiliary_functions_export.hpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010-2011
 */


#ifndef ACADO_TOOLKIT_AUXILIARY_FUNCTIONS_EXPORT_HPP
#define ACADO_TOOLKIT_AUXILIARY_FUNCTIONS_EXPORT_HPP

#include <acado/utils/acado_utils.hpp>
#include <acado/matrix_vector/matrix_vector.hpp>
#include <acado/code_generation/export_algorithm.hpp>


BEGIN_NAMESPACE_ACADO


/** 
 *	\brief Allows to export a couple of auxiliary functions for fast model predictive control.
 *
 *	\ingroup NumericalAlgorithms
 *
 *	The class AuxiliaryFunctionsExport allows to export a couple of auxiliary functions 
 *	for fast model predictive control.
 *
 *	\author Hans Joachim Ferreau, Boris Houska
 */
class AuxiliaryFunctionsExport : public ExportAlgorithm
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
        AuxiliaryFunctionsExport(	UserInteraction* _userInteraction = 0,
									const String& _commonHeaderName = ""
									);

		/** Copy constructor (deep copy).
		 *
		 *	@param[in] arg		Right-hand side object.
		 */
		AuxiliaryFunctionsExport(	const AuxiliaryFunctionsExport& arg
									);

		/** Destructor. 
		*/
		virtual ~AuxiliaryFunctionsExport( );

		/** Assignment operator (deep copy).
		 *
		 *	@param[in] arg		Right-hand side object.
		 */
		AuxiliaryFunctionsExport& operator=(	const AuxiliaryFunctionsExport& arg
												);


		/** Initializes export of a tailored condensing algorithm.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		virtual returnValue setup( );


		/** Adds all data declarations of the auto-generated auxiliary functions 
		 * to given list of declarations.
		 *
		 *	@param[in] declarations		List of declarations.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		virtual returnValue getDataDeclarations(	ExportStatementBlock& declarations,
													ExportStruct dataStruct = ACADO_ANY
													) const;

		/** Adds all function (forward) declarations of the auto-generated auxiliary functions 
		 *	to given list of declarations.
		 *
		 *	@param[in] declarations		List of declarations.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		virtual returnValue getFunctionDeclarations(	ExportStatementBlock& declarations
														) const;


		/** Exports source code of the auto-generated auxiliary functions
		 *  into the given directory.
		 *
		 *	@param[in] code				Code block containing the auto-generated auxiliary functions.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		virtual returnValue getCode(	ExportStatementBlock& code
										);


	protected:

		/** Copies all class members from given object.
		 *
		 *	@param[in] arg		Right-hand side object.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue copy(	const AuxiliaryFunctionsExport& arg
							);

		/** Frees internal dynamic memory to yield an empty function.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue clear( );


    protected:

		ExportFunction getAcadoVariablesX;				/**< Function returning pointer to exported differential states. */
		ExportFunction getAcadoVariablesU;				/**< Function returning pointer to exported control inputs. */
		ExportFunction getAcadoVariablesXRef;			/**< Function returning pointer to exported reference trajectory for differential states. */
		ExportFunction getAcadoVariablesURef;			/**< Function returning pointer to exported reference trajectory for control inputs. */

		ExportFunction printStates;						/**< Function that prints the exported differential states to screen. */
		ExportFunction printControls;					/**< Function that prints the exported control inputs to screen. */
		ExportFunction getTime;							/**< Function returning the current system time. */
		ExportFunction printHeader;						/**< Function that prints the ACADO copyright notice to screen. */
};


CLOSE_NAMESPACE_ACADO


#endif  // ACADO_TOOLKIT_AUXILIARY_FUNCTIONS_EXPORT_HPP

// end of file.
