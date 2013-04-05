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
		virtual returnValue setup( ) = 0;


		/** Assigns Differential Equation to be used by the integrator.
		 *
		 *	@param[in] rhs		Right-hand side expression.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		
		virtual returnValue setDifferentialEquation( const Expression& rhs ) = 0;


		/** Adds all data declarations of the auto-generated integrator to given list of declarations.
		 *
		 *	@param[in] declarations		List of declarations.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		virtual returnValue getDataDeclarations(	ExportStatementBlock& declarations,
													ExportStruct dataStruct = ACADO_ANY
													) const = 0;


		/** Adds all function (forward) declarations of the auto-generated integrator to given list of declarations.
		 *
		 *	@param[in] declarations		List of declarations.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		virtual returnValue getFunctionDeclarations(	ExportStatementBlock& declarations
														) const = 0;



		/** Exports source code of the auto-generated integrator into the given directory.
		 *
		 *	@param[in] code				Code block containing the auto-generated integrator.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		virtual returnValue getCode(	ExportStatementBlock& code
										) = 0;



	protected:

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


CLOSE_NAMESPACE_ACADO


#endif  // ACADO_TOOLKIT_DT_EXPORT_HPP

// end of file.
