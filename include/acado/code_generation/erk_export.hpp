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
 *    \file include/acado/integrator/erk_export.hpp
 *    \author Rien Quirynen
 *    \date 2012
 */


#ifndef ACADO_TOOLKIT_ERK_EXPORT_HPP
#define ACADO_TOOLKIT_ERK_EXPORT_HPP

#include <acado/code_generation/rk_export.hpp>


BEGIN_NAMESPACE_ACADO


/** 
 *	\brief Allows to export a tailored explicit Runge-Kutta integrator for fast model predictive control.
 *
 *	\ingroup NumericalAlgorithms
 *
 *	The class ExplicitRungeKuttaExport allows to export a tailored explicit Runge-Kutta integrator
 *	for fast model predictive control.
 *
 *	\author Rien Quirynen
 */
class ExplicitRungeKuttaExport : public RungeKuttaExport
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
        ExplicitRungeKuttaExport(	UserInteraction* _userInteraction = 0,
							const String& _commonHeaderName = ""
							);

		/** Copy constructor (deep copy).
		 *
		 *	@param[in] arg		Right-hand side object.
		 */
        ExplicitRungeKuttaExport(	const ExplicitRungeKuttaExport& arg
							);

        /** Destructor. 
		 */
        virtual ~ExplicitRungeKuttaExport( );


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
							
        
        /** Sets up the output with the grids for the different output functions.									\n
		*                                                                      										\n
		*  \param outputGrids_	  	The vector containing a grid for each output function.			  				\n
		*  \param rhs 	  	  		The expressions corresponding the output functions.								\n
		*                                                                      										\n
		*  \return SUCCESSFUL_RETURN
		*/
		virtual returnValue setupOutput( const std::vector<Grid> outputGrids_,
									  const std::vector<Expression> rhs );


	protected:

		
		/** This routine initializes the matrices AA, bb and cc which
		 * 	form the Butcher Tableau. */
		virtual returnValue initializeButcherTableau() = 0;


    protected:

};


CLOSE_NAMESPACE_ACADO


#endif  // ACADO_TOOLKIT_ERK_EXPORT_HPP

// end of file.
