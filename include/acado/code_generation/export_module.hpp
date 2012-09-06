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
 *    \file include/acado/code_generation/export_module.hpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010-2011
 */


#ifndef ACADO_TOOLKIT_EXPORT_MODULE_HPP
#define ACADO_TOOLKIT_EXPORT_MODULE_HPP

#include <acado/utils/acado_utils.hpp>
#include <acado/user_interaction/options.hpp>
#include <acado/user_interaction/user_interaction.hpp>
#include <acado/matrix_vector/matrix_vector.hpp>
#include <acado/function/function.hpp>
#include <acado/ocp/ocp.hpp>


BEGIN_NAMESPACE_ACADO


/** 
 *	\brief User-interface to automatically generate algorithms for fast model predictive control
 *
 *	\ingroup UserInterfaces
 *
 *  The class ExportModule is a user-interface to automatically generate tailored
 *  algorithms for fast model predictive control. It takes an optimal control 
 *  problem (OCP) formulation and generates code based on given user options.
 *
 *	\author Hans Joachim Ferreau, Boris Houska
 */
class ExportModule : public UserInteraction
{
    //
    // PUBLIC MEMBER FUNCTIONS:
    //
    public:

		/** Default constructor. 
		 */
		ExportModule( );

		/** Constructor which takes OCP formulation.
		 *
		 *	@param[in] _ocp		OCP formulation for code export.
		 */
		ExportModule(	const OCP& _ocp
						);

		/** Copy constructor (deep copy).
		 *
		 *	@param[in] arg		Right-hand side object.
		 */
		ExportModule(	const ExportModule& arg
						);

		/** Destructor. 
		 */
		virtual ~ExportModule( );

		/** Assignment operator (deep copy).
		 *
		 *	@param[in] arg		Right-hand side object.
		 */
        ExportModule& operator=(	const ExportModule& arg
									);


		/** Assigns OCP formulation to be used to export MPC algorithm.
		 *
		 *	@param[in] _ocp		OCP formulation for code export.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
        virtual returnValue setOCP(	const OCP& _ocp
									);

		/** Exports all files of the auto-generated code into the given directory.
		 *
		 *	@param[in] dirName			Name of directory to be used to export files.
		 *	@param[in] _realString		String to be used to declare real variables.
		 *	@param[in] _intString		String to be used to declare integer variables.
		 *	@param[in] _precision		Number of digits to be used for exporting real values.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
        virtual returnValue exportCode(	const String& dirName,
										const String& _realString = "real_t",
										const String& _intString = "int",
										int _precision = 16
										) = 0;


		/** Returns number of differential states.
		 *
		 *  \return Number of differential states
		 */
		uint getNX( ) const;


		/** Returns number of algebraic states.
		 *
		 *  \return Number of algebraic states
		 */
		uint getNXA( ) const;

		/** Returns number of control inputs.
		 *
		 *  \return Number of control inputs
		 */
		uint getNU( ) const;

		/** Returns number of parameters.
		 *
		 *  \return Number of parameters
		 */
		uint getNP( ) const;

		/** Returns number of control intervals.
		 *
		 *  \return Number of control intervals
		 */
		uint getN( ) const;


		/** Sets the name of common header file.
		 *
		 *	@param[in] _name			New name of common header file.
		 *
		 *	\return SUCCESSFUL_RETURN, \n
		 *	        RET_INVALID_ARGUMENTS
		 */
		returnValue	setCommonHeaderName(	const String& _name
											);

		/** Returns the name of common header file.
		 *
		 *	\return Name of common header file
		 */
		String getCommonHeaderName( ) const;


	protected:

		/** Copies all class members from given object.
		 *
		 *	@param[in] arg		Right-hand side object.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue copy(	const ExportModule& arg
							);

		/** Sets-up default options.
		 *
		 *  \return SUCCESSFUL_RETURN
		 */
		returnValue setupOptions( );


    protected:
		
		OCP ocp;							/**< OCP formulation used to export code. */

		uint NX;							/**< Number of differential states. */
		uint NDX;							/**< Number of differential states derivatives. */
		uint NXA;							/**< Number of algebraic states. */
		uint NU;							/**< Number of control inputs. */
		uint NP;							/**< Number of parameters. */
		uint N;								/**< Number of control intervals. */
		
		String commonHeaderName;			/**< Name of common header file. */
};


CLOSE_NAMESPACE_ACADO


#endif  // ACADO_TOOLKIT_EXPORT_MODULE_HPP

// end of file.
