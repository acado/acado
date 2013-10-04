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
 *    \file include/acado/code_generation/export_module.hpp
 *    \author Hans Joachim Ferreau, Boris Houska, Milan Vukov
 *    \date 2010 - 2013
 */


#ifndef ACADO_TOOLKIT_EXPORT_MODULE_HPP
#define ACADO_TOOLKIT_EXPORT_MODULE_HPP

#include <acado/utils/acado_utils.hpp>
#include <acado/user_interaction/options.hpp>
#include <acado/user_interaction/user_interaction.hpp>
#include <acado/matrix_vector/matrix_vector.hpp>
#include <acado/function/function.hpp>
#include <acado/ocp/ocp.hpp>
#include <acado/code_generation/export_file.hpp>
#include <acado/code_generation/integrators/integrator_export.hpp>


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


		/** Exports main header file for using the exported algorithm.
		 *
		 *	@param[in] _dirName			Name of directory to be used to export file.
		 *	@param[in] _fileName		Name of file to be exported.
		 *	@param[in] _realString		String to be used to declare real variables.
		 *	@param[in] _intString		String to be used to declare integer variables.
		 *	@param[in] _precision		Number of digits to be used for exporting real values.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		virtual returnValue exportAcadoHeader(	const String& _dirName,
										const String& _fileName,
										const String& _realString = "real_t",
										const String& _intString = "int",
										int _precision = 16
										) const = 0;


		/** Collects all data declarations of the auto-generated sub-modules to given
		 *	list of declarations.
		 *
		 *	@param[in] declarations		List of declarations.
		 *
		 *	\return SUCCESSFUL_RETURN, \n
		 *	        RET_UNABLE_TO_EXPORT_CODE
		 */
		virtual returnValue collectDataDeclarations(	ExportStatementBlock& declarations,
												ExportStruct dataStruct = ACADO_ANY
												) const = 0;


		/** Collects all function (forward) declarations of the auto-generated sub-modules
		 *	to given list of declarations.
		 *
		 *	@param[in] declarations		List of declarations.
		 *
		 *	\return SUCCESSFUL_RETURN, \n
		 *	        RET_UNABLE_TO_EXPORT_CODE
		 */
		virtual returnValue collectFunctionDeclarations(	ExportStatementBlock& declarations
													) const = 0;


		/** This function sets the number of calls performed for the timing results.
		 *
		 *	@param[in] _timingCalls		The new number of calls performed for the timing results.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		virtual returnValue setTimingCalls( uint _timingCalls
										);

		/** Returns the name of common header file.
		 *
		 *	\return Name of common header file
		 */
		String getCommonHeaderName( ) const;

		/** \name Get/set module name.
		 *
		 *  \todo Module name is used to prefix the exported files.
		 *
		 * @{ */

		/** Get module name. */
		String getName() const;

		/** Set module name. */
		returnValue setName(const String& _name);

		/** @} */

		/** \name Get/set export folder name.
		 * @{ */

		/** Get export folder name */
		const String& getExportFolderName() const;

		/** Set export folder name */
		void setExportFolderName(const String& _name);

		/** @} */

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
		String commonHeaderName;			/**< Name of common header file. */
		uint timingCalls;					/**< The number of calls to the exported function for the timing results. */

    private:
		/** Module name. */
		String name;
		/** Name of a folder where the code is going to be exported. */
		String exportFolderName;
};


CLOSE_NAMESPACE_ACADO


#endif  // ACADO_TOOLKIT_EXPORT_MODULE_HPP

// end of file.
