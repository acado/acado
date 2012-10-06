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
 *    \file include/acado/code_generation/mpc_export.hpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010-2011
 */


#ifndef ACADO_TOOLKIT_MPC_EXPORT_HPP
#define ACADO_TOOLKIT_MPC_EXPORT_HPP


#include <acado/code_generation/export_module.hpp>
#include <acado/code_generation/integrators/integrator_export.hpp>
#include <acado/code_generation/condensing_export.hpp>
#include <acado/code_generation/gauss_newton_export.hpp>
#include <acado/code_generation/auxiliary_functions_export.hpp>
#include <acado/code_generation/export_file.hpp>

BEGIN_NAMESPACE_ACADO

/** 
 *	\brief User-interface to automatically generate algorithms for fast model predictive control
 *
 *	\ingroup UserInterfaces
 *
 *  The class MPCexport is a user-interface to automatically generate tailored
 *  algorithms for fast model predictive control. It takes an optimal control 
 *  problem (OCP) formulation and generates code based on given user options, 
 *  e.g specifying the number of integrator steps or the online QP solver.
 *
 *	\author Hans Joachim Ferreau, Boris Houska
 */
class MPCexport : public ExportModule
{
    //
    // PUBLIC MEMBER FUNCTIONS:
    //
    public:

		/** Default constructor. 
		 */
		MPCexport( );

		/** Constructor which takes OCP formulation.
		 *
		 *	@param[in] _ocp		OCP formulation for code export.
		 */
		MPCexport(	const OCP& _ocp
					);

		/** Copy constructor (deep copy).
		 *
		 *	@param[in] arg		Right-hand side object.
		 */
		MPCexport(	const MPCexport& arg
					);

		/** Destructor. 
		 */
		virtual ~MPCexport( );

		/** Assignment operator (deep copy).
		 *
		 *	@param[in] arg		Right-hand side object.
		 */
        MPCexport& operator=(	const MPCexport& arg
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
										);


		/** Prints dimensions (i.e. number of variables and constraints) 
		 *  of underlying QP to screen.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue printDimensionsQP( );



    protected:

		/** Copies all class members from given object.
		 *
		 *	@param[in] arg		Right-hand side object.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue copy(	const MPCexport& arg
							);

		/** Frees internal dynamic memory to yield an empty function.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue clear( );


		/** Sets-up code export and initializes underlying export modules.
		 *
		 *	\return SUCCESSFUL_RETURN, \n
		 *	        RET_INVALID_OPTION, \n
		 *	        RET_INVALID_OBJECTIVE_FOR_CODE_EXPORT, \n
		 *	        RET_ONLY_ODE_FOR_CODE_EXPORT, \n
		 *	        RET_NO_DISCRETE_ODE_FOR_CODE_EXPORT, \n
		 *	        RET_ONLY_STATES_AND_CONTROLS_FOR_CODE_EXPORT, \n
		 *	        RET_ONLY_EQUIDISTANT_GRID_FOR_CODE_EXPORT, \n
		 *	        RET_ONLY_BOUNDS_FOR_CODE_EXPORT, \n
		 *	        RET_UNABLE_TO_EXPORT_CODE
		 */
		returnValue setup( );

		/** Checks whether OCP formulation is compatible with code export capabilities.
		 *
		 *	\return SUCCESSFUL_RETURN, \n
		 *	        RET_INVALID_OBJECTIVE_FOR_CODE_EXPORT, \n
		 *	        RET_ONLY_ODE_FOR_CODE_EXPORT, \n
		 *	        RET_NO_DISCRETE_ODE_FOR_CODE_EXPORT, \n
		 *	        RET_ONLY_STATES_AND_CONTROLS_FOR_CODE_EXPORT, \n
		 *	        RET_ONLY_EQUIDISTANT_GRID_FOR_CODE_EXPORT, \n
		 *	        RET_ONLY_BOUNDS_FOR_CODE_EXPORT
		 */
		returnValue checkConsistency( ) const;


		/** Collects all data declarations of the auto-generated sub-modules to given 
		 *	list of declarations.
		 *
		 *	@param[in] declarations		List of declarations.
		 *
		 *	\return SUCCESSFUL_RETURN, \n
		 *	        RET_UNABLE_TO_EXPORT_CODE
		 */
		returnValue collectDataDeclarations(	ExportStatementBlock& declarations,
												ExportStruct dataStruct = ACADO_ANY
												) const;

		/** Collects all function (forward) declarations of the auto-generated sub-modules 
		 *	to given list of declarations.
		 *
		 *	@param[in] declarations		List of declarations.
		 *
		 *	\return SUCCESSFUL_RETURN, \n
		 *	        RET_UNABLE_TO_EXPORT_CODE
		 */
		returnValue collectFunctionDeclarations(	ExportStatementBlock& declarations
													) const;


		/** Exports main header file for using the exported MPC algorithm.
		 *
		 *	@param[in] _dirName			Name of directory to be used to export file.
		 *	@param[in] _fileName		Name of file to be exported.
		 *	@param[in] _realString		String to be used to declare real variables.
		 *	@param[in] _intString		String to be used to declare integer variables.
		 *	@param[in] _precision		Number of digits to be used for exporting real values.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue exportAcadoHeader(	const String& _dirName,
										const String& _fileName,
										const String& _realString = "real_t",
										const String& _intString = "int",
										int _precision = 16
										) const;

		/** Exports file with template main function for using the 
		 *  exported MPC algorithm.
		 *
		 *	@param[in] _dirName			Name of directory to be used to export file.
		 *	@param[in] _fileName		Name of file to be exported.
		 *	@param[in] _realString		String to be used to declare real variables.
		 *	@param[in] _intString		String to be used to declare integer variables.
		 *	@param[in] _precision		Number of digits to be used for exporting real values.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue exportTemplateMain(	const String& _dirName,
										const String& _fileName,
										const String& _realString = "real_t",
										const String& _intString = "int",
										int _precision = 16
										) const;

		/** Exports GNU Makefile for compiling the exported MPC algorithm.
		 *
		 *	@param[in] _dirName			Name of directory to be used to export file.
		 *	@param[in] _fileName		Name of file to be exported.
		 *	@param[in] _realString		String to be used to declare real variables.
		 *	@param[in] _intString		String to be used to declare integer variables.
		 *	@param[in] _precision		Number of digits to be used for exporting real values.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue exportMakefile(	const String& _dirName,
									const String& _fileName,
										const String& _realString = "real_t",
										const String& _intString = "int",
										int _precision = 16
									) const;

		/** Exports files containing the interface to an external online 
		 *  QP solver to be used within the exported MPC algorithm.
		 *
		 *	@param[in] _dirName			Name of directory to be used to export files.
		 *	@param[in] _realString		String to be used to declare real variables.
		 *	@param[in] _intString		String to be used to declare integer variables.
		 *	@param[in] _precision		Number of digits to be used for exporting real values.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue exportQPsolverInterface(	const String& _dirName,
												const String& _realString = "real_t",
												const String& _intString = "int",
												int _precision = 16
												) const;

		/** Exports a Simulink interface for the exported MPC algorithm.
		 *
		 *	@param[in] _dirName			Name of directory to be used to export file.
		 *	@param[in] _sFcnFileName	Name of sfunction file to be exported.
		 *	@param[in] _makeFileName	Name of Matlab make script to be exported.
		 *	@param[in] _realString		String to be used to declare real variables.
		 *	@param[in] _intString		String to be used to declare integer variables.
		 *	@param[in] _precision		Number of digits to be used for exporting real values.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue exportSimulinkInterface(	const String& _dirName,
												const String& _sFcnFileName,
												const String& _makeFileName,
												const String& _realString = "real_t",
												const String& _intString = "int",
												int _precision = 16
												) const;



    protected:

		IntegratorExport*  integrator;			/**< Module for exporting a tailored integrator. */
		CondensingExport*  condenser;			/**< Module for exporting a tailored condensing algorithm. */
		GaussNewtonExport* gaussNewton;			/**< Module for exporting a tailored Gauss-Newton algorithm. */
		AuxiliaryFunctionsExport* auxFcns;		/**< Module for exporting a tailored Gauss-Newton algorithm. */
};


CLOSE_NAMESPACE_ACADO


#endif  // ACADO_TOOLKIT_MPC_EXPORT_HPP

// end of file.
