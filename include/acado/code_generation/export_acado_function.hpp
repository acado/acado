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
 *    \file include/acado/code_generation/export_ode_function.hpp
 *    \author Hans Joachim Ferreau, Boris Houska, Milan Vukov
 *    \date 2010 - 2013
 */

#ifndef ACADO_TOOLKIT_EXPORT_ODE_FUNCTION_HPP
#define ACADO_TOOLKIT_EXPORT_ODE_FUNCTION_HPP

#include <acado/code_generation/export_function.hpp>

BEGIN_NAMESPACE_ACADO

class Function;

/** 
 *	\brief Allows to export code of an ACADO function.
 *
 *	\ingroup AuxiliaryFunctionality
 *
 *	The class ExportAcadoFunction allows to export code of an ACADO function.
 *
 *	\author Hans Joachim Ferreau, Boris Houska, Milan Vukov
 */
class ExportAcadoFunction : public ExportFunction
{
public:
	/** Default constructor. */
	ExportAcadoFunction( );

	/** Constructor which takes the differential equation to be exported
	 *	as well as the name of the exported ODE.
	 *
	 *	@param[in] _f			Differential equation to be exported.
	 *	@param[in] _name		Name of exported ODE function.
	 */
	ExportAcadoFunction(	const Function& _f,
							const String& _name = "acadoFcn"
							);

	/** Copy constructor (deep copy).
	 *
	 *	@param[in] arg		Right-hand side object.
	 */
	ExportAcadoFunction(const ExportAcadoFunction& arg);

	/** Destructor. */
	virtual ~ExportAcadoFunction( );

	/** Assignment operator (deep copy).
	 *
	 *	@param[in] arg		Right-hand side object.
	 */
	ExportAcadoFunction& operator=(const ExportAcadoFunction& arg);

	/** Clone constructor (deep copy).
	 *
	 *	\return Pointer to cloned object.
	 */
	virtual ExportStatement* clone( ) const;

	/** Clone constructor for ExportFunction (deep copy).
	 *
	 *	\return Pointer to cloned object.
	 */
	virtual ExportFunction* cloneFunction( ) const;


	/** Initializes ODE function export by taking the differential equation
	 *	to be exported as well as the name of the exported ODE.
	 *
	 *	@param[in] _f			Differential equation to be exported.
	 *	@param[in] _name		Name of exported ODE function.
	 * 	@param[in] _numX		The number of states that are needed to evaluate the system of differential equations
	 * 							(needed when the number of equations is not equal to the number of given states).
	 * 	@param[in] _numXA		The number of algebraic states in the input for the evaluation of the system of equations.
	 * 	@param[in] _numU		The number of control inputs given for the evaluation of the system of equations.
	 * 	@param[in] _numP		The number of parameters given for the evaluation of the system of equations.
	 * 	@param[in] _numDX		The number of differential state derivatives given for the evaluation of the system of equations.
	 */
	returnValue init(	const Function& _f,
						const String& _name = "acadoFcn",
						const uint _numX = 0,
						const uint _numXA = 0,
						const uint _numU = 0,
						const uint _numP = 0,
						const uint _numDX = 0
						);

	/** Exports data declaration of the ODE function into given file. Its appearance can
	 *  can be adjusted by various options.
	 *
	 *	@param[in] file				Name of file to be used to export function.
	 *	@param[in] _realString		String to be used to declare real variables.
	 *	@param[in] _intString		String to be used to declare integer variables.
	 *	@param[in] _precision		Number of digits to be used for exporting real values.
	 *
	 *	\return SUCCESSFUL_RETURN
	 */
	virtual returnValue exportDataDeclaration(	FILE* file,
												const String& _realString = "real_t",
												const String& _intString = "int",
												int _precision = 16
												) const;

	/** Exports forward declaration of the ODE function into given file. Its appearance can
	 *  can be adjusted by various options.
	 *
	 *	@param[in] file				Name of file to be used to export statement.
	 *	@param[in] _realString		String to be used to declare real variables.
	 *	@param[in] _intString		String to be used to declare integer variables.
	 *	@param[in] _precision		Number of digits to be used for exporting real values.
	 *
	 *	\return SUCCESSFUL_RETURN
	 */
	virtual returnValue exportForwardDeclaration(	FILE* file,
													const String& _realString = "real_t",
													const String& _intString = "int",
													int _precision = 16
													) const;

	/** Exports source code of the ODE function into given file. Its appearance can
	 *  can be adjusted by various options.
	 *
	 *	@param[in] file				Name of file to be used to export function.
	 *	@param[in] _realString		String to be used to declare real variables.
	 *	@param[in] _intString		String to be used to declare integer variables.
	 *	@param[in] _precision		Number of digits to be used for exporting real values.
	 *
	 *	\return SUCCESSFUL_RETURN
	 */
	virtual returnValue exportCode(	FILE* file,
									const String& _realString = "real_t",
									const String& _intString = "int",
									int _precision = 16
									) const;

	/** Returns whether function has been defined.
	 *
	 *	\return BT_TRUE  iff function has been defined, \n
	 *	        BT_FALSE otherwise
	 */
	virtual BooleanType isDefined( ) const;

	/** Get output dimension of the ACADO function. */
	unsigned getFunctionDim( void );

	/** Get global export variable - a variable that holds intermediate values. */
	returnValue setGlobalExportVariable(const ExportVariable& var);

	/** Set global export variable - a variable that holds intermediate values. */
	ExportVariable getGlobalExportVariable( ) const;

protected:
	/** The number of states that are needed to evaluate the system of differential equations.
	 *  If this number isn't specified, then it will be set to the number of equations (minus
	 *  the number of algebraic states).  */
	unsigned numX;
	/** The number of algebraic states in the input for the evaluation of the system of
	 *  equations (similar to numX). */
	unsigned numXA;
	/** The number of control inputs given for the evaluation of the system of equations
	 *  (similar to numX). */
	unsigned numU;
	/** The number of parameters given for the evaluation of the system of equations. */
	unsigned numP;
	/** The number of differential state derivatives given for the evaluation of the
	 *  system of equations. */
	unsigned numDX;
	/** ACADO function to be exported. */
	std::tr1::shared_ptr< Function > f;
	/** A variable that holds intermediate values. */
	ExportVariable globalVar;
};

CLOSE_NAMESPACE_ACADO

#endif  // ACADO_TOOLKIT_EXPORT_FUNCTION_HPP
