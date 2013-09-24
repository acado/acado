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
 *    \file include/acado/code_generation/export_function.hpp
 *    \authors Hans Joachim Ferreau, Boris Houska, Milan Vukov
 *    \date 2010 - 2013
 */


#ifndef ACADO_TOOLKIT_EXPORT_FUNCTION_HPP
#define ACADO_TOOLKIT_EXPORT_FUNCTION_HPP

#include <acado/utils/acado_utils.hpp>
#include <acado/code_generation/export_variable.hpp>
#include <acado/code_generation/export_argument_list.hpp>
#include <acado/code_generation/export_statement_block.hpp>
#include <acado/code_generation/export_statement_string.hpp>
#include <acado/code_generation/memory_allocator.hpp>

#ifdef _WIN32
    #include <memory>
#else
    #include <tr1/memory>
#endif

BEGIN_NAMESPACE_ACADO


/** 
 *	\brief Allows to export code of an arbitrary function.
 *
 *	\ingroup AuxiliaryFunctionality
 *
 *	The class ExportFunction allows to export code of an arbitrary function.
 *
 */
class ExportFunction : public ExportStatementBlock
{
public:
	/** Default constructor which optionally takes the name of the function
	 *	as well as possible calling arguments.
	 *
	 *	@param[in] _name		Name of the function.
	 *	@param[in] _argument1	Calling argument no. 1.
	 *	@param[in] _argument2	Calling argument no. 2.
	 *	@param[in] _argument3	Calling argument no. 3.
	 *	@param[in] _argument4	Calling argument no. 4.
	 *	@param[in] _argument5	Calling argument no. 5.
	 *	@param[in] _argument6	Calling argument no. 6.
	 *	@param[in] _argument7	Calling argument no. 7.
	 *	@param[in] _argument8	Calling argument no. 8.
	 *	@param[in] _argument9	Calling argument no. 9.
	 */
	ExportFunction(	const String& _name = "defaultFunctionName",
					const ExportArgument& _argument1 = emptyConstExportArgument,
					const ExportArgument& _argument2 = emptyConstExportArgument,
					const ExportArgument& _argument3 = emptyConstExportArgument,
					const ExportArgument& _argument4 = emptyConstExportArgument,
					const ExportArgument& _argument5 = emptyConstExportArgument,
					const ExportArgument& _argument6 = emptyConstExportArgument,
					const ExportArgument& _argument7 = emptyConstExportArgument,
					const ExportArgument& _argument8 = emptyConstExportArgument,
					const ExportArgument& _argument9 = emptyConstExportArgument
					);

	/** Copy constructor (deep copy).
	 *
	 *	@param[in] arg		Right-hand side object.
	 */
	ExportFunction(	const ExportFunction& arg
					);

	/** Destructor.
	 */
	virtual ~ExportFunction( );

	/** Assignment operator (deep copy).
	 *
	 *	@param[in] arg		Right-hand side object.
	 */
	ExportFunction& operator=(	const ExportFunction& arg
								);

	/** Clone constructor (deep copy).
	 *
	 *	\return Pointer to cloned object.
	 */
	virtual ExportStatement* clone( ) const;

	virtual ExportFunction* cloneFunction( ) const;


	/** Initializes function with given name and possible calling arguments.
	 *
	 *	@param[in] _name		Name of the function.
	 *	@param[in] _argument1	Calling argument no. 1.
	 *	@param[in] _argument2	Calling argument no. 2.
	 *	@param[in] _argument3	Calling argument no. 3.
	 *	@param[in] _argument4	Calling argument no. 4.
	 *	@param[in] _argument5	Calling argument no. 5.
	 *	@param[in] _argument6	Calling argument no. 6.
	 *	@param[in] _argument7	Calling argument no. 7.
	 *	@param[in] _argument8	Calling argument no. 8.
	 *	@param[in] _argument9	Calling argument no. 9.
	 *
	 *	\return SUCCESSFUL_RETURN
	 */
	returnValue init(	const String& _name = "defaultFunctionName",
						const ExportArgument& _argument1 = emptyConstExportArgument,
						const ExportArgument& _argument2 = emptyConstExportArgument,
						const ExportArgument& _argument3 = emptyConstExportArgument,
						const ExportArgument& _argument4 = emptyConstExportArgument,
						const ExportArgument& _argument5 = emptyConstExportArgument,
						const ExportArgument& _argument6 = emptyConstExportArgument,
						const ExportArgument& _argument7 = emptyConstExportArgument,
						const ExportArgument& _argument8 = emptyConstExportArgument,
						const ExportArgument& _argument9 = emptyConstExportArgument
						);

	/** Initializes function with given name and possible calling arguments.
	 *
	 *	@param[in] _name		Name of the function.
	 *	@param[in] _argument1	Calling argument no. 1.
	 *	@param[in] _argument2	Calling argument no. 2.
	 *	@param[in] _argument3	Calling argument no. 3.
	 *	@param[in] _argument4	Calling argument no. 4.
	 *	@param[in] _argument5	Calling argument no. 5.
	 *	@param[in] _argument6	Calling argument no. 6.
	 *	@param[in] _argument7	Calling argument no. 7.
	 *	@param[in] _argument8	Calling argument no. 8.
	 *	@param[in] _argument9	Calling argument no. 9.
	 *
	 *	\return Reference to initialized object
	 */
	ExportFunction& setup(	const String& _name = "defaultFunctionName",
							const ExportArgument& _argument1 = emptyConstExportArgument,
							const ExportArgument& _argument2 = emptyConstExportArgument,
							const ExportArgument& _argument3 = emptyConstExportArgument,
							const ExportArgument& _argument4 = emptyConstExportArgument,
							const ExportArgument& _argument5 = emptyConstExportArgument,
							const ExportArgument& _argument6 = emptyConstExportArgument,
							const ExportArgument& _argument7 = emptyConstExportArgument,
							const ExportArgument& _argument8 = emptyConstExportArgument,
							const ExportArgument& _argument9 = emptyConstExportArgument
							);


	/** Adds up to nine calling arguments to the function.
	 *
	 *	@param[in] _name		Name of the function.
	 *	@param[in] _argument1	Calling argument no. 1.
	 *	@param[in] _argument2	Calling argument no. 2.
	 *	@param[in] _argument3	Calling argument no. 3.
	 *	@param[in] _argument4	Calling argument no. 4.
	 *	@param[in] _argument5	Calling argument no. 5.
	 *	@param[in] _argument6	Calling argument no. 6.
	 *	@param[in] _argument7	Calling argument no. 7.
	 *	@param[in] _argument8	Calling argument no. 8.
	 *	@param[in] _argument9	Calling argument no. 9.
	 *
	 *	\return SUCCESSFUL_RETURN
	 */
	returnValue addArgument(	const ExportArgument& _argument1,
								const ExportArgument& _argument2 = emptyConstExportArgument,
								const ExportArgument& _argument3 = emptyConstExportArgument,
								const ExportArgument& _argument4 = emptyConstExportArgument,
								const ExportArgument& _argument5 = emptyConstExportArgument,
								const ExportArgument& _argument6 = emptyConstExportArgument,
								const ExportArgument& _argument7 = emptyConstExportArgument,
								const ExportArgument& _argument8 = emptyConstExportArgument,
								const ExportArgument& _argument9 = emptyConstExportArgument
								);

	/** Assigns a return value to the function (by default, its return value is void).
	 *
	 *	@param[in] _functionReturnValue		New return value of the function.
	 *	@param[in] _returnAsPointer			Flag indicating whether value shall be returned as pointer.
	 *
	 *	\return SUCCESSFUL_RETURN
	 */
	ExportFunction& setReturnValue(	const ExportVariable& _functionReturnValue,
									BooleanType _returnAsPointer = BT_FALSE
									);


	/** Sets the name of the function. */
	ExportFunction&	setName(const String& _name);

	/** Returns the name of the function. */
	String getName( ) const;

	/** Exports data declaration of the function into given file. Its appearance can
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

	/** Exports forward declaration of the function into given file. Its appearance can
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

	/** Exports source code of the function into given file. Its appearance can
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

	/** Return number of calling arguments of the function. */
	unsigned getNumArguments( ) const;

	/** Add a new index (local) index to the function */
	ExportFunction& addIndex( const ExportIndex& _index );

	/** Add a new index (local) variable to the function. */
	ExportFunction& addVariable( const ExportVariable& _var );

	/** Acquire a local variable. */
	virtual ExportFunction& acquire( ExportIndex& obj );

	/** Release a local variable. */
	virtual ExportFunction& release( const ExportIndex& obj );

	/** Set a documentation string. */
	virtual ExportFunction& doc( const String& _doc );

	/** Set the function as private. If this is true, then do not export it's declaration. */
	virtual ExportFunction& setPrivate(	BooleanType _set = BT_TRUE );

	/** Is function private? */
	virtual BooleanType isPrivate() const;

protected:
	/** Frees internal dynamic memory to yield an empty function.
	 *
	 *	\return SUCCESSFUL_RETURN
	 */
	returnValue clear( );

	/** Name of the function. */
	String name;
	/** A description string. */
	String description;

	/** List of calling arguments. */
	ExportArgumentList functionArguments;
	/** Return value of the function (by default, if pointer is null, return value is void). */
	std::tr1::shared_ptr< ExportVariable > retVal;
	/** Flag indicating whether value shall be returned as pointer. */
	BooleanType returnAsPointer;
	/** Memory allocator */
	MemoryAllocatorPtr memAllocator;
	/** Vector of local variables. */
	std::vector< ExportVariable > localVariables;
	/** Private flag. In principle if this guy is true, do not export function declaration. */
	BooleanType flagPrivate;
};


CLOSE_NAMESPACE_ACADO


#endif  // ACADO_TOOLKIT_EXPORT_FUNCTION_HPP
