/*
 *	This file is part of ACADO Toolkit.
 *
 *	ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *	Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
 *	Developed within the Optimization in Engineering Center (OPTEC) under
 *	supervision of Moritz Diehl. All rights reserved.
 *
 *	ACADO Toolkit is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either
 *	version 3 of the License, or (at your option) any later version.
 *
 *	ACADO Toolkit is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *	Lesser General Public License for more details.
 *
 *	You should have received a copy of the GNU Lesser General Public
 *	License along with ACADO Toolkit; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/**
*	\file include/acado/code_generation/export_for_loop.hpp
*	\author Hans Joachim Ferreau, Boris Houska
*    \date 2010-2011
*/



#ifndef ACADO_TOOLKIT_EXPORT_FOR_LOOP_HPP
#define ACADO_TOOLKIT_EXPORT_FOR_LOOP_HPP


#include <acado/utils/acado_utils.hpp>
#include <acado/code_generation/export_index.hpp>
#include <acado/code_generation/export_statement_block.hpp>


BEGIN_NAMESPACE_ACADO


/** 
 *	\brief Allows to export code of a for-loop.
 *
 *	\ingroup AuxiliaryFunctionality
 *
 *	The class ExportForLoop allows to export code of a for-loop.
 *
 *	\author Hans Joachim Ferreau, Boris Houska
 */
class ExportForLoop : public ExportStatementBlock
{
	//
	// PUBLIC MEMBER FUNCTIONS:
	//
	public:

		/** Default constructor which optionally takes the name of the 
		 *	loop variable as well as other loop settings.
		 *
		 *	@param[in] _loopVariable		Name of the loop variable.
		 *	@param[in] _startValue			Start value of the loop counter.
		 *	@param[in] _finalValue			Final value of the loop counter.
		 *	@param[in] _increment			Increment of the loop counter.
		 *	@param[in] _doLoopUnrolling		Flag indicating whether loop shall be unrolled.
		 */
		ExportForLoop(	const String& _loopVariable = "run1",
						int _startValue = 0,
						int _finalValue = 0,
						int _increment = 1,
						BooleanType _doLoopUnrolling = BT_FALSE
						);

		/** Constructor which takes the loop variable as well as other 
		 *	loop settings.
		 *
		 *	@param[in] _loopVariable		Name of the loop variable.
		 *	@param[in] _startValue			Start value of the loop counter.
		 *	@param[in] _finalValue			Final value of the loop counter.
		 *	@param[in] _increment			Increment of the loop counter.
		 *	@param[in] _doLoopUnrolling		Flag indicating whether loop shall be unrolled.
		 */
		ExportForLoop(	const ExportIndex& _loopVariable,
						int _startValue = 0,
						int _finalValue = 0,
						int _increment = 1,
						BooleanType _doLoopUnrolling = BT_FALSE
						);

		/** Copy constructor (deep copy).
		 *
		 *	@param[in] arg	Right-hand side object.
		 */
		ExportForLoop(	const ExportForLoop& arg
						);

		/** Destructor.
		 */
		virtual ~ExportForLoop( );

		/** Assignment operator (deep copy).
		 *
		 *	@param[in] arg	Right-hand side object.
		 */
		ExportForLoop& operator=(	const ExportForLoop& arg
									);

		/** Clone constructor (deep copy).
		 *
		 *	\return Pointer to cloned object.
		 */
		virtual ExportStatement* clone( ) const;


		/** Initializes for-loop with given loop settings.
		 *
		 *	@param[in] _loopVariable		Name of the loop variable.
		 *	@param[in] _startValue			Start value of the loop counter.
		 *	@param[in] _finalValue			Final value of the loop counter.
		 *	@param[in] _increment			Increment of the loop counter.
		 *	@param[in] _doLoopUnrolling		Flag indicating whether loop shall be unrolled.
		 */
		returnValue init(	const String& _loopVariable = "run1",
							int _startValue = 0,
							int _finalValue = 0,
							int _increment = 1,
							BooleanType _doLoopUnrolling = BT_FALSE
							);

		/** Initializes for-loop with given loop settings.
		 *
		 *	@param[in] _loopVariable		Name of the loop variable.
		 *	@param[in] _startValue			Start value of the loop counter.
		 *	@param[in] _finalValue			Final value of the loop counter.
		 *	@param[in] _increment			Increment of the loop counter.
		 *	@param[in] _doLoopUnrolling		Flag indicating whether loop shall be unrolled.
		 */
		returnValue init(	const ExportIndex& _loopVariable,
							int _startValue = 0,
							int _finalValue = 0,
							int _increment = 1,
							BooleanType _doLoopUnrolling = BT_FALSE
							);


		/** Exports data declaration of the statement into given file. Its appearance can 
		 *  can be adjusted by various options.
		 *
		 *	@param[in] file				Name of file to be used to export statement.
		 *	@param[in] _realString		String to be used to declare real variables.
		 *	@param[in] _intString		String to be used to declare integer variables.
		 *	@param[in] _precision		Number of digits to be used for exporting real values.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		virtual returnValue exportDataDeclaration(	FILE *file,
													const String& _realString = "real_t",
													const String& _intString = "int",
													int _precision = 16
													) const;

		/** Exports source code of the statement into given file. Its appearance can 
		 *  can be adjusted by various options.
		 *
		 *	@param[in] file				Name of file to be used to export statement.
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


		/** Specifies to unroll for-loop.
		 *
		 *	\return Reference to for-loop object
		 */
		ExportForLoop& unrollLoop( );

		/** Specifies not to unroll for-loop.
		 *
		 *	\return Loop-unrolled copy of for-loop object
		 */
		ExportForLoop& keepLoop( );


	//
	// PROTECTED MEMBER FUNCTIONS:
	//
	protected:

		/** Frees internal dynamic memory to yield an empty for-loop.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue clear( );


	//
	// DATA MEMBERS:
	//
	protected:

		ExportIndex loopVariable;					/**< Loop variable. */
		
		int startValue;								/**< Start value of the loop counter. */
		int finalValue;								/**< Final value of the loop counter. */
		int increment;								/**< Increment of the loop counter. */
		
		BooleanType doLoopUnrolling;				/**< Flag indicating whether loop shall be unrolled when exporting the code. */
};


CLOSE_NAMESPACE_ACADO



#endif	// ACADO_TOOLKIT_EXPORT_FOR_LOOP_HPP


/*
 *	end of file
 */
