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
 *    \file include/acado/code_generation/export_statement.hpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010-2011
 */


#ifndef ACADO_TOOLKIT_EXPORT_STATEMENT_HPP
#define ACADO_TOOLKIT_EXPORT_STATEMENT_HPP

#include <acado/utils/acado_utils.hpp>



BEGIN_NAMESPACE_ACADO




/** 
 *	\brief Base class for all kind of statements to be exported by the code generation tool.
 *
 *	\ingroup AuxiliaryFunctionality
 *
 *	The class ExportStatement serves as a base class for all kind of statements to be exported 
 *	by the code generation tool.
 *
 *	\author Hans Joachim Ferreau, Boris Houska
 */
class ExportStatement
{
    //
    // PUBLIC MEMBER FUNCTIONS:
    //
    public:

		/** Default constructor. 
		 */
        ExportStatement( );

		/** Copy constructor (deep copy).
		 *
		 *	@param[in] arg		Right-hand side object.
		 */
		ExportStatement(	const ExportStatement& arg
							);

        /** Destructor. 
		 */
        virtual ~ExportStatement( );

		/** Assignment operator (deep copy).
		 *
		 *	@param[in] arg		Right-hand side object.
		 */
        ExportStatement& operator=(	const ExportStatement& arg
									);

		/** Clone constructor (deep copy).
		 *
		 *	\return Pointer to cloned object.
		 */
		virtual ExportStatement* clone( ) const = 0;


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
										) const = 0;


	//
    // PROTECTED MEMBER FUNCTIONS:
    //
    protected:

		/** Returns a string containing the name of the next unused run variable.
		 *
		 *	\return String containing the name of the next run variable.
		 */
		String getNextRunVariable( );



    protected:
		
		static uint runVariableCounter;				/**< Static counter for run variables. */

};


CLOSE_NAMESPACE_ACADO


#endif  // ACADO_TOOLKIT_EXPORT_STATEMENT_HPP

// end of file.
