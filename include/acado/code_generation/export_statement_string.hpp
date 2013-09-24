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
 *    \file include/acado/code_generation/export_statement_string.hpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010-2011
 */


#ifndef ACADO_TOOLKIT_EXPORT_STATEMENT_STRING_HPP
#define ACADO_TOOLKIT_EXPORT_STATEMENT_STRING_HPP

#include <acado/utils/acado_utils.hpp>
#include <acado/code_generation/export_statement.hpp>


BEGIN_NAMESPACE_ACADO


/** 
 *	\brief Allows to export code writing a string.
 *
 *	\ingroup AuxiliaryFunctionality
 *
 *  The class ExportStatementString allows to export code writing a string.
 *
 *	\author Hans Joachim Ferreau, Boris Houska
 */
class ExportStatementString : public ExportStatement
{
    //
    // PUBLIC MEMBER FUNCTIONS:
    //
    public:

		/** Default constructor which optionally takes the string to be exported.
		 *
		 *	@param[in] _statementString		String to be exported.
		 */
        ExportStatementString(	const String& _statementString = " "
								);

		/** Copy constructor (deep copy).
		 *
		 *	@param[in] arg		Right-hand side object.
		 */
        ExportStatementString(	const ExportStatementString& arg
								);

        /** Destructor.
		 */
        virtual ~ExportStatementString( );

		/** Assignment operator (deep copy).
		 *
		 *	@param[in] arg		Right-hand side object.
		 */
        ExportStatementString& operator=(	const ExportStatementString& arg
											);

		/** Clone constructor (deep copy).
		 *
		 *	\return Pointer to cloned object.
		 */
		virtual ExportStatement* clone( ) const;


		/** Exports source code of the string into given file. Its appearance can 
		 *  can be adjusted by various options.
		 *
		 *	@param[in] file				Name of file to be used to export string.
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


	//
    // PROTECTED MEMBER FUNCTIONS:
    //
    protected:



    protected:

		String statementString;					/**< String to be exported. */
};


CLOSE_NAMESPACE_ACADO


#endif  // ACADO_TOOLKIT_EXPORT_STATEMENT_STRING_HPP

// end of file.
