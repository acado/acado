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
 *    \file include/acado/integrator/export_file.hpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010-2011
 */


#ifndef ACADO_TOOLKIT_EXPORT_FILE_HPP
#define ACADO_TOOLKIT_EXPORT_FILE_HPP

#include <acado/utils/acado_utils.hpp>
#include <acado/user_interaction/algorithmic_base.hpp>
#include <acado/matrix_vector/matrix_vector.hpp>

#include <acado/code_generation/export_variable.hpp>
#include <acado/code_generation/export_statement_block.hpp>



BEGIN_NAMESPACE_ACADO


/** 
 *	\brief Allows to export files containing automatically generated algorithms for fast model predictive control
 *
 *	\ingroup NumericalAlgorithms
 *
 *	The class ExportFile allows to export files containing automatically generated 
 *	algorithms for fast model predictive control.
 *
 *	\author Hans Joachim Ferreau, Milan Vukov, Boris Houska
 */
class ExportFile : public ExportStatementBlock
{
    //
    // PUBLIC MEMBER FUNCTIONS:
    //

    public:

		/** Default constructor. 
		 *
		 *	@param[in] _fileName			Name of exported file.
		 *	@param[in] _commonHeaderName	Name of common header file to be included.
		 *	@param[in] _realString			String to be used to declare real variables.
		 *	@param[in] _intString			String to be used to declare integer variables.
		 *	@param[in] _precision			Number of digits to be used for exporting real values.
		 *	@param[in] _commentString		String to be used for exporting comments.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		ExportFile(	const String& _fileName,
					const String& _commonHeaderName = "",
					const String& _realString = "real_t",
					const String& _intString = "int",
					int _precision = 16,
					const String& _commentString = emptyConstString
					);

		/** Copy constructor (deep copy).
		 *
		 *	@param[in] arg		Right-hand side object.
		 */
        ExportFile(	const ExportFile& arg
					);

        /** Destructor. 
		 */
        virtual ~ExportFile( );

		/** Assignment operator (deep copy).
		 *
		 *	@param[in] arg		Right-hand side object.
		 */
		ExportFile& operator=(	const ExportFile& arg
								);


		/** Exports the file containing the auto-generated code.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		virtual returnValue exportCode( ) const;



	protected:

		/** Copies all class members from given object.
		 *
		 *	@param[in] arg		Right-hand side object.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue copy(	const ExportFile& arg
							);

		/** Opens given file and prepares it for exporting code.
		 *
		 *  \return Pointer to prepared file with given name, \n
		 *	        NULL iff file could not be opened
		 */
		FILE* openFile( ) const;



    protected:

		String fileName;					/**< Name of exported file. */
		String commonHeaderName;			/**< Name of common header file. */
		
		String realString;					/**< String to be used to declare real variables. */
		String intString;					/**< String to be used to declare integer variables. */
		int precision;						/**< Number of digits to be used for exporting real values. */
		String commentString;				/**< String to be used for exporting comments. */
};


CLOSE_NAMESPACE_ACADO


#endif  // ACADO_TOOLKIT_EXPORT_FILE_HPP

// end of file.
