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
 *    \file
 *    \author Milan Vukov
 *    \date 2012
 */


#ifndef ACADO_TOOLKIT_EXPORT_TEMPLATED_FILE_HPP
#define ACADO_TOOLKIT_EXPORT_TEMPLATED_FILE_HPP


#include <acado/code_generation/export_file.hpp>

#include <map>
#include <string>


BEGIN_NAMESPACE_ACADO

class ExportQpOasesInterface;


/** 
 *	\brief ...
 *
 *	\ingroup ...
 *
 *	\author Milan Vukov
 */
class ExportTemplatedFile : public ExportFile
{
    //
    // PUBLIC MEMBER FUNCTIONS:
    //

    public:

		friend class ExportQpOasesInterface;

		/** Default constructor. 
		 *
		 *	@param[in] _templateName		Name of a template.
		 *	@param[in] _fileName			Name of exported file.
		 *	@param[in] _commonHeaderName	Name of common header file to be included.
		 *	@param[in] _realString			String to be used to declare real variables.
		 *	@param[in] _intString			String to be used to declare integer variables.
		 *	@param[in] _precision			Number of digits to be used for exporting real values.
		 *	@param[in] _commentString		String to be used for exporting comments.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		ExportTemplatedFile(	const String& _templateName,
								const String& _fileName,
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
        ExportTemplatedFile(	const ExportTemplatedFile& arg
								);

        /** Destructor. 
		 */
        virtual ~ExportTemplatedFile( );

		/** Assignment operator (deep copy).
		 *
		 *	@param[in] arg		Right-hand side object.
		 */
		ExportTemplatedFile& operator=(	const ExportTemplatedFile& arg
										);

		/** Exports the file containing the auto-generated code.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue exportCode( ) const;
		
		/** Configure the template
		 *
		 *  \return SUCCESSFUL_RETURN
		 */
		virtual returnValue configure(  )
		{
			return fillTemplate( );
		}

	protected:

		/** Copies all class members from given object.
		 *
		 *	@param[in] arg		Right-hand side object.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue copy(	const ExportTemplatedFile& arg
							);
							
		returnValue fillTemplate( );
    
    	String templateName;
    
    	std::map< std::string, std::string > dictionary;	
};


CLOSE_NAMESPACE_ACADO


#endif  // ACADO_TOOLKIT_EXPORT_TEMPLATED_FILE_HPP
