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
 *    \file ...
 *    \author Rien Quirynen
 *    \date 2012
 */


#ifndef ACADO_TOOLKIT_EXPORT_MATLAB_RHS_HPP
#define ACADO_TOOLKIT_EXPORT_MATLAB_RHS_HPP


#include <acado/code_generation/export_templated_file.hpp>


BEGIN_NAMESPACE_ACADO


/** 
 *	\brief ...
 *
 *	\ingroup ...
 *
 *	\author Rien Quirynen
 */
class ExportMatlabRhs : public ExportTemplatedFile
{
    //
    // PUBLIC MEMBER FUNCTIONS:
    //

    public:

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
		ExportMatlabRhs(	const String& _templateName,
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
        ExportMatlabRhs(	const ExportMatlabRhs& arg
								);

        /** Destructor. 
		 */
        virtual ~ExportMatlabRhs( );

		/** Assignment operator (deep copy).
		 *
		 *	@param[in] arg		Right-hand side object.
		 */
		ExportMatlabRhs& operator=(	const ExportMatlabRhs& arg
										);
		
		/** Configure the template
		 *
		 *  \return SUCCESSFUL_RETURN
		 */
		returnValue configure(	const String& nameFunction );

	protected:

};


CLOSE_NAMESPACE_ACADO


#endif  // ACADO_TOOLKIT_EXPORT_MATLAB_RHS_HPP
