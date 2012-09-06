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
 *    \file include/acado/code_generation/export_index.hpp
 *    \author Hans Joachim Ferreau, Boris Houska
 */


#ifndef ACADO_TOOLKIT_EXPORT_INDEX_HPP
#define ACADO_TOOLKIT_EXPORT_INDEX_HPP

#include <acado/utils/acado_utils.hpp>
#include <acado/code_generation/export_data.hpp>


BEGIN_NAMESPACE_ACADO


class ExportArgument;


/** 
 *	\brief Defines a scalar-valued index variable to be used for exporting code.
 *
 *	\ingroup AuxiliaryFunctionality
 *
 *	The class ExportIndex defines a scalar-valued index variable to be used for 
 *	exporting code. Instances of this class can be used similar to usual integers 
 *	but offer additional functionality, e.g. they allow to export arithmetic 
 *	expressions involving indices of the form:
 *
 *	<factor> * index + <offset>,
 *
 *	where <factor> and <offset> are fixed integers.
 *
 *	\author Hans Joachim Ferreau, Boris Houska
 */
class ExportIndex : public ExportData
{
    //
    // PUBLIC MEMBER FUNCTIONS:
    //
    public:

		/** Default constructor which optionally takes name and type
		 *	of the index.
		 *
		 *	@param[in] _name			Name of the index.
		 *	@param[in] _typeString		String containing the type of the index.
		 *	@param[in] _value			Value of the index.
		 *	@param[in] _factor			Factor to create expression of the form "factor * index".
		 *	@param[in] _offset			Offset to create expression of the form "index + offset".
		 */
		ExportIndex(	const String& _name = "i",
						ExportType _type = INT,
						const int* const _value  = 0,
						const int* const _factor = 0,
						const int* const _offset = 0
						);

		/** Default constructor which optionally takes name and type
		 *	of the index.
		 *
		 *	@param[in] _value			Value of the index.
		 */
		ExportIndex(	int _value
						);

		/** Copy constructor (deep copy).
		 *
		 *	@param[in] arg		Right-hand side object.
		 */
        ExportIndex(	const ExportIndex& arg
						);

        /** Destructor.
		 */
        virtual ~ExportIndex( );

		/** Assignment operator (deep copy).
		 *
		 *	@param[in] arg		Right-hand side object.
		 */
        ExportIndex& operator=(	const ExportIndex& arg
								);

		/** Assignment operator for assigning a given integer to an ExportIndex.
		 *	Note that a possibly given offset or factor remain unchanged!
		 *
		 *	@param[in] arg		Right-hand side object.
		 */
		ExportIndex& operator=(	int _value
								);

		/** Clone constructor (deep copy).
		 *
		 *	\return Pointer to cloned object.
		 */
		virtual ExportData* clone( ) const;


		/** Initializes index with given name and type.
		 *
		 *	@param[in] _name			Name of the index.
		 *	@param[in] _typeString		String containing the type of the index.
		 *	@param[in] _value			Value of the index.
		 *	@param[in] _factor			Factor to create expression of the form "factor * index".
		 *	@param[in] _offset			Offset to create expression of the form "index + offset".
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue init(	const String& _name = "i",
							ExportType _type = INT,
							const int* const _value  = 0,
							const int* const _factor = 0,
							const int* const _offset = 0
							);

							
		/** Exports declaration of the index variable. Its appearance can 
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


		/** Returns a string containing the value of the index.
		 *
		 *	\return String containing the value of the index.
		 */
		const char* get( ) const;

		/** Returns the given value of the index (if defined).
		 *
		 *	\return Given value of the index or "undefinedValue".
		 */
		const int getGivenValue( ) const;


		/** Operator for adding two ExportIndices.
		 *
		 *	@param[in] arg		Index to be added.
		 *
		 *	\return Index containing the value after addition
		 */
		ExportIndex operator+(	const ExportIndex& arg
								) const;

		/** Operator for subtracting an ExportIndex from another one.
		 *
		 *	@param[in] arg		Index to be subtracted.
		 *
		 *	\return Index containing the value after subtraction
		 */
		ExportIndex operator-(	const ExportIndex& arg
								) const;


		/** Operator for adding a given integer to an ExportIndex.
		 *
		 *	@param[in] _offset		Integer to be added.
		 *
		 *	\return Index containing the value after addition
		 */
		ExportIndex operator+(	int _offset
								) const;

		/** Operator for subtracting a given integer from an ExportIndex.
		 *
		 *	@param[in] _offset		Integer to be subtracted.
		 *
		 *	\return Index containing the value after subtraction
		 */
		ExportIndex operator-(	int _offset
								) const;

		/** Operator for multiplying a given integer with an ExportIndex.
		 *
		 *	@param[in] _factor		Integer to be multiplied with.
		 *
		 *	\return Index containing the value after multiplication
		 */
		ExportIndex operator*(	int _factor
								) const;


		/** Returns whether the index is set to a given value.
		 *
		 *	\return BT_TRUE  iff index is set to a given value, \n
		 *	        BT_FALSE otherwise
		 */
		virtual BooleanType isGiven( ) const;


		/** Converts index into a calling argument.
		 *
		 *	\return Index converted into a calling argument.
		 */
		ExportArgument makeArgument( ) const;



	//
    // PROTECTED MEMBER FUNCTIONS:
    //
    protected:

		/** Frees internal dynamic memory to yield an empty index.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue clear( );


    protected:

		int* value;								/**< Value of the index. */
		int* factor;							/**< Factor to create expression of the form "factor * index". */
		int* offset;							/**< Offset to create expression of the form "index + offset". */
};


static const int emptyConstExportIndexValue = 0;
static const ExportIndex emptyConstExportIndex( (String)"i",INT,&emptyConstExportIndexValue );


CLOSE_NAMESPACE_ACADO


#endif  // ACADO_TOOLKIT_EXPORT_INDEX_HPP

// end of file.
