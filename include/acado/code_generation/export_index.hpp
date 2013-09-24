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
 *    \file include/acado/code_generation/export_index.hpp
 *    \author Hans Joachim Ferreau, Boris Houska, Milan Vukov
 */


#ifndef ACADO_TOOLKIT_EXPORT_INDEX_HPP
#define ACADO_TOOLKIT_EXPORT_INDEX_HPP

#include <acado/utils/acado_utils.hpp>
#include <acado/code_generation/export_data.hpp>

BEGIN_NAMESPACE_ACADO

class ExportIndexNode;
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
 *	\author Hans Joachim Ferreau, Boris Houska, Milan Vukov
 */

class ExportIndex : public ExportData
{
public:

	ExportIndex();

	ExportIndex(	const int _value );

	explicit ExportIndex(	const String& _name,
							const String& _prefix = emptyConstString
							);

	ExportIndexNode* operator->();

	const ExportIndexNode* operator->() const;

	friend ExportIndex operator+(	const ExportIndex& _arg1,
									const ExportIndex& _arg2
									);


	friend ExportIndex operator-(	const ExportIndex& _arg1,
									const ExportIndex& _arg2
									);


	friend ExportIndex operator*(	const ExportIndex& _arg1,
									const ExportIndex& _arg2
									);


	friend ExportIndex operator/(	const ExportIndex& _arg1,
									const ExportIndex& _arg2
									);

	friend ExportIndex operator%(	const ExportIndex& _arg1,
									const ExportIndex& _arg2
									);

	friend String operator==(	const ExportIndex& _arg1,
								const ExportIndex& _arg2
								);

	virtual returnValue exportDataDeclaration(	FILE* file,
												const String& _realString = "real_t",
												const String& _intString = "int",
												int _precision = 16
												) const;

	/** Returns a string containing the value of the index.
	 *
	 *	\return String containing the value of the index.
	 */
	const String get( ) const;

	/** Returns the given value of the index (if defined).
	 *
	 *	\return Given value of the index or "undefinedValue".
	 */
	int getGivenValue( ) const;

	/** Returns whether the index is set to a given value.
	 *
	 *	\return BT_TRUE  iff index is set to a given value, \n
	 *	        BT_FALSE otherwise
	 */
	BooleanType isGiven( ) const;

	BooleanType isBinary() const;

	BooleanType isVariable() const;

	/** Converts index into a calling argument.
	 *
	 *	\return Index converted into a calling argument.
	 */
	ExportArgument makeArgument( ) const;
};

struct ExportIndexComparator
{
    bool operator() (const ExportIndex& val1, const ExportIndex& val2) const
    {
    	int tmp = std::string( val1.getName().getName() ).compare( std::string( val2.getName().getName() ) );

    	return (tmp < 0) ? true : false;
    }
};

static const ExportIndex emptyConstExportIndex( int( 0 ) );
static const ExportIndex constExportIndexValueOne( int( 1 ) );

CLOSE_NAMESPACE_ACADO


#endif  // ACADO_TOOLKIT_EXPORT_INDEX_HPP

// end of file.
