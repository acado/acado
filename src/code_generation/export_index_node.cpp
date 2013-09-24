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
#include <acado/code_generation/export_index_node.hpp>
#include <sstream>

BEGIN_NAMESPACE_ACADO

using namespace std;


returnValue ExportIndexNode::exportDataDeclaration(	FILE* file,
													const String& _realString,
													const String& _intString,
													int _precision
													) const
{
	stringstream s;

	if (isGiven() == BT_TRUE)
		return ACADOERRORTEXT(RET_UNABLE_TO_EXPORT_CODE, "Declaration of given indices is not supported.");
	else if (isBinary() == BT_TRUE)
		return ACADOERRORTEXT(RET_UNABLE_TO_EXPORT_CODE, "Declaration of binary node indices is not supported.");

	s << _intString.getName() << " " << getFullName().getName() << ";" << endl;

	return acadoFPrintf(file, "%s", s.str().c_str());
}


const String ExportIndexNode::get( ) const
{
	stringstream s;

	switch ( varType )
	{
	case EVT_VALUE:
		s << value;
		break;

	case EVT_VARIABLE:
		if (factor == 1)
			s << getFullName().getName();
		else
			s << getFullName().getName() << " * " << factor;

		if (offset > 0)
			s << " + " << offset;
		else if (offset < 0)
			s << offset;

		break;

	case EVT_BINARY_OPERATOR:

		s << "(" << left.get().getName() << ")";
		switch ( op )
		{
		case ESO_ADD:
			s << " + ";
			break;

		case ESO_SUBTRACT:
			s << " - ";
			break;

		case ESO_MULTIPLY:
			s << " * ";
			break;

		case ESO_DIVIDE:
			s << " / ";
			break;
		case ESO_MODULO:
			s << " % ";
			break;
		}
		s << "(" << right.get().getName() << ")";
		break;
	}

	String str( s.str().c_str() );

	return str;
}


const int ExportIndexNode::getGivenValue( ) const
{
	if (varType == EVT_VALUE)
		return value;

	if (varType == EVT_VARIABLE)
		return -1;

	switch ( op )
	{
	case ESO_ADD:
		return left.getGivenValue() + right.getGivenValue();

	case ESO_SUBTRACT:
		return left.getGivenValue() - right.getGivenValue();

	case ESO_MULTIPLY:
		return left.getGivenValue() * right.getGivenValue();

	case ESO_DIVIDE:
		return left.getGivenValue() / right.getGivenValue();

	case ESO_MODULO:
		return left.getGivenValue() % right.getGivenValue();

	default:
		return 0;
	}
}


BooleanType ExportIndexNode::isGiven( ) const
{
	if (varType == EVT_VALUE)
		return BT_TRUE;

	return BT_FALSE;
}


CLOSE_NAMESPACE_ACADO
