/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2014 by Boris Houska, Hans Joachim Ferreau,
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
 *    \file src/code_generation/export_index.cpp
 *    \author Hans Joachim Ferreau, Boris Houska, Milan Vukov
 *    \date 2011 - 2013
 */

#include <acado/code_generation/export_index.hpp>
#include <acado/code_generation/export_argument.hpp>
#include <acado/code_generation/export_index_node.hpp>

BEGIN_NAMESPACE_ACADO

//
// PUBLIC MEMBER FUNCTIONS:
//

ExportIndex::ExportIndex()
{}


ExportIndex::ExportIndex(	const int _value )
{
	assignNode(new ExportIndexNode( _value ));
}

ExportIndex::ExportIndex(	const std::string& _name,
							const std::string& _prefix )
{
	assignNode(new ExportIndexNode(_name, _prefix));
}


ExportIndexNode* ExportIndex::operator->()
{
	return (ExportIndexNode*)(ExportData::operator->());
}


const ExportIndexNode* ExportIndex::operator->() const
{
	return (const ExportIndexNode*)(ExportData::operator->());
}


returnValue ExportIndex::exportDataDeclaration(	std::ostream& stream,
												const std::string& _realString ,
												const std::string& _intString,
												int _precision
												) const
{
	return (*this)->exportDataDeclaration(stream, _realString, _intString, _precision);
}


ExportIndex operator+(	const ExportIndex& _arg1,
						const ExportIndex& _arg2
						)
{
	ExportIndex tmp;

	if (_arg1.isGiven() == true && _arg2.isGiven() == true)
	{
		tmp.assignNode(new ExportIndexNode(_arg1.getGivenValue() + _arg2.getGivenValue()));

		return tmp;
	}

	if (_arg1.isVariable() == true && _arg2.isGiven() == true)
	{
		tmp.assignNode(new ExportIndexNode(_arg1.getName(), _arg1.getPrefix(), _arg1->getFactor(), _arg1->getOffset() + _arg2.getGivenValue()));

		return tmp;
	}

	if (_arg1.isGiven() == true && _arg2.isVariable() == true)
	{
		tmp.assignNode(new ExportIndexNode(_arg2.getName(), _arg2.getPrefix(),  _arg2->getFactor(), _arg2->getOffset() + _arg1.getGivenValue()));

		return tmp;
	}

	if(_arg1.isVariable() == true && _arg2.isVariable() == true && _arg1.getFullName() == _arg2.getFullName())
	{
		if ((_arg1->getFactor() + _arg2->getFactor()) == 0)
			tmp.assignNode(new ExportIndexNode(_arg1->getOffset() + _arg2->getOffset()));
		else
			tmp.assignNode(new ExportIndexNode(_arg1.getName(), _arg1.getPrefix(), _arg1->getFactor() + _arg2->getFactor(), _arg1->getOffset() + _arg2->getOffset()));
	}
	else
	{
		tmp.assignNode(new ExportIndexNode(ESO_ADD, _arg1, _arg2));
	}

	return tmp;
}


ExportIndex operator-(	const ExportIndex& _arg1,
						const ExportIndex& _arg2
						)
{
	ExportIndex tmp;

	if (_arg1.isGiven() == true && _arg2.isGiven() == true)
	{
		tmp.assignNode(new ExportIndexNode(_arg1.getGivenValue() - _arg2.getGivenValue()));

		return tmp;
	}

	if (_arg1.isVariable() == true && _arg2.isGiven() == true)
	{
		tmp.assignNode(new ExportIndexNode(_arg1.getName(), _arg1.getPrefix(), _arg1->getFactor(), _arg1->getOffset() - _arg2.getGivenValue()));

		return tmp;
	}

	if (_arg1.isGiven() == true && _arg2.isVariable() == true)
	{
		tmp.assignNode(new ExportIndexNode(_arg2.getName(), _arg2.getPrefix(), -1 * _arg2->getFactor(), _arg1->getGivenValue() - _arg2->getOffset()));

		return tmp;
	}

	if(_arg1.isVariable() == true && _arg2.isVariable() == true && _arg1.getFullName() == _arg2.getFullName())
	{
		if ((_arg1->getFactor() - _arg2->getFactor()) == 0)
			tmp.assignNode(new ExportIndexNode(_arg1->getOffset() - _arg2->getOffset()));
		else
			tmp.assignNode(new ExportIndexNode(_arg1.getName(), _arg1.getPrefix(), _arg1->getFactor() - _arg2->getFactor(), _arg1->getOffset() - _arg2->getOffset()));
	}
	else
		tmp.assignNode(new ExportIndexNode(ESO_SUBTRACT, _arg1, _arg2));

	return tmp;
}


ExportIndex operator*(	const ExportIndex& _arg1,
						const ExportIndex& _arg2
						)
{
	ExportIndex tmp;

	if (_arg1.isGiven() == true && _arg2.isGiven() == true)
	{
		tmp.assignNode(new ExportIndexNode(_arg1.getGivenValue() * _arg2.getGivenValue()));

		return tmp;
	}

	if (_arg1.isVariable() == true && _arg2.isGiven() == true)
	{
		tmp.assignNode(new ExportIndexNode(_arg1.getName(), _arg1.getPrefix(), _arg1->getFactor() * _arg2.getGivenValue(), _arg1->getOffset() * _arg2.getGivenValue()));

		return tmp;
	}

	if (_arg1.isGiven() == true && _arg2.isVariable() == true)
	{
		tmp.assignNode(new ExportIndexNode(_arg2.getName(), _arg2.getPrefix(), _arg2->getFactor() * _arg1.getGivenValue(), _arg2->getOffset() * _arg1.getGivenValue()));

		return tmp;
	}

	tmp.assignNode(new ExportIndexNode(ESO_MULTIPLY, _arg1, _arg2));

	return tmp;
}


ExportIndex operator/(	const ExportIndex& _arg1,
						const ExportIndex& _arg2
						)
{
	ExportIndex tmp;

	if (_arg1.isGiven() == true && _arg2.isGiven() == true)
	{
		tmp.assignNode(new ExportIndexNode(_arg1.getGivenValue() / _arg2.getGivenValue()));

		return tmp;
	}

	tmp.assignNode(new ExportIndexNode(ESO_DIVIDE, _arg1, _arg2));

	return tmp;
}

ExportIndex operator%(	const ExportIndex& _arg1,
						const ExportIndex& _arg2
						)
{
	ExportIndex tmp;

	if (_arg1.isGiven() == true && _arg2.isGiven() == true)
	{
		tmp.assignNode(new ExportIndexNode(_arg1.getGivenValue() % _arg2.getGivenValue()));

		return tmp;
	}

	tmp.assignNode(new ExportIndexNode(ESO_MODULO, _arg1, _arg2));

	return tmp;
}

std::string operator==(	const ExportIndex& _arg1,
						const ExportIndex& _arg2
						)
{
	std::stringstream ret;
	ret << _arg1.get() << " = " << _arg2.get() << ";\n";

	return ret.str();
}


const std::string ExportIndex::get( ) const
{
	return (*this)->get();
}


int ExportIndex::getGivenValue( ) const
{
	return (*this)->getGivenValue();
}


bool ExportIndex::isGiven( ) const
{
	return (*this)->isGiven();
}

bool ExportIndex::isBinary( ) const
{
	return (*this)->isBinary();
}

bool ExportIndex::isVariable() const
{
	return (*this)->isVariable();
}


ExportIndex::operator ExportArgument()
{
	std::string tmpName;

	// XXX In principle, this is an ugly hack. In case when an index is given,
	// We give it a name which is equal to its value. This is done in order
	// To be able to simplify function calls. Most probably much more sense
	// would make that ExportArgument is base class for this guy...

	if (isGiven() == true)
		tmpName = std::toString(getGivenValue());
	else
		tmpName = (*this)->getName();

	ExportArgument tmp(tmpName, 1 , 1, (*this)->getType(), ACADO_LOCAL, true, emptyConstExportIndex);
	tmp.setDoc( getDoc() );

	return tmp;
}


CLOSE_NAMESPACE_ACADO

// end of file.
