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
 *    \file src/code_generation/export_for_loop.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 */


#include <acado/code_generation/export_for_loop.hpp>

#include <sstream>

BEGIN_NAMESPACE_ACADO

using namespace std;


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportForLoop::ExportForLoop(	const ExportIndex& _loopVariable,
								const ExportIndex& _startValue,
								const ExportIndex& _finalValue,
								const ExportIndex& _increment,
								bool _doLoopUnrolling
							) : ExportStatementBlock( ),
									loopVariable( _loopVariable ),
									startValue( _startValue ),
									finalValue( _finalValue ),
									increment( _increment ),
									doLoopUnrolling( _doLoopUnrolling )
{
	sanityCheck();
}


ExportForLoop::ExportForLoop( const ExportForLoop& arg ) :	ExportStatementBlock( arg ),
															loopVariable( arg.loopVariable ),
															startValue( arg.startValue ),
															finalValue( arg.finalValue ),
															increment( arg.increment ),
															doLoopUnrolling( arg.doLoopUnrolling )
{
}


ExportForLoop::~ExportForLoop( )
{
	clear( );
}


ExportForLoop& ExportForLoop::operator=( const ExportForLoop& arg )
{
	if ( this != &arg )
	{
		ExportStatementBlock::operator=( arg );
		init(arg.loopVariable, arg.startValue, arg.finalValue, arg.increment, arg.doLoopUnrolling);
	}

	return *this;
}


ExportStatement* ExportForLoop::clone( ) const
{
	return new ExportForLoop( *this );
}


returnValue ExportForLoop::init(	const ExportIndex& _loopVariable,
									const ExportIndex& _startValue,
									const ExportIndex& _finalValue,
									const ExportIndex&  _increment,
									bool _doLoopUnrolling
									)
{
	clear();

	loopVariable = _loopVariable;
	startValue = _startValue;
	finalValue = _finalValue;
	increment  = _increment;
	doLoopUnrolling = _doLoopUnrolling;

	return SUCCESSFUL_RETURN;
}




returnValue ExportForLoop::exportDataDeclaration(	std::ostream& stream,
													const std::string& _realString,
													const std::string& _intString,
													int _precision
													) const
{
	return SUCCESSFUL_RETURN;
}


returnValue ExportForLoop::exportCode(	std::ostream& stream,
										const std::string& _realString,
										const std::string& _intString,
										int _precision
										) const
{
	returnValue status = sanityCheck();
	if (status != SUCCESSFUL_RETURN)
		return status;

	if (startValue.isGiven() == true && finalValue.isGiven() == true)
		if (startValue.getGivenValue() == finalValue.getGivenValue())
			return SUCCESSFUL_RETURN;

	if ( doLoopUnrolling == false )
	{
		stream << "for (" << loopVariable.get() << " = " << startValue.get() << "; ";

		if (increment.isGiven() ==  true && increment.getGivenValue() == -1)
			stream << finalValue.get() << " < " << loopVariable.get() << "; ";
		else
			stream << loopVariable.get() << " < " << finalValue.get() << "; ";
		
		if (increment.isGiven() == true)
		{
			switch ( increment.getGivenValue() )
			{
				case 1:
					stream << "++" << loopVariable.get();
					break;

				case -1:
					stream << "--" << loopVariable.get();
					break;

				default:
					stream << loopVariable.get() << " += " << increment.getGivenValue();
					break;
			}
		}
		else
		{
			stream << loopVariable.get() << " += " << increment.get();
		}

		stream << ")" << endl << "{" << endl;

		ExportStatementBlock::exportCode(stream, _realString, _intString, _precision);

		stream << "}\n";
	}
	
	return SUCCESSFUL_RETURN;
}



ExportForLoop& ExportForLoop::unrollLoop( )
{
	doLoopUnrolling = true;
	return *this;
}


ExportForLoop& ExportForLoop::keepLoop( )
{
	doLoopUnrolling = false;
	return *this;
}

ExportForLoop& ExportForLoop::allocate(MemoryAllocatorPtr allocator)
{
	//
	// For loop itself cannot allocate any memory. Thus it just forwards
	// the pointer, so that later statements can allocate some memory.
	//
	StatementPtrArray::const_iterator it = statements.begin();
	for(; it != statements.end(); ++it)
		(*it)->allocate( allocator );

	return *this;
}


//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue ExportForLoop::clear( )
{
	return SUCCESSFUL_RETURN;
}

//
// PRIVATE MEMBER FUNCTIONS:
//
returnValue ExportForLoop::sanityCheck() const
{
	if (doLoopUnrolling == true)
		return ACADOERRORTEXT(RET_NOT_IMPLEMENTED_YET, "Loop unrolling is not yet implemented");

	if (startValue.isGiven() == true && finalValue.isGiven() == true && increment.isGiven() == true)
	{
		if ( ( startValue.getGivenValue() > finalValue.getGivenValue() ) && ( increment.getGivenValue() >= 0 ) )
			return ACADOERRORTEXT(RET_INVALID_ARGUMENTS, "Export for loop arguments are invalid");

		if ( ( startValue.getGivenValue() < finalValue.getGivenValue() ) && ( increment.getGivenValue() <= 0 ) )
			return ACADOERRORTEXT(RET_INVALID_ARGUMENTS, "Export for loop arguments are invalid");
	}

	return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO

// end of file.
