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
								BooleanType _doLoopUnrolling
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
									BooleanType _doLoopUnrolling
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




returnValue ExportForLoop::exportDataDeclaration(	FILE* file,
													const String& _realString,
													const String& _intString,
													int _precision
													) const
{
//	ExportStatementBlock::exportDataDeclaration(file, _realString, _intString, _precision);

	return SUCCESSFUL_RETURN;
}


returnValue ExportForLoop::exportCode(	FILE* file,
										const String& _realString,
										const String& _intString,
										int _precision
										) const
{
	returnValue status = sanityCheck();
	if (status != SUCCESSFUL_RETURN)
		return status;

	if ( doLoopUnrolling == BT_FALSE )
	{
		stringstream s;

		s << "for (" << loopVariable.getName().getName() << " = ";

		if (startValue.isGiven() == BT_TRUE)
		{
			s << startValue.getGivenValue();
		}
		else
		{
			s << startValue.getName().getName();
		}
		s << "; ";

		s << loopVariable.getName().getName() << " < ";
		if (finalValue.isGiven() == BT_TRUE)
		{
			s << finalValue.getGivenValue();
		}
		else
		{
			s << finalValue.getName().getName();
		}
		s << "; ";
		
		if (increment.isGiven() == BT_TRUE)
		{
			switch ( increment.getGivenValue() )
			{
				case 1:
					s << "++" << loopVariable.getName().getName();
					break;

				case -1:
					s << "--" << loopVariable.getName().getName();
					break;

				default:
					s << loopVariable.getName().getName() << " += " << increment.getGivenValue();
					break;
			}
		}
		else
		{
			s << loopVariable.getName().getName() << " += " << increment.getName().getName();
		}

		s << ")" << endl << "{" << endl;

		acadoFPrintf(file, "%s", s.str().c_str());

		ExportStatementBlock::exportCode(file, _realString, _intString, _precision);

		acadoFPrintf( file,"\n}\n");
	}
	
	return SUCCESSFUL_RETURN;
}



ExportForLoop& ExportForLoop::unrollLoop( )
{
	doLoopUnrolling = BT_TRUE;
	return *this;
}


ExportForLoop& ExportForLoop::keepLoop( )
{
	doLoopUnrolling = BT_FALSE;
	return *this;
}

returnValue ExportForLoop::allocate(memoryAllocatorPtr allocator)
{
	//
	// For loop itself cannot allocate any memory. Thus it just forwards
	// the pointer, so that later statements can allocate some memory.
	//
	statementPtrArray::const_iterator it = statements.begin();
	for(; it != statements.end(); ++it)
		(*it)->allocate( allocator );

	return SUCCESSFUL_RETURN;
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
	if (doLoopUnrolling == BT_TRUE)
		return ACADOERRORTEXT(RET_NOT_IMPLEMENTED_YET, "Loop unrolling is not yet implemented");

	if (startValue.isGiven() == BT_TRUE && finalValue.isGiven() == BT_TRUE && increment.isGiven() == BT_TRUE)
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
