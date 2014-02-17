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
 *    \file include/code_generation/memory_allocator.hpp
 *    \author Milan Vukov
 *    \date 2012
 */

#include <acado/code_generation/memory_allocator.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO

returnValue MemoryAllocator::acquire(ExportIndex& _obj)
{
	if (indices.busy() == true)
	{
		ExportIndex ind( string("lRun") + toString(indices.size() + 1) );

		indices.add( ind );

		_obj = ind;

		return SUCCESSFUL_RETURN;
	}

	indices.acquire( _obj );

	return SUCCESSFUL_RETURN;
}

returnValue MemoryAllocator::release(const ExportIndex& _obj)
{
	if (indices.release( _obj ) == false)
	{
		LOG( LVL_ERROR ) << "Object '" << _obj.getFullName() << "' is not found in an object pool" << endl;
		return ACADOERROR( RET_INVALID_ARGUMENTS );
	}

	return SUCCESSFUL_RETURN;
}

returnValue MemoryAllocator::add(const ExportIndex& _obj)
{
	if (indices.add( _obj ) == false)
	{
		LOG( LVL_WARNING ) << "Object '" << _obj.getFullName() << "' already exists in an object pool" << endl;
		return ACADOERROR( RET_INVALID_ARGUMENTS );
	}

	return SUCCESSFUL_RETURN;
}

std::vector< ExportIndex > MemoryAllocator::getPool( void )
{
	return indices.getPool();
}

CLOSE_NAMESPACE_ACADO
