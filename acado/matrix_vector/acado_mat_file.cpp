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
 *    \file src/matrix_vector/acado_mat_file.cpp
 *    \author Carlo Savorgnan, Hans Joachim Ferreau, Boris Houska, Milan Vukov
 *    \date 2010 - 2013
 */

#include <acado/matrix_vector/acado_mat_file.hpp>

BEGIN_NAMESPACE_ACADO

template<typename T>
void MatFile< T >::write(	std::ostream& stream,
						const GenericMatrix< T >& mat,
						const char* name
						)
{
	Fmatrix x;
	double tmp;
	
	x.type = 0000;  // IEEE Little Endian - reserved - double precision (64 bits) - numeric full matrix
	x.mrows = mat.getNumRows();  // number of rows
	x.ncols = mat.getNumCols();  // number of columns
	x.imagf = 0;  // no imaginary part
	x.namelen = 1+strlen(name); // matrix name length 
	
	stream.write( (char*) &x, sizeof(Fmatrix));
	stream.write(name, x.namelen);
	
	// mat files store data in column-major format
	for (uint col = 0; col < mat.getNumCols(); ++col)
	{
		for (uint row = 0; row < mat.getNumRows(); ++row)
		{
			tmp = mat(row, col);
			stream.write((char*) &tmp, sizeof(double));
		}
	}
	
	// imaginary numbers should be stored just after the real ones
}

template<typename T>
void MatFile< T >::write(	std::ostream& stream,
						const GenericVector< T >& vec,
						const char* name
						)
{
	Fmatrix x;
	double tmp;
	
	x.type = 0000;  // IEEE Big Endian - reserved - double precision (64 bits) - numeric full matrix
	x.mrows = vec.getDim();  // number of rows
	x.ncols = 1;  // number of columns
	x.imagf = 0;  // no imaginary part
	x.namelen = 1+strlen(name); // matrix name length 
	
	stream.write( (char*) &x, sizeof(Fmatrix));
	stream.write(name, x.namelen);
	
	// mat files store data in column-major format
	for (uint row=0; row<vec.getDim(); ++row)
	{
		tmp = vec( row );
		stream.write((char*) &tmp, sizeof(double));
	}
	
	// imaginary numbers should be stored just after the real ones
}

//
// Explicit instantiations of templates.
//
template class MatFile< double >;
template class MatFile< int >;
template class MatFile< bool >;

CLOSE_NAMESPACE_ACADO
