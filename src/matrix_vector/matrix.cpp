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
 *    \file src/matrix_vector/matrix.cpp
 *    \author Hans Joachim Ferreau, Boris Houska, Milan Vukov
 *    \date 2008 - 2013
 */

#include <acado/matrix_vector/matrix.hpp>
#include <acado/matrix_vector/acado_mat_file.hpp>
#include <iomanip>

using namespace std;

BEGIN_NAMESPACE_ACADO

template<typename T>
GenericMatrix<T>::GenericMatrix(	unsigned _nRows,
									unsigned _nCols,
									std::vector< std::vector< T > >& _values
									)
	: Base(_nRows, _nCols)
{
	ASSERT( _values.size() > 0 );

	unsigned nRows = _values.size();
	unsigned nCols = _values[ 0 ].size();

	for (unsigned r = 0; r < nRows; ++r)
	{
		ASSERT( _values[ r ].size() == nCols );

		std::copy(_values[ r ].begin(), _values[ r ].end(), Base::data() + r * nCols);
	}
}

template<typename T>
GenericMatrix<T>& GenericMatrix<T>::appendRows(	const GenericMatrix& _arg
)
{
	ASSERT(Base::cols() == _arg.cols());

	unsigned oldRows = Base::rows();
	unsigned argRows = _arg.rows();

	Base::conservativeResize(oldRows + argRows, Base::cols());
	Base::block(oldRows, 0, argRows, Base::cols()) = _arg;

	return *this;
}

template<typename T>
GenericMatrix<T>& GenericMatrix<T>::appendCols(	const GenericMatrix& _arg
)
{
	ASSERT(Base::rows() == _arg.rows());

	unsigned oldCols = Base::cols();
	unsigned argCols = _arg.cols();

	Base::conservativeResize(Base::rows(), oldCols + argCols);
	Base::block(0, oldCols, Base::rows(), argCols) = _arg;

	return *this;
}

/** Computes the column-wise sum the Matrix
 *
 *  Example:
 *  \code
 *  a   |  b
 *  c   |  d
 *  \endcode
 *
 *  returns [a+b;c+d]
 */
template<typename T>
GenericVector< T > GenericMatrix< T >::sumCol() const
{
	GenericVector< T > foo(Base::rows());
	for (unsigned r = 0; r < Base::rows(); ++r)
		foo( r ) = Base::row( r ).sum();

	return foo;
}

template<typename T>
GenericVector< T > GenericMatrix<T>::sumRow() const
{
	GenericVector< T > foo(Base::cols());
	for (unsigned c = 0; c < Base::cols(); ++c)
		foo( c ) = Base::col( c ).sum();

	return foo;
}

template<typename T>
GenericMatrix<T>& GenericMatrix<T>::makeVector( )
{
	Base::_set(GenericVector< T >(Base::cols() * Base::rows(), Base::data()));
	return *this;
}

template<typename T>
GenericVector< T > GenericMatrix< T >::getDiag( ) const
{ return Base::diagonal(); }

template<typename T>
bool GenericMatrix<T>::isSquare() const
{ return Base::rows() == Base::cols(); }

template<typename T>
bool GenericMatrix<T>::isSymmetric( ) const
{ return Base::operator==(Base::transpose()); }

template<typename T>
returnValue GenericMatrix<T>::symmetrize( )
{
	if ( isSquare( ) == false )
		return ACADOERROR( RET_MATRIX_NOT_SQUARE );

	for (unsigned i = 0; i < getNumRows(); ++i)
		for (unsigned j = i + 1; j < getNumRows(); ++j)
		{
			T m = (Base::operator()(i, j) + Base::operator()(j, i)) / T( 2 );
			Base::operator()(i, j) = m;
			Base::operator()(j, i) = m;
		}

	return SUCCESSFUL_RETURN;
}

template<typename T>
bool GenericMatrix< T >::isPositiveSemiDefinite( ) const
{
	Eigen::LDLT< Base > foo = Base::ldlt();
	if (foo.info() == Eigen::Success && foo.isPositive() == true)
		return true;

	return false;
}

template<typename T>
bool GenericMatrix< T >::isPositiveDefinite( ) const
{
	if (Base::llt().info() == Eigen::Success)
		return true;

	return false;
}

template<typename T>
GenericMatrix< T > GenericMatrix< T >::absolute() const
{ return Base::cwiseAbs(); }

template<typename T>
GenericMatrix< T > GenericMatrix< T >::positive() const
{
	GenericMatrix foo = *this;
	unsigned dim = foo.rows() * foo.cols();
	for (unsigned el = 0; el < dim; ++el)
	{
		T bar = foo.data()[ el ];
		if (bar < T( 0 ))
			foo.data()[ el ] = T( 0 );
	}

	return foo;
}

template<typename T>
GenericMatrix<T> GenericMatrix<T>::negative() const
{
	GenericMatrix foo = *this;
	unsigned dim = foo.rows() * foo.cols();
	for (unsigned el = 0; el < dim; ++el)
	{
		T bar = foo.data()[ el ];
		if (bar > T( 0 ))
			foo.data()[ el ] = T( 0 );
	}

	return foo;
}

template<typename T>
T GenericMatrix< T >::getNorm( ) const
{ return Base::norm(); }

template<typename T>
T GenericMatrix< T >::getTrace( ) const
{ return Base::trace(); }

template<typename T>
returnValue GenericMatrix< T >::print(	std::ostream& _stream,
										const char* const _name,
										const char* const _startString,
										const char* const _endString,
										uint _width,
										uint _precision,
										const char* const _colSeparator,
										const char* const _rowSeparator
										) const
{
	if (_name != NULL && strlen(_name) > 0)
		_stream << _name << " = ";

	if (_startString != NULL && strlen(_startString) > 0)
		_stream << _startString;

	if (_precision > 0)
		_stream << scientific << setw( _width ) << setprecision( _precision );
	else
		_stream << setw( _width );

	for (unsigned r = 0; r < Base::rows(); ++r)
	{
		for (unsigned c = 0; c < Base::cols(); ++c)
		{
			if (_precision > 0)
				_stream << Base::operator()(r, c);
			else
				_stream << (int)Base::operator()(r, c);

			if (c < (Base::cols() - 1) && _colSeparator != NULL && strlen(_colSeparator) > 0)
				_stream << _colSeparator;
		}

		if (r < (Base::rows() - 1) && _rowSeparator != NULL && strlen(_rowSeparator) > 0)
			_stream << _rowSeparator;
	}
	if (_endString != NULL && strlen(_endString) > 0)
		_stream << _endString;

	return SUCCESSFUL_RETURN;
}

template<typename T>
returnValue GenericMatrix< T >::print(	std::ostream& _stream,
										const char* const _name,
										PrintScheme _printScheme
										) const
{
	MatFile< T >* matFile;

	switch ( _printScheme )
	{
	case PS_MATLAB_BINARY:
		matFile = new MatFile<T>();

		matFile->write(_stream, *this, _name);

		delete matFile;

		return SUCCESSFUL_RETURN;

	default:
		char* startString = 0;
		char* endString = 0;
		uint width = 0;
		uint precision = 0;
		char* colSeparator = 0;
		char* rowSeparator = 0;

		returnValue ret = getGlobalStringDefinitions(_printScheme, &startString,
				&endString, width, precision, &colSeparator, &rowSeparator);
		if (ret != SUCCESSFUL_RETURN)
			return ret;

		returnValue status = print(_stream, _name, startString, endString, width,
				precision, colSeparator, rowSeparator);

		if ( startString != 0 )   delete[] startString;
		if ( endString != 0 )     delete[] endString;
		if ( colSeparator != 0 )  delete[] colSeparator;
		if ( rowSeparator != 0 )  delete[] rowSeparator;

		return status;
	}
}

template<typename T>
returnValue GenericMatrix<T>::print(	const char* const _filename,
									const char* const _name,
									const char* const _startString,
									const char* const _endString,
									uint _width,
									uint _precision,
									const char* const _colSeparator,
									const char* const _rowSeparator
									) const
{
	ofstream stream( _filename );

	if ( stream.is_open() )
		return print(stream, _name, _startString, _endString, _width, _precision,
				_colSeparator, _rowSeparator);
	else
		return ACADOERROR( RET_FILE_CAN_NOT_BE_OPENED );

	stream.close();

	return SUCCESSFUL_RETURN;
}

template<typename T>
returnValue GenericMatrix< T >::print(	const char* const _filename,
							const char* const _name,
							PrintScheme _printScheme
							) const
{
	ofstream stream( _filename );
	returnValue status;

	if ( stream.is_open() )
		status = print(stream, _name, _printScheme);
	else
		return ACADOERROR( RET_FILE_CAN_NOT_BE_OPENED );

	stream.close();

	return status;
}

template<typename T>
returnValue GenericMatrix< T >::read( std::istream& _stream )
{
	vector< vector< T > > tmp;
	_stream >> tmp;

	if (tmp.size() == 0)
	{
		Base::_set(GenericMatrix< T >(0, 0));
		return SUCCESSFUL_RETURN;
	}

	// Sanity check
	unsigned nc = tmp[ 0 ].size();
	unsigned nr = tmp.size();
	for (unsigned r = 0; r < nr; ++r)
		if (tmp[ r ].size() != nc)
			return ACADOERROR( RET_INVALID_ARGUMENTS );

	Base::_set(GenericMatrix< T >(nr, nc, tmp));

	return SUCCESSFUL_RETURN;
}

template<typename T>
returnValue GenericMatrix< T >::read(const char* _filename)
{
	ifstream stream( _filename );
	returnValue status;

	if (stream.is_open())
		status = read( stream );
	else
		return ACADOERROR( RET_FILE_CAN_NOT_BE_OPENED );

	stream.close();

	return status;
}

//
// Explicit instantiation of templates.
//
template class GenericMatrix<double>;
template class GenericMatrix<int>;

CLOSE_NAMESPACE_ACADO
