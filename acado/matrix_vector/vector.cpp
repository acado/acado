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
 *    \file src/matrix_vector/vector.cpp
 *    \author Hans Joachim Ferreau, Boris Houska, Milan Vukov
 *    \date 2008 - 2013
 */

#include <acado/matrix_vector/vector.hpp>
#include <acado/matrix_vector/acado_mat_file.hpp>
#include <iomanip>

using namespace std;

BEGIN_NAMESPACE_ACADO

template<typename T>
GenericVector< T >& GenericVector<T>::append( 	const GenericVector<T>& _arg
)
{
	unsigned oldDim = Base::rows();
	unsigned argDim = _arg.rows();

	Base::conservativeResize(oldDim + argDim);
	Base::block(oldDim, 0, argDim, 1) = _arg;

	return *this;
}

template<typename T>
GenericVector<T>& GenericVector< T >::setUnitVector(	unsigned _idx
)
{
	ASSERT( _idx < Base::rows() );

	Base::setZero( );
	Base::operator()( _idx ) = T( 1 );

	return *this;
}

template<typename T>
T GenericVector< T >::getNorm(VectorNorm _norm ) const
{
	GenericVector scale( getDim() );
	scale.setOnes();
	return getNorm(_norm, scale);
}

template<typename T>
T GenericVector< T >::getNorm(	VectorNorm _norm,
								const GenericVector< T >& _scale ) const
{
	switch( _norm )
	{
	case VN_L1:
		return GenericVector<T>(this->cwiseQuotient( _scale )).getAbsolute().sum();

	case VN_L2:
		return Base::norm();

	case VN_LINF:
		return GenericVector<T>(this->cwiseQuotient(_scale)).getAbsolute().getMax();

	default:
		return T( -1 );
	}

	return T( -1 );
}

template<typename T>
returnValue GenericVector<T>::print(	std::ostream& stream,
										const std::string& name,
										const std::string& startString,
										const std::string& endString,
										uint width,
										uint precision,
										const std::string& colSeparator,
										const std::string& rowSeparator
										) const
{
	if (name.size())
		stream << name << " = ";

	stream << startString;

	for (unsigned i = 0; i < getDim(); ++i)
	{
		stream << Base::operator()( i );

		if (i < (getDim() - 1))
			stream << rowSeparator;
	}
	stream << endString;

	return SUCCESSFUL_RETURN;
}

template<>
returnValue GenericVector< double >::print(	std::ostream& stream,
											const std::string& name,
											const std::string& startString,
											const std::string& endString,
											uint width,
											uint precision,
											const std::string& colSeparator,
											const std::string& rowSeparator
											) const
{
	IoFormatter iof( stream );

	if (name.size())
		stream << name << " = ";

	stream << startString;

	iof.set(precision > 0 ? precision : iof.precision,
			width,
			precision > 0 ? ios::scientific | ios::right : ios::fixed);

	for (unsigned i = 0; i < getDim(); ++i)
	{
		stream << Base::operator()( i );

		if (i < (getDim() - 1))
			stream << rowSeparator;
	}
	stream << endString;

	iof.reset();

	return SUCCESSFUL_RETURN;
}

template<typename T>
returnValue GenericVector<T>::print(	const std::string& filename,
										const std::string& name,
										const std::string& startString,
										const std::string& endString,
										uint width,
										uint precision,
										const std::string& colSeparator,
										const std::string& rowSeparator
										) const
{
	ofstream stream( filename.c_str() );

	if ( stream.is_open() )
		return print(stream, name, startString, endString, width, precision,
				colSeparator, rowSeparator);
	else
		return ACADOERROR( RET_FILE_CAN_NOT_BE_OPENED );

	stream.close();

	return SUCCESSFUL_RETURN;
}

template<typename T>
returnValue GenericVector<T>::print(	std::ostream& stream,
										const std::string& name,
										PrintScheme printScheme
										) const
{
	MatFile<T>* matFile = 0;

	switch ( printScheme )
	{
	case PS_MATLAB_BINARY:
		matFile = new MatFile<T>;

		matFile->write(stream, *this, name.c_str());

		delete matFile;

		return SUCCESSFUL_RETURN;

	default:

		char* startString = 0;
		char* endString = 0;
		uint width = 0;
		uint precision = 0;
		char* colSeparator = 0;
		char* rowSeparator = 0;

		returnValue ret = getGlobalStringDefinitions(printScheme, &startString,
				&endString, width, precision, &colSeparator, &rowSeparator);
		if (ret != SUCCESSFUL_RETURN)
			return ret;

		returnValue status = print(stream, name, startString, endString, width,
				precision, colSeparator, rowSeparator);

		if ( startString != 0 )   delete[] startString;
		if ( endString != 0 )     delete[] endString;
		if ( colSeparator != 0 )  delete[] colSeparator;
		if ( rowSeparator != 0 )  delete[] rowSeparator;

		return status;
	}

	return SUCCESSFUL_RETURN;
}

template<typename T>
returnValue GenericVector<T>::print(	const std::string& filename,
										const std::string& name,
										PrintScheme printScheme
										) const
{
	ofstream stream( filename.c_str() );
	returnValue status;

	if ( stream.is_open() )
		status = print(stream, name, printScheme);
	else
		return ACADOERROR( RET_FILE_CAN_NOT_BE_OPENED );

	stream.close();

	return status;
}

template<typename T>
returnValue GenericVector<T>::read( std::istream& stream )
{
	vector< T > tmp;
	stream >> tmp;

	Base::_set(GenericVector<T>( tmp ));

	return SUCCESSFUL_RETURN;
}

template<typename T>
returnValue GenericVector<T>::read(	const std::string& filename
									)
{
	ifstream stream( filename.c_str() );
	returnValue status;

	if (stream.is_open())
		status = read( stream );
	else
		return ACADOERROR( RET_FILE_CAN_NOT_BE_OPENED );

	stream.close();

	return status;
}

//
// Explicit instantiation of templates
//
template class GenericVector<double>;
template class GenericVector<int>;
template class GenericVector<bool>;

CLOSE_NAMESPACE_ACADO
