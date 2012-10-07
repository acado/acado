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
 *    \file src/code_generation/export_variable.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010
 */


#include <acado/code_generation/export_variable.hpp>
#include <acado/code_generation/export_arithmetic_statement.hpp>


BEGIN_NAMESPACE_ACADO


static const double undefinedEntry = 1073741824.03125; // = 2^30 + 2^-5
static char exportDataString[1024];


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportVariable::ExportVariable( ) : ExportArgument( )
{
	init( "",0,0 );
}


ExportVariable::ExportVariable(	const String& _name,
								uint _nRows,
								uint _nCols,
								ExportType _type,
								ExportStruct _dataStruct,
								BooleanType _callItByValue,
								const String& _prefix
								)
{
	init( _name,_nRows,_nCols,_type,_dataStruct,_callItByValue, _prefix );
}


ExportVariable::ExportVariable(	const String& _name,
								const Matrix& _data,
								ExportType _type,
								ExportStruct _dataStruct,
								BooleanType _callItByValue,
								const String& _prefix
								)
{
	init( _name,_data,_type,_dataStruct,_callItByValue, _prefix );
}


ExportVariable::ExportVariable(	const ExportVariable& arg
								) : ExportArgument( arg )
{
	doAccessTransposed = arg.doAccessTransposed;
	
	rowOffset = arg.rowOffset;
	colOffset = arg.colOffset;
	
	setSubmatrixOffsets( arg.rowOffset, arg.colOffset, arg.colDim, arg.nRows, arg.nCols );
}


ExportVariable::ExportVariable(	const Matrix& _data
								)
{
	init( "M",_data );
}


ExportVariable::~ExportVariable( )
{
}


ExportVariable& ExportVariable::operator=(	const ExportVariable& arg
											)
{
	if( this != &arg )
	{
		ExportArgument::operator=( arg );
		
		doAccessTransposed = arg.doAccessTransposed;
	
		rowOffset = arg.rowOffset;
		colOffset = arg.colOffset;

		setSubmatrixOffsets( arg.rowOffset, arg.colOffset, arg.colDim, arg.nRows, arg.nCols );
	}

	return *this;
}


ExportVariable& ExportVariable::operator=(	const Matrix& arg
											)
{
	ExportArgument::operator=( arg );
	setSubmatrixOffsets( 0,0,arg.getNumCols(),0,0 );
	
	return *this;
}


ExportData* ExportVariable::clone( ) const
{
	return new ExportVariable(*this);
}



returnValue ExportVariable::init(	const String& _name,
									uint _nRows,
									uint _nCols,
									ExportType _type,
									ExportStruct _dataStruct,
									BooleanType _callItByValue,
									const String& _prefix
									)
{
	ExportArgument::init( _name,_nRows,_nCols,_type,_dataStruct,_callItByValue,
			emptyConstExportIndex, _prefix );

	doAccessTransposed = BT_FALSE;
	
	rowOffset.init( "run1" );
	colOffset.init( "run2" );
	
	return setSubmatrixOffsets( 0,0,_nCols,0,0 );
}


returnValue ExportVariable::init(	const String& _name,
									const Matrix& _data,
									ExportType _type,
									ExportStruct _dataStruct,
									BooleanType _callItByValue,
									const String& _prefix
									)
{
	ExportArgument::init( _name,_data,_type,_dataStruct,_callItByValue,
			emptyConstExportIndex, _prefix );

	doAccessTransposed = BT_FALSE;
	
	rowOffset.init( "run1" );
	colOffset.init( "run2" );

	return setSubmatrixOffsets( 0,0,_data.getNumCols(),0,0 );
}


ExportVariable& ExportVariable::setup(	const String& _name,
										uint _nRows,
										uint _nCols,
										ExportType _type,
										ExportStruct _dataStruct,
										BooleanType _callItByValue,
										const String& _prefix
										)
{
	init( _name,_nRows,_nCols,_type,_dataStruct,_callItByValue, _prefix );
	return *this;
}


ExportVariable& ExportVariable::setup(	const String& _name,
										const Matrix& _data,
										ExportType _type,
										ExportStruct _dataStruct,
										BooleanType _callItByValue,
										const String& _prefix
										)
{
	init( _name,_data,_type,_dataStruct,_callItByValue, _prefix );
	return *this;
}



double& ExportVariable::operator()(	uint rowIdx,
									uint colIdx
									)
{
	if ( isAccessedTransposed() == BT_FALSE )
		return data( rowIdx,colIdx );
	else
		return data( colIdx,rowIdx );
}


double ExportVariable::operator()(	uint rowIdx,
									uint colIdx
									) const
{
	if ( isAccessedTransposed() == BT_FALSE )
		return data( rowIdx,colIdx );
	else
		return data( colIdx,rowIdx );
}


double& ExportVariable::operator()(	uint totalIdx
									)
{
	return operator()( totalIdx/data.getNumCols(),totalIdx%data.getNumCols() );
}


double ExportVariable::operator()(	uint totalIdx
									) const
{
	return operator()( totalIdx/data.getNumCols(),totalIdx%data.getNumCols() );
}


ExportVariable ExportVariable::operator()(	const String& _name
											) const
{
	ExportVariable tmp( *this );
	tmp.setName( _name );
	return tmp;
}


returnValue ExportVariable::resetAll( )
{
	return data.setAll( undefinedEntry );
}


returnValue ExportVariable::resetDiagonal( )
{
	if ( getNumRows() != getNumCols() )
		return ACADOERROR( RET_MATRIX_NOT_SQUARE );

	for( uint i=0; i<getNumRows(); ++i )
		operator()( i,i ) = undefinedEntry;
	
	return SUCCESSFUL_RETURN;
}


BooleanType ExportVariable::isZero(	const ExportIndex& rowIdx,
									const ExportIndex& colIdx
									) const
{
	return hasValue( rowIdx,colIdx, 0.0 );
}


BooleanType ExportVariable::isZero(	const ExportIndex& rowIdx,
									uint colIdx
									) const
{
	return hasValue( rowIdx,colIdx, 0.0 );
}


BooleanType ExportVariable::isZero(	uint rowIdx,
									const ExportIndex& colIdx
									) const
{
	return hasValue( rowIdx,colIdx, 0.0 );
}


BooleanType ExportVariable::isZero(	uint rowIdx,
									uint colIdx
									) const
{
	return hasValue( rowIdx,colIdx, 0.0 );
}



BooleanType ExportVariable::isOne(	const ExportIndex& rowIdx,
									const ExportIndex& colIdx
									) const
{
	return hasValue( rowIdx,colIdx, 1.0 );
}


BooleanType ExportVariable::isOne(	const ExportIndex& rowIdx,
									uint colIdx
									) const
{
	return hasValue( rowIdx,colIdx, 1.0 );
}


BooleanType ExportVariable::isOne(	uint rowIdx,
									const ExportIndex& colIdx
									) const
{
	return hasValue( rowIdx,colIdx, 1.0 );
}


BooleanType ExportVariable::isOne(	uint rowIdx,
									uint colIdx
									) const
{
	return hasValue( rowIdx,colIdx, 1.0 );
}



BooleanType ExportVariable::isGiven(	const ExportIndex& rowIdx,
										const ExportIndex& colIdx
										) const
{
	if ( hasValue( rowIdx,colIdx, undefinedEntry ) == BT_TRUE )
		return BT_FALSE;
	else
		return BT_TRUE;
}


BooleanType ExportVariable::isGiven(	const ExportIndex& rowIdx,
										uint colIdx
										) const
{
	if ( hasValue( rowIdx,colIdx, undefinedEntry ) == BT_TRUE )
		return BT_FALSE;
	else
		return BT_TRUE;
}


BooleanType ExportVariable::isGiven(	uint rowIdx,
										const ExportIndex& colIdx
										) const
{
	if ( hasValue( rowIdx,colIdx, undefinedEntry ) == BT_TRUE )
		return BT_FALSE;
	else
		return BT_TRUE;
}


BooleanType ExportVariable::isGiven(	uint rowIdx,
										uint colIdx
										) const
{
	if ( hasValue( rowIdx,colIdx, undefinedEntry ) == BT_TRUE )
		return BT_FALSE;
	else
		return BT_TRUE;
}


BooleanType ExportVariable::isGiven( ) const
{
	return ExportArgument::isGiven( );
}



const char* ExportVariable::get(	const ExportIndex& rowIdx,
									const ExportIndex& colIdx
									) const
{
	ExportIndex totalIdx = getTotalIdx( rowIdx+rowOffset,colIdx+colOffset );
	
	if ( ( totalIdx.isGiven() == BT_TRUE ) && ( rowIdx.isGiven() == BT_TRUE ) && ( colIdx.isGiven() == BT_TRUE ) )
	{
		if ( isGiven( rowIdx,colIdx ) == BT_FALSE )
		{
			if ( ( isCalledByValue() == BT_TRUE ) && ( totalIdx.getGivenValue() == 0 ) )
				sprintf( exportDataString,"%s", getFullName().getName() );
			else
				sprintf( exportDataString,"%s[%d]", getFullName().getName(), totalIdx.getGivenValue() );
		}
		else
			sprintf( exportDataString,"(real_t)%.16e",operator()( rowIdx.getGivenValue(),colIdx.getGivenValue() ) );
	}
	else if ( ( rowIdx.isGiven() == BT_FALSE ) && ( colIdx.isGiven() == BT_FALSE ) )
	{
		// sprintf( exportDataString,"%s[%s]", getFullName().getName(),totalIdx.get( ) );

		if ( isAccessedTransposed() == BT_FALSE )
		{
			sprintf(exportDataString, "%s[%s * %d + %s]",
					getFullName().getName(),
					rowIdx.getFullName().getName(),
					getColDim(),
					colIdx.getFullName().getName()
					);
		}
		else
		{
			sprintf(exportDataString, "%s[%s * %d + %s]",
					getFullName().getName(),
					colIdx.getFullName().getName(),
					getColDim(),
					rowIdx.getFullName().getName()
			);
		}

//		if ( isAccessedTransposed() == BT_FALSE )
//			return rowIdx*getColDim() + colIdx;
//		else
//			return colIdx*getColDim() + rowIdx;
	}
	else
		sprintf( exportDataString,"%s[%s]", getFullName().getName(),totalIdx.get( ) );

	return exportDataString;
}


const char* ExportVariable::get(	uint rowIdx,
									uint colIdx
									) const
{
	if ( ( rowOffset.isGiven( ) == BT_TRUE ) && ( colOffset.isGiven( ) == BT_TRUE ) )
	{
		int totalIdx = getTotalIdx( rowIdx+rowOffset.getGivenValue(),colIdx+colOffset.getGivenValue() );

		if ( isGiven( rowIdx,colIdx ) == BT_FALSE )
		{
			if ( ( isCalledByValue() == BT_TRUE ) && ( totalIdx == 0 ) )
				sprintf( exportDataString,"%s", getFullName().getName() );
			else
				sprintf( exportDataString,"%s[%d]", getFullName().getName(),totalIdx );
		}
		else
			sprintf( exportDataString,"(real_t)%.16e",operator()(rowIdx,colIdx) );

		return exportDataString;
	}
	else
	{
		ExportIndex tmpRowIdx = rowIdx;
		ExportIndex tmpColIdx = colIdx;
		return get( tmpRowIdx,tmpColIdx );
	}
}



uint ExportVariable::getNumRows( ) const
{
	if ( isAccessedTransposed() == BT_FALSE )
	{
		if ( nRows > 0 )
			return nRows;
		else
			return data.getNumRows( );
	}
	else
	{
		if ( nCols > 0 )
			return nCols;
		else
			return data.getNumCols( );
	}
}


uint ExportVariable::getNumCols( ) const
{
	if ( isAccessedTransposed() == BT_FALSE )
	{
		if ( nCols > 0 )
			return nCols;
		else
			return data.getNumCols( );
	}
	else
	{
		if ( nRows > 0 )
			return nRows;
		else
			return data.getNumRows( );
	}
}


uint ExportVariable::getDim( ) const
{
	return ( getNumRows()*getNumCols() );
}




ExportArithmeticStatement ExportVariable::operator+(	const ExportVariable& arg
														) const
{
	ASSERT( getNumRows() == arg.getNumRows() );
	ASSERT( getNumCols() == arg.getNumCols() );

	return ExportArithmeticStatement( 0,ESO_ASSIGN,this,ESO_ADD,&arg );
}


ExportArithmeticStatement ExportVariable::operator-(	const ExportVariable& arg
														) const
{
	ASSERT( getNumRows() == arg.getNumRows() );
	ASSERT( getNumCols() == arg.getNumCols() );

	return ExportArithmeticStatement( 0,ESO_ASSIGN,this,ESO_SUBTRACT,&arg );
}


ExportArithmeticStatement ExportVariable::operator+=(	const ExportVariable& arg
														) const
{
	ASSERT( getNumRows() == arg.getNumRows() );
	ASSERT( getNumCols() == arg.getNumCols() );

	return ExportArithmeticStatement( this,ESO_ASSIGN,&arg,ESO_ADD_ASSIGN,0 );
}


ExportArithmeticStatement ExportVariable::operator-=(	const ExportVariable& arg
														) const
{
	ASSERT( getNumRows() == arg.getNumRows() );
	ASSERT( getNumCols() == arg.getNumCols() );

	return ExportArithmeticStatement( this,ESO_ASSIGN,&arg,ESO_SUBTRACT_ASSIGN,0 );
}


ExportArithmeticStatement ExportVariable::operator*(	const ExportVariable& arg
														) const
{
	ASSERT( getNumCols() == arg.getNumRows() );
	return ExportArithmeticStatement( 0,ESO_ASSIGN,this,ESO_MULTIPLY,&arg );
}


ExportArithmeticStatement ExportVariable::operator^(	const ExportVariable& arg
														) const
{
	ASSERT( getNumRows() == arg.getNumRows() );
	return ExportArithmeticStatement( 0,ESO_ASSIGN,this,ESO_MULTIPLY_TRANSPOSE,&arg );
}


ExportArithmeticStatement ExportVariable::operator==(	const ExportVariable& arg
														) const
{
	ASSERT( getNumRows() == arg.getNumRows() );
	ASSERT( getNumCols() == arg.getNumCols() );

	return ExportArithmeticStatement( this,ESO_ASSIGN,&arg,ESO_ASSIGN,0 );
}


ExportArithmeticStatement ExportVariable::operator==(	ExportArithmeticStatement arg
														) const
{
	ASSERT( getNumRows() == arg.getNumRows() );
	ASSERT( getNumCols() == arg.getNumCols() );

	return ExportArithmeticStatement( this,ESO_ASSIGN,arg.rhs1,arg.op1,arg.rhs2,arg.op2,arg.rhs3 );
}


ExportArithmeticStatement ExportVariable::operator+(	ExportArithmeticStatement arg
														) const
{
	ASSERT( getNumRows() == arg.getNumRows() );
	ASSERT( getNumCols() == arg.getNumCols() );

	ASSERT( ( arg.op1 == ESO_MULTIPLY ) || ( arg.op1 == ESO_MULTIPLY_TRANSPOSE ) );

	return ExportArithmeticStatement( 0,ESO_UNDEFINED,arg.rhs1,arg.op1,arg.rhs2,ESO_ADD,this );
}


ExportArithmeticStatement ExportVariable::operator-(	ExportArithmeticStatement arg
														) const
{
	ASSERT( getNumRows() == arg.getNumRows() );
	ASSERT( getNumCols() == arg.getNumCols() );

	ASSERT( ( arg.op1 == ESO_MULTIPLY ) || ( arg.op1 == ESO_MULTIPLY_TRANSPOSE ) );

	return ExportArithmeticStatement( 0,ESO_UNDEFINED,arg.rhs1,arg.op1,arg.rhs2,ESO_SUBTRACT,this );
}


ExportArithmeticStatement ExportVariable::operator+=(	ExportArithmeticStatement arg
														) const
{
	ASSERT( getNumRows() == arg.getNumRows() );
	ASSERT( getNumCols() == arg.getNumCols() );

	ASSERT( ( arg.op1 == ESO_MULTIPLY ) || 
			( arg.op1 == ESO_MULTIPLY_TRANSPOSE ) ||
			( arg.op1 == ESO_ADD ) || 
			( arg.op1 == ESO_SUBTRACT ) );

	return ExportArithmeticStatement( this,ESO_ADD_ASSIGN,arg.rhs1,arg.op1,arg.rhs2,arg.op2,arg.rhs3 );
}


ExportArithmeticStatement ExportVariable::operator-=(	ExportArithmeticStatement arg
														) const
{
	ASSERT( getNumRows() == arg.getNumRows() );
	ASSERT( getNumCols() == arg.getNumCols() );

	ASSERT( ( arg.op1 == ESO_MULTIPLY ) || 
			( arg.op1 == ESO_MULTIPLY_TRANSPOSE ) ||
			( arg.op1 == ESO_ADD ) || 
			( arg.op1 == ESO_SUBTRACT ) );

	return ExportArithmeticStatement( this,ESO_SUBTRACT_ASSIGN,arg.rhs1,arg.op1,arg.rhs2,arg.op2,arg.rhs3 );
}



ExportArithmeticStatement ExportVariable::operator+(	const Matrix& arg
														) const
{
	ExportVariable tmp( "tmp",arg );
	return operator+( tmp );
}


ExportArithmeticStatement ExportVariable::operator-(	const Matrix& arg
														) const
{
	ExportVariable tmp( "tmp",arg );
	return operator-( tmp );
}


ExportArithmeticStatement ExportVariable::operator+=(	const Matrix& arg
														) const
{
	ExportVariable tmp( "tmp",arg );
	return operator+=( tmp );
}


ExportArithmeticStatement ExportVariable::operator-=(	const Matrix& arg
														) const
{
	ExportVariable tmp( "tmp",arg );
	return operator-=( tmp );
}


ExportArithmeticStatement ExportVariable::operator*(	const Matrix& arg
														) const
{
	ExportVariable tmp( "tmp",arg );
	return operator*( tmp );
}


ExportArithmeticStatement ExportVariable::operator^(	const Matrix& arg
														) const
{
	ExportVariable tmp( "tmp",arg );
	return operator^( tmp );
}


ExportArithmeticStatement ExportVariable::operator==(	const Matrix& arg
														) const
{
	ExportVariable tmp( "tmp",arg );
	return operator==( tmp );
}



ExportVariable ExportVariable::getTranspose( ) const
{
	ExportVariable transposed( name,getNumCols(),getNumRows(),type,dataStruct,callItByValue, prefix);
	
	for( uint i=0; i<getNumCols(); ++i )
		for( uint j=0; j<getNumRows(); ++j )
			transposed( i,j ) = operator()( j,i );
		
	return transposed;
}


ExportVariable ExportVariable::accessTransposed( )
{
	doAccessTransposed = BT_TRUE;
	
	ExportIndex tmp = rowOffset;
	rowOffset = colOffset;
	colOffset = tmp;
	
	return *this;
}


BooleanType ExportVariable::isAccessedTransposed( ) const
{
	return doAccessTransposed;
}



ExportVariable ExportVariable::getRow(	uint idx
										) const
{
	ASSERT( isAccessedTransposed() == BT_FALSE );
	
	ExportVariable tmp( name,1,getNumCols( ),type,dataStruct,callItByValue, prefix );
	
	for( uint i=0; i<getNumCols(); ++i )
		tmp( 0,i ) = operator()( idx,i );
	
	tmp.setSubmatrixOffsets( idx,0,getNumCols( ) );

	return tmp;
}


ExportVariable ExportVariable::getRow(	const ExportIndex& idx
										) const
{
	ASSERT( isAccessedTransposed() == BT_FALSE );
	
	ExportVariable tmp( *this );
	ExportIndex tmpColOffset( idx.getName() );
	tmpColOffset = 0;
	
	tmp.setSubmatrixOffsets( idx,tmpColOffset,getNumCols( ), 1,getNumCols( ) );

	return tmp;
}


ExportVariable ExportVariable::getCol(	uint idx
										) const
{
	ASSERT( isAccessedTransposed() == BT_FALSE );
	
	ExportVariable tmp( name,getNumRows( ),1,type,dataStruct,callItByValue, prefix );
	
	for( uint i=0; i<getNumRows(); ++i )
		tmp( i,0 ) = operator()( i,idx );

	tmp.setSubmatrixOffsets( 0,idx,getNumCols( ) );

	return tmp;
}


ExportVariable ExportVariable::getCol(	const ExportIndex& idx
										) const
{
	ASSERT( isAccessedTransposed() == BT_FALSE );
	
	ExportVariable tmp = *this;
	ExportIndex tmpRowOffset( idx.getName() );
	tmpRowOffset = 0;

	tmp.setSubmatrixOffsets( tmpRowOffset,idx,getNumCols( ), getNumRows(),1 );

	return tmp;
}



ExportVariable ExportVariable::getRows(	uint idx1,
										uint idx2
										) const
{
	ASSERT( isAccessedTransposed() == BT_FALSE );
	
	ASSERT( 0 <= idx1 );
	ASSERT( idx1 <= idx2 );
	ASSERT( idx1 <= getNumRows( ) );

	ExportVariable tmp( name,idx2-idx1,getNumCols( ),type,dataStruct,callItByValue, prefix ); // idx2-idx1+1
	
	for( uint i=idx1; i<idx2; ++i ) // <idx2
		for( uint j=0; j<getNumCols(); ++j )
			tmp( i-idx1,j ) = operator()( i,j );

	tmp.setSubmatrixOffsets( idx1,0,getNumCols( ) );

	return tmp;
}


ExportVariable ExportVariable::getRows(	const ExportIndex& idx1,
										const ExportIndex& idx2
										) const
{
	ASSERT( isAccessedTransposed() == BT_FALSE );
	ASSERT( (idx2-idx1).isGiven() == BT_TRUE );
	
	if ( idx1.isGiven() == BT_TRUE )
	{
		ASSERT( 0 <= idx1.getGivenValue() );
		ASSERT( idx1.getGivenValue() <= idx2.getGivenValue() );
		ASSERT( idx1.getGivenValue() <= (int)getNumRows( ) );
	}

	ExportVariable tmp( *this );
	ExportIndex tmpColOffset( idx1.getName() );
	tmpColOffset = 0;

	tmp.setSubmatrixOffsets( idx1,tmpColOffset,getNumCols( ), (idx2-idx1).getGivenValue(),getNumCols( ) );
	return tmp;
}



ExportVariable ExportVariable::getCols(	uint idx1,
										uint idx2
										) const
{
	ASSERT( isAccessedTransposed() == BT_FALSE );
	ASSERT( 0 <= idx1 );
	ASSERT( idx1 <= idx2 );
	ASSERT( idx1 <= getNumCols( ) );

	ExportVariable tmp( name,getNumRows( ),idx2-idx1,type,dataStruct,callItByValue, prefix );// idx2-idx1+1
	
	for( uint i=0; i<getNumRows(); ++i )
		for( uint j=idx1; j<idx2; ++j )// <idx2
			tmp( i,j-idx1 ) = operator()( i,j );

	tmp.setSubmatrixOffsets( 0,idx1,getNumCols( ) );

	return tmp;
}


ExportVariable ExportVariable::getCols(	const ExportIndex& idx1,
										const ExportIndex& idx2
										) const
{
	ASSERT( isAccessedTransposed() == BT_FALSE );
	ASSERT( (idx2-idx1).isGiven() == BT_TRUE );
	
	if ( idx1.isGiven() == BT_TRUE )
	{
		ASSERT( 0 <= idx1.getGivenValue() );
		ASSERT( idx1.getGivenValue() <= idx2.getGivenValue() );
		ASSERT( idx1.getGivenValue() <= (int)getNumCols( ) );
	}

	ExportVariable tmp = *this;
	ExportIndex tmpRowOffset( idx1.getName() );
	tmpRowOffset = 0;

	tmp.setSubmatrixOffsets( tmpRowOffset,idx1,getNumCols( ), getNumRows( ),(idx2-idx1).getGivenValue() );
	return tmp;
}


ExportVariable ExportVariable::getSubMatrix(	uint rowIdx1,
												uint rowIdx2,
												uint colIdx1,
												uint colIdx2
												) const
{
	ASSERT( isAccessedTransposed() == BT_FALSE );
	ASSERT( 0 <= rowIdx1 );
	ASSERT( rowIdx1 <= rowIdx2 );
	ASSERT( rowIdx1 <= getNumRows( ) );
	ASSERT( 0 <= colIdx1 );
	ASSERT( colIdx1 <= colIdx2 );
	ASSERT( colIdx1 <= getNumCols( ) );

	ExportVariable tmp;
	
// 	if ( isAccessedTransposed() == BT_FALSE )
// 	{
		tmp.init( name,rowIdx2-rowIdx1,colIdx2-colIdx1,type,dataStruct,callItByValue, prefix );
		
		for( uint i=rowIdx1; i<rowIdx2; ++i )
			for( uint j=colIdx1; j<colIdx2; ++j )
				tmp( i-rowIdx1,j-colIdx1 ) = operator()( i,j );

		tmp.setSubmatrixOffsets( rowIdx1,colIdx1,getNumCols( ) );
// 	}
// 	else
// 	{
// 		tmp.init( name,type,colIdx2-colIdx1,rowIdx2-rowIdx1 );
// 		tmp.accessTransposed( );
// 		
// 		for( uint i=rowIdx1; i<rowIdx2; ++i )
// 			for( uint j=colIdx1; j<colIdx2; ++j )
// 				tmp( i-rowIdx1,j-colIdx1 ) = operator()( i,j );
// 			
// 		tmp.setSubmatrixOffsets( rowIdx1,colIdx1,getNumRows( ) );
// 	}

	return tmp;
}


ExportVariable ExportVariable::getSubMatrix(	const ExportIndex& rowIdx1,
												const ExportIndex& rowIdx2,
												uint colIdx1,
												uint colIdx2
												) const
{
	ASSERT( isAccessedTransposed() == BT_FALSE );
	ASSERT( (rowIdx2-rowIdx1).isGiven() == BT_TRUE );
	
	if ( rowIdx1.isGiven() == BT_TRUE )
	{
		ASSERT( 0 <= rowIdx1.getGivenValue() );
		ASSERT( rowIdx1.getGivenValue() <= rowIdx2.getGivenValue() );
		ASSERT( rowIdx1.getGivenValue() <= (int)getNumRows( ) );
	}

	ASSERT( 0 <= colIdx1 );
	ASSERT( colIdx1 <= colIdx2 );
	ASSERT( colIdx1 <= getNumCols( ) );

	ExportVariable tmp( *this );
	ExportIndex tmpColOffset( rowIdx1.getName() );
	tmpColOffset = colIdx1;

	tmp.setSubmatrixOffsets( rowIdx1,tmpColOffset,getNumCols( ), (rowIdx2-rowIdx1).getGivenValue(),colIdx2-colIdx1 );

	return tmp;
}


ExportVariable ExportVariable::getSubMatrix(	uint rowIdx1,
												uint rowIdx2,
												const ExportIndex& colIdx1,
												const ExportIndex& colIdx2
												) const
{
	ASSERT( isAccessedTransposed() == BT_FALSE );
	ASSERT( (colIdx2-colIdx1).isGiven() == BT_TRUE );
	
	ASSERT( 0 <= rowIdx1 );
	ASSERT( rowIdx1 <= rowIdx2 );
	ASSERT( rowIdx1 <= getNumRows( ) );
	
	if ( colIdx1.isGiven() == BT_TRUE )
	{
		ASSERT( 0 <= colIdx1.getGivenValue() );
		ASSERT( colIdx1.getGivenValue() <= colIdx2.getGivenValue() );
		ASSERT( colIdx1.getGivenValue() <= (int)getNumCols( ) );
	}

	ExportVariable tmp = *this;
	ExportIndex tmpRowOffset( colIdx1.getName() );
	tmpRowOffset = rowIdx1;

	tmp.setSubmatrixOffsets( tmpRowOffset,colIdx1,getNumCols( ), rowIdx2-rowIdx1,(colIdx2-colIdx1).getGivenValue() );

	return tmp;
}


ExportVariable ExportVariable::getSubMatrix(	const ExportIndex& rowIdx1,
												const ExportIndex& rowIdx2,
												const ExportIndex& colIdx1,
												const ExportIndex& colIdx2
												) const
{
	ASSERT( isAccessedTransposed() == BT_FALSE );
	ASSERT( (rowIdx2-rowIdx1).isGiven() == BT_TRUE );
	ASSERT( (colIdx2-colIdx1).isGiven() == BT_TRUE );
	
	if ( rowIdx1.isGiven() == BT_TRUE )
	{
		ASSERT( 0 <= rowIdx1.getGivenValue() );
		ASSERT( rowIdx1.getGivenValue() <= rowIdx2.getGivenValue() );
		ASSERT( rowIdx1.getGivenValue() <= (int)getNumRows( ) );
	}

	if ( colIdx1.isGiven() == BT_TRUE )
	{
		ASSERT( 0 <= colIdx1.getGivenValue() );
		ASSERT( colIdx1.getGivenValue() <= colIdx2.getGivenValue() );
		ASSERT( colIdx1.getGivenValue() <= (int)getNumCols( ) );
	}

	ExportVariable tmp( *this );

	tmp.setSubmatrixOffsets( rowIdx1,colIdx1,getNumCols( ), (rowIdx2-rowIdx1).getGivenValue(),(colIdx2-colIdx1).getGivenValue() );

	return tmp;
}



ExportVariable ExportVariable::makeRowVector( ) const
{
	ASSERT( ( nRows == 0 ) && ( nCols == 0 ) );
	
	ExportVariable tmp( name,1,getDim(),type,dataStruct,callItByValue, prefix );
	
	for ( uint i=0; i<getNumRows(); ++i )
		for ( uint j=0; j<getNumCols(); ++j )
			tmp( 0,i*getNumCols()+j ) = operator()( i,j );
	
	return tmp;
}


ExportVariable ExportVariable::makeColVector( ) const
{
	ASSERT( ( nRows == 0 ) && ( nCols == 0 ) );
	
	ExportVariable tmp( name,getDim(),1,type,dataStruct,callItByValue, prefix );
	
	for ( uint i=0; i<getNumRows(); ++i )
		for ( uint j=0; j<getNumCols(); ++j )
			tmp( i*getNumCols()+j,0 ) = operator()( i,j );
	
	return tmp;
}



BooleanType ExportVariable::isVector( ) const
{
	if ( ( getNumRows( ) == 1 ) || ( getNumCols( ) == 1 ) )
		return BT_TRUE;
	else
		return BT_FALSE;
}


Matrix ExportVariable::getGivenMatrix( ) const
{
	if ( isGiven() == BT_TRUE )
		return data;
	else
		return Matrix();
}



returnValue ExportVariable::print( ) const
{
	return data.print( name.getName() );
}



//
// PROTECTED MEMBER FUNCTIONS:
//

uint ExportVariable::getColDim( ) const
{
	return colDim;
}



uint ExportVariable::getTotalIdx(	uint rowIdx,
									uint colIdx
									) const
{
	if ( isAccessedTransposed() == BT_FALSE )
		return rowIdx*getColDim() + colIdx;
	else
		return colIdx*getColDim() + rowIdx;
}


ExportIndex	ExportVariable::getTotalIdx(	const ExportIndex& rowIdx,
											const ExportIndex& colIdx
											) const
{
	if ( isAccessedTransposed() == BT_FALSE )
		return rowIdx*getColDim() + colIdx;
	else
		return colIdx*getColDim() + rowIdx;
}



returnValue ExportVariable::setSubmatrixOffsets(	uint _rowOffset,
													uint _colOffset,
													uint _colDim,
													uint _nRows,
													uint _nCols
													)
{
	if ( _colOffset > _colDim )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	rowOffset = _rowOffset;
	colOffset = _colOffset;
	colDim    = _colDim;
	
	nRows = _nRows;
	nCols = _nCols;
	
	return SUCCESSFUL_RETURN;
}


returnValue ExportVariable::setSubmatrixOffsets(	const ExportIndex& _rowOffset,
													const ExportIndex& _colOffset,
													uint _colDim,
													uint _nRows,
													uint _nCols
													)
{
	if ( ( _rowOffset.isGiven() == BT_TRUE ) && ( _rowOffset.getGivenValue() < 0 ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );
	
	if ( ( _colOffset.isGiven() == BT_TRUE ) && ( _colOffset.getGivenValue() < 0 ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );
	
	if ( ( _colOffset.isGiven() == BT_TRUE ) && ( _colOffset.getGivenValue() > (int)_colDim ) )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	rowOffset = _rowOffset;
	colOffset = _colOffset;
	colDim    = _colDim;

	nRows = _nRows;
	nCols = _nCols;
	
	return SUCCESSFUL_RETURN;
}


BooleanType ExportVariable::hasValue(	const ExportIndex& rowIdx,
										const ExportIndex& colIdx,
										double _value
										) const
{
	if ( ( rowIdx.isGiven() == BT_TRUE ) && ( colIdx.isGiven() == BT_TRUE ) )
		return hasValue( rowIdx.getGivenValue(),colIdx.getGivenValue(), _value );
	else
		return BT_FALSE;
}


BooleanType ExportVariable::hasValue(	const ExportIndex& rowIdx,
										uint colIdx,
										double _value
										) const
{
	if ( rowIdx.isGiven() == BT_TRUE )
		return hasValue( rowIdx.getGivenValue(),colIdx, _value );
	else
		return BT_FALSE;
}


BooleanType ExportVariable::hasValue(	uint rowIdx,
										const ExportIndex& colIdx,
										double _value
										) const
{
	if ( colIdx.isGiven() == BT_TRUE )
		return hasValue( rowIdx,colIdx.getGivenValue(), _value );
	else
		return BT_FALSE;
}


BooleanType ExportVariable::hasValue(	uint rowIdx,
										uint colIdx,
										double _value
										) const
{
	return acadoIsEqual( operator()( rowIdx,colIdx ),_value );
}


CLOSE_NAMESPACE_ACADO

// end of file.
