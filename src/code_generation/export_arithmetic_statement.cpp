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
 *    \file src/code_generation/export_arithmetic_statement.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010-2011
 */

#include <acado/code_generation/export_arithmetic_statement.hpp>
#include <acado/code_generation/export_variable_internal.hpp>

#include <sstream>
#include <iomanip>


BEGIN_NAMESPACE_ACADO

using namespace std;

//
// PUBLIC MEMBER FUNCTIONS:
//

ExportArithmeticStatement::ExportArithmeticStatement( )
{
	op0 = ESO_UNDEFINED;
	op1 = ESO_UNDEFINED;
	op2 = ESO_UNDEFINED;
}


ExportArithmeticStatement::ExportArithmeticStatement(	const ExportVariable& _lhs,
														ExportStatementOperator _op0,
														const ExportVariable& _rhs1,
														ExportStatementOperator _op1,
														const ExportVariable& _rhs2,
														ExportStatementOperator _op2,
														const ExportVariable& _rhs3
														) : ExportStatement( )
{
	ASSERT( ( _op0 == ESO_UNDEFINED ) || ( _op0 == ESO_ASSIGN ) || ( _op0 == ESO_ADD_ASSIGN ) || ( _op0 == ESO_SUBTRACT_ASSIGN ) );
	ASSERT( ( _op2 == ESO_UNDEFINED ) || ( _op2 == ESO_ADD ) || ( _op2 == ESO_SUBTRACT ) );

	lhs = _lhs;
	rhs1 = _rhs1;
	rhs2 = _rhs2;
	rhs3 = _rhs3;

	op0  = _op0;
	op1  = _op1;
	op2  = _op2;
}


ExportArithmeticStatement::ExportArithmeticStatement( const ExportArithmeticStatement& arg ) : ExportStatement( arg )
{
	lhs = arg.lhs;
	rhs1 = arg.rhs1;
	rhs2 = arg.rhs2;
	rhs3 = arg.rhs3;

	op0  = arg.op0;
	op1  = arg.op1;
	op2  = arg.op2;

	memAllocator = arg.memAllocator;
}


ExportArithmeticStatement::~ExportArithmeticStatement( )
{}


ExportArithmeticStatement& ExportArithmeticStatement::operator=( const ExportArithmeticStatement& arg )
{
	if( this != &arg )
	{
		ExportStatement::operator=( arg );
		
		lhs = arg.lhs;
		rhs1 = arg.rhs1;
		rhs2 = arg.rhs2;
		rhs3 = arg.rhs3;

		op0  = arg.op0;
		op1  = arg.op1;
		op2  = arg.op2;

		memAllocator = arg.memAllocator;
	}

	return *this;
}


ExportStatement* ExportArithmeticStatement::clone( ) const
{
	return new ExportArithmeticStatement(*this);
}


uint ExportArithmeticStatement::getNumRows( ) const
{
	if ( rhs1.isNull() )
		return 0;
	else
	{
		if ( op1 != ESO_MULTIPLY_TRANSPOSE )
			return rhs1->getNumRows( );
		else
			return rhs1->getNumCols( );
	}
}


uint ExportArithmeticStatement::getNumCols( ) const
{
	if ( rhs1.isNull() )
		return 0;
	else
	{
		if ( rhs2.isNull() )
			return rhs1->getNumCols( );
		else
			return rhs2->getNumCols( );
	}
}


returnValue ExportArithmeticStatement::exportDataDeclaration(	FILE *file,
																const String& _realString,
																const String& _intString,
																int _precision
																) const
{
	return SUCCESSFUL_RETURN;
}


returnValue ExportArithmeticStatement::exportCode(	FILE* file,
													const String& _realString,
													const String& _intString,
													int _precision
													) const
{
	if (lhs->isGiven() == BT_TRUE && lhs->getDim() > 0)
	{
		LOG( LVL_ERROR ) << "Left hand side ('" << lhs.getFullName().getName( ) << "') of an arithmetic "
							"expression is given." << endl;
		return ACADOERROR(RET_INVALID_ARGUMENTS);
	}

	if (memAllocator == 0)
		return ACADOERRORTEXT(RET_INVALID_ARGUMENTS, "Memory allocator is not defined.");
	
	switch( op1 )
	{
		case ESO_ADD:
			return exportCodeAddSubtract(file, "+", _realString, _intString, _precision);

		case ESO_SUBTRACT:
			return exportCodeAddSubtract(file, "-", _realString, _intString, _precision);
			
		case ESO_ADD_ASSIGN:
			return exportCodeAssign( file,"+=",_realString,_intString,_precision );

		case ESO_SUBTRACT_ASSIGN:
			return exportCodeAssign( file,"-=",_realString,_intString,_precision );

		case ESO_MULTIPLY:
			return exportCodeMultiply( file,BT_FALSE,_realString,_intString,_precision );
			
		case ESO_MULTIPLY_TRANSPOSE:
			return exportCodeMultiply( file,BT_TRUE,_realString,_intString,_precision );

		case ESO_ASSIGN:
			return exportCodeAssign( file,"=",_realString,_intString,_precision );

		default:
			return ACADOERROR( RET_UNKNOWN_BUG );
	}
	
	return ACADOERROR( RET_UNKNOWN_BUG );
}

//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue ExportArithmeticStatement::exportCodeAddSubtract(	FILE* file,
																const String& _sign,
																const String& _realString,
																const String& _intString,
																int _precision
																) const
{
//	if ( ( rhs1.isNull() ) || ( rhs2.isNull() ) || ( !rhs3.isNull() ) )
//		return ACADOERROR( RET_UNABLE_TO_EXPORT_STATEMENT );

	if (rhs1.getDim() == 0)
		return SUCCESSFUL_RETURN;

	if ( ( rhs1->getNumRows() != rhs2->getNumRows() ) || 
		 ( rhs1->getNumCols() != rhs2->getNumCols() ) )
		return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );
	
	if ( !lhs.isNull() )
	{
		if ( ( rhs1->getNumRows() != lhs->getNumRows() ) || 
		     ( rhs1->getNumCols() != lhs->getNumCols() ) )
		{
			cout << "lhs name is " << lhs.getName().getName() <<
					", size: " << lhs.getNumRows() << " x " << lhs.getNumCols() << endl;
			cout << "rhs1 name is " << rhs1.getName().getName() <<
					", size: " << rhs1.getNumRows() << " x " << rhs1.getNumCols() << endl;

			return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );
		}
	}
	
	String assignString;
	if ( getAssignString( assignString ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_STATEMENT );
	
	//
	// Rough approximation of flops needed for matrix multiplication
	//
	unsigned numberOfFlops = lhs->getNumRows() * lhs->getNumCols();

	//
	// Optimization can be performed only if both matrices are not given.
	// Currently, optimizations cannot be performed on hard-coded matrices.
	//
	int optimizationsAllowed =
			( rhs1->isGiven() == BT_FALSE ) && ( rhs2->isGiven() == BT_FALSE );

	if ((numberOfFlops < 4096) || (optimizationsAllowed == 0))
	{
		for( uint i=0; i<getNumRows( ); ++i )
			for( uint j=0; j<getNumCols( ); ++j )
			{
				if ( ( op0 != ESO_ASSIGN ) &&
						( rhs1->isGiven(i,j) == BT_TRUE ) && ( rhs2->isGiven(i,j) == BT_TRUE ) )
				{
					// check for zero value in case of "+=" or "-="
					if ( ( op1 == ESO_ADD ) && ( acadoIsZero(rhs1(i, j) + rhs2(i, j)) == BT_TRUE ) )
						continue;

					if ( ( op1 == ESO_SUBTRACT ) && ( acadoIsZero( rhs1(i, j) - rhs2(i, j)) == BT_TRUE ) )
						continue;
				}

				if ( !lhs.isNull() )
					acadoFPrintf( file,"%s %s ", lhs.get(i,j).getName(),assignString.getName() );

				if ( rhs1->isZero(i, j) == BT_FALSE )
				{
					acadoFPrintf( file,"%s", rhs1->get(i,j).getName() );
					if ( rhs2->isZero(i,j) == BT_FALSE )
						acadoFPrintf( file," %s %s;\n", _sign.getName(),rhs2->get(i,j).getName() );
					else
						acadoFPrintf( file,";\n" );
				}
				else
				{
					if ( rhs2->isZero(i,j) == BT_FALSE )
						acadoFPrintf( file,"%s %s;\n", _sign.getName(),rhs2->get(i,j).getName() );
					else
						acadoFPrintf( file,"0.0;\n" );
				}
			}
	}
	else if ( numberOfFlops < 32768 )
	{
		ExportIndex ii;
		memAllocator->acquire( ii );

		acadoFPrintf(file, "for (%s = 0; ", ii.getName().getName());
		acadoFPrintf(file, "%s < %d; ", ii.getName().getName(), getNumRows());
		acadoFPrintf(file, "++%s)\n{\n", ii.getName().getName());

		for(unsigned j = 0; j < getNumCols( ); ++j)
		{
			acadoFPrintf( file,"%s %s ", lhs->get(ii, j).getName(), assignString.getName() );
			acadoFPrintf( file,"%s %s;\n", _sign.getName(), rhs2->get(ii,j).getName() );
		}

		acadoFPrintf(file, "\n{\n");

		memAllocator->release( ii );
	}
	else
	{
		ExportIndex ii, jj;
		memAllocator->acquire( ii );
		memAllocator->acquire( jj );

		acadoFPrintf(file, "for (%s = 0; ", ii.getName().getName());
		acadoFPrintf(file, "%s < %d; ", ii.getName().getName(), getNumRows());
		acadoFPrintf(file, "++%s)\n{\n", ii.getName().getName());

		acadoFPrintf(file, "for (%s = 0; ", jj.getName().getName());
		acadoFPrintf(file, "%s < %d; ", jj.getName().getName(), getNumRows());
		acadoFPrintf(file, "++%s)\n{\n", jj.getName().getName());

		acadoFPrintf( file,"%s %s ", lhs->get(ii, jj).getName(), assignString.getName() );
		acadoFPrintf( file,"%s %s;\n", _sign.getName(), rhs2->get(ii,jj).getName() );

		acadoFPrintf(file, "\n{\n");
		acadoFPrintf(file, "\n{\n");

		memAllocator->release( ii );
		memAllocator->release( jj );
	}

	return SUCCESSFUL_RETURN;
}

returnValue ExportArithmeticStatement::exportCodeMultiply(	FILE* file,
															BooleanType transposeRhs1,
															const String& _realString,
															const String& _intString,
															int _precision
															) const
{
//	if ( ( lhs.isNull() ) || ( rhs1.isNull() ) || ( rhs2.isNull() ) )
//		return ACADOERROR( RET_UNABLE_TO_EXPORT_STATEMENT );

	if (lhs.getDim() == 0 || rhs1.getDim() == 0 || rhs2.getDim() == 0)
		return SUCCESSFUL_RETURN;

	String assignString;
	if ( getAssignString( assignString ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_STATEMENT );

	uint nRowsRhs1;
	uint nColsRhs1;

	if ( transposeRhs1 == BT_FALSE )
	{
		nRowsRhs1 = rhs1->getNumRows( );
		nColsRhs1 = rhs1->getNumCols( );
	}
	else
	{
		nRowsRhs1 = rhs1->getNumCols( );
		nColsRhs1 = rhs1->getNumRows( );
	}

	if ( ( nColsRhs1 != rhs2->getNumRows( ) ) ||
			( nRowsRhs1 != lhs->getNumRows( ) ) ||
			( rhs2->getNumCols( ) != lhs->getNumCols( ) ) )
	return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );

	char sign[2] = "+";

	if ( op2 != ESO_UNDEFINED )
	{
		if ( ( rhs3->getNumRows( ) != lhs->getNumRows( ) ) ||
				( rhs3->getNumCols( ) != lhs->getNumCols( ) ) )
		return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );

		if ( op2 == ESO_SUBTRACT )
		sign[0] = '-';
	}

	BooleanType allZero;

	ExportIndex ii, iiRhs1;
	ExportIndex jj, jjRhs1;
	ExportIndex kk, kkRhs1;

	//
	// Rough approximation of flops needed for matrix multiplication
	//
	unsigned numberOfFlops = nRowsRhs1 * rhs2->getNumRows( ) * rhs2->getNumCols();

	//
	// Optimization can be performed only if both matrices are not given.
	// Currently, optimizations cannot be performed on hard-coded matrices.
	//
	int optimizationsAllowed =
			( rhs1->isGiven() == BT_FALSE ) && ( rhs2->isGiven() == BT_FALSE );

	//
	// Depending on the flops count different export strategies are performed
	//
	if ( ( numberOfFlops < 4096 ) || ( optimizationsAllowed == 0) )
	{
		//
		// Unroll all loops
		//

		for(uint i = 0; i < getNumRows( ); ++i)
		{
			ii = i;

			for(uint j = 0; j < getNumCols( ); ++j)
			{
				allZero = BT_TRUE;

				acadoFPrintf( file,"%s %s", lhs->get(ii,j).getName(), assignString.getName() );

				for(uint k = 0; k < nColsRhs1; ++k)
				{
					kk = k;
					if ( transposeRhs1 == BT_FALSE )
					{
						iiRhs1 = ii;
						kkRhs1 = kk;
					}
					else
					{
						iiRhs1 = kk;
						kkRhs1 = ii;
					}

					if ( ( rhs1->isZero(iiRhs1,kkRhs1) == BT_FALSE ) &&
							( rhs2->isZero(kk,j) == BT_FALSE ) )
					{
						allZero = BT_FALSE;

						if ( rhs1->isOne(iiRhs1,kkRhs1) == BT_FALSE )
						{
							acadoFPrintf( file," %s %s", sign,rhs1->get(iiRhs1,kkRhs1).getName() );
							if ( rhs2->isOne(kk,j) == BT_FALSE )
							acadoFPrintf( file,"*%s", rhs2->get(kk,j).getName() );
						}
						else
						{
							if ( rhs2->isOne(kk,j) == BT_FALSE )
							acadoFPrintf( file," %s %s", sign,rhs2->get(kk,j).getName() );
							else
							acadoFPrintf( file," %s 1.0", sign );
						}
					}
				}

				if ( ( op2 == ESO_ADD ) || ( op2 == ESO_SUBTRACT ) )
					acadoFPrintf( file," + %s;\n", rhs3->get(ii,j).getName() );

				if ( op2 == ESO_UNDEFINED )
				{
					if ( allZero == BT_TRUE )
						acadoFPrintf( file," 0.0;\n" );
					else
						acadoFPrintf( file,";\n" );
				}
			}
		}
	}
	else if ( numberOfFlops < 32768 )
	{
		//
		// Unroll two inner loops
		//

		memAllocator->acquire( ii );

		acadoFPrintf(file, "for (%s = 0; ", ii.getName().getName());
		acadoFPrintf(file, "%s < %d; ",ii.getName().getName(), getNumRows());
		acadoFPrintf(file, "++%s)\n{\n",ii.getName().getName());

		for(uint j = 0; j < getNumCols( ); ++j)
		{
			allZero = BT_TRUE;

			acadoFPrintf( file,"%s %s", lhs->get(ii,j).getName(), assignString.getName() );

			for(uint k = 0; k < nColsRhs1; ++k)
			{
				kk = k;
				if ( transposeRhs1 == BT_FALSE )
				{
					iiRhs1 = ii;
					kkRhs1 = kk;
				}
				else
				{
					iiRhs1 = kk;
					kkRhs1 = ii;
				}

				if ( ( rhs1->isZero(iiRhs1,kkRhs1) == BT_FALSE ) &&
					( rhs2->isZero(kk,j) == BT_FALSE ) )
				{
					allZero = BT_FALSE;

					if ( rhs1->isOne(iiRhs1,kkRhs1) == BT_FALSE )
					{
						acadoFPrintf( file," %s %s", sign,rhs1->get(iiRhs1,kkRhs1).getName() );
						if ( rhs2->isOne(kk,j) == BT_FALSE )
							acadoFPrintf( file,"*%s", rhs2->get(kk,j).getName() );
					}
					else
					{
						if ( rhs2->isOne(kk,j) == BT_FALSE )
							acadoFPrintf( file," %s %s", sign,rhs2->get(kk,j).getName() );
						else
							acadoFPrintf( file," %s 1.0", sign );
					}
				}
			}

			if ( ( op2 == ESO_ADD ) || ( op2 == ESO_SUBTRACT ) )
				acadoFPrintf( file," + %s;\n", rhs3->get(ii,j).getName() );

			if ( op2 == ESO_UNDEFINED )
			{
				if ( allZero == BT_TRUE )
					acadoFPrintf( file," 0.0;\n" );
				else
					acadoFPrintf( file,";\n" );
			}
		}
		acadoFPrintf( file,"\n}\n" );

		memAllocator->release( ii );
	}
	else
	{
		//
		// Keep rolled first two outer loops
		//

		memAllocator->acquire( ii );
		memAllocator->acquire( jj );

		// First loop
		acadoFPrintf(file, "for (%s = 0; ", ii.getName().getName());
		acadoFPrintf(file, "%s < %d; ", ii.getName().getName(), getNumRows());
		acadoFPrintf(file, "++%s)\n{\n", ii.getName().getName());

		// Second loop

		acadoFPrintf(file, "for (%s = 0; ", jj.getName().getName());
		acadoFPrintf(file, "%s < %d; ", jj.getName().getName(), getNumCols());
		acadoFPrintf(file, "++%s)\n{\n", jj.getName().getName());

		allZero = BT_TRUE;

		acadoFPrintf( file,"%s %s", lhs->get(ii,jj).getName(), assignString.getName() );

		for(uint k = 0; k < nColsRhs1; ++k)
		{
			kk = k;
			if ( transposeRhs1 == BT_FALSE )
			{
				iiRhs1 = ii;
				kkRhs1 = kk;
			}
			else
			{
				iiRhs1 = kk;
				kkRhs1 = ii;
			}

			if ( ( rhs1->isZero(iiRhs1,kkRhs1) == BT_FALSE ) &&
					( rhs2->isZero(kk,jj) == BT_FALSE ) )
			{
				allZero = BT_FALSE;

				if ( rhs1->isOne(iiRhs1,kkRhs1) == BT_FALSE )
				{
					acadoFPrintf( file," %s %s", sign,rhs1->get(iiRhs1,kkRhs1).getName() );
					if ( rhs2->isOne(kk,jj) == BT_FALSE )
						acadoFPrintf( file,"*%s", rhs2->get(kk,jj).getName() );
				}
				else
				{
					if ( rhs2->isOne(kk,jj) == BT_FALSE )
						acadoFPrintf( file," %s %s", sign,rhs2->get(kk,jj).getName() );
					else
						acadoFPrintf( file," %s 1.0", sign );
				}
			}
		}

		if ( ( op2 == ESO_ADD ) || ( op2 == ESO_SUBTRACT ) )
			acadoFPrintf( file," + %s;\n", rhs3->get(ii,jj).getName() );

		if ( op2 == ESO_UNDEFINED )
		{
			if ( allZero == BT_TRUE )
				acadoFPrintf( file," 0.0;\n" );
			else
				acadoFPrintf( file,";\n" );
		}

		acadoFPrintf( file,"\n}\n" );
		acadoFPrintf( file,"\n}\n" );

		memAllocator->release( ii );
		memAllocator->release( jj );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ExportArithmeticStatement::exportCodeAssign(	FILE* file,
															const String& _op,
															const String& _realString,
															const String& _intString,
															int _precision
															) const
{
	if (lhs.getDim() == 0 || rhs1.getDim() == 0 || rhs2.getDim() == 0)
		return SUCCESSFUL_RETURN;

	if ( ( rhs1.getNumRows( ) != lhs.getNumRows( ) ) ||
		 ( rhs1.getNumCols( ) != lhs.getNumCols( ) ) )
	{
		LOG( LVL_DEBUG ) << "lhs name is " << lhs.getName().getName()
				<< ", size: " << lhs.getNumRows() << " x " << lhs.getNumCols()
				<< "rhs1 name is " << rhs1.getName().getName()
				<< ", size: " << rhs1.getNumRows() << " x " << rhs1.getNumCols() << endl;

		return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );
	}

	stringstream s;
	s.precision( 16 );

	unsigned numOps = lhs.getNumRows() * lhs.getNumCols();

	if (	lhs.isSubMatrix() == BT_FALSE && lhs.getDim() > 1 &&
			rhs1.isGiven() == BT_TRUE && rhs1.getGivenMatrix().isZero() == BT_TRUE)
	{
		s 	<< "{ int lCopy; for (lCopy = 0; lCopy < "<< lhs.getDim() << "; lCopy++) "
			<< lhs.getFullName() << "[ lCopy ] = 0.0; }" << endl;
	}
	else if ((numOps < 128) || (rhs1.isGiven() == BT_TRUE))
	{
		for(unsigned i = 0; i < lhs.getNumRows( ); ++i)
			for(unsigned j = 0; j < lhs.getNumCols( ); ++j)
				if ( ( _op == (String)"=" ) || ( rhs1.isZero(i,j) == BT_FALSE ) )
				{
					s << lhs->get(i, j).getName() << " " << _op.getName() << " ";
					if (rhs1->isGiven() == BT_TRUE)
					{
						s << scientific << rhs1(i, j);
					}
					else
					{
						s << rhs1->get(i, j).getName();
					}
					s << ";" << endl;
				}
	}
	else
	{
		ExportIndex ii, jj;

		if (lhs.getNumCols() == 1 || lhs.getNumRows() == 1)
		{
			memAllocator->acquire( ii );

			s << "for (" << ii.get().getName() << " = 0; " << ii.get().getName() << " < ";

			if (lhs->getNumCols() == 1)
			{
				s << lhs->getNumRows() << "; ++" << ii.getName().getName() << ")" << endl
						<< lhs.get(ii, 0).getName() << " " << _op.getName() << " " << rhs1.get(ii, 0).getName()
						<< ";" << endl << endl;
			}
			else
			{
				s << lhs.getNumCols() << "; ++" << ii.getName().getName() << ")" << endl;
				s << lhs.get(0, ii).getName() << " " << _op.getName() << " " << rhs1.get(0, ii).getName()
						<< ";" << endl << endl;
			}

			memAllocator->release( ii );
		}
		else
		{
			memAllocator->acquire( ii );
			memAllocator->acquire( jj );

			s << "for (" << ii.getName().getName() << " = 0;" << ii.getName().getName() << " < "
					<< lhs->getNumRows() << "; ++" << ii.getName().getName() << ")" << endl;

			s << "for (" << jj.getName().getName() << " = 0;" << jj.getName().getName() << " < "
					<< lhs->getNumCols() << "; ++" << jj.getName().getName() << ")" << endl;

			s << lhs->get(ii, jj).getName() << " " << _op.getName( ) << " " << rhs1->get(ii, jj).getName() << ";" << endl;

			memAllocator->release( ii );
			memAllocator->release( jj );
		}
	}
	
	acadoFPrintf(file, "%s", s.str().c_str());

	return SUCCESSFUL_RETURN;
}


returnValue ExportArithmeticStatement::getAssignString(	String& _assignString
														) const
{
	switch ( op0 )
	{
		case ESO_ASSIGN:
			_assignString = "=";
			return SUCCESSFUL_RETURN;
		
		case ESO_ADD_ASSIGN:
			_assignString = "+=";
			return SUCCESSFUL_RETURN;
			
		case ESO_SUBTRACT_ASSIGN:
			_assignString = "-=";
			return SUCCESSFUL_RETURN;
			
		default:
			return RET_UNABLE_TO_EXPORT_STATEMENT;
	}
}

ExportArithmeticStatement& ExportArithmeticStatement::allocate( MemoryAllocatorPtr allocator )
{
	memAllocator = allocator;

	return *this;
}

CLOSE_NAMESPACE_ACADO

// end of file.
