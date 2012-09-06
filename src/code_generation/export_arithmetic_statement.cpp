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
 *    \file src/code_generation/export_arithmetic_statement.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010-2011
 */

#include <acado/code_generation/export_arithmetic_statement.hpp>


BEGIN_NAMESPACE_ACADO

//
// By default, we do not want to export C99-compliant code.
//
BooleanType ExportArithmeticStatement::allowedC99 = NO;

//
// PUBLIC MEMBER FUNCTIONS:
//

ExportArithmeticStatement::ExportArithmeticStatement( )
{
	lhs  = 0;
	rhs1 = 0;
	rhs2 = 0;
	rhs3 = 0;

	op0 = ESO_UNDEFINED;
	op1 = ESO_UNDEFINED;
	op2 = ESO_UNDEFINED;
	
	outerLoopVariable = -1;
}


ExportArithmeticStatement::ExportArithmeticStatement(	const ExportVariable* const _lhs,
														ExportStatementOperator _op0,
														const ExportVariable* const _rhs1,
														ExportStatementOperator _op1,
														const ExportVariable* const _rhs2,
														ExportStatementOperator _op2,
														const ExportVariable* const _rhs3
														) : ExportStatement( )
{
	ASSERT( ( _op0 == ESO_UNDEFINED ) || ( _op0 == ESO_ASSIGN ) || ( _op0 == ESO_ADD_ASSIGN ) || ( _op0 == ESO_SUBTRACT_ASSIGN ) );
	ASSERT( ( _op2 == ESO_UNDEFINED ) || ( _op2 == ESO_ADD ) || ( _op2 == ESO_SUBTRACT ) );

	if ( _lhs != 0 )
		lhs  = new ExportVariable( *_lhs );
	else
		lhs = 0;
	
	if ( _rhs1 != 0 )
		rhs1  = new ExportVariable( *_rhs1 );
	else
		rhs1 = 0;
	
	if ( _rhs2 != 0 )
		rhs2  = new ExportVariable( *_rhs2 );
	else
		rhs2 = 0;

	if ( _rhs3 != 0 )
		rhs3  = new ExportVariable( *_rhs3 );
	else
		rhs3 = 0;

	op0  = _op0;
	op1  = _op1;
	op2  = _op2;
	
	outerLoopVariable = -1;
}


ExportArithmeticStatement::ExportArithmeticStatement( const ExportArithmeticStatement& arg ) : ExportStatement( arg )
{
	if ( arg.lhs != 0 )
		lhs  = new ExportVariable( *(arg.lhs) );
	else
		lhs = 0;
	
	if ( arg.rhs1 != 0 )
		rhs1  = new ExportVariable( *(arg.rhs1) );
	else
		rhs1 = 0;
	
	if ( arg.rhs2 != 0 )
		rhs2  = new ExportVariable( *(arg.rhs2) );
	else
		rhs2 = 0;
	
	if ( arg.rhs3 != 0 )
		rhs3  = new ExportVariable( *(arg.rhs3) );
	else
		rhs3 = 0;

	op0  = arg.op0;
	op1  = arg.op1;
	op2  = arg.op2;
	
	outerLoopVariable = arg.outerLoopVariable;
}


ExportArithmeticStatement::~ExportArithmeticStatement( )
{
	if ( lhs != 0 )
		delete lhs;

	if ( rhs1 != 0 )
		delete rhs1;

	if ( rhs2 != 0 )
		delete rhs2;
	
	if ( rhs3 != 0 )
		delete rhs3;
}


ExportArithmeticStatement& ExportArithmeticStatement::operator=( const ExportArithmeticStatement& arg )
{
	if( this != &arg )
	{
		if ( lhs != 0 )
			delete lhs;

		if ( rhs1 != 0 )
			delete rhs1;

		if ( rhs2 != 0 )
			delete rhs2;

		if ( rhs3 != 0 )
			delete rhs3;

		ExportStatement::operator=( arg );
		
		if ( arg.lhs != 0 )
			lhs  = new ExportVariable( *(arg.lhs) );
		else
			lhs = 0;
		
		if ( arg.rhs1 != 0 )
			rhs1  = new ExportVariable( *(arg.rhs1) );
		else
			rhs1 = 0;
		
		if ( arg.rhs2 != 0 )
			rhs2  = new ExportVariable( *(arg.rhs2) );
		else
			rhs2 = 0;

		if ( arg.rhs3 != 0 )
			rhs3  = new ExportVariable( *(arg.rhs3) );
		else
			rhs3 = 0;

		op0  = arg.op0;
		op1  = arg.op1;
		op2  = arg.op2;
		
		outerLoopVariable = arg.outerLoopVariable;
	}

	return *this;
}


ExportStatement* ExportArithmeticStatement::clone( ) const
{
	return new ExportArithmeticStatement(*this);
}


uint ExportArithmeticStatement::getNumRows( ) const
{
	if ( rhs1 == 0 )
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
	if ( rhs1 == 0 )
		return 0;
	else
	{
		if ( rhs2 == 0 )
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
	if ( (outerLoopVariable.isGiven() == BT_FALSE) && (allowedC99 == NO))
		outerLoopVariable.exportDataDeclaration( file,_realString,_intString,_precision );
	
	return SUCCESSFUL_RETURN;
}


returnValue ExportArithmeticStatement::exportCode(	FILE* file,
													const String& _realString,
													const String& _intString,
													int _precision
													) const
{
// 	if ( lhs == 0 )
// 		return ACADOERROR( RET_UNABLE_TO_EXPORT_STATEMENT );
	
	switch( op1 )
	{
		case ESO_ADD:
			return exportCodeAddSubtract( file,"+",_realString,_intString,_precision );

		case ESO_SUBTRACT:
			return exportCodeAddSubtract( file,"-",_realString,_intString,_precision );
			
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


ExportArithmeticStatement& ExportArithmeticStatement::keepOuterLoop( )
{
	outerLoopVariable.init( "run1" );
	return *this;
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
	if ( ( rhs1 == 0 ) || ( rhs2 == 0 ) || ( rhs3 != 0 ) )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_STATEMENT );

	if ( ( rhs1->getNumRows() != rhs2->getNumRows() ) || 
		 ( rhs1->getNumCols() != rhs2->getNumCols() ) )
		return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );
	
	if ( lhs != 0 )
	{
		if ( ( rhs1->getNumRows() != lhs->getNumRows() ) || 
		     ( rhs1->getNumCols() != lhs->getNumCols() ) )
			return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );
	}
	
	String assignString;
	if ( getAssignString( assignString ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_STATEMENT );
	
	for( uint i=0; i<getNumRows( ); ++i )
		for( uint j=0; j<getNumCols( ); ++j )
		{
			if ( ( op0 != ESO_ASSIGN ) &&
				 ( rhs1->isGiven(i,j) == BT_TRUE ) && ( rhs2->isGiven(i,j) == BT_TRUE ) )
			{
				// check for zero value in case of "+=" or "-="
				if ( ( op1 == ESO_ADD ) && ( acadoIsZero( rhs1->operator()(i,j)+rhs2->operator()(i,j) ) == BT_TRUE ) )
					continue;

				if ( ( op1 == ESO_SUBTRACT ) && ( acadoIsZero( rhs1->operator()(i,j)-rhs2->operator()(i,j) ) == BT_TRUE ) )
					continue;
			}

			if ( lhs != 0 )
				acadoFPrintf( file,"%s %s ", lhs->get(i,j),assignString.getName() );

			if ( rhs1->isZero(i,j) == BT_FALSE )
			{
				acadoFPrintf( file,"%s", rhs1->get(i,j) );
				if ( rhs2->isZero(i,j) == BT_FALSE )
					acadoFPrintf( file," %s %s;\n", _sign.getName(),rhs2->get(i,j) );
				else
					acadoFPrintf( file,";\n" );
			}
			else
			{
				if ( rhs2->isZero(i,j) == BT_FALSE )
					acadoFPrintf( file,"%s %s;\n", _sign.getName(),rhs2->get(i,j) );
				else
					acadoFPrintf( file,"0.0;\n" );
			}
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
	if ( ( lhs == 0 ) || ( rhs1 == 0 ) || ( rhs2 == 0 ) )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_STATEMENT );

	if ( allowedC99 == YES )
	{
		exportCodeMultiplyC99(file, transposeRhs1, _realString, _intString, _precision);

		return SUCCESSFUL_RETURN;
	}

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
// 	uint ii, kk;
// 	uint iiRhs1, kkRhs1;

	ExportIndex ii( "run1" ), iiRhs1( "run1" );
	ExportIndex kk( "run3" ), kkRhs1( "run3" );

	uint iMax = getNumRows( );
	if ( outerLoopVariable.isGiven( ) == BT_FALSE )
		iMax = 1;
	
	for( uint i=0; i<iMax; ++i )
	{
		if ( outerLoopVariable.isGiven( ) == BT_FALSE )
		{
			acadoFPrintf( file, "for ( %s=0; ", ii.getName().getName() );
			acadoFPrintf( file, "%s<%d; ", ii.getName().getName(),getNumRows() );
			acadoFPrintf( file, "++%s ){\n", ii.getName().getName() );
		}
		else
			ii = i;
	
		for( uint j=0; j<getNumCols( ); ++j )
		{
			allZero = BT_TRUE;
			acadoFPrintf( file,"%s %s", lhs->get(ii,j),assignString.getName() );

			for( uint k=0; k<nColsRhs1; ++k )
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
						acadoFPrintf( file," %s %s", sign,rhs1->get(iiRhs1,kkRhs1) );
						if ( rhs2->isOne(kk,j) == BT_FALSE )
							acadoFPrintf( file,"*%s", rhs2->get(kk,j) );
					}
					else
					{
						if ( rhs2->isOne(kk,j) == BT_FALSE )
							acadoFPrintf( file," %s %s", sign,rhs2->get(kk,j) );
						else
							acadoFPrintf( file," %s 1.0", sign );
					}
				}
			}
			
			if ( ( op2 == ESO_ADD ) || ( op2 == ESO_SUBTRACT ) )
				acadoFPrintf( file," + %s;\n", rhs3->get(ii,j) );

			if ( op2 == ESO_UNDEFINED )
			{
				if ( allZero == BT_TRUE )
					acadoFPrintf( file," 0.0;\n" );
				else
					acadoFPrintf( file,";\n" );
			}
		}
	}
	
	if ( outerLoopVariable.isGiven( ) == BT_FALSE )
		acadoFPrintf( file,"}\n" );
	

	return SUCCESSFUL_RETURN;
}

returnValue ExportArithmeticStatement::exportCodeMultiplyC99(
	FILE* file,
	BooleanType transposeRhs1,
	const String& _realString,
	const String& _intString,
	int _precision
) const
{
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

	ExportIndex ii( "run1" ), iiRhs1( "run1" );
	ExportIndex jj( "run2" ), jjRhs1( "run2" );
	ExportIndex kk( "run3" ), kkRhs1( "run3" );

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

				acadoFPrintf( file,"%s %s", lhs->get(ii,j), assignString.getName() );

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
							acadoFPrintf( file," %s %s", sign,rhs1->get(iiRhs1,kkRhs1) );
							if ( rhs2->isOne(kk,j) == BT_FALSE )
							acadoFPrintf( file,"*%s", rhs2->get(kk,j) );
						}
						else
						{
							if ( rhs2->isOne(kk,j) == BT_FALSE )
							acadoFPrintf( file," %s %s", sign,rhs2->get(kk,j) );
							else
							acadoFPrintf( file," %s 1.0", sign );
						}
					}
				}

				if ( ( op2 == ESO_ADD ) || ( op2 == ESO_SUBTRACT ) )
					acadoFPrintf( file," + %s;\n", rhs3->get(ii,j) );

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

		acadoFPrintf( file, "for ( int %s=0; ", ii.getName().getName() );
		acadoFPrintf( file, "%s<%d; ", ii.getName().getName(),getNumRows() );
		acadoFPrintf( file, "++%s )\n{\n", ii.getName().getName() );

		for(uint j = 0; j < getNumCols( ); ++j)
		{
			allZero = BT_TRUE;

			acadoFPrintf( file,"%s %s", lhs->get(ii,j), assignString.getName() );

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
						acadoFPrintf( file," %s %s", sign,rhs1->get(iiRhs1,kkRhs1) );
						if ( rhs2->isOne(kk,j) == BT_FALSE )
							acadoFPrintf( file,"*%s", rhs2->get(kk,j) );
					}
					else
					{
						if ( rhs2->isOne(kk,j) == BT_FALSE )
							acadoFPrintf( file," %s %s", sign,rhs2->get(kk,j) );
						else
							acadoFPrintf( file," %s 1.0", sign );
					}
				}
			}

			if ( ( op2 == ESO_ADD ) || ( op2 == ESO_SUBTRACT ) )
				acadoFPrintf( file," + %s;\n", rhs3->get(ii,j) );

			if ( op2 == ESO_UNDEFINED )
			{
				if ( allZero == BT_TRUE )
					acadoFPrintf( file," 0.0;\n" );
				else
					acadoFPrintf( file,";\n" );
			}
		}
		acadoFPrintf( file,"\n}\n" );
	}
	else
	{
		//
		// Keep rolled first two outer loops
		//

		// First loop
		acadoFPrintf( file, "for ( int %s = 0; ", ii.getName().getName() );
		acadoFPrintf( file, "%s<%d; ", ii.getName().getName(),getNumRows() );
		acadoFPrintf( file, "++%s )\n{\n", ii.getName().getName() );

		// Second loop

		acadoFPrintf( file, "for ( int %s = 0; ", jj.getName().getName() );
		acadoFPrintf( file, "%s < %d; ", jj.getName().getName(), getNumCols() );
		acadoFPrintf( file, "++%s )\n{\n", jj.getName().getName() );

		allZero = BT_TRUE;

		acadoFPrintf( file,"%s %s", lhs->get(ii,jj), assignString.getName() );

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
					acadoFPrintf( file," %s %s", sign,rhs1->get(iiRhs1,kkRhs1) );
					if ( rhs2->isOne(kk,jj) == BT_FALSE )
						acadoFPrintf( file,"*%s", rhs2->get(kk,jj) );
				}
				else
				{
					if ( rhs2->isOne(kk,jj) == BT_FALSE )
						acadoFPrintf( file," %s %s", sign,rhs2->get(kk,jj) );
					else
						acadoFPrintf( file," %s 1.0", sign );
				}
			}
		}

		if ( ( op2 == ESO_ADD ) || ( op2 == ESO_SUBTRACT ) )
			acadoFPrintf( file," + %s;\n", rhs3->get(ii,jj) );

		if ( op2 == ESO_UNDEFINED )
		{
			if ( allZero == BT_TRUE )
				acadoFPrintf( file," 0.0;\n" );
			else
				acadoFPrintf( file,";\n" );
		}

		acadoFPrintf( file,"\n}\n" );
		acadoFPrintf( file,"\n}\n" );
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
	if ( ( lhs == 0 ) || ( rhs1 == 0 ) || ( rhs2 != 0 ) )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_STATEMENT );

	if ( ( rhs1->getNumRows( ) != lhs->getNumRows( ) ) || 
		 ( rhs1->getNumCols( ) != lhs->getNumCols( ) ) )
		return ACADOERROR( RET_VECTOR_DIMENSION_MISMATCH );

	
	for( uint i=0; i<getNumRows( ); ++i )
		for( uint j=0; j<getNumCols( ); ++j )
		{
			if ( ( _op == (String)"=" ) || ( rhs1->isZero(i,j) == BT_FALSE ) )
			{
				acadoFPrintf( file,"%s %s ", lhs->get(i,j),_op.getName() );
				acadoFPrintf( file,"%s;\n", rhs1->get(i,j) );
			}
		}
	
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



CLOSE_NAMESPACE_ACADO

// end of file.
