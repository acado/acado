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
 *    \file src/matrix_vector/block_matrix.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 19.01.2009
 */


#include <acado/matrix_vector/matrix_vector.hpp>



BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//

BlockMatrix::BlockMatrix( ){

	nRows    = 0;
	nCols    = 0;
    elements = 0;
    types    = 0;
}


BlockMatrix::BlockMatrix( uint _nRows, uint _nCols )
{
    uint run1, run2;

	nRows = _nRows;
	nCols = _nCols;

    elements = new Matrix            *[nRows];
    types    = new SubBlockMatrixType*[nRows];

    for( run1 = 0; run1 < nRows; run1++ ){

        elements[run1] = new Matrix            [nCols];
        types   [run1] = new SubBlockMatrixType[nCols];

        for( run2 = 0; run2 < nCols; run2++ ){

            types[run1][run2] = SBMT_ZERO;
        }
    }
}


BlockMatrix::BlockMatrix(	const Matrix& value
							)
{
	nRows = 1;
	nCols = 1;

	elements = new Matrix            *[nRows];
	types    = new SubBlockMatrixType*[nRows];

	elements[0] = new Matrix            [nCols];
	types   [0] = new SubBlockMatrixType[nCols];

	setDense( 0,0,value );
}


BlockMatrix::BlockMatrix( const BlockMatrix& rhs ){

    uint run1, run2;

	nRows = rhs.nRows;
	nCols = rhs.nCols;

    if( rhs.elements != 0 ){

        elements = new Matrix            *[nRows];
        types    = new SubBlockMatrixType*[nRows];

        for( run1 = 0; run1 < nRows; run1++ ){

            elements[run1] = new Matrix            [nCols];
            types   [run1] = new SubBlockMatrixType[nCols];

            for( run2 = 0; run2 < nCols; run2++ ){

                elements[run1][run2] = rhs.elements[run1][run2];
                types   [run1][run2] = rhs.types   [run1][run2];
            }
        }
    }
    else{
        elements = 0;
        types    = 0;
    }
}


BlockMatrix::~BlockMatrix( ){

    uint run1;

    if( elements != 0 ){

        for( run1 = 0; run1 < nRows; run1++ ){

            if( elements[run1] != 0 )
                delete[] elements[run1];
        }

        delete[] elements;
    }

    if( types != 0 ){

        for( run1 = 0; run1 < nRows; run1++ ){

            if( types[run1] != 0 )
                delete[] types[run1];
        }

        delete[] types;
    }
}


BlockMatrix& BlockMatrix::operator=( const BlockMatrix& rhs ){

    uint run1, run2;

    if ( this != &rhs ){

        if( elements != 0 ){

            for( run1 = 0; run1 < nRows; run1++ ){

                if( elements[run1] != 0 )
                    delete[] elements[run1];
            }

            delete[] elements;
        }

        if( types != 0 ){

            for( run1 = 0; run1 < nRows; run1++ ){

                if( types[run1] != 0 )
                    delete[] types[run1];
            }

            delete[] types;
        }

        nRows = rhs.nRows;
        nCols = rhs.nCols;

        if( rhs.elements != 0 ){

            elements = new Matrix            *[nRows];
            types    = new SubBlockMatrixType*[nRows];

            for( run1 = 0; run1 < nRows; run1++ ){

                elements[run1] = new Matrix            [nCols];
                types   [run1] = new SubBlockMatrixType[nCols];

                for( run2 = 0; run2 < nCols; run2++ ){
                    elements[run1][run2] = rhs.elements[run1][run2];
                    types   [run1][run2] = rhs.types   [run1][run2];
                }
            }
        }
        else{
            elements = 0;
            types    = 0;
        }
    }

    return *this;
}


returnValue BlockMatrix::init( uint _nRows, uint _nCols ){


    uint run1, run2;

    if( elements != 0 ){

        for( run1 = 0; run1 < nRows; run1++ ){

            if( elements[run1] != 0 )
                delete[] elements[run1];
        }

        delete[] elements;
    }

    if( types != 0 ){

        for( run1 = 0; run1 < nRows; run1++ ){

            if( types[run1] != 0 )
                delete[] types[run1];
        }

        delete[] types; 
    }

    nRows = _nRows;
    nCols = _nCols;

    elements = new Matrix            *[nRows];
    types    = new SubBlockMatrixType*[nRows];

    for( run1 = 0; run1 < nRows; run1++ ){

        elements[run1] = new Matrix            [nCols];
        types   [run1] = new SubBlockMatrixType[nCols];

        for( run2 = 0; run2 < nCols; run2++ ){

            types[run1][run2] = SBMT_ZERO;
        }
    }

	return SUCCESSFUL_RETURN;
}


BlockMatrix BlockMatrix::operator+( const BlockMatrix& arg ) const{

	ASSERT( ( getNumRows( ) == arg.getNumRows( ) ) && ( getNumCols( ) == arg.getNumCols( ) ) );

	uint i,j;

	BlockMatrix tmp( arg );

	for( i=0; i < getNumRows(); i++ ){
        for( j=0; j < getNumCols(); j++ ){

            if( tmp.types[i][j] == SBMT_ZERO ){

                tmp.types    [i][j] = types    [i][j];
                tmp.elements [i][j] = elements [i][j];
            }
            else{
                if( types[i][j] != SBMT_ZERO ){

                   tmp.types    [i][j]  = SBMT_DENSE     ;
                   tmp.elements [i][j] += elements [i][j];
                }
            }
        }
    }

	return tmp;
}


BlockMatrix& BlockMatrix::operator+=( const BlockMatrix& arg ){

	ASSERT( ( getNumRows( ) == arg.getNumRows( ) ) && ( getNumCols( ) == arg.getNumCols( ) ) );

	uint i, j;

	for( i = 0; i < getNumRows(); i++ ){
        for( j = 0; j < getNumCols(); j++ ){

            if( types[i][j] == SBMT_ZERO ){

                types    [i][j] = arg.types    [i][j];
                elements [i][j] = arg.elements [i][j];
            }
            else{
                if( arg.types[i][j] != SBMT_ZERO ){

                   types    [i][j]  = SBMT_DENSE         ;
                   elements [i][j] += arg.elements [i][j];
                }
            }
        }
    }
	return *this;
}


BlockMatrix BlockMatrix::operator-( const BlockMatrix& arg ) const{

    ASSERT( ( getNumRows( ) == arg.getNumRows( ) ) && ( getNumCols( ) == arg.getNumCols( ) ) );

    uint i,j;

    BlockMatrix tmp( getNumRows(), getNumCols() );

    for( i=0; i < getNumRows(); i++ ){
        for( j=0; j < getNumCols(); j++ ){

            if( arg.types[i][j] == SBMT_ZERO ){

                tmp.types    [i][j] = types    [i][j];
                tmp.elements [i][j] = elements [i][j];
            }
            else{
                if( types[i][j] != SBMT_ZERO ){

                   tmp.types    [i][j]  = SBMT_DENSE;
                   tmp.elements [i][j]  = elements [i][j] - arg.elements[i][j];
                }
                else{

                   Matrix nn(arg.elements[i][j].getNumRows(),arg.elements[i][j].getNumCols());
                   nn.setZero();

                   tmp.types    [i][j]  = SBMT_DENSE;
                   tmp.elements [i][j]  = nn - arg.elements[i][j];
                }
            }
        }
    }

    return tmp;
}



// 
// 
// BlockMatrix BlockMatrix::operator-=(	const BlockMatrix& arg ){
// 
// 	ASSERT( ( getNumRows( ) == arg.getNumRows( ) ) && ( getNumCols( ) == arg.getNumCols( ) ) );
// 
// 
// 
// 	return *this;
// }


BlockMatrix BlockMatrix::operator*=( double scalar ){

	uint i,j;

 	for( i=0; i < getNumRows(); i++ ){
        for( j=0; j < getNumCols(); j++ ){
            if( types[i][j] != SBMT_ZERO ){
                types   [i][j]  = SBMT_DENSE;
                elements[i][j] *= scalar    ;
            }
        }
    }
	return *this;
}


BlockMatrix BlockMatrix::operator*( const BlockMatrix& arg ) const{

    ASSERT( getNumCols( ) == arg.getNumRows( ) );

    uint i,j,k;

    uint newNumRows = getNumRows( );
    uint newNumCols = arg.getNumCols( );
    BlockMatrix result( newNumRows,newNumCols );

    for( i=0; i<newNumRows; ++i ){
        for( k=0; k<getNumCols( ); ++k ){

            switch( types[i][k] ){

                case SBMT_DENSE:

                    for( j=0; j<newNumCols; ++j ){

                        if( arg.types[k][j] == SBMT_DENSE ){

                            if( result.types[i][j] != SBMT_ZERO )
                                  result.elements[i][j] += elements[i][k] * arg.elements[k][j];
                            else  result.elements[i][j]  = elements[i][k] * arg.elements[k][j];
                        }

                        if( arg.types[k][j] == SBMT_ONE ){

                            if( result.types[i][j] != SBMT_ZERO )
                                  result.elements[i][j] += elements[i][k];
                            else  result.elements[i][j]  = elements[i][k];
                        }

                        if( arg.types[k][j] != SBMT_ZERO )
                            result.types[i][j]  = SBMT_DENSE;
                    }
                    break;


                case SBMT_ONE:

                    for( j=0; j<newNumCols; ++j ){

                         if( arg.types[k][j] == SBMT_DENSE ){

                             if( result.types[i][j] != SBMT_ZERO )
                                   result.elements[i][j] += arg.elements[k][j];
                             else  result.elements[i][j]  = arg.elements[k][j];

                             result.types[i][j]  = SBMT_DENSE;
                         }

                         if( arg.types[k][j] == SBMT_ONE ){

                             if( result.types[i][j] == SBMT_ZERO ){
                                   result.elements[i][j]  = elements[i][k];
                                   result.types   [i][j]  = SBMT_ONE      ;
                             }
                             else{
                                   result.elements[i][j] += elements[i][k];
                                   result.types   [i][j]  = SBMT_DENSE    ;
                             }
                         }
                     }
                     break;

                case SBMT_ZERO:

                     break;

                default:
                     break;
            }
        }
    }

    return result;
}


BlockMatrix BlockMatrix::operator^( const BlockMatrix& arg ) const{

	ASSERT( getNumRows( ) == arg.getNumRows( ) );

	uint i,j,k;

	uint newNumRows = getNumCols( );
	uint newNumCols = arg.getNumCols( );
	BlockMatrix result( newNumRows,newNumCols );

    for( i=0; i<newNumRows; ++i ){
        for( k=0; k<getNumRows( ); ++k ){

            switch( types[k][i] ){

                case SBMT_DENSE:

                    for( j=0; j<newNumCols; ++j ){

                        if( arg.types[k][j] == SBMT_DENSE ){
                            if( result.types[i][j] != SBMT_ZERO )
                                  result.elements[i][j] += elements[k][i]^arg.elements[k][j];
                            else  result.elements[i][j]  = elements[k][i]^arg.elements[k][j];
                        }

                        if( arg.types[k][j] == SBMT_ONE ){
                            if( result.types[i][j] != SBMT_ZERO )
                                  result.elements[i][j] += elements[k][i].transpose();
                            else  result.elements[i][j]  = elements[k][i].transpose();
                        }

                        if( arg.types[k][j] != SBMT_ZERO )
                             result.types[i][j]  = SBMT_DENSE;
                    }
                    break;


                case SBMT_ONE:

                    for( j=0; j<newNumCols; ++j ){

                        if( arg.types[k][j] == SBMT_DENSE ){
                            if( result.types[i][j] != SBMT_ZERO )
                                  result.elements[i][j] += arg.elements[k][j];
                            else  result.elements[i][j]  = arg.elements[k][j];
                            result.types[i][j]  = SBMT_DENSE;
                        }

                        if( arg.types[k][j] == SBMT_ONE ){

                            if( result.types[i][j] == SBMT_ZERO ){
                                  result.elements[i][j]  = elements[k][i];
                                  result.types   [i][j]  = SBMT_ONE      ;
                            }
                            else{
                                  result.elements[i][j] += elements[k][i];
                                  result.types   [i][j]  = SBMT_DENSE    ;
                            }
                        }
                    }
                    break;

                case SBMT_ZERO:
                     break;

                default:
                     break;
            }
        }
    }
	return result;
}


BlockMatrix BlockMatrix::transpose() const{

     BlockMatrix result( getNumCols(), getNumRows() );

     uint i,j;

     for( i = 0; i < getNumRows(); i++ ){
         for( j = 0; j < getNumCols(); j++ ){
             result.elements[j][i] = elements[i][j].transpose();
             result.types   [j][i] = types   [i][j]            ;
         }
     }

     return result;
}


BlockMatrix BlockMatrix::getAbsolute() const{

    uint run1, run2;
    BlockMatrix result( nRows, nCols );

    for( run1 = 0; run1 < nRows; run1++ ){
        for( run2 = 0; run2 < nCols; run2++ ){

            if( types[run1][run2] == SBMT_ONE )
                result.setIdentity( run1, run2, elements[run1][run2].getNumRows() );

            if( types[run1][run2] == SBMT_DENSE )
                result.setDense( run1, run2, elements[run1][run2].absolute() );
        }
    }

    return result;
}


BlockMatrix BlockMatrix::getPositive() const{

    uint run1, run2;
    BlockMatrix result( nRows, nCols );

    for( run1 = 0; run1 < nRows; run1++ ){
        for( run2 = 0; run2 < nCols; run2++ ){

            if( types[run1][run2] == SBMT_ONE )
                result.setIdentity( run1, run2, elements[run1][run2].getNumRows() );

            if( types[run1][run2] == SBMT_DENSE )
                result.setDense( run1, run2, elements[run1][run2].positive() );
        }
    }

    return result;
}


BlockMatrix BlockMatrix::getNegative() const{

    uint run1, run2;
    BlockMatrix result( nRows, nCols );

    for( run1 = 0; run1 < nRows; run1++ ){
        for( run2 = 0; run2 < nCols; run2++ ){

            if( types[run1][run2] == SBMT_DENSE )
                result.setDense( run1, run2, elements[run1][run2].negative() );
        }
    }

    return result;
}


returnValue BlockMatrix::print( ) const{

	uint i,j;

    acadoPrintf("Printing Block Matrix: \n\n");

	for( i=0; i<getNumRows( ); ++i ){

		acadoPrintf( "Row %d \n", i );

		for( j=0; j<getNumCols( ); ++j ){

            if( elements != 0 ){
                if( types[i][j] == SBMT_DENSE ) elements[i][j].print();
                if( types[i][j] == SBMT_ONE   ) acadoPrintf("ONE \n");
                if( types[i][j] == SBMT_ZERO  ) acadoPrintf("ZERO \n");
            }
            else acadoPrintf("ZERO \n");
		    acadoPrintf( "\n" );
		}
		acadoPrintf( "\n\n" );
	}

	return SUCCESSFUL_RETURN;
}


returnValue BlockMatrix::setDense( uint rowIdx, uint colIdx, const Matrix& value ){

	ASSERT( rowIdx < getNumRows( ) );
	ASSERT( colIdx < getNumCols( ) );

    elements[rowIdx][colIdx] = value     ;
    types   [rowIdx][colIdx] = SBMT_DENSE;

    return SUCCESSFUL_RETURN;
}


returnValue BlockMatrix::addDense( uint rowIdx, uint colIdx, const Matrix& value ){

	ASSERT( rowIdx < getNumRows( ) );
	ASSERT( colIdx < getNumCols( ) );

    if( types[rowIdx][colIdx] == SBMT_DENSE || types[rowIdx][colIdx] == SBMT_ONE ){
        types[rowIdx][colIdx] = SBMT_DENSE;
        elements[rowIdx][colIdx] += value;
        return SUCCESSFUL_RETURN;
    }

    return setDense( rowIdx, colIdx, value );
}


returnValue BlockMatrix::getSubBlock( uint rowIdx, uint colIdx,
                                      Matrix &value, uint nR, uint nC )  const{


    ASSERT( rowIdx < getNumRows( ) );
    ASSERT( colIdx < getNumCols( ) );


    if( types[rowIdx][colIdx] != SBMT_ZERO ){

        ASSERT( nR == elements[rowIdx][colIdx].getNumRows( ) );
        ASSERT( nC == elements[rowIdx][colIdx].getNumCols( ) );

        value = elements[rowIdx][colIdx];
    }
    else{
        value.init( nR, nC );
        value.setZero();
    }

    return SUCCESSFUL_RETURN;
}

//
// PROTECTED MEMBER FUNCTIONS:
//





CLOSE_NAMESPACE_ACADO

/*
 *	end of file
 */
