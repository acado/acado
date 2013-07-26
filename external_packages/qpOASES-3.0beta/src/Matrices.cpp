/*
 *	This file is part of qpOASES.
 *
 *	qpOASES -- An Implementation of the Online Active Set Strategy.
 *	Copyright (C) 2007-2011 by Hans Joachim Ferreau, Andreas Potschka,
 *	Christian Kirches et al. All rights reserved.
 *
 *	qpOASES is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either
 *	version 2.1 of the License, or (at your option) any later version.
 *
 *	qpOASES is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *	See the GNU Lesser General Public License for more details.
 *
 *	You should have received a copy of the GNU Lesser General Public
 *	License along with qpOASES; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/**
 *	\file src/Matrices.cpp
 *	\author Hans Joachim Ferreau, Andreas Potschka, Christian Kirches
 *	\version 3.0beta
 *	\date 2007-2011
 *
 *	Implementation of the matrix classes.
 */


#include <qpOASES.hpp>

#include <qpOASES/PrivateUtils.hpp>

BEGIN_NAMESPACE_QPOASES

/** String for calling LAPACK/BLAS routines with transposed matrices. */
const char * const TRANS = "TRANS";

/** String for calling LAPACK/BLAS routines without transposing the matrix. */
const char * const NOTRANS = "NOTRANS";

/*****************************************************************************
 *  P U B L I C                                                              *
 *****************************************************************************/

DenseMatrix::~DenseMatrix()
{
	if ( needToFreeMemory( ) == BT_TRUE )
		free( );
}

void DenseMatrix::free( )
{
	if (val != 0)
		delete[] val;
	val = 0;
}

Matrix *DenseMatrix::duplicate( ) const
{
	DenseMatrix *dupl = 0;

	if ( needToFreeMemory( ) == BT_TRUE )
	{
		real_t* val_new = new real_t[nRows*nCols];
		memcpy( val_new,val, nRows*nCols*sizeof(real_t) );
		dupl = new DenseMatrix(nRows, nCols, nCols, val_new);
		dupl->doFreeMemory( );
	}
	else
	{
		dupl = new DenseMatrix(nRows, nCols, nCols, val);
	}

	return dupl;
}

real_t DenseMatrix::diag(	int i
							) const
{
	return val[i*(leaDim+1)];
}

BooleanType DenseMatrix::isDiag( ) const
{
	int i, j;

	if (nRows != nCols)
		return BT_FALSE;

	for ( i=0; i<nRows; ++i )
		for ( j=0; j<i; ++j )
			if ( ( fabs( val[i*leaDim+j] ) > EPS ) || ( fabs( val[j*leaDim+i] ) > EPS ) )
				return BT_FALSE;

	return BT_TRUE;
}

returnValue DenseMatrix::getRow(int rNum, const Indexlist* const icols, real_t alpha, real_t *row) const
{
	int i;
    if (icols != 0)
    {
	    if (isExactlyOne(alpha))
		    for (i = 0; i < icols->length; i++)
			    row[i] = val[rNum*leaDim+icols->number[i]];
	    else if (isExactlyMinusOne(alpha))
		    for (i = 0; i < icols->length; i++)
			    row[i] = -val[rNum*leaDim+icols->number[i]];
	    else
		    for (i = 0; i < icols->length; i++)
			    row[i] = alpha*val[rNum*leaDim+icols->number[i]];
    }
    else
    {
	    if (isExactlyOne(alpha))
		    for (i = 0; i < nCols; i++)
			    row[i] = val[rNum*leaDim+i];
	    else if (isExactlyMinusOne(alpha))
		    for (i = 0; i < nCols; i++)
			    row[i] = -val[rNum*leaDim+i];
	    else
		    for (i = 0; i < nCols; i++)
			    row[i] = alpha*val[rNum*leaDim+i];
    }
	return SUCCESSFUL_RETURN;
}

returnValue DenseMatrix::getCol(int cNum, const Indexlist* const irows, real_t alpha, real_t *col) const
{
	int i;

	if (isExactlyOne(alpha))
		for (i = 0; i < irows->length; i++)
			col[i] = val[irows->number[i]*leaDim+cNum];
	else if (isExactlyMinusOne(alpha))
		for (i = 0; i < irows->length; i++)
			col[i] = -val[irows->number[i]*leaDim+cNum];
	else
		for (i = 0; i < irows->length; i++)
			col[i] = alpha*val[irows->number[i]*leaDim+cNum];

	return SUCCESSFUL_RETURN;
}

returnValue DenseMatrix::times(int xN, real_t alpha, const real_t *x, int xLD, real_t beta, real_t *y, int yLD) const
{
	unsigned long _xN=xN, _nRows=nRows, _nCols=nCols, /*_leaDim=getMax(1,leaDim),*/ _xLD=getMax(1,xLD), _yLD=getMax(1,yLD);
	/* Call BLAS. Mind row major format! */
	GEMM(TRANS, NOTRANS, &_nRows, &_xN, &_nCols, &alpha, val, &_nCols, x, &_xLD, &beta, y, &_yLD);
	return SUCCESSFUL_RETURN;
}

returnValue DenseMatrix::transTimes(int xN, real_t alpha, const real_t *x, int xLD, real_t beta, real_t *y, int yLD) const
{
	/* Call BLAS. Mind row major format! */
	unsigned long _xN=xN, _nRows=nRows, _nCols=nCols, /*_leaDim=getMax(1,leaDim),*/ _xLD=getMax(1,xLD), _yLD=getMax(1,yLD);
	GEMM(NOTRANS, NOTRANS, &_nCols, &_xN, &_nRows, &alpha, val, &_nCols, x, &_xLD, &beta, y, &_yLD);
	return SUCCESSFUL_RETURN;
}

returnValue DenseMatrix::times(const Indexlist* const irows, const Indexlist* const icols,
		int xN, real_t alpha, const real_t *x, int xLD, real_t beta, real_t *y, int yLD,
		BooleanType yCompr) const
{
	int i, j, k, row, col, iy, irA;

	if (yCompr == BT_TRUE)
	{
		if (isExactlyZero(beta))
			for (k = 0; k < xN; k++)
				for (j = 0; j < irows->length; j++)
					y[j+k*yLD] = 0.0;
		else if (isExactlyMinusOne(beta))
			for (k = 0; k < xN; k++)
				for (j = 0; j < irows->length; j++)
					y[j+k*yLD] = -y[j+k*yLD];
		else if (!isExactlyOne(beta))
			for (k = 0; k < xN; k++)
				for (j = 0; j < irows->length; j++)
					y[j+k*yLD] *= beta;

		if (icols == 0)
			if (isExactlyOne(alpha))
				for (k = 0; k < xN; k++)
					for (j = 0; j < irows->length; j++)
					{
						row = irows->iSort[j];
						iy = row + k * yLD;
						irA = irows->number[row] * leaDim;
						for (i = 0; i < nCols; i++)
							y[iy] += val[irA+i] * x[k*xLD+i];
					}
			else if (isExactlyMinusOne(alpha))
				for (k = 0; k < xN; k++)
					for (j = 0; j < irows->length; j++)
					{
						row = irows->iSort[j];
						iy = row + k * yLD;
						irA = irows->number[row] * leaDim;
						for (i = 0; i < nCols; i++)
							y[iy] -= val[irA+i] * x[k*xLD+i];
					}
			else
				for (k = 0; k < xN; k++)
					for (j = 0; j < irows->length; j++)
					{
						row = irows->iSort[j];
						iy = row + k * yLD;
						irA = irows->number[row] * leaDim;
						for (i = 0; i < nCols; i++)
							y[iy] += alpha * val[irA+i] * x[k*xLD+i];
					}
		else /* icols != 0 */
			if (isExactlyOne(alpha))
				for (k = 0; k < xN; k++)
					for (j = 0; j < irows->length; j++)
					{
						row = irows->iSort[j];
						iy = row + k * yLD;
						irA = irows->number[row] * leaDim;
						for (i = 0; i < icols->length; i++)
						{
							col = icols->iSort[i];
							y[iy] += val[irA+icols->number[col]] * x[k*xLD+col];
						}
					}
			else if (isExactlyMinusOne(alpha))
				for (k = 0; k < xN; k++)
					for (j = 0; j < irows->length; j++)
					{
						row = irows->iSort[j];
						iy = row + k * yLD;
						irA = irows->number[row] * leaDim;
						for (i = 0; i < icols->length; i++)
						{
							col = icols->iSort[i];
							y[iy] -= val[irA+icols->number[col]] * x[k*xLD+col];
						}
					}
			else
				for (k = 0; k < xN; k++)
					for (j = 0; j < irows->length; j++)
					{
						row = irows->iSort[j];
						iy = row + k * yLD;
						irA = irows->number[row] * leaDim;
						for (i = 0; i < icols->length; i++)
						{
							col = icols->iSort[i];
							y[iy] += alpha * val[irA+icols->number[col]] * x[k*xLD+col];
						}
					}
	}
	else /* y not compressed */
	{
		if (isExactlyZero(beta))
			for (k = 0; k < xN; k++)
				for (j = 0; j < irows->length; j++)
					y[irows->number[j]+k*yLD] = 0.0;
		else if (isExactlyMinusOne(beta))
			for (k = 0; k < xN; k++)
				for (j = 0; j < irows->length; j++)
					y[irows->number[j]+k*yLD] = -y[j+k*yLD];
		else if (!isExactlyOne(beta))
			for (k = 0; k < xN; k++)
				for (j = 0; j < irows->length; j++)
					y[irows->number[j]+k*yLD] *= beta;

		if (icols == 0)
			if (isExactlyOne(alpha))
				for (k = 0; k < xN; k++)
					for (j = 0; j < irows->length; j++)
					{
						row = irows->number[irows->iSort[j]];
						iy = row + k * yLD;
						irA = row * leaDim;
						for (i = 0; i < nCols; i++)
							y[iy] += val[irA+i] * x[k*xLD+i];
					}
			else if (isExactlyMinusOne(alpha))
				for (k = 0; k < xN; k++)
					for (j = 0; j < irows->length; j++)
					{
						row = irows->number[irows->iSort[j]];
						iy = row + k * yLD;
						irA = row * leaDim;
						for (i = 0; i < nCols; i++)
							y[iy] -= val[irA+i] * x[k*xLD+i];
					}
			else
				for (k = 0; k < xN; k++)
					for (j = 0; j < irows->length; j++)
					{
						row = irows->number[irows->iSort[j]];
						iy = row + k * yLD;
						irA = row * leaDim;
						for (i = 0; i < nCols; i++)
							y[iy] += alpha * val[irA+i] * x[k*xLD+i];
					}
		else /* icols != 0 */
			if (isExactlyOne(alpha))
				for (k = 0; k < xN; k++)
					for (j = 0; j < irows->length; j++)
					{
						row = irows->number[irows->iSort[j]];
						iy = row + k * yLD;
						irA = row * leaDim;
						for (i = 0; i < icols->length; i++)
						{
							col = icols->iSort[i];
							y[iy] += val[irA+icols->number[col]] * x[k*xLD+col];
						}
					}
			else if (isExactlyMinusOne(alpha))
				for (k = 0; k < xN; k++)
					for (j = 0; j < irows->length; j++)
					{
						row = irows->number[irows->iSort[j]];
						iy = row + k * yLD;
						irA = row * leaDim;
						for (i = 0; i < icols->length; i++)
						{
							col = icols->iSort[i];
							y[iy] -= val[irA+icols->number[col]] * x[k*xLD+col];
						}
					}
			else
				for (k = 0; k < xN; k++)
					for (j = 0; j < irows->length; j++)
					{
						row = irows->number[irows->iSort[j]];
						iy = row + k * yLD;
						irA = row * leaDim;
						for (i = 0; i < icols->length; i++)
						{
							col = icols->iSort[i];
							y[iy] += alpha * val[irA+icols->number[col]] * x[k*xLD+col];
						}
					}
	}

	return SUCCESSFUL_RETURN;
}

returnValue DenseMatrix::transTimes(const Indexlist* const irows, const Indexlist* const icols,
		int xN, real_t alpha, const real_t *x, int xLD, real_t beta, real_t *y, int yLD) const
{
	int i, j, k, row, col;

	if (isExactlyZero(beta))
		for (k = 0; k < xN; k++)
			for (j = 0; j < icols->length; j++)
				y[j+k*yLD] = 0.0;
	else if (isExactlyMinusOne(beta))
		for (k = 0; k < xN; k++)
			for (j = 0; j < icols->length; j++)
				y[j+k*yLD] = -y[j+k*yLD];
	else if (!isExactlyOne(beta))
		for (k = 0; k < xN; k++)
			for (j = 0; j < icols->length; j++)
				y[j+k*yLD] *= beta;

	if (isExactlyOne(alpha))
		for (k = 0; k < xN; k++)
			for (j = 0; j < irows->length; j++)
			{
				row = irows->iSort[j];
				for (i = 0; i < icols->length; i++)
				{
					col = icols->iSort[i];
					y[col+k*yLD] += val[irows->number[row]*leaDim+icols->number[col]] * x[row+k*xLD];
				}
			}
	else if (isExactlyMinusOne(alpha))
		for (k = 0; k < xN; k++)
			for (j = 0; j < irows->length; j++)
			{
				row = irows->iSort[j];
				for (i = 0; i < icols->length; i++)
				{
					col = icols->iSort[i];
					y[col+k*yLD] -= val[irows->number[row]*leaDim+icols->number[col]] * x[row+k*xLD];
				}
			}
	else
		for (k = 0; k < xN; k++)
			for (j = 0; j < irows->length; j++)
			{
				row = irows->iSort[j];
				for (i = 0; i < icols->length; i++)
				{
					col = icols->iSort[i];
					y[col+k*yLD] += alpha * val[irows->number[row]*leaDim+icols->number[col]] * x[row+k*xLD];
				}
			}

	return SUCCESSFUL_RETURN;
}

returnValue DenseMatrix::addToDiag(real_t alpha)
{
	int i;
	for (i = 0; i < nRows && i < nCols; i++)
		val[i*(leaDim+1)] += alpha;

	return SUCCESSFUL_RETURN;
}


returnValue DenseMatrix::print( ) const
{
	myPrintf( "aaa\n" );
	return qpOASES::print( val,nRows,nCols );
}



Matrix *SymDenseMat::duplicate( ) const
{
	/* "same" as duplicate() in DenseMatrix */
	SymDenseMat *dupl = 0;

	if ( needToFreeMemory( ) == BT_TRUE )
	{
		real_t* val_new = new real_t[nRows*nCols];
		memcpy( val_new,val, nRows*nCols*sizeof(real_t) );
		dupl = new SymDenseMat(nRows, nCols, nCols, val_new);
		dupl->doFreeMemory( );
	}
	else
	{
		dupl = new SymDenseMat(nRows, nCols, nCols, val);
	}

	return dupl;
}


returnValue SymDenseMat::bilinear(const Indexlist* const icols,
		int xN, const real_t *x, int xLD, real_t *y, int yLD) const
{
	int ii, jj, kk, col;
	int i,j,k,irA;

	for (ii = 0; ii < xN; ii++)
		for (jj = 0; jj < xN; jj++)
			y[ii*yLD+jj] = 0.0;
			
	real_t *Ax = new real_t[icols->length * xN];			

	for (i=0;i<icols->length * xN;++i)
		Ax[i]=0.0;

	/* exploit symmetry of A ! */
	for (j = 0; j < icols->length; j++) {
		irA = icols->number[j] * leaDim;	
		for (i = 0; i < icols->length; i++)
		{
			real_t h = val[irA+icols->number[i]];
			for (k = 0; k < xN; k++)
				Ax[j + k * icols->length] += h * x[k*xLD+icols->number[i]];
		}
	}
							
	for (ii = 0; ii < icols->length; ++ii) {
		col = icols->number[ii];
		for (jj = 0; jj < xN; ++jj) {
			for (kk = 0; kk < xN; ++kk) {
				y[kk + jj*yLD] += x[col + jj*xLD] * Ax[ii + kk*icols->length];
			}
		}
	}
	delete[] Ax;

	return SUCCESSFUL_RETURN;
}


SparseMatrix::SparseMatrix() : nRows(0), nCols(0), ir(0), jc(0), jd(0), val(0) {}

SparseMatrix::SparseMatrix(long nr, long nc, long *r, long *c, real_t *v, long *d)
	: nRows(nr), nCols(nc), ir(r), jc(c), jd(d), val(v) { doNotFreeMemory(); }

SparseMatrix::SparseMatrix(int nr, int nc, int ld, const real_t * const v) : nRows(nr), nCols(nc), jd(0)
{
	int i, j, nnz;

	jc = new long[nc+1];
	ir = new long[nr*nc];
	val = new real_t[nr*nc];

	nnz = 0;
	for (j = 0; j < nCols; j++)
	{
		jc[j] = nnz;
		for (i = 0; i < nRows; i++)
			if (!isExactlyZero(v[i*ld+j]))
			{
				ir[nnz] = i;
				val[nnz++] = v[i*ld+j];
			}
	}
	jc[nCols] = nnz;

	doFreeMemory( );
}

SparseMatrix::~SparseMatrix()
{
	if ( needToFreeMemory() == BT_TRUE ) 
		free( );
}

void SparseMatrix::free( )
{
	if (ir != 0) delete[] ir;
	ir = 0;
	if (jc != 0) delete[] jc;
	jc = 0;
	if (jd != 0) delete[] jd;
	jd = 0;
	if (val != 0) delete[] val;
	val = 0;
	doNotFreeMemory( );
}

Matrix *SparseMatrix::duplicate() const
{
	long i, length = jc[nCols];
	SparseMatrix *dupl = new SparseMatrix;

	dupl->nRows = nRows;
	dupl->nCols = nCols;
	dupl->ir = new long[length];
	dupl->jc = new long[nCols+1];
	dupl->jd = new long[nCols];
	dupl->val = new real_t[length];

	for (i = 0; i < length; i++) dupl->ir[i] = ir[i];
	for (i = 0; i <= nCols; i++) dupl->jc[i] = jc[i];
	for (i = 0; i < nCols; i++) dupl->jd[i] = jd[i];
	for (i = 0; i < length; i++) dupl->val[i] = val[i];
	
	dupl->doFreeMemory( );

	return dupl;
}

real_t SparseMatrix::diag(int i) const
{
	int entry = jd[i];
	return (entry < jc[i+1] && ir[entry] == i) ? val[entry] : 0.0;
}

BooleanType SparseMatrix::isDiag() const
{
	return BT_FALSE;
}

returnValue SparseMatrix::getRow(int rNum, const Indexlist* const icols, real_t alpha, real_t *row) const
{
	long i, j, k;

    if (icols != 0)
    {
	    if (isExactlyOne(alpha))
		    for (k = 0; k < icols->length; k++)
		    {
			    j = icols->number[icols->iSort[k]];
			    for (i = jc[j]; i < jc[j+1] && ir[i] < rNum; i++);
			    row[icols->iSort[k]] = (i < jc[j+1] && ir[i] == rNum) ? val[i] : 0.0;
		    }
	    else if (isExactlyMinusOne(alpha))
		    for (k = 0; k < icols->length; k++)
		    {
			    j = icols->number[icols->iSort[k]];
			    for (i = jc[j]; i < jc[j+1] && ir[i] < rNum; i++);
			    row[icols->iSort[k]] = (i < jc[j+1] && ir[i] == rNum) ? -val[i] : 0.0;
		    }
	    else
		    for (k = 0; k < icols->length; k++)
		    {
			    j = icols->number[icols->iSort[k]];
			    for (i = jc[j]; i < jc[j+1] && ir[i] < rNum; i++);
			    row[icols->iSort[k]] = (i < jc[j+1] && ir[i] == rNum) ? alpha*val[i] : 0.0;
		    }
    }
    else
    {
	    if (isExactlyOne(alpha))
		    for (j = 0; j < nCols; j++)
		    {
			    for (i = jc[j]; i < jc[j+1] && ir[i] < rNum; i++);
			    row[j] = (i < jc[j+1] && ir[i] == rNum) ? val[i] : 0.0;
		    }
	    else if (isExactlyMinusOne(alpha))
		    for (j = 0; j < icols->length; j++)
		    {
			    for (i = jc[j]; i < jc[j+1] && ir[i] < rNum; i++);
			    row[j] = (i < jc[j+1] && ir[i] == rNum) ? -val[i] : 0.0;
		    }
	    else
		    for (j = 0; j < icols->length; j++)
		    {
			    for (i = jc[j]; i < jc[j+1] && ir[i] < rNum; i++);
			    row[j] = (i < jc[j+1] && ir[i] == rNum) ? alpha*val[i] : 0.0;
		    }
    }
	return SUCCESSFUL_RETURN;
}

returnValue SparseMatrix::getCol(int cNum, const Indexlist* const irows, real_t alpha, real_t *col) const
{
	long i, j;

	i = jc[cNum];
	j = 0;
	if (isExactlyOne(alpha))
		while (i < jc[cNum+1] && j < irows->length)
			if (ir[i] == irows->number[irows->iSort[j]])
				col[irows->iSort[j++]] = val[i++];
			else if (ir[i] > irows->number[irows->iSort[j]])
				col[irows->iSort[j++]] = 0.0;
			else
				i++;
	else if (isExactlyMinusOne(alpha))
		while (i < jc[cNum+1] && j < irows->length)
			if (ir[i] == irows->number[irows->iSort[j]])
				col[irows->iSort[j++]] = -val[i++];
			else if (ir[i] > irows->number[irows->iSort[j]])
				col[irows->iSort[j++]] = 0.0;
			else
				i++;
	else
		while (i < jc[cNum+1] && j < irows->length)
			if (ir[i] == irows->number[irows->iSort[j]])
				col[irows->iSort[j++]] = alpha * val[i++];
			else if (ir[i] > irows->number[irows->iSort[j]])
				col[irows->iSort[j++]] = 0.0;
			else
				i++;

	/* fill in remaining zeros */
	while (j < irows->length)
		col[irows->iSort[j++]] = 0.0;

	return SUCCESSFUL_RETURN;
}

returnValue SparseMatrix::times(int xN, real_t alpha, const real_t *x, int xLD,
		real_t beta, real_t *y, int yLD) const
{
	long i, j, k;

	if (isExactlyZero(beta))
		for (k = 0; k < xN; k++)
			for (j = 0; j < nRows; j++)
				y[j+k*yLD] = 0.0;
	else if (isExactlyMinusOne(beta))
		for (k = 0; k < xN; k++)
			for (j = 0; j < nRows; j++)
				y[j+k*yLD] = -y[j+k*yLD];
	else if (!isExactlyOne(beta))
		for (k = 0; k < xN; k++)
			for (j = 0; j < nRows; j++)
				y[j+k*yLD] *= beta;

	if (isExactlyOne(alpha))
		for (k = 0; k < xN; k++)
			for (j = 0; j < nCols; j++)
				for (i = jc[j]; i < jc[j+1]; i++)
					y[ir[i]+k*yLD] += val[i] * x[j+k*xLD];
	else if (isExactlyMinusOne(alpha))
		for (k = 0; k < xN; k++)
			for (j = 0; j < nCols; j++)
				for (i = jc[j]; i < jc[j+1]; i++)
					y[ir[i]+k*yLD] -= val[i] * x[j+k*xLD];
	else
		for (k = 0; k < xN; k++)
			for (j = 0; j < nCols; j++)
				for (i = jc[j]; i < jc[j+1]; i++)
					y[ir[i]+k*yLD] += alpha * val[i] * x[j+k*xLD];

	return SUCCESSFUL_RETURN;
}

returnValue SparseMatrix::transTimes(int xN, real_t alpha, const real_t *x, int xLD,
		real_t beta, real_t *y, int yLD) const
{
	long i, j, k;

	if (isExactlyZero(beta))
		for (k = 0; k < xN; k++)
			for (j = 0; j < nCols; j++)
				y[j+k*yLD] = 0.0;
	else if (isExactlyMinusOne(beta))
		for (k = 0; k < xN; k++)
			for (j = 0; j < nCols; j++)
				y[j+k*yLD] = -y[j+k*yLD];
	else if (!isExactlyOne(beta))
		for (k = 0; k < xN; k++)
			for (j = 0; j < nCols; j++)
				y[j+k*yLD] *= beta;

	if (isExactlyOne(alpha))
		for (k = 0; k < xN; k++)
			for (j = 0; j < nCols; j++)
				for (i = jc[j]; i < jc[j+1]; i++)
					y[j+k*yLD] += val[i] * x[ir[i]+k*xLD];
	else if (isExactlyMinusOne(alpha))
		for (k = 0; k < xN; k++)
			for (j = 0; j < nCols; j++)
				for (i = jc[j]; i < jc[j+1]; i++)
					y[j+k*yLD] -= val[i] * x[ir[i]+k*xLD];
	else
		for (k = 0; k < xN; k++)
			for (j = 0; j < nCols; j++)
				for (i = jc[j]; i < jc[j+1]; i++)
					y[j+k*yLD] += alpha * val[i] * x[ir[i]+k*xLD];

	return SUCCESSFUL_RETURN;
}

returnValue SparseMatrix::times(const Indexlist* const irows, const Indexlist* const icols,
		int xN, real_t alpha, const real_t *x, int xLD, real_t beta, real_t *y, int yLD,
		BooleanType yCompr) const
{
	long i, j, k, l, col;

	if (yCompr == BT_TRUE)
	{
		if (isExactlyZero(beta))
			for (k = 0; k < xN; k++)
				for (j = 0; j < irows->length; j++)
					y[j+k*yLD] = 0.0;
		else if (isExactlyMinusOne(beta))
			for (k = 0; k < xN; k++)
				for (j = 0; j < irows->length; j++)
					y[j+k*yLD] = -y[j+k*yLD];
		else if (!isExactlyOne(beta))
			for (k = 0; k < xN; k++)
				for (j = 0; j < irows->length; j++)
					y[j+k*yLD] *= beta;

		if (icols == 0)
			if (isExactlyOne(alpha))
				for (l = 0; l < nCols; l++)
				{
					i = jc[l];
					j = 0;
					while (i < jc[l+1] && j < irows->length)
						if (ir[i] == irows->number[irows->iSort[j]])
						{
							for (k = 0; k < xN; k++)
								y[k*yLD+irows->iSort[j]] += val[i] * x[k*xLD+l];
							i++, j++;
						}
						else if (ir[i] > irows->number[irows->iSort[j]]) j++;
						else i++;
				}
			else if (isExactlyMinusOne(alpha))
				for (l = 0; l < nCols; l++)
				{
					i = jc[l];
					j = 0;
					while (i < jc[l+1] && j < irows->length)
						if (ir[i] == irows->number[irows->iSort[j]])
						{
							for (k = 0; k < xN; k++)
								y[k*yLD+irows->iSort[j]] -= val[i] * x[k*xLD+l];
							i++, j++;
						}
						else if (ir[i] > irows->number[irows->iSort[j]]) j++;
						else i++;
				}
			else
				for (l = 0; l < nCols; l++)
				{
					i = jc[l];
					j = 0;
					while (i < jc[l+1] && j < irows->length)
						if (ir[i] == irows->number[irows->iSort[j]])
						{
							for (k = 0; k < xN; k++)
								y[k*yLD+irows->iSort[j]] += alpha * val[i] * x[k*xLD+l];
							i++, j++;
						}
						else if (ir[i] > irows->number[irows->iSort[j]]) j++;
						else i++;
				}
		else /* icols != 0 */
			if (isExactlyOne(alpha))
				for (l = 0; l < icols->length; l++)
				{
					col = icols->iSort[l];
					i = jc[icols->number[col]];
					j = 0;
					while (i < jc[icols->number[col]+1] && j < irows->length)
						if (ir[i] == irows->number[irows->iSort[j]])
						{
							for (k = 0; k < xN; k++)
								y[k*yLD+irows->iSort[j]] += val[i] * x[k*xLD+col];
							i++, j++;
						}
						else if (ir[i] > irows->number[irows->iSort[j]]) j++;
						else i++;
				}
			else if (isExactlyMinusOne(alpha))
				for (l = 0; l < icols->length; l++)
				{
					col = icols->iSort[l];
					i = jc[icols->number[col]];
					j = 0;
					while (i < jc[icols->number[col]+1] && j < irows->length)
						if (ir[i] == irows->number[irows->iSort[j]])
						{
							for (k = 0; k < xN; k++)
								y[k*yLD+irows->iSort[j]] -= val[i] * x[k*xLD+col];
							i++, j++;
						}
						else if (ir[i] > irows->number[irows->iSort[j]]) j++;
						else i++;
				}
			else
				for (l = 0; l < icols->length; l++)
				{
					col = icols->iSort[l];
					i = jc[icols->number[col]];
					j = 0;
					while (i < jc[icols->number[col]+1] && j < irows->length)
						if (ir[i] == irows->number[irows->iSort[j]])
						{
							for (k = 0; k < xN; k++)
								y[k*yLD+irows->iSort[j]] += alpha * val[i] * x[k*xLD+col];
							i++, j++;
						}
						else if (ir[i] > irows->number[irows->iSort[j]]) j++;
						else i++;
				}
	}
	else /* y not compressed */
	{
		if (isExactlyZero(beta))
			for (k = 0; k < xN; k++)
				for (j = 0; j < irows->length; j++)
					y[irows->number[j]+k*yLD] = 0.0;
		else if (isExactlyMinusOne(beta))
			for (k = 0; k < xN; k++)
				for (j = 0; j < irows->length; j++)
					y[irows->number[j]+k*yLD] = -y[j+k*yLD];
		else if (!isExactlyOne(beta))
			for (k = 0; k < xN; k++)
				for (j = 0; j < irows->length; j++)
					y[irows->number[j]+k*yLD] *= beta;

		if (icols == 0)
			if (isExactlyOne(alpha))
				for (l = 0; l < nCols; l++)
				{
					i = jc[l];
					j = 0;
					while (i < jc[l+1] && j < irows->length)
						if (ir[i] == irows->number[irows->iSort[j]])
						{
							for (k = 0; k < xN; k++)
								y[k*yLD+irows->number[irows->iSort[j]]] += val[i] * x[k*xLD+l];
							i++, j++;
						}
						else if (ir[i] > irows->number[irows->iSort[j]]) j++;
						else i++;
				}
			else if (isExactlyMinusOne(alpha))
				for (l = 0; l < nCols; l++)
				{
					i = jc[l];
					j = 0;
					while (i < jc[l+1] && j < irows->length)
						if (ir[i] == irows->number[irows->iSort[j]])
						{
							for (k = 0; k < xN; k++)
								y[k*yLD+irows->number[irows->iSort[j]]] -= val[i] * x[k*xLD+l];
							i++, j++;
						}
						else if (ir[i] > irows->number[irows->iSort[j]]) j++;
						else i++;
				}
			else
				for (l = 0; l < nCols; l++)
				{
					i = jc[l];
					j = 0;
					while (i < jc[l+1] && j < irows->length)
						if (ir[i] == irows->number[irows->iSort[j]])
						{
							for (k = 0; k < xN; k++)
								y[k*yLD+irows->number[irows->iSort[j]]] += alpha * val[i] * x[k*xLD+l];
							i++, j++;
						}
						else if (ir[i] > irows->number[irows->iSort[j]]) j++;
						else i++;
				}
		else /* icols != 0 */
			if (isExactlyOne(alpha))
				for (l = 0; l < icols->length; l++)
				{
					col = icols->iSort[l];
					i = jc[icols->number[col]];
					j = 0;
					while (i < jc[icols->number[col]+1] && j < irows->length)
						if (ir[i] == irows->number[irows->iSort[j]])
						{
							for (k = 0; k < xN; k++)
								y[k*yLD+irows->number[irows->iSort[j]]] += val[i] * x[k*xLD+col];
							i++, j++;
						}
						else if (ir[i] > irows->number[irows->iSort[j]]) j++;
						else i++;
				}
			else if (isExactlyMinusOne(alpha))
				for (l = 0; l < icols->length; l++)
				{
					col = icols->iSort[l];
					i = jc[icols->number[col]];
					j = 0;
					while (i < jc[icols->number[col]+1] && j < irows->length)
						if (ir[i] == irows->number[irows->iSort[j]])
						{
							for (k = 0; k < xN; k++)
								y[k*yLD+irows->number[irows->iSort[j]]] -= val[i] * x[k*xLD+col];
							i++, j++;
						}
						else if (ir[i] > irows->number[irows->iSort[j]]) j++;
						else i++;
				}
			else
				for (l = 0; l < icols->length; l++)
				{
					col = icols->iSort[l];
					i = jc[icols->number[col]];
					j = 0;
					while (i < jc[icols->number[col]+1] && j < irows->length)
						if (ir[i] == irows->number[irows->iSort[j]])
						{
							for (k = 0; k < xN; k++)
								y[k*yLD+irows->number[irows->iSort[j]]] += alpha * val[i] * x[k*xLD+col];
							i++, j++;
						}
						else if (ir[i] > irows->number[irows->iSort[j]]) j++;
						else i++;
				}
	}
	return SUCCESSFUL_RETURN;
}

returnValue SparseMatrix::transTimes(const Indexlist* const irows, const Indexlist* const icols,
		int xN, real_t alpha, const real_t *x, int xLD, real_t beta, real_t *y, int yLD) const
{
	long i, j, k, l, col;

	if (isExactlyZero(beta))
		for (k = 0; k < xN; k++)
			for (j = 0; j < icols->length; j++)
				y[j+k*yLD] = 0.0;
	else if (isExactlyMinusOne(beta))
		for (k = 0; k < xN; k++)
			for (j = 0; j < icols->length; j++)
				y[j+k*yLD] = -y[j+k*yLD];
	else if (!isExactlyOne(beta))
		for (k = 0; k < xN; k++)
			for (j = 0; j < icols->length; j++)
				y[j+k*yLD] *= beta;

	if (isExactlyOne(alpha))
		for (l = 0; l < icols->length; l++)
		{
			col = icols->iSort[l];
			i = jc[icols->number[col]];
			j = 0;
			while (i < jc[icols->number[col]+1] && j < irows->length)
				if (ir[i] == irows->number[irows->iSort[j]])
				{
					for (k = 0; k < xN; k++)
						y[k*yLD+col] += val[i] * x[k*xLD+irows->iSort[j]];
					i++, j++;
				}
				else if (ir[i] > irows->number[irows->iSort[j]]) j++;
				else i++;
		}
	else if (isExactlyMinusOne(alpha))
		for (l = 0; l < icols->length; l++)
		{
			col = icols->iSort[l];
			i = jc[icols->number[col]];
			j = 0;
			while (i < jc[icols->number[col]+1] && j < irows->length)
				if (ir[i] == irows->number[irows->iSort[j]])
				{
					for (k = 0; k < xN; k++)
						y[k*yLD+col] -= val[i] * x[k*xLD+irows->iSort[j]];
					i++, j++;
				}
				else if (ir[i] > irows->number[irows->iSort[j]]) j++;
				else i++;
		}
	else
		for (l = 0; l < icols->length; l++)
		{
			col = icols->iSort[l];
			i = jc[icols->number[col]];
			j = 0;
			while (i < jc[icols->number[col]+1] && j < irows->length)
				if (ir[i] == irows->number[irows->iSort[j]])
				{
					for (k = 0; k < xN; k++)
						y[k*yLD+col] += alpha * val[i] * x[k*xLD+irows->iSort[j]];
					i++, j++;
				}
				else if (ir[i] > irows->number[irows->iSort[j]]) j++;
				else i++;
		}

	return SUCCESSFUL_RETURN;
}

returnValue SparseMatrix::addToDiag(real_t alpha)
{
	long i;

	if (!isExactlyZero(alpha))
	{
		for (i = 0; i < nRows && i < nCols; i++)
			if (ir[jd[i]] == i) val[jd[i]] += alpha;
			else return RET_NO_DIAGONAL_AVAILABLE;
	}

	return SUCCESSFUL_RETURN;
}

long *SparseMatrix::createDiagInfo()
{
	long i, j;

	if (jd == 0) {
		jd = new long[nCols];

		for (j = 0; j < nCols; j++)
		{
			for (i = jc[j]; i < jc[j+1] && ir[i] < j; i++);
			jd[j] = i;
		}
	}

	return jd;
}

real_t *SparseMatrix::full() const
{
	long i, j;
	real_t *v = new real_t[nRows*nCols];

	for (i = 0; i < nCols*nRows; i++)
		v[i] = 0.0;

	for (j = 0; j < nCols; j++)
		for (i = jc[j]; i < jc[j+1]; i++)
			v[ir[i] * nCols + j] = val[i];

	return v;
}


returnValue SparseMatrix::print( ) const
{
	return THROWERROR( RET_NOT_YET_IMPLEMENTED );
}



Matrix *SymSparseMat::duplicate() const
{
	/* "same" as duplicate() in SparseMatrix */
	long i, length = jc[nCols];
	SymSparseMat *dupl = new SymSparseMat;

	dupl->nRows = nRows;
	dupl->nCols = nCols;
	dupl->ir = new long[length];
	dupl->jc = new long[nCols+1];
	dupl->jd = new long[nCols];
	dupl->val = new real_t[length];

	for (i = 0; i < length; i++) dupl->ir[i] = ir[i];
	for (i = 0; i <= nCols; i++) dupl->jc[i] = jc[i];
	for (i = 0; i < nCols; i++) dupl->jd[i] = jd[i];
	for (i = 0; i < length; i++) dupl->val[i] = val[i];
	
	dupl->doFreeMemory( );

	return dupl;
}

returnValue SymSparseMat::bilinear(const Indexlist* const icols,
		int xN, const real_t *x, int xLD, real_t *y, int yLD) const
{
	int i, j, k, l, idx, row, col;

	/* clear output */
	for (i = 0; i < xN*xN; i++)
		y[i] = 0.0;

	/* compute lower triangle */
	for (l = 0; l < icols->length; l++)
	{
		col = icols->number[icols->iSort[l]];
		idx = jd[col];
		k = 0;
		while (idx < jc[col+1] && k < icols->length)
		{
			row = icols->number[icols->iSort[k]];
			if (ir[idx] == row)
			{
				/* TODO: It is possible to formulate this as DSYR and DSYR2
				 * operations. */
				if (row == col) /* diagonal element */
					for (i = 0; i < xN; i++)
						for (j = i; j < xN; j++)
							y[i*yLD+j] += val[idx] * x[i*xLD+col] * x[j*xLD+col];
				else /* subdiagonal elements */
					for (i = 0; i < xN; i++)
						for (j = i; j < xN; j++)
							y[i*yLD+j] += val[idx] * (x[i*xLD+col] * x[j*xLD+row] + x[i*xLD+row] * x[j*xLD+col]);
				idx++, k++;
			}
			else if (ir[idx] > row) k++;
			else idx++;
		}
	}

	/* fill upper triangle */
	for (i = 0; i < xN; i++)
		for (j = i; j < xN; j++)
			y[j*yLD+i] = y[i*yLD+j];

	return SUCCESSFUL_RETURN;
}


END_NAMESPACE_QPOASES


/*
 *	end of file
 */
