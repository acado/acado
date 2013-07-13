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
 *	\file src/BLASReplacement.cpp
 *	\author Hans Joachim Ferreau, Andreas Potschka, Christian Kirches
 *	\version 3.0beta
 *	\date 2007-2011
 *
 *	BLAS Level 3 replacement routines.
 */

#include <qpOASES/PrivateUtils.hpp>

USING_NAMESPACE_QPOASES

extern "C" void dgemm_ ( const char *TRANSA, const char *TRANSB,
		const unsigned long *M, const unsigned long *N, const unsigned long *K,
		const double *ALPHA, const double *A, const unsigned long *LDA, const double *B, const unsigned long *LDB,
		const double *BETA, double *C, const unsigned long *LDC)
{
	unsigned int i, j, k;

	if (isExactlyZero(*BETA))
		for (k = 0; k < *N; k++)
			for (j = 0; j < *M; j++)
				C[j+(*LDC)*k] = 0.0;
	else if (isExactlyMinusOne(*BETA))
		for (k = 0; k < *N; k++)
			for (j = 0; j < *M; j++)
				C[j+(*LDC)*k] = -C[j+(*LDC)*k];
	else if (!isExactlyOne(*BETA))
		for (k = 0; k < *N; k++)
			for (j = 0; j < *M; j++)
				C[j+(*LDC)*k] *= *BETA;

	if (TRANSA[0] == 'N')
		if (isExactlyOne(*ALPHA))
			for (k = 0; k < *N; k++)
				for (j = 0; j < *M; j++)
					for (i = 0; i < *K; i++)
						C[j+(*LDC)*k] += A[j+(*LDA)*i] * B[i+(*LDB)*k];
		else if (isExactlyMinusOne(*ALPHA))
			for (k = 0; k < *N; k++)
				for (j = 0; j < *M; j++)
					for (i = 0; i < *K; i++)
						C[j+(*LDC)*k] -= A[j+(*LDA)*i] * B[i+(*LDB)*k];
		else
			for (k = 0; k < *N; k++)
				for (j = 0; j < *M; j++)
					for (i = 0; i < *K; i++)
						C[j+(*LDC)*k] += *ALPHA * A[j+(*LDA)*i] * B[i+(*LDB)*k];
	else
		if (isExactlyOne(*ALPHA))
			for (k = 0; k < *N; k++)
				for (j = 0; j < *M; j++)
					for (i = 0; i < *K; i++)
						C[j+(*LDC)*k] += A[i+(*LDA)*j] * B[i+(*LDB)*k];
		else if (isExactlyMinusOne(*ALPHA))
			for (k = 0; k < *N; k++)
				for (j = 0; j < *M; j++)
					for (i = 0; i < *K; i++)
						C[j+(*LDC)*k] -= A[i+(*LDA)*j] * B[i+(*LDB)*k];
		else
			for (k = 0; k < *N; k++)
				for (j = 0; j < *M; j++)
					for (i = 0; i < *K; i++)
						C[j+(*LDC)*k] += *ALPHA * A[i+(*LDA)*j] * B[i+(*LDB)*k];
}

extern "C" void sgemm_ ( const char *TRANSA, const char *TRANSB,
		const unsigned long *M, const unsigned long *N, const unsigned long *K,
		const float *ALPHA, const float *A, const unsigned long *LDA, const float *B, const unsigned long *LDB,
		const float *BETA, float *C, const unsigned long *LDC)
{
	unsigned int i, j, k;

	if (isExactlyZero(*BETA))
		for (k = 0; k < *N; k++)
			for (j = 0; j < *M; j++)
				C[j+(*LDC)*k] = 0.0;
	else if (isExactlyMinusOne(*BETA))
		for (k = 0; k < *N; k++)
			for (j = 0; j < *M; j++)
				C[j+(*LDC)*k] = -C[j+(*LDC)*k];
	else if (!isExactlyOne(*BETA))
		for (k = 0; k < *N; k++)
			for (j = 0; j < *M; j++)
				C[j+(*LDC)*k] *= *BETA;

	if (TRANSA[0] == 'N')
		if (isExactlyOne(*ALPHA))
			for (k = 0; k < *N; k++)
				for (j = 0; j < *M; j++)
					for (i = 0; i < *K; i++)
						C[j+(*LDC)*k] += A[j+(*LDA)*i] * B[i+(*LDB)*k];
		else if (isExactlyMinusOne(*ALPHA))
			for (k = 0; k < *N; k++)
				for (j = 0; j < *M; j++)
					for (i = 0; i < *K; i++)
						C[j+(*LDC)*k] -= A[j+(*LDA)*i] * B[i+(*LDB)*k];
		else
			for (k = 0; k < *N; k++)
				for (j = 0; j < *M; j++)
					for (i = 0; i < *K; i++)
						C[j+(*LDC)*k] += *ALPHA * A[j+(*LDA)*i] * B[i+(*LDB)*k];
	else
		if (isExactlyOne(*ALPHA))
			for (k = 0; k < *N; k++)
				for (j = 0; j < *M; j++)
					for (i = 0; i < *K; i++)
						C[j+(*LDC)*k] += A[i+(*LDA)*j] * B[i+(*LDB)*k];
		else if (isExactlyMinusOne(*ALPHA))
			for (k = 0; k < *N; k++)
				for (j = 0; j < *M; j++)
					for (i = 0; i < *K; i++)
						C[j+(*LDC)*k] -= A[i+(*LDA)*j] * B[i+(*LDB)*k];
		else
			for (k = 0; k < *N; k++)
				for (j = 0; j < *M; j++)
					for (i = 0; i < *K; i++)
						C[j+(*LDC)*k] += *ALPHA * A[i+(*LDA)*j] * B[i+(*LDB)*k];
}

