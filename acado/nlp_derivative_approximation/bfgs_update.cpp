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
 *    \file src/nlp_derivative_approximation/bfgs_update.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */


#include <acado/nlp_derivative_approximation/bfgs_update.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

BFGSupdate::BFGSupdate( ) : ConstantHessian( )
{
	modification = MOD_POWELLS_MODIFICATION;
	nBlocks = 0;
}


BFGSupdate::BFGSupdate(	UserInteraction* _userInteraction,
						uint _nBlocks
						) : ConstantHessian( _userInteraction )
{
	modification = MOD_POWELLS_MODIFICATION;
	nBlocks = _nBlocks;
}


BFGSupdate::BFGSupdate( const BFGSupdate& rhs ) : ConstantHessian( rhs )
{
    modification = rhs.modification;
	nBlocks = rhs.nBlocks;
}


BFGSupdate::~BFGSupdate( )
{
}


BFGSupdate& BFGSupdate::operator=( const BFGSupdate& rhs )
{
	if ( this != &rhs )
	{
		ConstantHessian::operator=( rhs );

		modification = rhs.modification;
		nBlocks = rhs.nBlocks;
	}

	return *this;
}


NLPderivativeApproximation* BFGSupdate::clone( ) const
{
	return new BFGSupdate( *this );
}



returnValue BFGSupdate::initHessian(	BlockMatrix& B,
										uint N,
										const OCPiterate& iter
										)
{
	return ConstantHessian::initHessian( B,N,iter );
}


returnValue BFGSupdate::initScaling(	BlockMatrix& B,
										const BlockMatrix& x,
										const BlockMatrix& y
										)
{
    return ConstantHessian::initScaling( B,x,y );
}



returnValue BFGSupdate::apply(	BlockMatrix &B,
								const BlockMatrix &x,
								const BlockMatrix &y
								)
{
	if ( performsBlockUpdates( ) == BT_TRUE )
    	return applyBlockDiagonalUpdate( B,x,y );
	else
    	return applyUpdate( B,x,y );
}



//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue BFGSupdate::applyUpdate(	BlockMatrix &B,
										const BlockMatrix &x,
										const BlockMatrix &y
										)
{
    // CONSTANTS FOR POWELL'S STRATEGY:
    // --------------------------------
    const double epsilon = 0.2;   // constant epsilon for a positive curvature
                                  // check of the form x^T y > epsilon x^T B x
    double       theta   = 1.0;   // constant theta for Powell's strategy.


    // OTHER CONSTANTS:
    // ----------------
    const double regularisation = 100.0*EPS; // safe-guard constant for devisions
                                             // (to avoid numerical devision by 0).


    // DEFINITTIONS:
    // --------------             -------------------------------------------------------
    BlockMatrix Bx  ;             // the vector  B*x.
    BlockMatrix BxxB;             // the matrix  B*x*x^T*B
    DMatrix      xBx ;             // the scalar  x^T B x
    DMatrix      xy  ;             // the scalar  x^T y
    BlockMatrix z   ;             // the modified "gradient" y (needed for Powell's strategy)
    double      xz = 0.0;         // the scalar  x^T z
    BlockMatrix zz  ;             // the matrix  z*z^T


    // COMPUTATION OF Bx, xBx, xy AND BxxB:
    // -------------------------------------
    Bx = B*x;
    (x^Bx).getSubBlock( 0, 0, xBx, 1, 1 );
    (y^x ).getSubBlock( 0, 0, xy , 1, 1 );
    BxxB = Bx*Bx.transpose();


    // CURVATURE CHECK:
    // -------------------------------------
    if( xy(0,0) > epsilon*xBx(0,0) ){

        z  = y      ; // do not modify z if  x^T y > epsilon x^T B x
        xz = xy(0,0); // in this case we have x^T z = x^T y.
    }
    else{

        switch( modification ){

            case MOD_NO_MODIFICATION : // In this case no modification is applied
                                       // and B might become indefinite after the update
                 z  = y      ;
                 xz = xy(0,0);
                 break;


            case MOD_NOCEDALS_MODIFICATION:  // just skip the update

                 return SUCCESSFUL_RETURN;


            case MOD_POWELLS_MODIFICATION:  // apply Powell's modification of y

                 theta =      (1.0-epsilon)*xBx(0,0)/
                         (xBx(0,0)-xy(0,0)+regularisation);

                 z   = y               ;
                 z  *= theta           ;
                 Bx *= (1.0-theta)     ;
                 z  += Bx              ;
                 xz  = epsilon*xBx(0,0);
                 break;
        }
    }

    zz = z*z.transpose();

    BxxB *= 1.0/(xBx(0,0) + regularisation);
    zz   *= 1.0/(xz       + regularisation);


    // PERFORM THE UPDATE:
    // -------------------

    B += zz - BxxB;

    return SUCCESSFUL_RETURN;
}


returnValue BFGSupdate::applyBlockDiagonalUpdate(	BlockMatrix &B,
													const BlockMatrix &x,
													const BlockMatrix &y
													)
{
    int run1;
    BlockMatrix a(5,1), b(5,1);
    BlockMatrix block(5,5);

    for( run1 = 0; run1 < (int)nBlocks; run1++ ){

        a.setZero();
        b.setZero();
        block.setZero();

        getSubBlockLine( nBlocks, 0, 0, run1, x, a );
        getSubBlockLine( nBlocks, 0, 0, run1, y, b );

        getSubBlockLine( nBlocks,           run1, 0, run1, B, block );
        getSubBlockLine( nBlocks,   nBlocks+run1, 1, run1, B, block );
        getSubBlockLine( nBlocks, 2*nBlocks+run1, 2, run1, B, block );
        getSubBlockLine( nBlocks, 3*nBlocks+run1, 3, run1, B, block );
        getSubBlockLine( nBlocks, 4*nBlocks+run1, 4, run1, B, block );

        applyUpdate( block, a, b );

        setSubBlockLine( nBlocks,           run1, 0, run1, B, block );
        setSubBlockLine( nBlocks,   nBlocks+run1, 1, run1, B, block );
        setSubBlockLine( nBlocks, 2*nBlocks+run1, 2, run1, B, block );
        setSubBlockLine( nBlocks, 3*nBlocks+run1, 3, run1, B, block );
        setSubBlockLine( nBlocks, 4*nBlocks+run1, 4, run1, B, block );
    }

    return SUCCESSFUL_RETURN;
}



returnValue BFGSupdate::getSubBlockLine( const int         &N     ,
                                         const int         &line1 ,
                                         const int         &line2 ,
                                         const int         &offset,
                                         const BlockMatrix &M     ,
                                               BlockMatrix &O      )
{
	DMatrix tmp;

	M.getSubBlock(     offset, line1, tmp );
	if( tmp.getDim() != 0 ) O.setDense(0,line2,tmp);

	M.getSubBlock(   N+offset, line1, tmp );
	if( tmp.getDim() != 0 ) O.setDense(1,line2,tmp);

	M.getSubBlock( 2*N+offset, line1, tmp );
	if( tmp.getDim() != 0 ) O.setDense(2,line2,tmp);

	M.getSubBlock( 3*N+offset, line1, tmp );
	if( tmp.getDim() != 0 ) O.setDense(3,line2,tmp);

	M.getSubBlock( 4*N+offset, line1, tmp );
	if( tmp.getDim() != 0 ) O.setDense(4,line2,tmp);

	return SUCCESSFUL_RETURN;
}


returnValue BFGSupdate::setSubBlockLine( const int         &N     ,
                                         const int         &line1 ,
                                         const int         &line2 ,
                                         const int         &offset,
                                               BlockMatrix &M     ,
                                         const BlockMatrix &O      )
{
	DMatrix tmp;

	O.getSubBlock( 0, line2, tmp );

	if( tmp.getDim() != 0 ) M.setDense(     offset, line1, tmp );
	O.getSubBlock( 1, line2, tmp );
	if( tmp.getDim() != 0 ) M.setDense(   N+offset, line1, tmp );

	O.getSubBlock(2,line2,tmp);
	if( tmp.getDim() != 0 ) M.setDense( 2*N+offset, line1, tmp );

	O.getSubBlock(3,line2,tmp);
	if( tmp.getDim() != 0 ) M.setDense( 3*N+offset, line1, tmp );

	O.getSubBlock(4,line2,tmp);
	if( tmp.getDim() != 0 ) M.setDense( 4*N+offset, line1, tmp );

	return SUCCESSFUL_RETURN;
}





//
// PROTECTED MEMBER FUNCTIONS:
//





CLOSE_NAMESPACE_ACADO

// end of file.
