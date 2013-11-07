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
 *    \file include/acado/matrix_vector/condensing_matrix.ipp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 19.01.2009
 */


//
// PUBLIC MEMBER FUNCTIONS:
//


BEGIN_NAMESPACE_ACADO


inline CondensingMatrix CondensingMatrix::hessianMultiplication( const BlockMatrix& hessian ) const{

    uint run1, run2, run3;

    CondensingMatrix tmp2( N, nx, nxa, np, nu, nw );

    BlockMatrix Hblock(5,5  );
    BlockMatrix Tblock(5,3*N);
    BlockMatrix HTblock      ;

    for( run1 = 0; run1 < N; run1++ ){

        // copy Hblock:
        // ------------
        for( run2 = 0; run2 < 5; run2++ ){
           for( run3 = 0; run3 < 5; run3++ ){
               Hblock.types[run2][run3] = hessian.types[run1+N*run2][run1+N*run3];
               if( (hessian.elements[run1+N*run2][run1+N*run3]).getDim() != 0 )
                   Hblock.elements[run2][run3] = hessian.elements[run1+N*run2][run1+N*run3];
           }
        }

        // copy Tblock:
        // ------------
        for( run2 = 0; run2 < 5; run2++ ){
           for( run3 = 0; run3 < 3*N; run3++ ){
               Tblock.types[run2][run3] = types[run1+N*run2][run3];
               if( elements[run1+N*run2][run3].getDim() != 0 )
                   Tblock.elements[run2][run3] = elements[run1+N*run2][run3];
           }
        }

        // multiply the sub-blocks:
        // ------------------------
        HTblock = Hblock*Tblock;


        // copy back:
        // ------------
        for( run2 = 0; run2 < 5; run2++ ){
           for( run3 = 0; run3 < 3*N; run3++ ){
               tmp2.types[run1+N*run2][run3] = HTblock.types[run2][run3];
               if( (HTblock.elements[run2][run3]).getDim() != 0 )
                   tmp2.elements[run1+N*run2][run3] = HTblock.elements[run2][run3];
           }
        }
    }
    return tmp2;
}



inline CondensingMatrix CondensingMatrix::operator^( const CondensingMatrix& HDense ) const{

    uint run1, run2, run3;

    CondensingMatrix tmp;
    tmp.init( 3*N, 3*N );

    uint  *length = new uint[3*N];
    uint **index  = new uint*[3*N];

    RealClock clock;


    clock.start();

    // determine the lengths of the indices:
    // -------------------------------------
    for( run1 = 0; run1 < 3*N; run1++ ){
        length[run1] = 0;
        for( run2 = 0; run2 < 5*N; run2++ )
             if( types[run2][run1] != SBMT_ZERO )
                 length[run1]++;
    }

    // determine the indices:
    // ------------------------------------------
    for( run1 = 0; run1 < 3*N; run1++ ){
        index[run1] = new uint[length[run1]];
        run3 = 0;
        for( run2 = 0; run2 < 5*N; run2++ ){
             if( types[run2][run1] != SBMT_ZERO ){
                 index[run1][run3] = run2;
                 run3++;
             }
        }
    }

    clock.stop();

     printf("index generation = %f s \n", clock.getTime() );


    clock.reset();

    clock.start();

    // perform the actual multiplication:
    // ----------------------------------

    for( run1 = 0; run1 < 3*N; run1++ ){

        if( run1 == 1 && nxa == 0 )
            run1 += N;

        if( run1 == N+1 && np == 0 )
            run1 ++;

        if( run1 == N+2 && nu == 0 )
            run1 += N-1;

        if( run1 == 2*N+1 && nw == 0 )
            break;

        for( run2 = 0; run2 < 3*N; run2++ ){

            if( run2 == 1 && nxa == 0 )
                run2 += N;

            if( run2 == N+1 && np == 0 )
                run2 ++;

            if( run2 == N+2 && nu == 0 )
                run2 += N-1;

            if( run2 == 2*N+1 && nw == 0 )
                break;

            for( run3 = 0; run3 < length[run1]; run3++ ){
                if( tmp.types[run1][run2] != SBMT_ZERO ){
                    if( HDense.types[index[run1][run3]][run2] != SBMT_ZERO ){
                        tmp.elements[run1][run2] += elements[index[run1][run3]][run1]^ HDense.elements[index[run1][run3]][run2];
                        tmp.types[run1][run2] = SBMT_DENSE;
                    }
                }
                else{
                    if( HDense.types[index[run1][run3]][run2] != SBMT_ZERO ){
                        tmp.elements[run1][run2] = elements[index[run1][run3]][run1] ^ HDense.elements[index[run1][run3]][run2];
                        tmp.types[run1][run2] = SBMT_DENSE;
                    }
                }
            }
        }
    }

    clock.stop();

     printf("multiplication = %f s \n", clock.getTime() );


    for( run1 = 0; run1 < 3*N; run1++ )
        delete[] index[run1];
    delete[] index;
    delete[] length;

    return tmp;
}


CLOSE_NAMESPACE_ACADO


/*
 *	end of file
 */
