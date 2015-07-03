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
 *    \file src/objective/lsq_term.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */

#include <acado/objective/lsq_term.hpp>
#include <acado/curve/curve.hpp>

// #define SIM_DEBUG


BEGIN_NAMESPACE_ACADO

//
// PUBLIC MEMBER FUNCTIONS:
//


LSQTerm::LSQTerm( )
        :ObjectiveElement(){

    S       = 0;
    r       = 0;

    S_temp  = 0;
    r_temp  = 0;

    S_h_res = 0;
}


LSQTerm::LSQTerm( const MatrixVariablesGrid *S_,
                  const Function&            h ,
                  const VariablesGrid       *r_  )
        :ObjectiveElement(){

    S       = 0;
    r       = 0;
    S_h_res = 0;
    fcn     = h;

    if( S_ != 0 ) S_temp = new MatrixVariablesGrid(*S_);
    else          S_temp = 0                           ;

    if( r_ != 0 ) r_temp = new VariablesGrid(*r_);
    else          r_temp = 0                     ;
}


LSQTerm::LSQTerm( const LSQTerm& rhs )
        :ObjectiveElement( rhs ){

    uint run1, run2;

    if( rhs.S_temp != 0 ) S_temp = new MatrixVariablesGrid(*rhs.S_temp);
    else                  S_temp = 0;

    if( rhs.r_temp != 0 ) r_temp = new VariablesGrid(*rhs.r_temp);
    else                  r_temp = 0;


    if( rhs.S != NULL ){
        S = new DMatrix[grid.getNumPoints()];
        for( run1 = 0; run1 < grid.getNumPoints(); run1++ )
            S[run1] = rhs.S[run1];
    }
    else S = NULL;

    if( rhs.r != NULL ){
        r = new DVector[grid.getNumPoints()];
        for( run1 = 0; run1 < grid.getNumPoints(); run1++ )
            r[run1] = rhs.r[run1];
    }
    else r = NULL;

    if( rhs.S_h_res != 0 ){
        S_h_res = new double*[grid.getNumPoints()];
        for( run1 = 0; run1 < grid.getNumPoints(); run1++ ){
            S_h_res[run1] = new double[fcn.getDim()];
            for( run2 = 0; (int) run2 < fcn.getDim(); run2++ ){
                S_h_res[run1][run2] = rhs.S_h_res[run1][run2];
            }
        }
    }
}


LSQTerm::~LSQTerm( ){

    uint run1;

    if( S_temp != 0 )
        delete S_temp;

    if( r_temp != 0 )
        delete r_temp;

    if( S != 0 )
        delete[] S;

    if( r != 0 )
        delete[] r;

    if( S_h_res != 0 ){
        for( run1 = 0; run1 < grid.getNumPoints(); run1++ )
            delete[] S_h_res[run1];
        delete[] S_h_res;
    }
}


LSQTerm& LSQTerm::operator=( const LSQTerm& rhs ){

    uint run1, run2;

    if ( this != &rhs ){


        if( S_temp != 0 )
            delete S_temp;

        if( r_temp != 0 )
           delete[] r_temp;

        if( S!= NULL )
            delete[] S;

        if( r!= NULL )
            delete[] r;

        if( S_h_res != 0 ){
            for( run1 = 0; run1 < grid.getNumPoints(); run1++ )
                delete[] S_h_res[run1];
            delete[] S_h_res;
        }

        ObjectiveElement::operator=(rhs);

        if( rhs.S_temp != 0 ) S_temp = new MatrixVariablesGrid(*rhs.S_temp);
        else                  S_temp = 0;

        if( rhs.r_temp != 0 ) r_temp = new VariablesGrid(*rhs.r_temp);
        else                  r_temp = 0;

        if( rhs.S != NULL ){
            S = new DMatrix[grid.getNumPoints()];
            for( run1 = 0; run1 < grid.getNumPoints(); run1++ )
                S[run1] = rhs.S[run1];
        }
        else S = NULL;

        if( rhs.r != NULL ){
            r = new DVector[grid.getNumPoints()];
            for( run1 = 0; run1 < grid.getNumPoints(); run1++ )
                r[run1] = rhs.r[run1];
        }
        else r = NULL;

        if( rhs.S_h_res != 0 ){
            S_h_res = new double*[grid.getNumPoints()];
            for( run1 = 0; run1 < grid.getNumPoints(); run1++ ){
                S_h_res[run1] = new double[fcn.getDim()];
                for( run2 = 0; (int) run2 < fcn.getDim(); run2++ ){
                    S_h_res[run1][run2] = rhs.S_h_res[run1][run2];
                }
            }
        }
    }
    return *this;
}



returnValue LSQTerm::evaluate( const OCPiterate &x ){

    uint run1, run2, run3;

    DVector h_res;

    const uint nh = fcn.getDim();
    const uint N  = grid.getNumPoints();

    ObjectiveElement::init( x );

    obj = 0.0;

	double currentValue;
	VariablesGrid allValues( 1,grid );

    for( run1 = 0; run1 < N; run1++ ){

		currentValue = 0.0;

        // EVALUATE THE LSQ-FUCNTION:
        // --------------------------
        z.setZ( run1, x );
        h_res = fcn.evaluate( z, (int) run1 );

	
	#ifdef SIM_DEBUG
	if( run1 == 0 || run1 == 1 ){
	  
	    h_res.print("hres"); 
	    r[run1].print("reference");
	  
	}
	#endif
	
	

        // EVALUATE THE OBJECTIVE:
        // -----------------------

        if( r != NULL )
            h_res -= r[run1];

        if( S != NULL ){

        	if ( !(S[run1].getNumCols() == nh && S[run1].getNumRows() == nh) )
        		return ACADOERRORTEXT(RET_ASSERTION,
        				The weighting matrix in the LSQ objective has a wrong dimension.);

            for( run2 = 0; run2 < nh; run2++ ){
                S_h_res[run1][run2] = 0.0;
                for( run3 = 0; run3 < nh; run3++ )
                    S_h_res[run1][run2] += S[run1].operator()(run2,run3)*h_res(run3);
            }

            for( run2 = 0; run2 < nh; run2++ ){
                 currentValue += 0.5*h_res(run2)*S_h_res[run1][run2];
            }
        }
        else{
            for( run2 = 0; run2 < nh; run2++ ){
                S_h_res[run1][run2] = h_res(run2);
                currentValue += 0.5*h_res(run2)*h_res(run2);
            }
        }

		allValues( run1,0 ) = currentValue;
    }

	if (N > 1) {
		DVector tmp(1);
		allValues.getIntegral(IM_CONSTANT, tmp);
		obj = tmp(0);
	} else {
		obj = allValues(0, 0);
	}

    return SUCCESSFUL_RETURN;
}



returnValue LSQTerm::evaluateSensitivities( BlockMatrix *hessian ){

    if( hessian == 0 ) return evaluateSensitivitiesGN(0);

    int run1, run2, run3, run4;
    const int N = grid.getNumPoints();
    const int nh = fcn.getDim();

    if( bSeed != 0 ){

        if( xSeed  != 0 || pSeed  != 0 || uSeed  != 0 || wSeed  != 0 ||
            xSeed2 != 0 || pSeed2 != 0 || uSeed2 != 0 || wSeed2 != 0 )
            return ACADOERROR( RET_WRONG_DEFINITION_OF_SEEDS );

        double *bseed   = new double [nh];
        double **J      = new double*[nh];

        for( run2 = 0; run2 < nh; run2++ )
             J[run2] = new double[fcn.getNumberOfVariables() +1];

        if( bSeed->getNumRows( 0, 0 ) != 1 ) return ACADOWARNING( RET_WRONG_DEFINITION_OF_SEEDS );

        DMatrix bseed_;
        bSeed->getSubBlock( 0, 0, bseed_);

        dBackward.init( 1, 5*N );

        DMatrix Dx ( 1, nx );
        DMatrix Dxa( 1, na );
        DMatrix Dp ( 1, np );
        DMatrix Du ( 1, nu );
        DMatrix Dw ( 1, nw );

        for( run1 = 0; run1 < N; run1++ ){

            Dx .setZero();
            Dxa.setZero();
            Dp .setZero();
            Du .setZero();
            Dw .setZero();

            for( run2 = 0; run2 < nh; run2++ ) bseed[run2] = 0;

            for( run2 = 0; run2 < nh; run2++ ){
                 for(run3 = 0; (int) run3 < fcn.getNumberOfVariables() +1; run3++ )
                     J[run2][run3] = 0.0;

                 bseed[run2] = 1.0;
                 fcn.AD_backward( run1, bseed, J[run2] );
                 bseed[run2] = 0.0;

                 for( run3 = 0; run3 < nx; run3++ ){
                      Dx( 0, run3 ) += bseed_(0,0)*J[run2][y_index[run3]]*S_h_res[run1][run2];
                 }
                 for( run3 = nx; run3 < nx+na; run3++ ){
                      Dxa( 0, run3-nx ) += bseed_(0,0)*J[run2][y_index[run3]]*S_h_res[run1][run2];
                 }
                 for( run3 = nx+na; run3 < nx+na+np; run3++ ){
                      Dp( 0, run3-nx-na ) += bseed_(0,0)*J[run2][y_index[run3]]*S_h_res[run1][run2];
                 }
                 for( run3 = nx+na+np; run3 < nx+na+np+nu; run3++ ){
                      Du( 0, run3-nx-na-np ) += bseed_(0,0)*J[run2][y_index[run3]]*S_h_res[run1][run2];
                 }
                 for( run3 = nx+na+np+nu; run3 < nx+na+np+nu+nw; run3++ ){
                      Dw( 0, run3-nx-na-np-nu ) += bseed_(0,0)*J[run2][y_index[run3]]*S_h_res[run1][run2];
                 }
            }
            if( nx > 0 ) dBackward.setDense( 0,     run1, Dx  );
            if( na > 0 ) dBackward.setDense( 0,   N+run1, Dxa );
            if( np > 0 ) dBackward.setDense( 0, 2*N+run1, Dp  );
            if( nu > 0 ) dBackward.setDense( 0, 3*N+run1, Du  );
            if( nw > 0 ) dBackward.setDense( 0, 4*N+run1, Dw  );


            // COMPUTATION OF THE EXACT HESSIAN:
            // ---------------------------------

            const int nnn = nx+na+np+nu+nw;
            DMatrix tmp( nh, nnn );

            for( run3 = 0; run3 < nnn; run3++ ){
                for( run2 = 0; run2 < nh; run2++ ){
                    if( S != 0 ){
                        tmp( run2, run3 ) = 0.0;
                        for( run4 = 0; run4 < nh; run4++ ){
                            tmp( run2, run3 ) += S[run1].operator()(run2,run4)*J[run4][y_index[run3]];
                        }
                    }
                    else{
                        tmp( run2, run3 ) = J[run2][y_index[run3]];
                    }
                }
            }
            DMatrix tmp2;
            int i,j;
            int *Sidx = new int[6];
            int *Hidx = new int[5];

            Sidx[0] = 0;
            Sidx[1] = nx;
            Sidx[2] = nx+na;
            Sidx[3] = nx+na+np;
            Sidx[4] = nx+na+np+nu;
            Sidx[5] = nx+na+np+nu+nw;

            Hidx[0] =     run1;
            Hidx[1] =   N+run1;
            Hidx[2] = 2*N+run1;
            Hidx[3] = 3*N+run1;
            Hidx[4] = 4*N+run1;

            for( i = 0; i < 5; i++ ){
                for( j = 0; j < 5; j++ ){

                    tmp2.init(Sidx[i+1]-Sidx[i],Sidx[j+1]-Sidx[j]);
                    tmp2.setZero();

                    for( run3 = Sidx[i]; run3 < Sidx[i+1]; run3++ )
                        for( run4 = Sidx[j]; run4 < Sidx[j+1]; run4++ )
                            for( run2 = 0; run2 < nh; run2++ )
                                tmp2(run3-Sidx[i],run4-Sidx[j]) += J[run2][y_index[run3]]*tmp(run2,run4);

                    if( tmp2.getDim() != 0 ) hessian->addDense(Hidx[i],Hidx[j],tmp2);
                }
            }
            delete[] Sidx;
            delete[] Hidx;
        }

        for( run2 = 0; run2 < nh; run2++ )
            delete[] J[run2];
        delete[] J;
        delete[] bseed;
        return SUCCESSFUL_RETURN;
    }
    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}


returnValue LSQTerm::evaluateSensitivitiesGN( BlockMatrix *GNhessian ){

    int run1, run2, run3, run4;
    const int N = grid.getNumPoints();
    const int nh = fcn.getDim();

    if( bSeed != 0 ){

        if( xSeed  != 0 || pSeed  != 0 || uSeed  != 0 || wSeed  != 0 ||
            xSeed2 != 0 || pSeed2 != 0 || uSeed2 != 0 || wSeed2 != 0 )
            return ACADOERROR( RET_WRONG_DEFINITION_OF_SEEDS );

        double *bseed   = new double [nh];
        double **J      = new double*[nh];

        for( run2 = 0; run2 < nh; run2++ )
             J[run2] = new double[fcn.getNumberOfVariables() +1];

        if( bSeed->getNumRows( 0, 0 ) != 1 ) return ACADOWARNING( RET_WRONG_DEFINITION_OF_SEEDS );

        DMatrix bseed_;
        bSeed->getSubBlock( 0, 0, bseed_);

        dBackward.init( 1, 5*N );

        DMatrix Dx ( 1, nx );
        DMatrix Dxa( 1, na );
        DMatrix Dp ( 1, np );
        DMatrix Du ( 1, nu );
        DMatrix Dw ( 1, nw );

        for( run1 = 0; run1 < N; run1++ ){

            Dx .setZero();
            Dxa.setZero();
            Dp .setZero();
            Du .setZero();
            Dw .setZero();

            for( run2 = 0; run2 < nh; run2++ ) bseed[run2] = 0;

            if( fcn.ADisSupported() == BT_FALSE ){

                double *fseed = new double[fcn.getNumberOfVariables()+1];
                for( run3 = 0; (int) run3 < fcn.getNumberOfVariables()+1; run3++ )
                     fseed[run3] = 0.0;

                for( run3 = 0; (int) run3 < fcn.getNumberOfVariables()+1; run3++ ){
                     fseed[run3] = 1.0;
                     fcn.AD_forward( run1, fseed, bseed );
                     fseed[run3] = 0.0;
                     for( run2 = 0; run2 < nh; run2++ )
                         J[run2][run3] = bseed[run2];
                }
                delete[] fseed;
            }

            for( run2 = 0; run2 < nh; run2++ ){

                 if( fcn.ADisSupported() == BT_TRUE ){
                     for(run3 = 0; (int) run3 < fcn.getNumberOfVariables() +1; run3++ )
                         J[run2][run3] = 0.0;
                     bseed[run2] = 1.0;
                     fcn.AD_backward( run1, bseed, J[run2] );
                     bseed[run2] = 0.0;
                 }

                 for( run3 = 0; run3 < nx; run3++ ){
                      Dx( 0, run3 ) += bseed_(0,0)*J[run2][y_index[run3]]*S_h_res[run1][run2];
                 }
                 for( run3 = nx; run3 < nx+na; run3++ ){
                      Dxa( 0, run3-nx ) += bseed_(0,0)*J[run2][y_index[run3]]*S_h_res[run1][run2];
                 }
                 for( run3 = nx+na; run3 < nx+na+np; run3++ ){
                      Dp( 0, run3-nx-na ) += bseed_(0,0)*J[run2][y_index[run3]]*S_h_res[run1][run2];
                 }
                 for( run3 = nx+na+np; run3 < nx+na+np+nu; run3++ ){
                      Du( 0, run3-nx-na-np ) += bseed_(0,0)*J[run2][y_index[run3]]*S_h_res[run1][run2];
                 }
                 for( run3 = nx+na+np+nu; run3 < nx+na+np+nu+nw; run3++ ){
                      Dw( 0, run3-nx-na-np-nu ) += bseed_(0,0)*J[run2][y_index[run3]]*S_h_res[run1][run2];
                 }
            }
            if( nx > 0 ) dBackward.setDense( 0,     run1, Dx  );
            if( na > 0 ) dBackward.setDense( 0,   N+run1, Dxa );
            if( np > 0 ) dBackward.setDense( 0, 2*N+run1, Dp  );
            if( nu > 0 ) dBackward.setDense( 0, 3*N+run1, Du  );
            if( nw > 0 ) dBackward.setDense( 0, 4*N+run1, Dw  );

            // COMPUTE GAUSS-NEWTON HESSIAN APPROXIMATION IF REQUESTED:
            // --------------------------------------------------------

            if( GNhessian != 0 ){

                const int nnn = nx+na+np+nu+nw;
                DMatrix tmp( nh, nnn );

                for( run3 = 0; run3 < nnn; run3++ ){
                    for( run2 = 0; run2 < nh; run2++ ){
                        if( S != 0 ){
                            tmp( run2, run3 ) = 0.0;
                            for( run4 = 0; run4 < nh; run4++ ){
                                tmp( run2, run3 ) += S[run1].operator()(run2,run4)*J[run4][y_index[run3]];
                            }
                        }
                        else{
                            tmp( run2, run3 ) = J[run2][y_index[run3]];
                        }
                    }
                }
                DMatrix tmp2;
                int i,j;
                int *Sidx = new int[6];
                int *Hidx = new int[5];

                Sidx[0] = 0;
                Sidx[1] = nx;
                Sidx[2] = nx+na;
                Sidx[3] = nx+na+np;
                Sidx[4] = nx+na+np+nu;
                Sidx[5] = nx+na+np+nu+nw;

                Hidx[0] =     run1;
                Hidx[1] =   N+run1;
                Hidx[2] = 2*N+run1;
                Hidx[3] = 3*N+run1;
                Hidx[4] = 4*N+run1;

                for( i = 0; i < 5; i++ ){
                    for( j = 0; j < 5; j++ ){

                        tmp2.init(Sidx[i+1]-Sidx[i],Sidx[j+1]-Sidx[j]);
                        tmp2.setZero();

                        for( run3 = Sidx[i]; run3 < Sidx[i+1]; run3++ )
                            for( run4 = Sidx[j]; run4 < Sidx[j+1]; run4++ )
                                for( run2 = 0; run2 < nh; run2++ )
                                    tmp2(run3-Sidx[i],run4-Sidx[j]) += J[run2][y_index[run3]]*tmp(run2,run4);

                        if( tmp2.getDim() != 0 ) GNhessian->addDense(Hidx[i],Hidx[j],tmp2);
                    }
                }
                delete[] Sidx;
                delete[] Hidx;
            }
        }

        for( run2 = 0; run2 < nh; run2++ )
            delete[] J[run2];
        delete[] J;
        delete[] bseed;
        return SUCCESSFUL_RETURN;
    }

    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}

returnValue LSQTerm::getWeigthingtMatrix(const unsigned _index, DMatrix& _matrix) const
{
	if ( S_temp )
	{
		_matrix= S_temp->getMatrix( _index );

		return SUCCESSFUL_RETURN;
	}

	return RET_INITIALIZE_FIRST;
}

returnValue LSQTerm::setReference( const VariablesGrid &ref ){
    uint run1;
    const uint N  = grid.getNumPoints();
    //ASSERT( N == ref.getNumPoints() );
// 	printf("setReference!!!!\n");
    if( r == 0 ) r = new DVector[N];
    if ( N == ref.getNumPoints() ) {
      for( run1 = 0; run1 < N; run1++ )
	  r[run1] = ref.getVector(run1);
// 	  r[run1].print( "ref[run1]" );
    } else {
      Curve tmp;tmp.add(ref);
      for( run1 = 0; run1 < N; run1++ ) {
	  tmp.evaluate(grid.getTime(run1)+ref.getFirstTime(), r[run1]);
// 	  printf( "time = %e\n", grid.getTime(run1)+ref.getFirstTime() );
// 	  r[run1].print( "ref[run1]" );
      }
    }
    return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO

// end of file.
