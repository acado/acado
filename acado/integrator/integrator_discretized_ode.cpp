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
 *    \file src/integrator/integrator_discretized_ode.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 */

#include <acado/utils/acado_utils.hpp>
#include <acado/matrix_vector/matrix_vector.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/function/function_.hpp>
#include <acado/function/differential_equation.hpp>
#include <acado/function/discretized_differential_equation.hpp>
#include <acado/integrator/integrator.hpp>
#include <acado/integrator/integrator_runge_kutta.hpp>
#include <acado/integrator/integrator_runge_kutta12.hpp>
#include <acado/integrator/integrator_discretized_ode.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

IntegratorDiscretizedODE::IntegratorDiscretizedODE( )
                         :IntegratorRK12( ){
}


IntegratorDiscretizedODE::IntegratorDiscretizedODE( const DifferentialEquation &rhs_ )
                         :IntegratorRK12( rhs_ ){

    if ( rhs_.isDiscretized( ) == BT_FALSE ){
        ACADOERROR( RET_CANNOT_TREAT_CONTINUOUS_DE );
        ASSERT( 1 == 0 );
    }
    if( rhs_.isDiscretized( ) == BT_FALSE ){
        ACADOERROR( RET_MEMBER_NOT_INITIALISED );
        ASSERT( 1 == 0 );
    }

    stepLength = rhs_.getStepLength();
}


IntegratorDiscretizedODE::IntegratorDiscretizedODE( const IntegratorDiscretizedODE& arg )
                         :IntegratorRK12( arg ){

    stepLength = arg.stepLength;
}


IntegratorDiscretizedODE::~IntegratorDiscretizedODE( ){

}


IntegratorDiscretizedODE& IntegratorDiscretizedODE::operator=( const IntegratorDiscretizedODE& arg ){

    if( this != &arg ){
        IntegratorRK12::operator=( arg );
        stepLength = arg.stepLength;
    }
    return *this;
}


Integrator* IntegratorDiscretizedODE::clone() const{

    return new IntegratorDiscretizedODE(*this);
}



returnValue IntegratorDiscretizedODE::init( const DifferentialEquation &rhs_ )
{
	stepLength = rhs_.getStepLength( );
	return IntegratorRK12::init( rhs_ );
}



returnValue IntegratorDiscretizedODE::step(int number){

    // DEFINE SOME LOCAL VARIABLES:
    // ----------------------------

    int         run1       ;   // simple run variable
    returnValue returnvalue;   // return value for error handling


    // PERFORM ONE STEP OF THE ITERATION:
    // ----------------------------------

    if( soa == SOA_FREEZING_ALL )  returnvalue = performDiscreteStep(number);
    else                           returnvalue = performDiscreteStep( 0    );

    if( returnvalue != SUCCESSFUL_RETURN ) return ACADOWARNING( returnvalue );



    // COMPUTE DERIVATIVES IF REQUESTED:
    // ---------------------------------

    if( nFDirs > 0 && nBDirs2 == 0 && nFDirs2 == 0 ){

        if( soa != SOA_EVERYTHING_FROZEN ){
            return ACADOERROR(RET_NOT_FROZEN);
        }
        if( nBDirs != 0 ){
            return ACADOERROR(RET_WRONG_DEFINITION_OF_SEEDS);
        }
        performADforwardStep(number);
    }
    if( nBDirs > 0 ){

        if( soa != SOA_EVERYTHING_FROZEN ){
            return ACADOERROR(RET_NOT_FROZEN);
        }
        if( nFDirs != 0 || nBDirs2 != 0 || nFDirs2 != 0 ){
            return ACADOERROR(RET_WRONG_DEFINITION_OF_SEEDS);
        }
        performADbackwardStep(number);
    }
    if( nFDirs2 > 0 ){

        if( soa != SOA_EVERYTHING_FROZEN ){
            return ACADOERROR(RET_NOT_FROZEN);
        }
        if( nBDirs != 0 || nBDirs2 != 0 || nFDirs != 1 ){
            return ACADOERROR(RET_WRONG_DEFINITION_OF_SEEDS);
        }
        performADforwardStep2(number);
    }
    if( nBDirs2 > 0 ){

        if( soa != SOA_EVERYTHING_FROZEN ){
            return ACADOERROR(RET_NOT_FROZEN);
        }
        if( nBDirs != 0 || nFDirs2 != 0 || nFDirs != 1 ){
            return ACADOERROR(RET_WRONG_DEFINITION_OF_SEEDS);
        }
        performADbackwardStep2(number);
    }


     // ACTUALLY INCREASE THE TIME:
     // ---------------------------

     if( nBDirs > 0 || nBDirs2 > 0 ){

         t = t - stepLength;   // IN BACKWARD THE TIME IS RUNNING BACKWARDS.
     }
     else{

         t = t + stepLength;   // IN FORWARD MODE THE TIME IS RUNNING FORWARDS.
     }



    // STORE THE INTERMEDIATE VALUES IF THIS IS REQUESTED:
    // ---------------------------------------------------

    if( soa == SOA_FREEZING_MESH || soa == SOA_FREEZING_ALL || soa == SOA_MESH_FROZEN_FREEZING_ALL ){

        if( number >= maxAlloc){
            maxAlloc = 2*maxAlloc;
            h = (double*)realloc(h,maxAlloc*sizeof(double));
        }
        h[number] = stepLength;
    }


     if( nFDirs == 0 && nBDirs == 0 && nFDirs2 == 0 && nBDirs == 0 ){

         int i1 = timeInterval.getFloorIndex( t-h[0] );
         int i2 = timeInterval.getFloorIndex( t      );
         int jj;

         for( jj = i1+1; jj <= i2; jj++ ){

             for( run1 = 0; run1 < m; run1++ )
                 xStore( jj, run1 ) = eta4[run1];

             for( run1 = 0; run1 < mn; run1++ )
                 iStore( jj, run1 ) = x[rhs->index( VT_INTERMEDIATE_STATE, run1 )];
         }
     }


    // CHECK WHETHER THE END OF THE TIME HORIZON IS ALREADY ACHIEVED:
    // --------------------------------------------------------------


    if( nBDirs == 0 || nBDirs2 == 0 ){

        // Stop the algorithm if  t >= tend:
        // ----------------------------------------------
        if( t >= timeInterval.getLastTime() - 0.5*stepLength ){
            x[time_index] = timeInterval.getLastTime();
            for( run1 = 0; run1 < m; run1++ ){
                x[diff_index[run1]] = eta4[run1];
            }
            if( soa == SOA_FREEZING_MESH ){
                soa = SOA_MESH_FROZEN;
            }
            if( soa == SOA_FREEZING_ALL || soa == SOA_MESH_FROZEN_FREEZING_ALL ){
                soa = SOA_EVERYTHING_FROZEN;
            }

            return SUCCESSFUL_RETURN;
        }
    }


    return RET_FINAL_STEP_NOT_PERFORMED_YET;
}





// PROTECTED ROUTINES:
// -------------------


returnValue IntegratorDiscretizedODE::performDiscreteStep( const int& number_ ){

    int run1;

    x[time_index] = t;

    for( run1 = 0; run1 < m; run1++ )
        x[diff_index[run1]] = eta4[run1];

    if( rhs[0].evaluate( number_, x, k[0] ) != SUCCESSFUL_RETURN )
        return RET_UNSUCCESSFUL_RETURN_FROM_INTEGRATOR_RK45;

    for( run1 = 0; run1 < m; run1++ )
        eta4[run1] = k[0][run1];

    return SUCCESSFUL_RETURN;
}



returnValue IntegratorDiscretizedODE::performADforwardStep( const int& number_ ){

    int run1, run2;

    for( run1 = 0; run1 < nFDirs; run1++ ){

        for( run2 = 0; run2 < m; run2++ )
            G[diff_index[run2]] = etaG[run2];

        if( rhs[0].AD_forward( number_, G, k[0] ) != SUCCESSFUL_RETURN )
            return RET_UNSUCCESSFUL_RETURN_FROM_INTEGRATOR_RK45;

        for( run2 = 0; run2 < m; run2++ )
            etaG[run2] = k[0][run2];
    }

    return SUCCESSFUL_RETURN;
}



returnValue IntegratorDiscretizedODE::performADbackwardStep( const int& number_ ){

    int run2;
    const int ndir = rhs->getNumberOfVariables() + 1 + m;

    for( run2 = 0; run2 < ndir; run2++ )
        l[0][run2] = 0.0;

    for( run2 = 0; run2 < m; run2++ )
        H[run2] = etaH[diff_index[run2]];

    if( rhs[0].AD_backward( number_, H, l[0] )!= SUCCESSFUL_RETURN )
        return RET_UNSUCCESSFUL_RETURN_FROM_INTEGRATOR_RK45;

    for( run2 = 0; run2 < ndir; run2++ )
        etaH[run2] += l[0][run2];

    for( run2 = 0; run2 < m; run2++ )
        etaH[diff_index[run2]] = l[0][diff_index[run2]];

    return SUCCESSFUL_RETURN;
}



returnValue IntegratorDiscretizedODE::performADforwardStep2( const int& number_ ){

    int run2;

    for( run2 = 0; run2 < m; run2++ ){
        G2[diff_index[run2]] = etaG2[run2];
        G3[diff_index[run2]] = etaG3[run2];
    }

    if( rhs[0].AD_forward2( number_, G2, G3, k[0], k2[0] ) != SUCCESSFUL_RETURN )
        return RET_UNSUCCESSFUL_RETURN_FROM_INTEGRATOR_RK45;

    for( run2 = 0; run2 < m; run2++ ){
        etaG2[run2] = k [0][run2];
        etaG3[run2] = k2[0][run2];
    }

    return SUCCESSFUL_RETURN;
}



returnValue IntegratorDiscretizedODE::performADbackwardStep2( const int& number_ ){

    int run2;
    const int ndir = rhs->getNumberOfVariables() + 1 + m;

    for( run2 = 0; run2 < ndir; run2++ ){
        l [0][run2] = 0.0;
        l2[0][run2] = 0.0;
    }

    for( run2 = 0; run2 < m; run2++ ){
        H2[run2] = etaH2[diff_index[run2]];
        H3[run2] = etaH3[diff_index[run2]];
    }

    if( rhs[0].AD_backward2( number_, H2, H3, l[0], l2[0] )!= SUCCESSFUL_RETURN )
        return RET_UNSUCCESSFUL_RETURN_FROM_INTEGRATOR_RK45;

    for( run2 = 0; run2 < ndir; run2++ ){
        etaH2[run2] += l [0][run2];
        etaH3[run2] += l2[0][run2];
    }
    for( run2 = 0; run2 < m; run2++ ){
        etaH2[diff_index[run2]] = l [0][diff_index[run2]];
        etaH3[diff_index[run2]] = l2[0][diff_index[run2]];
    }

    return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO


// end of file.
