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
 *    \file   examples/ocp/powerkite.cpp
 *    \author Boris Houska
 *    \date   2009
 */


#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>



int main( ){


   USING_NAMESPACE_ACADO


// DIFFERENTIAL STATES :
// -------------------------
   DifferentialState      r;      //  the length r of the cable
   DifferentialState    phi;      //  the angle phi
   DifferentialState  theta;      //  the angle theta
// -------------------------      //  -------------------------------------------
   DifferentialState     dr;      //  first  derivative of r0    with respect to t
   DifferentialState   dphi;      //  first  derivative of phi   with respect to t
   DifferentialState dtheta;      //  first  derivative of theta with respect to t
// -------------------------      //  -------------------------------------------
   DifferentialState      n;      //  winding number
// -------------------------      //  -------------------------------------------
   DifferentialState    Psi;      //  the roll angle Psi
   DifferentialState     CL;      //  the aerodynamic lift coefficient
// -------------------------      //  -------------------------------------------
   DifferentialState      W;      //  integral over the power at the generator
                                  //  ( = ENERGY )


// CONTROL :
// -------------------------
   Control             ddr0;      //  second derivative of r0    with respect to t
   Control             dPsi;      //  first  derivative of Psi   with respect to t
   Control              dCL;      //  first  derivative of CL    with respect to t


// PARAMETERS :
// ------------------------
                                     //  PARAMETERS OF THE KITE :
                                     //  -----------------------------
   double         mk =  850.00;      //  mass of the kite               //  [ kg    ]
   double          A =  500.00;      //  effective area                 //  [ m^2   ]
   double          V =  720.00;      //  volume                         //  [ m^3   ]
   double        cd0 =    0.04;      //  aerodynamic drag coefficient   //  [       ]
                                     //  ( cd0: without downwash )
   double          K =    0.04;      //  induced drag constant          //  [       ]


                                     //   PHYSICAL CONSTANTS :
                                     //  -----------------------------
   double          g =    9.81;      //  gravitational constant         //  [ m /s^2]
   double        rho =    1.23;      //  density of the air             //  [ kg/m^3]

                                     //  PARAMETERS OF THE CABLE :
                                     //  -----------------------------
   double       rhoc = 1450.00;      //  density of the cable           //  [ kg/m^3]
   double         cc =    1.00;      //  frictional constant            //  [       ]
   double         dc = 0.05614;      //  diameter                       //  [ m     ]


                                     //  PARAMETERS OF THE WIND :
                                     //  -----------------------------
   double         w0 =   10.00;      //  wind velocity at altitude h0   //  [ m/s   ]
   double         h0 =  100.00;      //  the altitude h0                //  [ m     ]
   double         hr =    0.10;      //  roughness length               //  [ m     ]



// OTHER VARIABLES :
// ------------------------

   double AQ               ;      //  cross sectional area

   IntermediateState     mc;      //  mass of the cable
   IntermediateState     m ;      //  effective inertial mass
   IntermediateState     m_;      //  effective gravitational mass

   IntermediateState     Cg;

   IntermediateState     dm;      //  first  derivative of m     with respect to t


// DEFINITION OF PI :
// ------------------------

   double PI = 3.1415926535897932;


// ORIENTATION AND FORCES :
// ------------------------

   IntermediateState h               ;      //  altitude of the kite
   IntermediateState w               ;      //  the wind at altitude h
   IntermediateState we          [ 3];      //  effective wind vector
   IntermediateState nwe             ;      //  norm of the effective wind vector
   IntermediateState nwep            ;      //  -
   IntermediateState ewep        [ 3];      //  projection of ewe
   IntermediateState eta            ;      //  angle eta: cf. [2]
   IntermediateState et          [ 3];      //  unit vector from the left to the right wing tip
   IntermediateState Caer            ;
   IntermediateState Cf              ;      //  simple constants
   IntermediateState CD              ;      //  the aerodynamic drag coefficient
   IntermediateState Fg          [ 3];      //  the gravitational force
   IntermediateState Faer        [ 3];      //  the aerodynamic force
   IntermediateState Ff          [ 3];      //  the frictional force
   IntermediateState F           [ 3];      //  the total force


// TERMS ON RIGHT-HAND-SIDE
// OF THE DIFFERENTIAL
// EQUATIONS              :
// ------------------------

   IntermediateState a_pseudo    [ 3];      //  the pseudo accelaration
   IntermediateState dn              ;      //  the time derivate of the kite's winding number
   IntermediateState ddr             ;      //  second derivative of s     with respect to t
   IntermediateState ddphi           ;      //  second derivative of phi   with respect to t
   IntermediateState ddtheta         ;      //  second derivative of theta with respect to t
   IntermediateState power           ;      //  the power at the generator
// ------------------------                 //  ----------------------------------------------
   IntermediateState regularisation  ;      //  regularisation of controls



//                        MODEL EQUATIONS :
// ===============================================================



// CROSS AREA OF THE CABLE :
// ---------------------------------------------------------------

   AQ      =  PI*dc*dc/4.0                                       ;

// THE EFECTIVE MASS' :
// ---------------------------------------------------------------

   mc      =  rhoc*AQ*r        ;   // mass of the cable
   m       =  mk + mc     / 3.0;   // effective inertial mass
   m_      =  mk + mc     / 2.0;   // effective gravitational mass
// -----------------------------   // ----------------------------
   dm      =  (rhoc*AQ/ 3.0)*dr;   // time derivative of the mass


// WIND SHEAR MODEL :
// ---------------------------------------------------------------

   h       =  r*cos(theta)                                       ;
   w       =  log(h/hr) / log(h0/hr) *w0                         ;


// EFFECTIVE WIND IN THE KITE`S SYSTEM :
// ---------------------------------------------------------------

   we[0]   =  w*sin(theta)*cos(phi) -              dr    ;
   we[1]   = -w*sin(phi)            - r*sin(theta)*dphi  ;
   we[2]   = -w*cos(theta)*cos(phi) + r           *dtheta;


// CALCULATION OF THE KITE`S TRANSVERSAL AXIS :
// ---------------------------------------------------------------

   nwep    =  pow(                we[1]*we[1] + we[2]*we[2], 0.5 );
   nwe     =  pow(  we[0]*we[0] + we[1]*we[1] + we[2]*we[2], 0.5 );
   eta     =  asin( we[0]*tan(Psi)/ nwep )                       ;

// ---------------------------------------------------------------

// ewep[0] =  0.0                                                ;
   ewep[1] =  we[1] / nwep                                       ;
   ewep[2] =  we[2] / nwep                                       ;

// ---------------------------------------------------------------

   et  [0] =  sin(Psi)                                                  ;
   et  [1] =  (-cos(Psi)*sin(eta))*ewep[1] - (cos(Psi)*cos(eta))*ewep[2];
   et  [2] =  (-cos(Psi)*sin(eta))*ewep[2] + (cos(Psi)*cos(eta))*ewep[1];




// CONSTANTS FOR SEVERAL FORCES :
// ---------------------------------------------------------------

   Cg      =  (V*rho-m_)*g                                       ;
   Caer    =  (rho*A/2.0 )*nwe                                   ;
   Cf      =  (rho*dc/8.0)*r*nwe                                 ;


// THE DRAG-COEFFICIENT :
// ---------------------------------------------------------------

   CD      =  cd0 + K*CL*CL                                      ;



// SUM OF GRAVITATIONAL AND LIFTING FORCE :
// ---------------------------------------------------------------

   Fg  [0] =  Cg  *  cos(theta)                                  ;
// Fg  [1] =  Cg  *  0.0                                         ;
   Fg  [2] =  Cg  *  sin(theta)                                  ;


// SUM OF THE AERODYNAMIC FORCES :
// ---------------------------------------------------------------

   Faer[0] =  Caer*( CL*(we[1]*et[2]-we[2]*et[1]) + CD*we[0] )   ;
   Faer[1] =  Caer*( CL*(we[2]*et[0]-we[0]*et[2]) + CD*we[1] )   ;
   Faer[2] =  Caer*( CL*(we[0]*et[1]-we[1]*et[0]) + CD*we[2] )   ;


// THE FRICTION OF THE CABLE :
// ---------------------------------------------------------------

// Ff  [0] =  Cf  *  cc* we[0]                                   ;
   Ff  [1] =  Cf  *  cc* we[1]                                   ;
   Ff  [2] =  Cf  *  cc* we[2]                                   ;



// THE TOTAL FORCE :
// ---------------------------------------------------------------

   F   [0] = Fg[0] + Faer[0]                                     ;
   F   [1] =         Faer[1] + Ff[1]                             ;
   F   [2] = Fg[2] + Faer[2] + Ff[2]                             ;



// THE PSEUDO ACCELERATION:
// ---------------------------------------------------------------

   a_pseudo [0] =  - ddr0
                   + r*(                         dtheta*dtheta
                         + sin(theta)*sin(theta)*dphi  *dphi   )
                   - dm/m*dr                                     ;

   a_pseudo [1] =  - 2.0*cos(theta)/sin(theta)*dphi*dtheta
                   - 2.0*dr/r*dphi
                   - dm/m*dphi                                   ;

   a_pseudo [2] =    cos(theta)*sin(theta)*dphi*dphi
                   - 2.0*dr/r*dtheta
                   - dm/m*dtheta                                 ;




// THE EQUATIONS OF MOTION:
// ---------------------------------------------------------------

   ddr          =  F[0]/m                + a_pseudo[0]           ;
   ddphi        =  F[1]/(m*r*sin(theta)) + a_pseudo[1]           ;
   ddtheta      = -F[2]/(m*r           ) + a_pseudo[2]           ;





// THE DERIVATIVE OF THE WINDING NUMBER :
// ---------------------------------------------------------------

   dn           =  (        dphi*ddtheta - dtheta*ddphi     ) /
                   (2.0*PI*(dphi*dphi    + dtheta*dtheta)   )      ;



// THE POWER AT THE GENERATOR :
// ---------------------------------------------------------------

   power        =   m*ddr*dr                                     ;



// REGULARISATION TERMS :
// ---------------------------------------------------------------


   regularisation =    5.0e2 * ddr0    * ddr0
                     + 1.0e8 * dPsi    * dPsi
                     + 1.0e5 * dCL     * dCL
                     + 2.5e5 * dn      * dn
                     + 2.5e7 * ddphi   * ddphi;
                     + 2.5e7 * ddtheta * ddtheta;
                     + 2.5e6 * dtheta  * dtheta;
//                   ---------------------------



// THE "RIGHT-HAND-SIDE" OF THE ODE:
// ---------------------------------------------------------------
   DifferentialEquation f;

   f  << dot(r)      ==  dr                             ;
   f  << dot(phi)    ==  dphi                           ;
   f  << dot(theta)  ==  dtheta                         ;
   f  << dot(dr)     ==  ddr0                           ;
   f  << dot(dphi)   ==  ddphi                          ;
   f  << dot(dtheta) ==  ddtheta                        ;
   f  << dot(n)      ==  dn                             ;
   f  << dot(Psi)    ==  dPsi                           ;
   f  << dot(CL)     ==  dCL                            ;
   f  << dot(W)      == (-power + regularisation)*1.0e-6;


    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    OCP ocp( 0.0, 18.0, 18 );
    ocp.minimizeMayerTerm( W );
    ocp.subjectTo( f );


    // INITIAL VALUE CONSTRAINTS:
    // ---------------------------------
    ocp.subjectTo( AT_START, n == 0.0 );
    ocp.subjectTo( AT_START, W == 0.0 );


    // PERIODIC BOUNDARY CONSTRAINTS:
    // ----------------------------------------
    ocp.subjectTo( 0.0, r     , -r     , 0.0 );
    ocp.subjectTo( 0.0, phi   , -phi   , 0.0 );
    ocp.subjectTo( 0.0, theta , -theta , 0.0 );
    ocp.subjectTo( 0.0, dr    , -dr    , 0.0 );
    ocp.subjectTo( 0.0, dphi  , -dphi  , 0.0 );
    ocp.subjectTo( 0.0, dtheta, -dtheta, 0.0 );
    ocp.subjectTo( 0.0, Psi   , -Psi   , 0.0 );
    ocp.subjectTo( 0.0, CL    , -CL    , 0.0 );

    ocp.subjectTo( -0.34   <= phi   <= 0.34   );
    ocp.subjectTo(  0.85   <= theta <= 1.45   );
    ocp.subjectTo( -40.0   <= dr    <= 10.0   );
    ocp.subjectTo( -0.29   <= Psi   <= 0.29   );
    ocp.subjectTo(  0.1    <= CL    <= 1.50   );
    ocp.subjectTo( -0.7    <= n     <= 0.90   );
    ocp.subjectTo( -25.0   <= ddr0  <= 25.0   );
    ocp.subjectTo( -0.065  <= dPsi  <= 0.065  );
    ocp.subjectTo( -3.5    <= dCL   <= 3.5    );


    // CREATE A PLOT WINDOW AND VISUALIZE THE RESULT:
    // ----------------------------------------------

    GnuplotWindow window;
    window.addSubplot( r,    "CABLE LENGTH  r [m]" );
    window.addSubplot( phi,  "POSITION ANGLE  phi [rad]" );
    window.addSubplot( theta,"POSITION ANGLE theta [rad]" );
    window.addSubplot( Psi,  "ROLL ANGLE  psi [rad]" );
    window.addSubplot( CL,   "LIFT COEFFICIENT  CL" );
    window.addSubplot( W,    "ENERGY  W [MJ]" );
	window.addSubplot( F[0], "FORCE IN CABLE [N]" );
	window.addSubplot( phi,theta, "Kite Orbit","theta [rad]","phi [rad]" );

    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
    // ---------------------------------------------------

    OptimizationAlgorithm algorithm(ocp);

    algorithm.initializeDifferentialStates("powerkite_states.txt"    );
    algorithm.initializeControls          ("powerkite_controls.txt"  );
    algorithm.set                         ( MAX_NUM_ITERATIONS, 100  );
    algorithm.set                         ( KKT_TOLERANCE    , 1e-2 );

    algorithm << window;

    algorithm.set( PRINT_SCP_METHOD_PROFILE, YES );

    algorithm.solve();
    return 0;
}



