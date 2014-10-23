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
  *    DISTILLATION COLUMN MODEL
  *    See http://www.iwr.uni-heidelberg.de/~Moritz.Diehl/DISSERTATION/index.html
  *
  *    \file examples/integrator/hydroscal_model.cpp
  *    \author Boris Houska, Hans Joachim Ferreau, Moritz Diehl, David Ariens
  *    \date 2008-2010
  */


#include <acado_integrators.hpp>
#include "hydroscal_model.hpp"
#include <acado_gnuplot.hpp>


void ffcn_model( double *x, double *f, void *user_data ){

    int i;

    double *xd = new double[NXD];
    double *xa = new double[NXA];
    double *u  = new double[ NU];
    double *p  = new double[ NP];

    for( i = 0; i < NXD; i++ ) xd[i] = x[         1+i ];
    for( i = 0; i < NXA; i++ ) xa[i] = x[     NXD+1+i ];
    for( i = 0; i <  NU; i++ )  u[i] = x[ NXA+NXD+1+i ];

    p[ 0] = 1.5458567140000001E-01;
    p[ 1] = 1.7499999999999999E-01;
    p[ 2] = 3.4717208398678062E-01;
    p[ 3] = 6.1895708603484367E-01;
    p[ 4] = 1.6593025789999999E-01;
    p[ 5] = 5.0695122527590109E-01;
    p[ 6] = 8.5000000000000000E+00;
    p[ 7] = 1.7000000000000001E-01;
    p[ 8] = 9.3885430857029321E+04;
    p[ 9] = 2.5000000000000000E+02;
    p[10] = 1.4026000000000000E+01;
    p[11] = 3.2000000000000001E-01;
    p[12] = 7.1054000000000002E+01;
    p[13] = 4.7163089489100003E+01;
    p[14] = 4.1833910753991770E+00;
    p[15] = 2.4899344810136301E+00;
    p[16] = 1.8760537088149468E+02;

    ffcn( &x[0], xd, xa, u, p, f );
    gfcn( &x[0], xd, xa, u, p, &(f[NXD]) );

    delete[] xd;
    delete[] xa;
    delete[]  u;
    delete[]  p;
}




int main( ){

    USING_NAMESPACE_ACADO

	const int  nx  =  82;  // the number of differential states
	const int  nxa = 122;  // the number of algebraic states
	const int  nu  =   2;  // the number of controls
	//const int  np  =   0;  // the number of parameters
	//const int  nw  =   0;  // the number of disturbances

    TIME t;
    DifferentialState x("", nx, 1);
    AlgebraicState z("", nxa, 1);
    Control u("", nu, 1);


    IntermediateState is(1+nx+nxa+nu);
    is(0) = t;
    for (int i=0; i < nx; ++i)  is(1+i)        = x(i);
    for (int i=0; i < nxa; ++i) is(1+nx+i)     = z(i);
    for (int i=0; i < nu; ++i)  is(1+nx+nxa+i) = u(i);

    CFunction hydroscalModel( nx+nxa, ffcn_model );


    // Define a Right-Hand-Side:
    // -------------------------

    DifferentialEquation f;
    f << hydroscalModel(is);


    // DEFINE AN INTEGRATOR:
    // ---------------------
    IntegratorBDF integrator( f );


    // DEFINE INITIAL VALUES:
    // ----------------------

    double xd_init[NXD] = { 2.1936116177990631E-01,
                       3.3363028623863722E-01,
                       3.7313133250625952E-01,
                       3.9896472354654333E-01,
                       4.1533719381260475E-01,
                       4.2548399372287182E-01,
                       4.3168379354213621E-01,
                       4.3543569751236455E-01,
                       4.3768918647214428E-01,
                       4.3903262905928286E-01,
                       4.3982597315656735E-01,
                       4.4028774979047969E-01,
                       4.4055002518902953E-01,
                       4.4069238917008052E-01,
                       4.4076272408112094E-01,
                       4.4078980543461005E-01,
                       4.4079091412311144E-01,
                       4.4077642312834125E-01,
                       4.4075255679998443E-01,
                       4.4072304911231042E-01,
                       4.4069013958173919E-01,
                       6.7041926189645151E-01,
                       7.3517997375758948E-01,
                       7.8975978943631409E-01,
                       8.3481725159539033E-01,
                       8.7125377077380739E-01,
                       9.0027275078767721E-01,
                       9.2312464536394301E-01,
                       9.4096954980798608E-01,
                       9.5481731262797742E-01,
                       9.6551271145368878E-01,
                       9.7374401773010488E-01,
                       9.8006186072166701E-01,
                       9.8490109485675337E-01,
                       9.8860194771099286E-01,
                       9.9142879342008328E-01,
                       9.9358602331847468E-01,
                       9.9523105632238640E-01,
                       9.9648478785701988E-01,
                       9.9743986301741971E-01,
                       9.9816716097314861E-01,
                       9.9872084014280071E-01,
                       3.8633811956730968E+00,
                       3.9322260498028840E+00,
                       3.9771965626392531E+00,
                       4.0063070333869728E+00,
                       4.0246026844143410E+00,
                       4.0358888958821835E+00,
                       4.0427690398786789E+00,
                       4.0469300433477020E+00,
                       4.0494314648020326E+00,
                       4.0509267560029381E+00,
                       4.0518145583397631E+00,
                       4.0523364846379799E+00,
                       4.0526383977460299E+00,
                       4.0528081437632766E+00,
                       4.0528985491134542E+00,
                       4.0529413510270169E+00,
                       4.0529556049324462E+00,
                       4.0529527471448805E+00,
                       4.0529396392278008E+00,
                       4.0529203970496912E+00,
                       3.6071164950918582E+00,
                       3.7583754503438387E+00,
                       3.8917148481441974E+00,
                       4.0094300698741563E+00,
                       4.1102216725798293E+00,
                       4.1944038520620675E+00,
                       4.2633275166560596E+00,
                       4.3188755452109175E+00,
                       4.3630947909857642E+00,
                       4.3979622247841386E+00,
                       4.4252580012497740E+00,
                       4.4465128947193868E+00,
                       4.4630018314791968E+00,
                       4.4757626150015568E+00,
                       4.4856260094946823E+00,
                       4.4932488551808500E+00,
                       4.4991456959629330E+00,
                       4.5037168116896273E+00,
                       4.5072719605639726E+00,
                       4.5100498969782414E+00  };


    double xa_init[NXA] = { 8.7651079143636981E+00,
                       8.7871063316432316E+00,
                       8.7893074703670067E+00,
                       8.7901954544445342E+00,
                       8.7901233416606477E+00,
                       8.7894020661781447E+00,
                       8.7882641216255362E+00,
                       8.7868655382627203E+00,
                       8.7853059232818165E+00,
                       8.7836472367940104E+00,
                       8.7819274715696096E+00,
                       8.7801697317787344E+00,
                       8.7783879979338462E+00,
                       8.7765907033291164E+00,
                       8.7747829241037341E+00,
                       8.7729677102977046E+00,
                       8.7711468912374286E+00,
                       8.7693215615475513E+00,
                       8.7674923739534876E+00,
                       8.7656597155017142E+00,
                       2.7825469403413372E+00,
                       2.8224111125799740E+00,
                       2.8351257821612172E+00,
                       2.8455862495713884E+00,
                       2.8539999172723634E+00,
                       2.8606290594307993E+00,
                       2.8657653801220269E+00,
                       2.8696861889639877E+00,
                       2.8726352758900391E+00,
                       2.8748174364382795E+00,
                       2.8763998227654772E+00,
                       2.8775162576841131E+00,
                       2.8782724559458406E+00,
                       2.8787511355838511E+00,
                       2.8790165741126224E+00,
                       2.8791184656798956E+00,
                       2.8790950843473126E+00,
                       2.8789758246804231E+00,
                       2.8787832131565576E+00,
                       2.8785344845386325E+00,
                       3.7489688386445703E+00,
                       3.7511699771858589E+00,
                       3.7520579611269311E+00,
                       3.7519858482265618E+00,
                       3.7512645726312401E+00,
                       3.7501266279652898E+00,
                       3.7487280444903774E+00,
                       3.7471684294005221E+00,
                       3.7455097428065072E+00,
                       3.7437899774766037E+00,
                       3.7420322375793442E+00,
                       3.7402505036278120E+00,
                       3.7384532089192324E+00,
                       3.7366454295969547E+00,
                       3.7348302157041928E+00,
                       3.7330093965681632E+00,
                       3.7311840668122449E+00,
                       3.7293548791598456E+00,
                       3.7275222206556560E+00,
                       3.5295879437000068E+00,
                       3.5694521158072119E+00,
                       3.5821667852381145E+00,
                       3.5926272524800611E+00,
                       3.6010409200004330E+00,
                       3.6076700619776987E+00,
                       3.6128063825021988E+00,
                       3.6167271912042991E+00,
                       3.6196762780219980E+00,
                       3.6218584384893231E+00,
                       3.6234408247540211E+00,
                       3.6245572596202216E+00,
                       3.6253134578357198E+00,
                       3.6257921374340887E+00,
                       3.6260575759313443E+00,
                       3.6261594674751652E+00,
                       3.6261360861254119E+00,
                       3.6260168264456856E+00,
                       3.6258242149122157E+00,
                       3.6255754862872984E+00,
                       3.5278596344996789E+00,
                       8.7075189603180618E+01,
                       8.2548857813137090E+01,
                       8.1085116711116342E+01,
                       8.0140698658642663E+01,
                       7.9532219332963521E+01,
                       7.9135751980236464E+01,
                       7.8870294691608407E+01,
                       7.8684803169784061E+01,
                       7.8547757165561293E+01,
                       7.8439916843235665E+01,
                       7.8349617512390665E+01,
                       7.8269815515229979E+01,
                       7.8196267858564511E+01,
                       7.8126422374424024E+01,
                       7.8058745287858954E+01,
                       7.7992315317564561E+01,
                       7.7926579216690257E+01,
                       7.7861204749145912E+01,
                       7.7795992344269862E+01,
                       7.7730822041668659E+01,
                       7.7665621642015878E+01,
                       7.1094961608415730E+01,
                       6.9448805116206330E+01,
                       6.8122261394548488E+01,
                       6.7060799125769435E+01,
                       6.6217795308026254E+01,
                       6.5550027436041674E+01,
                       6.5020432750221772E+01,
                       6.4598581400619508E+01,
                       6.4260125467265169E+01,
                       6.3985893352644759E+01,
                       6.3760944540013256E+01,
                       6.3573715428857163E+01,
                       6.3415297789173543E+01,
                       6.3278851294230762E+01,
                       6.3159135609293749E+01,
                       6.3052142835899517E+01,
                       6.2954811408060124E+01,
                       6.2864804764641939E+01,
                       6.2780340868928761E+01,
                       6.2700061296306565E+01,
                       6.2622930929692828E+01  };

    double u_init[ NU] = { 4.1833910982822058E+00, 2.4899344742988991E+00 };

    double t_start    =    0.0;
    double t_end      =  120.0;


    // START THE INTEGRATION:
    // ----------------------

    integrator.set( INTEGRATOR_PRINTLEVEL, MEDIUM );
    integrator.set( INTEGRATOR_TOLERANCE, 1e-6 );
    integrator.set( ABSOLUTE_TOLERANCE  , 1e-2 );
	integrator.set( PRINT_INTEGRATOR_PROFILE, YES );
    //integrator.set( LINEAR_ALGEBRA_SOLVER, SPARSE_LU );

    Grid tt( t_start, t_end, 100 );

    integrator.integrate( tt, xd_init, xa_init, 0, u_init );

    integrator.printRunTimeProfile();


    // GET THE RESULTS
    // ---------------

    VariablesGrid differentialStates;
    VariablesGrid algebraicStates   ;
    VariablesGrid intermediateStates;

    integrator.getX ( differentialStates );
    integrator.getXA( algebraicStates    );
    integrator.getI ( intermediateStates );

    GnuplotWindow window;
        window.addSubplot( differentialStates(0) );
        window.addSubplot( algebraicStates   (0) );

    window.plot();


    return 0;
}



