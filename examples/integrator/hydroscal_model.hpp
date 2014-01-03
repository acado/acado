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
 *  \file examples/integrator/hydroscal_model.cpp
 *  \author Moritz Diehl, Boris Houska, Hans Joachim Ferreau, David Ariens
 *  \date 2009-2010
 *
 *
 *  Distillation model
 *  82 diff + 122 alg var
 *  Zoltan Nagy, Moritz Diehl, 2000
 *
 *  For a description see e.g. Chapter 7 in the PhD thesis 
 *  "Real-Time Optimization for Large Scale Nonlinear Processes"
 *  (2001) by Moritz Diehl
 *  Download at: http://www.ub.uni-heidelberg.de/archiv/1659/
 *
 *  Optimization problem is to steer the column 
 *  from a disturbed state back into the nominal 
 *  operating point, minimizing an integrated
 *  least-squares deviation of two temperatures 
 *  and the controls.
 *
 *  EXPRESSIONS:
 *  u = (  L_{vol} , Q                                           )^T
 *  x = (  X_0, ..., X_{41}, n_1, ..., n_{40}                    )^T
 *  z = (  L_1, ..., L_{40}, V_1, ..., V_{40}, T_0, ..., T_{41}  )^T.
 */


//#define PSEUDO_STATES
/* problem setup */

/* #define  NMOS  2*/

#define  NMOS   2
#define  NP     17
#define  NRC    0
#define  NRCE   0

#define  NPR    0
#define  NLSQ   4 

#ifdef PSEUDO_STATES
#define  NXD    84
#else
#define  NXD    82
#endif

#define  NXA    122
#define  NU     2
#define  NW     1

/* model parameters */

#define  qF      1
#define  Nfeed   20

#define A1       23.48
#define B1       3626.6
#define C1       -34.29

#define A2       22.437
#define B2       3166.4
#define C2       -80.15

#define Rconst        8.3147

#define T14s    (70)
#define T28s    (88)

#define QT14    1    
#define QT28    1    
	             
#define  R11  (2/2)
#define  R22  (0.1/2)
#define  RPENALTY  (0.1)


typedef struct {

    double x[42]; 
    double n[42]; 
    double L[42]; 
    double V[42]; 
    double Temp[42];

    double refvol[42];
    double Pres[42];
    double P1[42];
    double P2[42];
    double y[42];

    double F; 
    double xF; 


    double  nvolB;
    double  nvolD;
    double  nvolM;
    double  nvolK;
    double  flowwidth;

    double  alphastrip;
    double  alpharect;
    double  Ptop;
    double  DPstrip;
    double  DPrect;
    double  Tfeed;
    double  Tcond;

} Parameters;

static Parameters par;




static double T_x(double T, double x)
     /*
      Calculates derivative of T with respect to x, 
      according to implicit function theorem for
      0=F=x*P1(T)+(1-x)*P2(T)-P
      */
{
     double P1,P2,P1_T,P2_T, F_T, F_x, T_x;

     P1   = exp(A1-B1/(C1+T));  
     P1_T = P1 * B1/(C1+T)/(C1+T); 

     P2   = exp(A2-B2/(C2+T));  
     P2_T = P2 * B2/(C2+T)/(C2+T);

     F_T=x*P1_T+(1-x)*P2_T;
     F_x=P1-P2;

     T_x=- 1/F_T*F_x;
     return T_x;
}


static void enthalpies_1(double Temp, double Pres, double *hL1, double *hV1, double *hL1_T)
     /* 
	gives enthalpies of pure liquid/gaseous Methanol at Temperature Temp and Pressure Pres 
     */

{
  const double  h11    =   18.31    ;
  const double  h21    =   1.713E-2 ;
  const double  h31    =   6.399E-5 ;
  const double  Tc1    =   512.6    ;
  const double  Pc1    =   8.096E6  ;
  const double  OMEGA1 =   0.557    ;

  double TR1, PR1, Dhv1;

  TR1=Temp/Tc1;
  PR1=Pres/Pc1;
  Dhv1 = Rconst*Tc1*sqrt(1-PR1/pow(TR1,3))*(6.09648-1.28862*TR1+
      1.016*pow(TR1,7)+OMEGA1*(15.6875-13.4721*TR1+2.615*pow(TR1,7)));
  *hL1=(h11*(Temp-273.15)+h21*pow(Temp-273.15,2)+h31*pow(Temp-273.15,3))*4.186;
  *hV1=*hL1+Dhv1;
  *hL1_T=(h11+h21*2*(Temp-273.15)+h31*3*pow(Temp-273.15,2))*4.186;
}

static void enthalpies_2(double Temp, double Pres, double *hL2, double *hV2, double *hL2_T)
     /* 
	gives enthalpies of pure liquid/gaseous n-Propanol at Temperature Temp and Pressure Pres 
     */

{
  const double  h12     =   31.92      ;
  const double  h22     =   4.49E-2    ;
  const double  h32     =   9.663E-5   ;
  const double  Tc2     =   536.7      ;
  const double  Pc2     =   5.166E6    ;
  const double  OMEGA2  =   0.612      ;

  double TR2, PR2, Dhv2;

  TR2=Temp/Tc2;
  PR2=Pres/Pc2;
  Dhv2 = Rconst*Tc2*sqrt(1-PR2/pow(TR2,3))*(6.09648-1.28862*TR2+
      1.016*pow(TR2,7)+OMEGA2*(15.6875-13.4721*TR2+2.615*pow(TR2,7)));
  *hL2=(h12*(Temp-273.15)+h22*pow(Temp-273.15,2)+h32*pow(Temp-273.15,3))*4.186;
  *hV2=*hL2+Dhv2;
  *hL2_T=(h12+h22*2*(Temp-273.15)+h32*3*pow(Temp-273.15,2))*4.186;
}



static void enthalpies(double Temp, double Pres, double x, double y, 
		       double *hL, double *hV, double *hL_x_total)
     /* 
	gives enthalpies  of mixed liquid/vapour
	- and (total) derivative of liquid -
	at Temperature Temp and Pressure Pres 
     */

{
  double hL1, hV1, hL1_T, hL2, hV2, hL2_T, hL_x, hL_T;
  enthalpies_1(Temp, Pres, &hL1,&hV1, &hL1_T);
  enthalpies_2(Temp, Pres, &hL2,&hV2, &hL2_T);
  *hL=hL1*x+hL2*(1-x);
  *hV=hV1*y+hV2*(1-y);
  hL_x=hL1-hL2;
  hL_T=hL1_T*x+hL2_T*(1-x);
  *hL_x_total=hL_x
              +hL_T*T_x(Temp,x)
             ;
}


static double litre_per_kmol(double TK, double x_m)
     /* gives molar Volume in 
	     litre / kmol
	from 
             Temp in Kelvin, 
	     x_m the Methanol concentration
     */

{
  double d_m, vm, d_p, vp, vol_p, vol_m, vol_sum, v_mix;//, F;
  d_m =  2.288/ pow(0.2685,(1+ pow((1 - TK/512.4),0.2453))); 
  vm = 1/d_m;

  d_p =  1.235/ pow(0.27136, (1+ pow((1- TK/536.4),0.24)));
  vp = 1/d_p;

  vol_m = x_m*vm;
  vol_p = (1-x_m)*vp;
  vol_sum = vol_m + vol_p;
  v_mix= vol_sum*1000;

  return v_mix;
}


static double ramp(double x)
{
  /*
   smooth version of the "__/" function 1/2 ( x + |x| ) 
   */
  const double R=20; 
  double xo;
   if (x<-R)  xo=0;
  else if (x<R) xo=0.5*(x+log(2*cosh(x)));
  else xo=x;

  return xo;

}
static double kmol_per_sec_outflow(double n, double TK, double x_m, double refvol, 
				   double flowwidth)
     /* gives Tray outflow in 
             kmol / sec
	from 
             n molar tray holdup
             Temp in Kelvin, 
	     x_m the Methanol concentration
	     refvol reference volume in litre
     */

{
  double volumeholdup, volumeoutflow, outflow, l_per_kmol;
  const double alpha=500;
  l_per_kmol=litre_per_kmol(TK, x_m);
  volumeholdup=l_per_kmol * n;
  volumeoutflow = flowwidth*(
			     pow(1/alpha*ramp(alpha*(volumeholdup-refvol)),1.5)
			     /* -1/pow(alpha,1.5)*ramp(-alpha*(volumeholdup-refvol)) */
			     );
  outflow=volumeoutflow / l_per_kmol;

  /*
  acadoFPrintf(stderr,"%g  ",refvol/volumeholdup);
  */

  return outflow;
}



static void insert(double *xd, 
		   double *xa,  
		   double *u, 
		   double *p
)
     /* 
	Extracts variables out of xd, xa, u, p, puts them into
	par.x, par.Temp, par.V, par.L, etc.
       

	Attention: par.V[41] and par.L[0] are only specified in prepare(), as they need y.

     */
{
  double Lc, Vc;
  double Q;//, R;
  double L_lh,F_lh;
  //double xC;
  double holdup = 0.0;
  double dummy, hL, hV, deltaH, Qloss;
  long i;
  extern Parameters par;
  
  L_lh   = u[0];
  Q      = u[1];

  par.alpharect  = p[2];
  par.alphastrip = p[3];

  Qloss   = p[5];

  par.nvolB      = p[6];
  par.nvolD      = p[7];
  par.nvolM      = p[0];
  par.nvolK      = p[0];
  par.flowwidth  = p[4];


  par.Ptop  = p[8];
  par.DPstrip    = p[9];
  par.DPrect     = p[9];

#ifdef PSEUDO_STATES
  F_lh      = xd[82];
  par.xF    = xd[83];
#else
  F_lh      = p[10];
  par.xF    = p[11];
#endif


  par.Tfeed = p[12] +273.15;
  par.Tcond = p[13] +273.15;

  /* Let's just put the variables in a nicer form */  

  for (i=0; i<= 39; i++) 
    {
    par.L[i+1]=xa[i];
    par.V[i+1]=xa[40+i]; 
    par.Temp[i]=xa[80+i];
    }
  par.Temp[40]=xa[120];
  par.Temp[41]=xa[121];

  for (i=0; i<= 41; i++) 
    par.x[i]=xd[i];

  for (i=1; i<= 40; i++) 
    par.n[i]=xd[41+i];


  /* Pressure  */
  par.Pres[41]=par.Ptop;
  for (i=41; i>=1; i--)
    {
      if (i>Nfeed)
	{par.Pres[i-1]=par.Pres[i]+par.DPrect;}
      else
	{par.Pres[i-1]=par.Pres[i]+par.DPstrip;}
    }

  /* vapor concentrations */
  for (i=0; i<= 41; i++)
    {
    par.P1[i] = exp(A1-B1/(C1+par.Temp[i]));  
    par.P2[i] = exp(A2-B2/(C2+par.Temp[i]));  
    par.y[i]=par.P1[i]*par.x[i]/par.Pres[i];
    }
 /* Tray efficiency */
  for (i=1; i<= Nfeed; i++)
    par.y[i]=par.alphastrip*par.y[i]+(1-par.alphastrip)*par.y[i-1]; 

  for (i=Nfeed+1; i<= 40; i++)
    par.y[i]=par.alpharect*par.y[i]+(1-par.alpharect)*par.y[i-1]; 


  par.y[41]=par.x[41]; /* Condenser outflow out of column is liquid */

  /* recalculate input fluxes from litres/hour into kmol/sec */
  par.F     = F_lh / (litre_per_kmol(par.Tfeed, par.xF)*3600); /* [kmol/s] */
  Lc     = L_lh / (litre_per_kmol(par.Tcond, par.x[41])*3600); /* [kmol/s] */
  /* Lc     = L_lh / (litre_per_kmol(par.Temp[41], par.x[41])*3600); */
  /* Lc     = R/(1+R) * par.V[40]; */
  par.L[41]=Lc;  

  /* calculate vapour flow */  
  i=0;
  enthalpies(par.Temp[i], par.Pres[i], par.x[i], par.y[i], &hL,&hV, &dummy);
  deltaH=hV-hL;
  Vc  = (Q-Qloss)/deltaH;
  par.V[0]=Vc;  

  for (i=0; i<= 41; i++) 
    { 
      if (i==0) holdup=par.nvolB;
      else if (i<Nfeed) holdup=par.nvolK;
      if (i==Nfeed) holdup=par.nvolM;
      else if (i<41) holdup=par.nvolK;
      else if (i==41) holdup=par.nvolD;
      
      par.refvol[i]=holdup;
    }
  par.n[0] =par.refvol[0]/litre_per_kmol(par.Temp[0], par.x[0]);
  par.n[41]=par.refvol[41]/litre_per_kmol(par.Temp[41], par.x[41]);
		       
    /* Volume of Reboiler is fixed */
  par.L[0]=1/litre_per_kmol(par.Temp[0], par.x[0])
           * (  litre_per_kmol(par.Temp[0], par.x[1]) * par.L[1]
	      - litre_per_kmol(par.Temp[0], par.y[0]) * par.V[0]
	   );

    /* Volume of Condenser is fixed */
  par.V[41]=1/litre_per_kmol(par.Temp[41], par.y[41])
            * (  litre_per_kmol(par.Temp[41], par.y[40]) * par.V[40]
	       - litre_per_kmol(par.Temp[41], par.x[41]) * par.L[41]
	    );

}

static void diffeq(
		   double *dx,
		   double *dn
)
     /* 
	Calculates dx[42] and dn[40] from
	par.x, par.Temp, par.V, par.L, par.y

     */
{

  extern Parameters par;
  long i;
  double Vmol, dVmol_x;

  /* Reboiler with fixed volume */
  i=0;
  Vmol=litre_per_kmol(par.Temp[i],  par.x[i]);
  dVmol_x=litre_per_kmol(par.Temp[i], 1.0)-litre_per_kmol(par.Temp[i], 0.0);

  dx[i] = 1 / (par.n[i]*(1 + dVmol_x * par.x[i] / Vmol) ) 
            * (par.L[i+1]*par.x[i+1] - par.V[i]*par.y[i] - par.L[i]*par.x[i]);

  /* Stripping + Rectifying + Feed */

  for (i=1; i<=40; i++) 
    {
      dx[i] = -(1/par.n[i])*par.V[i]*(par.y[i]-par.x[i]);
      dn[i-1] = -par.V[i]-par.L[i];

      dx[i] += (1/par.n[i])*par.L[i+1]*(par.x[i+1]-par.x[i]);
      dn[i-1] += par.L[i+1];

      dx[i] += (1/par.n[i])*par.V[i-1]*(par.y[i-1]-par.x[i]); 
      dn[i-1] += par.V[i-1]; 
      
      if (i==Nfeed) 
	{
	  dx[i] += (1/par.n[i])*par.F*(par.xF-par.x[i]); 
	  dn[i-1] += par.F;
	}
    }

  /* Condenser  with fixed volume */
  i=41;
  Vmol=litre_per_kmol(par.Temp[i],  par.x[i]);
  dVmol_x=litre_per_kmol(par.Temp[i], 1.0)-litre_per_kmol(par.Temp[i], 0.0);

  dx[i] = 1 / (par.n[i]*(1 + dVmol_x * par.x[i] / Vmol) ) 
            * (par.V[i-1]*par.y[i-1] - par.V[i]*par.y[i] - par.L[i]*par.x[i]);
}


static void ffcn( double *t, double *xd, double *xa, double *u, 
                  double *p, double *rhs ){

  long i;
#ifdef PSEUDO_STATES
  double xdcopy[84];
#else
  double xdcopy[82];
#endif
  double xacopy[122];
  /* scaling */
  for(i=0;i<42;i++)
    xdcopy[i]=xd[i];
  for(i=42;i<82;i++)
    xdcopy[i]=xd[i]*1e-3;
#ifdef PSEUDO_STATES
   xdcopy[82]=xd[82];
   xdcopy[83]=xd[83];
#endif
  for(i=0;i<80;i++)
    xacopy[i]=xa[i]*1e-5;
  for(i=80;i<122;i++)
    xacopy[i]=xa[i]+273.15;

  insert(xdcopy, xacopy, u, p);   
  diffeq(rhs, &rhs[42]);

  /* scaling */
  for(i=42;i<82;i++)
    rhs[i]=rhs[i]/1e-3;

#ifdef PSEUDO_STATES
  rhs[82]=0;
  rhs[83]=0;
#endif

}



static void gfcn(double *t, double *xd, double *xa, double *u, 
                 double *p, double *rhs ){

  extern Parameters par;

  long i;
  double hL_x[42], hL[42], hV[42]; 
  double dx[42],dn[42];
  double Pfeed, yfeed, P1, P2, dummy;
  double hLfeed, hVfeed; 

#ifdef PSEUDO_STATES
  double xdcopy[84];
#else
  double xdcopy[82];
#endif
  double xacopy[122];

  /* scaling */ 
  for(i=0;i<42;i++)
    xdcopy[i]=xd[i];
  for(i=42;i<82;i++)
    xdcopy[i]=xd[i]*1e-3;
#ifdef PSEUDO_STATES
   xdcopy[82]=xd[82];
   xdcopy[83]=xd[83];
#endif

  for(i=0;i<80;i++)
    xacopy[i]=xa[i]*1e-5;
  
  for(i=80;i<122;i++)
    xacopy[i]=xa[i]+273.15;


  
  insert(xdcopy, xacopy, u, p);   
  diffeq(dx, dn);

  /* enthalpies */
  for (i=0; i<= 41; i++)
    {
      enthalpies(par.Temp[i], par.Pres[i], par.x[i], par.y[i], &hL[i],&hV[i], &hL_x[i]);
    }
   
  /* entalphy of the feed */
 
  P1 = exp(A1-B1/(C1+par.Tfeed));
  P2 = exp(A2-B2/(C2+par.Tfeed));
  Pfeed=P1*par.xF+(1-par.xF)*P2;
  yfeed=P1*par.xF/Pfeed; 	 
  
  enthalpies(par.Tfeed, Pfeed, par.xF, yfeed, &hLfeed,&hVfeed, &dummy);
  
  /* enthalphy of the condenser reflux, when Temperature different from equilibrium */
      i=41;
      par.Temp[i]=par.Tcond;
      enthalpies(par.Temp[i], par.Pres[i], par.x[i], par.y[i], &hL[i],&hV[i], &hL_x[i]);
  
  
  /* ++++++++++++++++++++ Algebraic equations ++++++++++++++++++++++ */
  
  
  /* Stripping + Rectifying + Feed - algebraic */
  
  for (i=1; i<=40; i++)
    {
      rhs[i-1] = par.L[i]-kmol_per_sec_outflow(par.n[i], par.Temp[i], par.x[i], 
					       par.refvol[i], par.flowwidth);
      /*
      rhs[40+i-1] =   par.V[i-1]*hV[i-1] 
                    - par.V[i]*hV[i] 
		    + par.L[i+1]*hL[i+1] 
		    - par.L[i]*hL[i] 
	            - par.n[i]*hL_x[i]*dx[i]
		    - dn[i-1]*hL[i];
      if (i==Nfeed) 
	{
	  rhs[40+i-1] += par.F*hLfeed + (1-qF)*hVfeed; 
	}
      */

      rhs[40+i-1] =   par.V[i-1]*(hV[i-1] - hL[i])  
	            - par.V[i]*(hV[i] -hL[i]) 
	            + par.L[i+1]*(hL[i+1]-hL[i])  
	- par.n[i]*hL_x[i]*dx[i];
      if (i==Nfeed) 
	{
	  rhs[40+i-1] += par.F*(hLfeed - hL[i]) + (1-qF)*hVfeed; 
	}

    }
  
  /* Temperatures - algebraic */
  for (i=0; i<= 41; i++)
    rhs[80+i]=1-par.P1[i]/par.Pres[i]*par.x[i]-(1-par.x[i])*par.P2[i]/par.Pres[i];

  /* scaling */
  for(i=0;i<40;i++)
    rhs[i]=rhs[i]/1e-5;
}

