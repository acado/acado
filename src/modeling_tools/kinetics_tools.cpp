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
 *    \author Joris Gillis, Boris Houska, Joachim Ferreau
 *
 */


#include <acado/modeling_tools/kinetics_tools.hpp>
#include <math.h>

USING_NAMESPACE_ACADO

#ifndef __MATLAB__
using namespace std;
#endif


enum AXIS{AX_X,AX_Y,AX_Z};


int sign(int a) {
    if (a<0)
      return -1;
    if (a>0)
      return 1;
    return 0;
}

Expression Rperm(int a, int b, int c) {
  Expression R=zeros(3,3);
  R(abs(a)-1,0)=sign(a);
  R(abs(b)-1,1)=sign(b);
  R(abs(c)-1,2)=sign(c);
  return R;
}

int cosquadrant(const int quadrant) {
  int k=quadrant % 4; 
  if (k < 0) k+=4;// force mod to be positive
  int e=0;
  if (k==0) e=1;
  if (k==2) e=-1;
  return e;
}

int sinquadrant(const int quadrant) {
  int k=quadrant % 4; 
  if (k < 0) k+=4;// force mod to be positive
  int e=0;
  if (k==1) e=1;
  if (k==3) e=-1;
  return e;
}

Expression Rxp(const int quadrant) {
  Expression R=zeros(3,3);
  Expression ca=cosquadrant(quadrant);Expression sa=sinquadrant(quadrant);
  //matrix([1,0,0],[0,cos(alpha),-sin(alpha)],[0,sin(alpha),cos(alpha)])
  R(0,0)=1;R(1,1)=ca;R(1,2)=-sa;R(2,1)=sa;R(2,2)=ca;
  return R;
}


Expression Rx(const Expression &angle){

  IntermediateState R  = zeros(3,3);
  IntermediateState ca = cos(angle);
  IntermediateState sa = sin(angle);

  R(0,0) = 1;
               R(1,1) = ca;  R(1,2) = -sa;
               R(2,1) = sa;  R(2,2) =  ca;

  return R;
}


Expression Ry(const Expression &angle) {

    IntermediateState R  = zeros(3,3);
    IntermediateState ca = cos(angle);
    IntermediateState sa = sin(angle);

    R(0,0) =  ca;                 R(0,2) =  sa;
                   R(1,1) = 1.0;
    R(2,0) = -sa;                 R(2,2) =  ca;

    return R;
}


Expression Rz(const Expression &angle) {

    IntermediateState R  = zeros(3,3);
    IntermediateState ca = cos(angle);
    IntermediateState sa = sin(angle);

    R(0,0) = ca;  R(0,1) = -sa;
    R(1,0) = sa;  R(1,1) =  ca;
                                  R(2,2) = 1.0;

    return R;
}



Expression Ryp(const int quadrant) {
  Expression R=zeros(3,3);
  Expression ca=cosquadrant(quadrant);Expression sa=sinquadrant(quadrant);
  //matrix([cos(alpha),0,sin(alpha)],[0,1,0],[-sin(alpha),0,cos(alpha)])
  R(0,0)=ca;R(0,2)=sa;R(1,1)=0;R(2,0)=-sa;R(2,2)=ca;
  return R;
}


Expression Rzp(const int quadrant) {
  Expression R=zeros(3,3);
  Expression ca=cosquadrant(quadrant);Expression sa=sinquadrant(quadrant);
  //matrix([cos(alpha),0,sin(alpha)],[0,1,0],[-sin(alpha),0,cos(alpha)])
  R(0,0)=ca;R(0,1)=-sa;R(1,0)=sa;R(1,1)=ca;R(2,2)=1;
  return R;
}


Expression TRx(const Expression &angle) {

    IntermediateState T = zeros(4,4);
    IntermediateState R = Rx(angle);

    T(0,0) = R(0,0);  T(0,1) = R(0,1);  T(0,2) = R(0,2);
    T(1,0) = R(1,0);  T(1,1) = R(1,1);  T(1,2) = R(1,2);
    T(2,0) = R(2,0);  T(2,1) = R(2,1);  T(2,2) = R(2,2);

    T(3,3) = 1.0;

    return T;
}



Expression TRy(const Expression &angle) {

    IntermediateState T = zeros(4,4);
    IntermediateState R = Ry(angle);

    T(0,0) = R(0,0);  T(0,1) = R(0,1);  T(0,2) = R(0,2);
    T(1,0) = R(1,0);  T(1,1) = R(1,1);  T(1,2) = R(1,2);
    T(2,0) = R(2,0);  T(2,1) = R(2,1);  T(2,2) = R(2,2);

    T(3,3) = 1.0;

    return T;
}



Expression TRz(const Expression &angle) {

  IntermediateState T = zeros(4,4);
  IntermediateState R = Rz(angle);

  T(0,0) = R(0,0);  T(0,1) = R(0,1);  T(0,2) = R(0,2);
  T(1,0) = R(1,0);  T(1,1) = R(1,1);  T(1,2) = R(1,2);
  T(2,0) = R(2,0);  T(2,1) = R(2,1);  T(2,2) = R(2,2);

  T(3,3) = 1.0;

  return T;
}



Expression TRxp(const int quadrant) {
  Expression T=zeros(4,4);
  T(3,3)=1;
  Expression R=Rxp(quadrant);
  T(0,0)=R(0,0);T(0,1)=R(0,1);T(0,2)=R(0,2);
  T(1,0)=R(1,0);T(1,1)=R(1,1);T(1,2)=R(1,2);
  T(2,0)=R(2,0);T(2,1)=R(2,1);T(2,2)=R(2,2);
  return T;
}

Expression TRyp(const int quadrant) {
  Expression T=zeros(4,4);
  T(3,3)=1;
  Expression R=Ryp(quadrant);
  T(0,0)=R(0,0);T(0,1)=R(0,1);T(0,2)=R(0,2);
  T(1,0)=R(1,0);T(1,1)=R(1,1);T(1,2)=R(1,2);
  T(2,0)=R(2,0);T(2,1)=R(2,1);T(2,2)=R(2,2);
  return T;
}

Expression TRzp(const int quadrant) {
  Expression T=zeros(4,4);
  T(3,3)=1;
  Expression R=Rzp(quadrant);
  T(0,0)=R(0,0);T(0,1)=R(0,1);T(0,2)=R(0,2);
  T(1,0)=R(1,0);T(1,1)=R(1,1);T(1,2)=R(1,2);
  T(2,0)=R(2,0);T(2,1)=R(2,1);T(2,2)=R(2,2);
  return T;
}
Expression TRperm(int a, int b, int c) {
  Expression T=zeros(4,4);
  T(3,3)=1;
  Expression R=Rperm(a,b,c);
  T(0,0)=R(0,0);T(0,1)=R(0,1);T(0,2)=R(0,2);
  T(1,0)=R(1,0);T(1,1)=R(1,1);T(1,2)=R(1,2);
  T(2,0)=R(2,0);T(2,1)=R(2,1);T(2,2)=R(2,2);
  return T;
}


Expression tr( const Expression &x, const Expression &y, const Expression &z ) {

    return translate( x, y, z );
}


Expression translate( const Expression &x, const Expression &y, const Expression &z ) {

    IntermediateState T = zeros(4,4);

    T(0,0) = 1;                            T(0,3) = x;
                 T(1,1) = 1;               T(1,3) = y;
                              T(2,2) = 1;  T(2,3) = z;
                                           T(3,3) = 1;
    return T;
}



// It seems this is impossible:
//Expression::Expression(KinVec &vec ) {
//    copy     (vec.getCoords());
//}


// Expression rotate (AXIS X,const Expression &angle) {
//   switch ( X ) {
//     case AX_X : 
//       return TRx(angle);
//       break;
//     case AX_Y : 
//       return TRy(angle);
//       break;
//     case AX_Z : 
//       return TRz(angle);
//       break;
//   }
//  
// }
