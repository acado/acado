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


// Note: replace jacobian(a,b)*c by a.ADforward(b,c);

#include <acado/modeling_tools/kinvec.hpp>
#include <math.h>


USING_NAMESPACE_ACADO

#ifndef __MATLAB__
using namespace std;
#endif


KinVec::KinVec(){ }

KinVec::KinVec( const Expression &x   ,
                const Expression &y   ,
                const Expression &z   ,
                bool  type_           ,
                const Frame      &ref_  ){

    type = type_;
    ref  = ref_ ;

    IntermediateState xyz(3);

    xyz(0) = x;
    xyz(1) = y;
    xyz(2) = z;

    Expression nn;
    Initialize( xyz, type_, ref_, nn, nn, nn, nn, nn, 0 );
}


returnValue KinVec::Initialize( const Expression &xyz ,
                                bool  type_           ,
                                const Frame      &ref_,
                                const Expression &J_  ,
                                const Expression &c_  ,
                                const Expression &q_  ,
                                const Expression &dq_ ,
                                const Expression &ddq_,
                                int   order_            ){

    type  = type_;
    v     = xyz  ;
    ref   = ref_ ;
    J     = J_   ;
    c     = c_   ;
    q     = q_   ;
    dq    = dq_  ;
    ddq   = ddq_ ;
    order = order_;

    return SUCCESSFUL_RETURN;
}


returnValue KinVec::copy(const KinVec &vec) {

    type   = vec.type ;
    v      = vec.v    ;
    ref    = vec.ref  ;
    J      = vec.J    ;
    c      = vec.c    ;
    q      = vec.q    ;
    dq     = vec.dq   ;
    ddq    = vec.ddq  ;
    order  = vec.order;

    return SUCCESSFUL_RETURN;
}


KinVec::KinVec(const KinVec &vec)    {

    copy( vec );
}



KinVec::KinVec( const Expression& xyz, bool type_, const Frame& ref_ ){

    Expression nn;
    Initialize( xyz, type_, ref_, nn, nn, nn, nn, nn, 0 );
}


KinVec::KinVec( const Expression & xyz   ,
                bool               type_ ,
                const Frame      & ref_  ,
                const Expression & J_    ,
                const Expression & q_    ,
                const Expression & dq_   ,
                const Expression & ddq_  ,
                int                order_  ) {

    Expression nn;
    Initialize( xyz, type_, ref_, J_, nn, q_, dq_, ddq_, order_ );
}


KinVec::KinVec( const Expression & xyz  ,
                bool               type_,
                const Frame      & ref_ ,
                const Expression & J_   ,
                const Expression & c_   ,
                const Expression & q_   ,
                const Expression & dq_  ,
                const Expression & ddq_ ,
                int                order_  ){

     Initialize( xyz, type_, ref_, J_, c_, q_, dq_, ddq_, order_ );
}


KinVec::KinVec( const Expression & xyz   ,
                bool               type_ ,
                const Frame      & ref_  ,
                const Expression & q_    ,
                const Expression & dq_   ,
                const Expression & ddq_  ,
                int                order_  ){

    Expression nn;
    Initialize( xyz, type_, ref_, nn, nn, q_, dq_, ddq_, order_ );
}


Stream operator<<(Stream &stream, const KinVec &vec){

    return vec.print(stream);
}


Stream KinVec::print(Stream &stream) const{

    stream << "KinVec" << "\n";
    stream << "  Components: " << v <<  "\n";
    stream << "  type: "<< type << "\n";
 // stream << "  Expressed in: " << ref << "\n";
    stream << "  J: " << J  <<  "\n";
    stream << "  c: " << c  <<  "\n";
    stream << "  order: " << order <<  "\n";
    stream << "  q: " << q <<  "\n";
    stream << "  dq: " << dq <<  "\n";

    return stream;
}


Expression der( const Expression &v  ,
                const Expression &q  ,
                const Expression &dq ,
                const Expression &ddq  ){

    //return jacobian(v,q)*dq + jacobian(v,dq)*ddq;  // NO !  SHOULD BE FORWARD DERIVATIVE !!!
							// okay, it's on the todo list
    return v.ADforward(q,dq) + v.ADforward(dq,ddq);
}


Expression KinVec::explicitize(const Expression & ddq_) const {

    if ( q.getDim() != 3 ){ ACADOERROR( RET_DDQ_DIMENSION_MISMATCH );           ASSERT( 1 == 0 ); }
    //if ( abs(order) !=2  ){ ACADOERROR( RET_CAN_ONLY_SOLVE_2ND_ORDER_KINVECS ); ASSERT( 1 == 0 ); }
    if ( !J.getDim()) throw("KinVec jacobian information was lost. Some operators destroy this information. (e.g. cross)");

    //Expression Jc;               // SHOULD BE INTERMEDIATE STATE.
    IntermediateState Jc((uint) 3,ddq_.getDim());       
    IntermediateState cc=c;

    uint i;
    
    std::map<uint,uint> di;
    
    uint j;
    uint count;
    Matrix p=ddq.getDependencyPattern(ddq_);

    for ( j = 0; j < ddq_.getDim(); j++ ){
       for ( i = 0; i < ddq.getDim(); i++ ){
	 if (p(i,j)>0.0) {
	    di[i]=count;
	    count++;
	 }
       }
    }
    
    for ( j = 0; j < ddq.getDim(); j++ ){
      for ( i = 0; i < 3; i++ ){
	if (di.count(j)) {
	  Jc(i,di[j])=J(i,j);
	} else {
	  cc(i)+=J(i,j)*ddq(j);
	}
      }
    }

    printf("test\n");
    Function ff;
    ff << Jc;
    FILE *myfile=fopen("debug.txt","w");
    myfile << ff;
    fclose(myfile);
    Jc.getInverse();
    return -Jc.getInverse()*cc;
}

Expression KinVec::explicitizeJc(std::map<uint,uint> &di) const {
    std::map<uint,uint> ri;
    ri[0]=0;ri[1]=1;ri[2]=2;
    return explicitizeJc(di,ri);
}

Expression KinVec::explicitizeJc(std::map<uint,uint> &di,std::map<uint,uint> &ri) const {

    //if ( abs(order) !=2  ){ ACADOERROR( RET_CAN_ONLY_SOLVE_2ND_ORDER_KINVECS ); ASSERT( 1 == 0 ); }
    if ( !J.getDim()) throw("KinVec jacobian information was lost. Some operators destroy this information. (e.g. cross)");

    //Expression Jc;               // SHOULD BE INTERMEDIATE STATE.
    IntermediateState Jc((uint) ri.size(),(uint) di.size());       
    IntermediateState cc=zeros( (uint) ri.size(),1);

    for (uint i = 0; i < 3; i++ ){
	  if (ri.count(i)) {
	    cc(ri[i])=c(i);
	  }
    }
    
    for (uint j = 0; j < ddq.getDim(); j++ ){
      for (uint i = 0; i < 3; i++ ){
	if (di.count(j)) {
	  if (ri.count(i)) {
	    Jc(ri[i],di[j])=J(i,j);
	  }
	} else {
	  if (ri.count(i)) {
	    cc(ri[i])+=J(i,j)*ddq(j);
	  }
	}
      }
    }

    return Jc;
}

Expression KinVec::explicitize(std::map<uint,uint> &di) const {
    std::map<uint,uint> ri;
    ri[0]=0;ri[1]=1;ri[2]=2;
    return explicitize(di,ri);
}

Expression KinVec::explicitize(std::map<uint,uint> &di,std::map<uint,uint> &ri) const {

    //if ( abs(order) !=2  ){ ACADOERROR( RET_CAN_ONLY_SOLVE_2ND_ORDER_KINVECS ); ASSERT( 1 == 0 ); }
    if ( !J.getDim()) throw("KinVec jacobian information was lost. Some operators destroy this information. (e.g. cross)");

    //Expression Jc;               // SHOULD BE INTERMEDIATE STATE.
    IntermediateState Jc((uint) ri.size(),(uint) di.size());       
    IntermediateState cc=zeros((uint) ri.size(),1);

    for (uint i = 0; i < 3; i++ ){
	  if (ri.count(i)) {
	    cc(ri[i])=c(i);
	  }
    }
    
    for (uint j = 0; j < ddq.getDim(); j++ ){
      for (uint i = 0; i < 3; i++ ){
	if (di.count(j)) {
	  if (ri.count(i)) {
	    Jc(ri[i],di[j])=J(i,j);
	  }
	} else {
	  if (ri.count(i)) {
	    cc(ri[i])+=J(i,j)*ddq(j);
	  }
	}
      }
    }

    return -Jc.getInverse()*cc;
}


KinVec KinVec::expressedIn(const Frame& f) const {

    return KinVec( ref.chain( v, f, type),
                   type,
                   f,
                   ref.chain( J, f, 0),
                   ref.chain( c, f, 0),
                   q,
                   dq,
                   ddq,
                   order );
}

void KinVec::expressIn(const Frame& f) {
    J = ref.chain( J, f, 0);
    c = ref.chain( c, f, 0);
    v = ref.chain( v, f, type);
}


Expression KinVec::getCoords() const{

  return v;
}

Expression KinVec::getCoords(const Frame& ref_) const{
  return expressedIn(ref_).v;
}

const Expression& KinVec::getQ() const{
  if (q.getDim()) return q;
  return ref.getQ();
}

const Expression& KinVec::getDQ() const{
  if (dq.getDim()) return dq;
  return ref.getDQ();
}

const Expression& KinVec::getDDQ() const{
  if (ddq.getDim()) return ddq;
  return ref.getDDQ();
}

returnValue KinVec::setDDQ(const Expression& ddq_){
  ddq=ddq_;
  v=J*ddq+c;
  return SUCCESSFUL_RETURN;
}

KinVec pos( const Frame& f, const Frame& ei ){

    IntermediateState p = zeros(3,1);

    return KinVec( f.chain( p, ei, 1 ), 1, ei );
}

KinVec vel( const Frame& f ,
            const Frame& wt,
            const Frame& ei,
            const Expression & q  ,
            const Expression & dq ,
            const Expression & ddq  ){

   IntermediateState p = zeros(3,1);

   p = f.chain(p,wt,1);

   IntermediateState J = jacobian( p, q );

   J = wt.chain( J, ei, 0 );

   return KinVec( J*dq, 0, ei, J, q, dq, ddq,1 );
}


KinVec acc( const Frame      & f  ,
            const Frame      & wt ,
            const Frame      & ei ,
            const Expression & q  ,
            const Expression & dq ,
            const Expression & ddq  ){

   IntermediateState p = zeros(3,1);

   p=f.chain(p,wt,1);

   IntermediateState J     = jacobian( p   ,q); // this thing is segfaulting
   //IntermediateState acnst = jacobian( J*dq,q)*dq;    // NO, SHOULD BE FORWARD DIFFERENTIATION !
   IntermediateState acnst = (J*dq).ADforward(q,dq);    // NO, SHOULD BE FORWARD DIFFERENTIATION !

   J     = wt.chain( J    , ei, 0 );
   acnst = wt.chain( acnst, ei, 0 );

   return KinVec( J*ddq + acnst, 0, ei, J, acnst, q, dq, ddq, 2 );
}


KinVec rotVel( const Frame      & f  ,
               const Frame      & wt ,
               const Frame      & ei ,
               const Expression & q  ,
               const Expression & dq ,
               const Expression & ddq  ){

   IntermediateState R = eye(3);

   R=f.chain(R,wt,0);

   //Expression J;
   //J << R.getRow(1)*jacobian(R.getRow(2).transpose(),q);
   //J << R.getRow(2)*jacobian(R.getRow(0).transpose(),q);
   //J << R.getRow(0)*jacobian(R.getRow(1).transpose(),q);

   IntermediateState J1 = jacobian(R.getRow(2).transpose(),q);
   IntermediateState J2 = jacobian(R.getRow(0).transpose(),q);
   IntermediateState J3 = jacobian(R.getRow(1).transpose(),q);

   Expression J;
   J.appendRows(R.getRow(1)*J1); 
   J.appendRows(R.getRow(2)*J2);
   J.appendRows(R.getRow(0)*J3);

   J = wt.chain(J,ei,0);

   return KinVec( J*dq, 0, ei, J, q, dq, ddq, -1 );
}


KinVec rotAcc( const Frame      & f  ,
               const Frame      & wt ,
               const Frame      & ei ,
               const Expression & q  ,
               const Expression & dq ,
               const Expression & ddq  ){

   IntermediateState R = eye(3);
   R = f.chain(R,wt,0);

   IntermediateState J1=jacobian(R.getRow(2).transpose(),q);
   IntermediateState J2=jacobian(R.getRow(0).transpose(),q);
   IntermediateState J3=jacobian(R.getRow(1).transpose(),q);

   Expression J;
   J.appendRows(R.getRow(1)*J1);
   J.appendRows(R.getRow(2)*J2);
   J.appendRows(R.getRow(0)*J3);

   J=wt.chain(J,ei,0);

   //IntermediateState tempJ = jacobian( J*dq, q );
   //IntermediateState acnst = tempJ*dq;

   //IntermediateState tempJ = jacobian( J*dq, q );
   IntermediateState acnst = (J*dq).ADforward(q,dq);
   
   return KinVec( J*ddq+acnst, 0, ei, J, acnst, q, dq, ddq, -2 );
}


KinVec vel(const Frame& f, const Frame& wt, const Frame& ei) {return vel(f,wt,ei,f.getQ(),f.getDQ(),f.getDDQ());}
KinVec acc(const Frame& f, const Frame& wt, const Frame& ei) {return acc(f,wt,ei,f.getQ(),f.getDQ(),f.getDDQ());}
KinVec rotVel(const Frame& f, const Frame& wt, const Frame& ei) {return rotVel(f,wt,ei,f.getQ(),f.getDQ(),f.getDDQ());}
KinVec rotAcc(const Frame& f, const Frame& wt, const Frame& ei) {return rotAcc(f,wt,ei,f.getQ(),f.getDQ(),f.getDDQ());}

KinVec ex(const Frame& f) {return KinVec(1,0,0,0,f);}
KinVec ey(const Frame& f) {return KinVec(0,1,0,0,f);}
KinVec ez(const Frame& f) {return KinVec(0,0,1,0,f);}
KinVec ex(const Frame& f,const Frame& ei) {return ex(f).expressedIn(ei);}
KinVec ey(const Frame& f,const Frame& ei) {return ey(f).expressedIn(ei);}
KinVec ez(const Frame& f,const Frame& ei) {return ez(f).expressedIn(ei);}

KinVec cross(const KinVec &a,const KinVec &b) {
 KinVec A=a; 
 KinVec B=b; 
 
 Frame c=expressCommon(A,B);
 if (A.type && B.type) throw("Cannot multiply 2 position vectors/ 1-vectors");
 //if (!A.q.isEqual(B.q)) throw("Vectors were not constructed with same states (q)");
 //if (!A.dq.isEqual(B.dq)) throw("Vectors were not constructed with same states (dq)");

 return KinVec(A.v(1)*B.v(2)-B.v(1)*A.v(2),B.v(0)*A.v(2)-A.v(0)*B.v(2),A.v(0)*B.v(1)-B.v(0)*A.v(1),0,c);
}

// Operator overloads

Frame expressCommon(KinVec &a,KinVec &b) {
   Frame c=a.ref.getCommonFrame(b.ref); 
   a=a.expressedIn(c);
   b=b.expressedIn(c);
   return c;
}

KinVec KinVec::operator+(const KinVec &b) {
 KinVec A=*this; 
 KinVec B=b; 

 Expression q_,dq_,ddq_;
 if (!A.q.getDim() && B.q.getDim()) {q_=B.q;dq_=B.dq;ddq_=B.ddq;};
 if (A.q.getDim() && !B.q.getDim()) {q_=A.q;dq_=A.dq;ddq_=A.ddq;};
 if (A.q.getDim() && B.q.getDim()) {
  //if (!A.dq.isEqual(B.dq)) throw("Vectors were not constructed with same states (q,dq,ddq)");
  q_=A.q;dq_=A.dq;ddq_=A.ddq;
 }
 Frame c_=expressCommon(A,B);
 if (A.type && B.type) throw("Cannot add 2 position vectors/ 1-vectors");
 if (A.order && B.order && A.order != B.order) throw("Cannot add vectors of different order");
 IntermediateState J_,cn;
 if (A.J.getDim() && B.J.getDim()) {
   J_=A.J+B.J;
    if (A.c.getDim() && B.c.getDim()) {
    	cn=A.c+B.c;
    } else if (A.c.getDim()) {
	cn=A.c;
    } else {
	cn=B.c;
    }
 } else if (A.J.getDim()) {
   J_=A.J;
   if (A.c.getDim()) cn=A.c + B.v;
   if (!A.c.getDim()) cn=B.v;
 } else {
   J_=B.J;
   if (B.c.getDim()) cn=B.c + A.v;
   if (!B.c.getDim()) cn=A.v;
 }
 IntermediateState sum=A.v+B.v;
 return KinVec(sum,A.type || B.type,c_,J_,cn,q_,dq_,ddq_,A.order);
}

KinVec& KinVec::operator+=(const KinVec &b) {
 return *this = *this + b;
}

KinVec KinVec::operator-(const KinVec &b) {
 return *this+(-b);
}

KinVec KinVec::operator-() const{
  return KinVec(-v,type,ref,-J,-c,q,dq,ddq,order);
}

KinVec& KinVec::operator-=(const KinVec &b) {
 return *this = *this - b;
}

Expression norm(const KinVec &v) {return sqrt(v.v(0)*v.v(0)+v.v(1)*v.v(1)+v.v(2)*v.v(2));}
    
KinVec operator*(const KinVec &a,const Expression &b) {return KinVec(a.v*b,a.type,a.ref,a.J*b,a.c*b,a.q,a.dq,a.ddq,a.order);}
KinVec operator*(const Expression &a,const KinVec &b) {
  //Expression J,c;
  //if (b.J.getDim()) J = a*b.J;
  //if (b.c.getDim()) c = a*b.c;
  return KinVec(a*b.v,b.type,b.ref,a*b.J,a*b.c,b.q,b.dq,b.ddq,b.order);
}
KinVec operator/(const KinVec &a,const Expression &b) {return KinVec(a.v/b,a.type,a.ref,a.J/b,a.c/b,a.q,a.dq,a.ddq,a.order);}






Expression operator*(const KinVec &a,const KinVec &b) {
 KinVec A=a; 
 KinVec B=b; 
 
 Frame c=expressCommon(A,B);
 if (A.type && B.type) throw("Cannot multiply 2 position vectors/ 1-vectors");
 if (A.order != B.order ) throw("Cannot multiply vectors of different order");
 //if (!A.q.isEqual(B.q)) throw("Vectors were not constructed with same states (q)");
 //if (!A.dq.isEqual(B.dq)) throw("Vectors were not constructed with same states (dq)");

 return A.v(0)*B.v(0)+A.v(1)*B.v(1)+A.v(2)*B.v(2);
}

Operator& KinVec::operator[](int i) {
   Expression t1;
   Expression t2;
   J=t1;
   c=t2;
   return v(i);
}

Expression KinVec::operator[](int i) const{
  return v(i);
}

Operator& KinVec::operator()(int i) {
   Expression t1;
   Expression t2;
   J=t1;
   c=t2;
   return v(i);
}

Expression KinVec::operator()(int i) const{
  return v(i);
}


KinVec KinVec::der() {
  if ( abs(order)>= 3 ) throw("Cannot take derivatives of accelerations");

  //Expression q_=getQ();
  //Expression dq_=getDQ();
  //Expression ddq_=getDDQ();
  //Expression J_=jacobian(v,dq_);
  //Expression c_=jacobian(v,q_)*dq_;
 // Expression c_=v.ADforward(q_,dq_);
  //return KinVec(J_*ddq_+c_,0,ref,J_,c_,q_,dq_,ddq_,copysign(abs(order)+1,order));
  
  Expression J_=jacobian(v,getDQ());
  //Expression c_=jacobian(v,q_)*dq_;
  Expression c_=v.ADforward(getQ(),getDQ());
  return KinVec(); // does not compile!! J_*getDDQ()+c_,0,ref,J_,c_,getQ(),getDQ(),getDDQ(),copysign(abs(order)+1,order));
}
