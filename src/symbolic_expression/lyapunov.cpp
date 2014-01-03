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
 *    \file src/symbolic_expression/parameter.cpp
 *    \author Julia Sternberg, Boris Houska, Hans Joachim Ferreau
 *    \date 2010
 */

#include <acado/symbolic_expression/lyapunov.hpp>

BEGIN_NAMESPACE_ACADO


Lyapunov::Lyapunov(){
}

Lyapunov::Lyapunov(const Expression &rhs1_,                                                        const Expression &A_,                                                           const Expression &B_,                                                           const Expression &P_,                                                           const Expression &x1_,                                                          const Expression &u_,                                                           const Expression &p_){

    rhs1=rhs1_;
    A=A_;
    B=B_;
    P=P_;
    x1=x1_;
    u=u_;
    p=p_;
}

Lyapunov::Lyapunov(const Expression &rhs1_,                                                        const Expression &rhs2_,                                                        const Expression &A_,                                                           const Expression &B_,                                                           const Expression &P_,                                                           const Expression &x1_,                                                          const Expression &x2_,                                                          const Expression &u_,                                                           const Expression &p_){

    rhs1=rhs1_;
    rhs2=rhs2_;
    A=A_;
    B=B_;
    P=P_;
    x1=x1_;
    x2=x2_;
    u=u_;
    p=p_;
}

Lyapunov::Lyapunov(const Expression &rhs1_,                                                        const Expression &rhs2_,                                                        const Expression &A_,                                                           const Expression &B_,                                                           const Expression &P_,                                                           const Expression &x1_,                                                          const Expression &x2_,                                                          const Expression &u_,                                                           const Expression &p_,                                                           const Expression &useed_,                                                       const Expression &pseed_,                                                       const Expression &Yx1_,                                                         const Expression &Yx2_,                                                         const Expression &YP_){

    rhs1=rhs1_;
    rhs2=rhs2_;
    A=A_;
    B=B_;
    P=P_;
    x1=x1_;
    x2=x2_;
    u=u_;
    p=p_;
    useed=useed_;
    pseed=pseed_;
    Yx1=Yx1_;
    Yx2=Yx2_;
    YP=YP_;
}


Lyapunov::Lyapunov(const Expression &rhs1_,                                                        const Expression &A_,                                                           const Expression &B_,                                                           const Expression &P_,                                                           const Expression &x1_,                                                          const Expression &u_,                                                           const Expression &p_,                                                           const Expression &w_){

    rhs1=rhs1_;
    A=A_;
    B=B_;
    P=P_;
    x1=x1_;
    u=u_;
    p=p_;
    w=w_;
}

Lyapunov::Lyapunov(const Expression &rhs1_,                                                        const Expression &rhs2_,                                                        const Expression &A_,                                                           const Expression &B_,                                                           const Expression &P_,                                                           const Expression &x1_,                                                          const Expression &x2_,                                                          const Expression &u_,                                                           const Expression &p_,                                                           const Expression &w_){

    rhs1=rhs1_;
    rhs2=rhs2_;
    A=A_;
    B=B_;
    P=P_;
    x1=x1_;
    x2=x2_;
    u=u_;
    p=p_;
    w=w_;
}

Lyapunov::Lyapunov(const Expression &rhs1_,                                                        const Expression &rhs2_,                                                        const Expression &A_,                                                           const Expression &B_,                                                           const Expression &P_,                                                           const Expression &x1_,                                                          const Expression &x2_,                                                          const Expression &u_,                                                           const Expression &p_,                                                           const Expression &w_,                                                           const Expression &useed_,                                                       const Expression &pseed_,                                                       const Expression &Yx1_,                                                         const Expression &Yx2_,                                                         const Expression &YP_){

    rhs1=rhs1_;
    rhs2=rhs2_;
    A=A_;
    B=B_;
    P=P_;
    x1=x1_;
    x2=x2_;
    u=u_;
    p=p_;
    w=w_;
    useed=useed_;
    pseed=pseed_;
    Yx1=Yx1_;
    Yx2=Yx2_;
    YP=YP_;
}


Lyapunov::Lyapunov( const Lyapunov &arg ){


    rhs1=arg.rhs1;
    rhs2=arg.rhs2;
    A=arg.A;
    B=arg.B;
    P=arg.P;
    x1=arg.x1;
    x2=arg.x2;
    u=arg.u;
    p=arg.p;
    w=arg.w;
    useed=arg.useed;
    pseed=arg.pseed;
    Yx1=arg.Yx1;
    Yx2=arg.Yx2;
    YP=arg.YP;
}


Lyapunov::~Lyapunov(){ }


Lyapunov& Lyapunov::operator=( const Lyapunov &arg ){

    if( this != &arg ){

        rhs1=arg.rhs1;
        rhs2=arg.rhs2;
        A=arg.A;
        B=arg.B;
        P=arg.P;
        x1=arg.x1;
        x2=arg.x2;
        u=arg.u;
        p=arg.p;
        w=arg.w;
        useed=arg.useed;
        pseed=arg.pseed;
        Yx1=arg.Yx1;
        Yx2=arg.Yx2;
        YP=arg.YP;
    }
    return *this;
}

BooleanType Lyapunov::isEmpty() const{
  if ((P).getDim() == 0)  return BT_TRUE;
  return BT_FALSE;
}


//}


CLOSE_NAMESPACE_ACADO

// end of file.
