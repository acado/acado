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
 *    \file src/modeling_tools/modeling_tools.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */


#include <acado/utils/acado_utils.hpp>
#include <acado/modeling_tools/modeling_tools.hpp>


USING_NAMESPACE_ACADO




returnValue LagrangianFormalism(  DifferentialEquation &f  ,
                                  const Expression     &L  ,
                                  const Expression &q  ,
                                  const Expression &dq   ){

    f << dot(q) - dq;
    f << forwardDerivative( backwardDerivative(L,dq) ,  q, dq      )
         + forwardDerivative( backwardDerivative(L,dq) , dq, dot(dq) )
         - backwardDerivative(L,q);

    return SUCCESSFUL_RETURN;
}


returnValue LagrangianFormalism(  DifferentialEquation &f  ,
                                  const Expression     &L  ,
                                  const Expression &Q  ,
                                  const Expression &q  ,
                                  const Expression &dq ){

    f << dot(q) - dq;
    f << forwardDerivative( backwardDerivative(L,dq) ,  q, dq      )
         + forwardDerivative( backwardDerivative(L,dq) , dq, dot(dq) )
         - backwardDerivative(L,q) - Q;

    return SUCCESSFUL_RETURN;
}


returnValue LagrangianFormalism(  DifferentialEquation &f  ,
                                  Expression           &CF ,
                                  const Expression     &L  ,
                                  const Expression     &Q  ,
                                  const Expression     &q  ,
                                  const Expression     &dq ,
                                  const Expression     &z  ,
                                  const Expression     &dz   ){

//     Expression tmp1 = (  getForwardDerivative( getBackwardDerivative(L,dq) ,  q, dq      )
//                            + getForwardDerivative( getBackwardDerivative(L,dq) , dq, dot(dq) )
//                            - getBackwardDerivative(L,q) - Q).clone();
// 
//     Expression tmp2 = (getBackwardDerivative(L,z)).clone();
// 
//     int run1;
//     int idx;
//     int nz = z.getDim();
// 
//     for( run1 = 0; run1 < nz; run1++ ){
//         idx = z.element[run1]->getVariableIndex();
//         tmp1->substitute( idx, DoubleConstant( 0.0, NE_ZERO ) );
//         tmp2->substitute( idx, DoubleConstant( 0.0, NE_ZERO ) );
//         idx = dz.element[run1]->getVariableIndex();
//         tmp1->substitute( idx, DoubleConstant( 0.0, NE_ZERO ) );
//         tmp2->substitute( idx, DoubleConstant( 0.0, NE_ZERO ) );
//     }
// 
//     f << dot(q) - dq;
//     f << *tmp1;
// 
//     for( run1 = 0; run1 < nz; run1++ ){
//         if( CF.element[run1] != 0 ) delete CF.element[run1];
//         CF.element[run1] = (tmp2.element[run1]).clone();
//     }
    return SUCCESSFUL_RETURN;
}


returnValue HamiltonianFormalism( DifferentialEquation &f ,
                                  const Expression     &H ,
                                  const Expression &Q ,
                                  const Expression &p ,
                                  const Expression &q   ){

    f << dot(q) - backwardDerivative(H,p);
    f << dot(p) + backwardDerivative(H,q) - Q;

    return SUCCESSFUL_RETURN;
}



returnValue NewtonEulerFormalism( DifferentialEquation &f     ,
                                  const Expression &R     ,
                                  const Expression &g     ,
                                  const Expression &x     ,
                                  const Expression &v     ,
                                  const Expression &a     ,
                                  const Expression &lambda  ){

    TIME t;

    f << dot(x) - v;
    f << dot(v) - a;

    f.addAlgebraic(   R - backwardDerivative(g,x,lambda) );

    f.addAlgebraic(   forwardDerivative(forwardDerivative(g,t),t)
                    + forwardDerivative( g, x, a )
                    + forwardDerivative( forwardDerivative( g, x, v ), x, v ) );

    return SUCCESSFUL_RETURN;
}



// end of file.
