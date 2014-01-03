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
  *    \file   examples/ocp/rocket_with_templates_ocp.hpp
  *    \author Joel Andersson
  *    \date   2009
  */

#ifndef ROCKET_WITH_TEMPLATES_HPP
#define ROCKET_WITH_TEMPLATES_HPP

// problem dimensions
const int nxd = 3; // number of differential states
const int nu  = 1; // number of controls

// macros for variables
#define S xd[0]
#define V xd[1]
#define M xd[2]

#define U u[0]

// Names (e.g. for plotting)
const char* xd_names[nxd] = {"DifferentialState s","DifferentialState v","DifferentialState m"};
const char* u_names[nu] = {"Control u"};

// parameters
const double t_start =  0.0;
const double t_end   = 10.0;

const bool   xd_0_fixed[nxd] = { true , true , true };
const bool   xd_f_fixed[nxd] = { true , true , false};

const double xd_0[nxd]       = {  0.0 ,  0.0 ,  1.0 };
const double xd_f[nxd]       = { 10.0 ,  0.0 ,  0.0 };

const bool   xd_has_lb[nxd]  = { false, true , false};
const bool   xd_has_ub[nxd]  = { false, true,  false};

const double xd_lb[nxd]       = {  0.0 , -0.01,  0.0 };
const double xd_ub[nxd]       = {  0.0 ,   1.3,  0.0 };


// evaluate objective function
template <class DifferentialStateType, class ControlType, class ReturnType>
void eval_F(const DifferentialStateType *xd, 
	    const ControlType *u,
	    ReturnType &lfun){
  
  lfun = U*U;
  
}

// evaluate right hand side of the ODE
template <class DifferentialStateType, class ControlType, class ReturnType>
void eval_G(const DifferentialStateType *xd, 
	    const ControlType *u, 
	    ReturnType *rhs){
  
  rhs[0] = V;
  rhs[1] = (U-0.02*V*V)/M;
  rhs[2] = -0.01*U*U;
  
} 


#undef U
#undef V
#undef S
#undef M


#endif // ROCKET_WITH_TEMPLATES_HPP

