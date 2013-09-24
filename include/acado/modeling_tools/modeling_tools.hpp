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
 *    \file include/acado/modeling_tools/modeling_tools.hpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */


#ifndef ACADO_TOOLKIT_MODELING_TOOLS_HPP
#define ACADO_TOOLKIT_MODELING_TOOLS_HPP

#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/function/function.hpp>


/**
 *  \author Boris Houska, Hans Joachim Ferreau
 *
 */



/** An Implementation of the Lagrangian Formalism         \n
 *                                                        \n
 *  \param  f    the differential equation to be computed \n
 *  \param  L    the Lagrangian function  L = T-V         \n
 *  \param  q    the generalized coordinates              \n
 *  \param dq    the generalized velocities               \n
 *                                                        \n
 *                                                        \n
 *  \return The equations of motion.                      \n
 */


REFER_NAMESPACE_ACADO returnValue LagrangianFormalism( REFER_NAMESPACE_ACADO DifferentialEquation &f ,
                                                       const REFER_NAMESPACE_ACADO Expression     &L ,
                                                       const REFER_NAMESPACE_ACADO Expression     &q ,
                                                       const REFER_NAMESPACE_ACADO Expression     &dq  );



/** An Implementation of the Lagrangian Formalism         \n
 *                                                        \n
 *  \param  f    the differential equation to be computed \n
 *  \param  L    the Lagrangian function  L = T-V         \n
 *  \param  Q    the generalized force                    \n
 *  \param  q    the generalized coordinates              \n
 *  \param dq    the generalized velocities               \n
 *                                                        \n
 *                                                        \n
 *  \return The equations of motion.                      \n
 */


REFER_NAMESPACE_ACADO returnValue LagrangianFormalism( REFER_NAMESPACE_ACADO DifferentialEquation &f ,
                                                       const REFER_NAMESPACE_ACADO Expression     &L ,
                                                       const REFER_NAMESPACE_ACADO Expression     &Q ,
                                                       const REFER_NAMESPACE_ACADO Expression     &q ,
                                                       const REFER_NAMESPACE_ACADO Expression     &dq  );



/** An Implementation of the Lagrangian Formalism         \n
 *                                                        \n
 *  \param  f    the differential equation to be computed \n
 *  \param  L    the Lagrangian function  L = T-V         \n
 *  \param  Q    the generalized force                    \n
 *  \param  q    the generalized coordinates              \n
 *  \param dq    the generalized velocities               \n
 *                                                        \n
 *                                                        \n
 *  \return The equations of motion.                      \n
 */


REFER_NAMESPACE_ACADO returnValue LagrangianFormalism( REFER_NAMESPACE_ACADO DifferentialEquation &f ,
                                                       REFER_NAMESPACE_ACADO Expression       &CF,
                                                       const REFER_NAMESPACE_ACADO Expression &L ,
                                                       const REFER_NAMESPACE_ACADO Expression &Q ,
                                                       const REFER_NAMESPACE_ACADO Expression &q ,
                                                       const REFER_NAMESPACE_ACADO Expression &dq,
                                                       const REFER_NAMESPACE_ACADO Expression &z ,
                                                       const REFER_NAMESPACE_ACADO Expression &dz  );



/** An Implementation of the Hamiltonian Formalism        \n
 *                                                        \n
 *  \param  H    the Hamiltonian function  H = T+V        \n
 *  \param  f    the differential equation to be computed \n
 *  \param  Q    the generalized force                    \n
 *  \param  q    the generalized coordinates              \n
 *  \param  p    the generalized moments                  \n
 *                                                        \n
 *                                                        \n
 *  \return The equations of motion.                      \n
 */

REFER_NAMESPACE_ACADO returnValue HamiltonianFormalism( REFER_NAMESPACE_ACADO DifferentialEquation &f ,
                                                        const REFER_NAMESPACE_ACADO Expression     &H ,
                                                        const REFER_NAMESPACE_ACADO Expression     &Q ,
                                                        const REFER_NAMESPACE_ACADO Expression     &p ,
                                                        const REFER_NAMESPACE_ACADO Expression     &q   );




/** An Implementation of the Newton Euler Formalism           \n
 *                                                            \n
 *  \param  f      the differential equation to be computed   \n
 *  \param  R      the force residuum  R := m*a - F           \n
 *  \param  g      the geometric constraints                  \n
 *  \param  x      the system coordinates                     \n
 *  \param  v      the associated velocities                  \n
 *  \param  a      the associated accelerations               \n
 *  \param  lambda the associated constraint forces           \n
 *                                                            \n
 *                                                            \n
 *  \return The equations of motion.                          \n
 */

REFER_NAMESPACE_ACADO returnValue NewtonEulerFormalism( REFER_NAMESPACE_ACADO DifferentialEquation &f     ,
                                                        const REFER_NAMESPACE_ACADO Expression     &R     ,
                                                        const REFER_NAMESPACE_ACADO Expression     &g     ,
                                                        const REFER_NAMESPACE_ACADO Expression     &x     ,
                                                        const REFER_NAMESPACE_ACADO Expression     &v     ,
                                                        const REFER_NAMESPACE_ACADO Expression     &a     ,
                                                        const REFER_NAMESPACE_ACADO Expression     &lambda );


#include <acado/modeling_tools/kinetics_tools.hpp>

#endif  // ACADO_TOOLKIT_MODELING_TOOLS_HPP


/*
 *   end of file
 */
