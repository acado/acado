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
 *  \brief Kinetics toolbox
 *
 *  \author Joris Gillis, Boris Houska, Hans Joachim Ferreau
 *
 */


#ifndef ACADO_TOOLKIT_MODELING_TOOLS_KINETICS_HPP
#define ACADO_TOOLKIT_MODELING_TOOLS_KINETICS_HPP


#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/modeling_tools/frame.hpp>
#include <acado/modeling_tools/kinvec.hpp>

/** \brief creates a 4x4 transformation matrix for a translation
* 
*   \param x The amount to shift over the x-axis
*   \param y The amount to shift over the y-axis
*   \param z The amount to shift over the z-axis
*
* \code
*  1 | 0 | 0 | x 
*  0 | 0 | 0 | y
*  0 | 0 | 1 | z
*  0 | 0 | 0 | 1
* \endcode
*/
REFER_NAMESPACE_ACADO Expression translate(const REFER_NAMESPACE_ACADO Expression &x,const REFER_NAMESPACE_ACADO  Expression &y,const REFER_NAMESPACE_ACADO Expression &z);
/** \brief shorter notation for translate()
*/
REFER_NAMESPACE_ACADO Expression tr(const REFER_NAMESPACE_ACADO Expression &x,const REFER_NAMESPACE_ACADO Expression &y,const REFER_NAMESPACE_ACADO Expression &z);

//REFER_NAMESPACE_ACADO Expression rotate(AXIS X,const REFER_NAMESPACE_ACADO Expression &angle); // Rotate about an axis

/** \brief creates a 3x3 rotation matrix for a rotation about the x-axis
* 
*   creates 3x3 rotation matrix for a rotation about the x-axis
*   \param angle angle for which to rotate in radians
*                Looking from the origin towards the endpoint of the x-defining unit vector, a positive rotation is clockwise
*
*   ca = cos(angle)
*   sa = sin(angle)
*
* \code
*  1 | 0  | 0  
*  0 | ca | -sa
*  0 | sa | ca  
* \endcode
*/
REFER_NAMESPACE_ACADO Expression Rx(const REFER_NAMESPACE_ACADO Expression &angle);
/** \brief creates a 3x3 rotation matrix for a rotation about the y-axis
* 
*   creates 3x3 rotation matrix for a rotation about the y-axis
*   \param angle angle for which to rotate in radians
*                Looking from the origin towards the endpoint of the y-defining unit vector, a positive rotation is clockwise
*
*   ca = cos(angle)
*   sa = sin(angle)
*
* \code
*  ca | -sa | 0 | 0 
*  sa | ca  | 0 | 0
*  0  | 0   | 0 | 0 
* \endcode
*/
REFER_NAMESPACE_ACADO Expression Ry(const REFER_NAMESPACE_ACADO Expression &angle);
/** \brief creates a 3x3 rotation matrix for a rotation about the z-axis
* 
*   creates 3x3 rotation matrix for a rotation about the z-axis
*   \param angle angle for which to rotate in radians
*                Looking from the origin towards the endpoint of the z-defining unit vector, a positive rotation is clockwise
*
*   ca = cos(angle)
*   sa = sin(angle)
*
* \code
*  1 | 0  | 0  
*  0 | ca | -sa
*  0 | sa | ca  
* \endcode
*/
REFER_NAMESPACE_ACADO Expression Rz(const REFER_NAMESPACE_ACADO Expression &angle);
/**
* \brief creates a 3x3 rotation matrix for a rotation about the x-axis with an angle that is a multiple of PI/2
* \param quadrant rotate over PI/2*quadrant
*/
REFER_NAMESPACE_ACADO Expression Rxp(const int quadrant);
/**
* \brief creates a 3x3 rotation matrix for a rotation about the y-axis with an angle that is a multiple of PI/2
* \param quadrant rotate over PI/2*quadrant
*/
REFER_NAMESPACE_ACADO Expression Ryp(const int quadrant);
/**
* \brief creates a 3x3 rotation matrix for a rotation about the z-axis with an angle that is a multiple of PI/2
* \param quadrant rotate over PI/2*quadrant
*/
REFER_NAMESPACE_ACADO Expression Rzp(const int quadrant);
REFER_NAMESPACE_ACADO Expression TRx(const REFER_NAMESPACE_ACADO Expression &angle);
REFER_NAMESPACE_ACADO Expression TRy(const REFER_NAMESPACE_ACADO Expression &angle);
REFER_NAMESPACE_ACADO Expression TRz(const REFER_NAMESPACE_ACADO Expression &angle);
REFER_NAMESPACE_ACADO Expression TRxp(const int quadrant); // Rotate multiple of PI/2
REFER_NAMESPACE_ACADO Expression TRyp(const int quadrant);
REFER_NAMESPACE_ACADO Expression TRzp(const int quadrant);

/**
*  \brief Make a 3x3 rotation matrix that expresses a permutation of the axes
*  
* Make a 3x3 rotation matrix that expresses a permutation if the axes.
*
* Axes x,y,z are labeled as integers 1,2,3. A minus sign indicates a reversed direction
*
* \param a The new 1-axis is the old a-axis
* \param a The new 2-axis is the old b-axis
* \param a The new 3-axis is the old c-axis
*
*  The following example expresses a mirror operation around the z=0 plane:
* \code
*  Expression R=R(1,2,-3);
* \endcode
* Note that this would shift handedness of the frame. Not a good idea in mechanics. Better stick to conventional right-handed frames.
*/
REFER_NAMESPACE_ACADO Expression Rperm(int a,int b,int c);
/**
*  \brief Make a 4x4 transformation matrix that expresses a permutation of the axes
*
*  \see Rperm(int a,int b,int c)
*/
REFER_NAMESPACE_ACADO Expression TRperm(int a,int b,int c);

// BEGIN_NAMESPACE_ACADO
//class Expression{
//  Expression(KinVec &vec);
//}
// END_NAMESPACE_ACADO
#endif // ACADO_TOOLKIT_MODELING_TOOLS_KINETICS_HPP
