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
 *  \brief Internal class for representing nodes in a transformation chain
 *
 *  \author Joris Gillis, Boris Houska, Hans Joachim Ferreau
 *
 */

#ifndef ACADO_TOOLKIT_MODELING_TOOLS_FRAME_NODE_HPP
#define ACADO_TOOLKIT_MODELING_TOOLS_FRAME_NODE_HPP

#include <acado/modeling_tools/frame.hpp>


BEGIN_NAMESPACE_ACADO


class Frame;


/** \brief Internal class to make Frame trees reference-safe
 */


class FrameNode{

    friend class Frame;

    public:


    explicit FrameNode( const String     & name,
                        const Expression &    q,
                        const Expression &   dq,
                        const Expression &  ddq);


    FrameNode( const String     &name,
               const Frame      &ref ,
               const Expression &  T   );


    ~FrameNode();


    Stream print(Stream &stream) const;


    protected:

        String      name;
        int        count;
        Frame        ref;
        Expression  time;
        Expression     q;
        Expression    dq;
        Expression   ddq;
        Expression     R;  /**< the 3x3 rotation matrix */
        Expression     p;  /**< position vector         */

};

CLOSE_NAMESPACE_ACADO

#endif //ACADO_TOOLKIT_MODELING_FRAME_NODE_HPP

