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
 *    \file   include/utils/acado_stream.hpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2010
 *
 */


#ifndef ACADO_TOOLKIT_ACADO_STREAM_HPP
#define ACADO_TOOLKIT_ACADO_STREAM_HPP


#include <acado/utils/acado_string.hpp>


BEGIN_NAMESPACE_ACADO

/**
 *  \brief Provides a very rudimentary class to handle streams.
 *
 *	\ingroup BasicDataStructures
 *
 *  The class Stream provides a very rudimentary class to handle streams. It \n
 *  can be used as a replacement for the standard iostream in C++.           \n
 *
 *  \author Boris Houska, Hans Joachim Ferreau
 */

class Stream: public String{

    public:

        /** Default constructor */
        Stream();

        /** Constructor taking a char-pointer. */
        Stream( const char *name_ );

        /** Constructor taking a char-pointer. */
        Stream( const double &val_ );

        /** Constructor taking a char-pointer. */
        Stream( const int &val_ );

        /** Copy constructor */
        Stream( const Stream& arg );

        /** Assignment operator */
        Stream& operator=( const Stream& arg );

        /** Assignment operator */
        Stream& operator=( const char *name_ );

        /** Destructor */
        ~Stream();


        /** stream operator */
        Stream& operator<<( const Stream& arg );

        /** stream operator */
        Stream& operator<<( const char* arg );

        /** stream operator */
        Stream& operator<<( const double& arg );

        /** stream operator */
        Stream& operator<<( const int& arg );

        /** stream operator */
        Stream& operator<<( const uint& arg );

        /** stream operator */
        Stream& operator<<( const String& arg );


        /** print operator */
        friend returnValue operator<<( FILE *file, const Stream &stream );


    protected:

};


CLOSE_NAMESPACE_ACADO

#endif  // ACADO_TOOLKIT_ACADO_STREAM_HPP

//end of file
