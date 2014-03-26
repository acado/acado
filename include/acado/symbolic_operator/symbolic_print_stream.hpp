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
 *    \file include/acado/symbolic_operator/symbolic_print_stream.hpp
 *    \author Boris Houska, Hans Joachim Ferreau
 */


#ifndef ACADO_TOOLKIT_C_EXPORT_STREAM_HPP
#define ACADO_TOOLKIT_C_EXPORT_STREAM_HPP


#include <acado/symbolic_operator/symbolic_operator_fwd.hpp>


BEGIN_NAMESPACE_ACADO

/**
 *	\brief A stream class that is based on stl::ostream but more powerful.
 *
 *	\ingroup BasicDataStructures
 *
 *	The class CExportStream acts like a standard stream and the
 *      the "operator<<" simply prints into the main stream. However, the
 *      class additionally acts as a struct to carry other print information
 *      through the symbolic tree such as addditional memory requirements
 *      that are needed for printing user linked C functions as well the
 *      names of variables.
 * 
 *	\author Boris Houska
 */

class CExportStream{

public:


    /** Default constructor. */
    CExportStream();
    
    /** Copy constructor. */
    CExportStream( const CExportStream &arg );
    
    /** Assignment operator. */
    CExportStream& operator=( const CExportStream &arg );
    
    /** Default destructor. */
    ~CExportStream();
    
    /** Printing into main stream. */
    inline CExportStream& operator<<( const CExportStream &arg ){
      
           mainStream<<arg.mainStream;
           return *this;
    }
    
    
    
// PUBLIC MEMBERS:
// -------------------------
    
    std::ostream   mainStream;  /** the main stream containing the output  */
    StringMap            name;  /** a map that defines the variable names
                                 *  that are used for printing C-functions */
    
    std::ostream    preStream;  /** function calls that should be printed
                                 *  before the main stream. (needed for printing
                                 *  user defined functions) */
    
    std::ostream forwardStream; /** for printing forward declarations */
    
    std::ostream   allocStream; /**  */
    
    std::string     realString; /** Definition of real type */
    bool        allocateMemory; /** Whether memory should be allocated */
    bool          staticMemory; /** Whether memory is static */
};


CLOSE_NAMESPACE_ACADO



#endif



