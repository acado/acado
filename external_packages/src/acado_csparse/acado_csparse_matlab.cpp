/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC) under
 *    supervision of Moritz Diehl. All rights reserved.
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
 *    \file   external_packages/src/acado_csparse/acado_csparse.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2009
 */


#ifndef ACADO_CMAKE_BUILD

// THIS FILE IS A TEMPORY PATCH FOR THE  MATLAB INTERFACE

#include "acado_csparse/acado_csparse.hpp"


BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//


ACADOcsparse::ACADOcsparse( ){


}


ACADOcsparse::ACADOcsparse( const ACADOcsparse &arg ){


}


ACADOcsparse::~ACADOcsparse( ){

}


ACADOcsparse* ACADOcsparse::clone() const{

	return new ACADOcsparse(*this);
}



returnValue ACADOcsparse::solve( double *b ){

	return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}


returnValue ACADOcsparse::setDimension( const int &n ){

	return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}



returnValue ACADOcsparse::setNumberOfEntries( const int &nDense_ ){

	return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}



returnValue ACADOcsparse::setIndices( const int *rowIdx_, const int *colIdx_  ){

	return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}



returnValue ACADOcsparse::setMatrix( double *A_ ){

	return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}

returnValue ACADOcsparse::solveTranspose( double *b ){

	return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}


returnValue ACADOcsparse::getX( double *x_ ){

	return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}


returnValue ACADOcsparse::setTolerance( double TOL_ ){

	return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}


returnValue ACADOcsparse::setPrintLevel( PrintLevel printLevel_ ){

	return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}



CLOSE_NAMESPACE_ACADO


/*
 *   end of file
 */

#endif // ACADO_CMAKE_BUILD
