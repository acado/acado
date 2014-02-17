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
 *    \file   src/set_arithmetics/interval.cpp
 *    \author Boris Houska, Mario Villanueva, Benoit Chachuat
 *    \date   2013
 */


#include <acado/set_arithmetics/interval.hpp>


BEGIN_NAMESPACE_ACADO



BooleanType Interval::isCompact() const{
 
	BooleanType result = BT_TRUE;
	
	if( acadoIsNaN   ( _l ) == BT_TRUE  ) result = BT_FALSE;
	if( acadoIsFinite( _l ) == BT_FALSE ) result = BT_FALSE;
	if( acadoIsNaN   ( _u ) == BT_TRUE  ) result = BT_FALSE;
	if( acadoIsFinite( _u ) == BT_FALSE ) result = BT_FALSE;
	
	return result;
}



double Interval::mid( const double convRel, const double concRel, const double valCut, int &indexMid ) const{

  int ICONV = 0;
  int ICUT  = 1;
  int ICONC = 2;

  if( indexMid < 0 ){
    if ( valCut < convRel ){
      indexMid = ICONV;
      return convRel;
    }
    else
    if ( valCut < concRel ){
      indexMid = ICUT;
      return valCut;
    }
    else{
      indexMid = ICONC;
      return concRel;
    }
  }

  if ( indexMid == ICONV )
    return convRel;

  else
  if ( indexMid == ICUT )
    return valCut;

  else
    return concRel;
}


CLOSE_NAMESPACE_ACADO


/*
 *	end of file
 */
