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
 *    \file include/acado/integrator/radau_IIA5_export.hpp
 *    \author Rien Quirynen
 *    \date 2012
 */


#ifndef ACADO_TOOLKIT_RADAU_IIA5_EXPORT_HPP
#define ACADO_TOOLKIT_RADAU_IIA5_EXPORT_HPP

#include <acado/code_generation/irk_export.hpp>


BEGIN_NAMESPACE_ACADO


/** 
 *	\brief Allows to export a tailored Radau IIA method of order 5 for fast model predictive control.
 *
 *	\ingroup NumericalAlgorithms
 *
 *	The class RadauIIA5Export allows to export a tailored Radau IIA method of order 5
 *	for fast model predictive control.
 *
 *	\author Rien Quirynen
 */
class RadauIIA5Export : public ImplicitRungeKuttaExport
{
    //
    // PUBLIC MEMBER FUNCTIONS:
    //

    public:

		/** Default constructor. 
		 *
		 *	@param[in] _userInteraction		Pointer to corresponding user interface.
		 *	@param[in] _commonHeaderName	Name of common header file to be included.
		 */
        RadauIIA5Export(	UserInteraction* _userInteraction = 0,
							const String& _commonHeaderName = ""
							);

		/** Copy constructor (deep copy).
		 *
		 *	@param[in] arg		Right-hand side object.
		 */
        RadauIIA5Export(	const RadauIIA5Export& arg
							);

        /** Destructor. 
		 */
        virtual ~RadauIIA5Export( );


	protected:

		
		/** This routine initializes the matrices AA, bb and cc which
		 * 	form the Butcher Tableau. */
		virtual returnValue initializeButcherTableau();

    protected:

};

static struct RegisterRadauIIA5Export
{
	RegisterRadauIIA5Export();
} registerRadauIIA5Export;

CLOSE_NAMESPACE_ACADO


#endif  // ACADO_TOOLKIT_RADAU_IIA5_EXPORT_HPP

// end of file.
