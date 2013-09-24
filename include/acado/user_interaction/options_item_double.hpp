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
 *    \file include/acado/user_interaction/options_item_double.hpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 04.08.2008
 */


#ifndef ACADO_TOOLKIT_OPTIONS_ITEM_double_HPP
#define ACADO_TOOLKIT_OPTIONS_ITEM_double_HPP


#include <acado/user_interaction/options_item.hpp>


BEGIN_NAMESPACE_ACADO


/**
 *	\brief Provides real-valued items within the OptionsList (for internal use).
 *
 *	\ingroup AuxiliaryFunctionality
 *
 *  The class OptionsItemDouble provides real-valued items within the OptionsList.
 *
 *	\author Hans Joachim Ferreau, Boris Houska
 */
class OptionsItemDouble : public OptionsItem
{
	//
	// PUBLIC MEMBER FUNCTIONS:
	//
	public:
		/** Default constructor.
		 */
		OptionsItemDouble( );

		/** Constructor which takes the name and initial value
		 *	of the item.
		 *
		 *	@param[in] _name			Name of item.
		 *	@param[in] _value			Initial value of item.
		 */
		OptionsItemDouble(	OptionsName _name,
							double _value
							);

		/** Copy constructor (deep copy).
		 *
		 *	@param[in] rhs	Right-hand side object.
		 */
		OptionsItemDouble(	const OptionsItemDouble& rhs
							);

		/** Destructor.
		 */
		virtual ~OptionsItemDouble( );

		/** Assignment operator (deep copy).
		 *
		 *	@param[in] rhs	Right-hand side object.
		 */
		OptionsItemDouble& operator=(	const OptionsItemDouble& rhs
										);


		/** Returns value of item of integer type.
		 *
		 *	@param[out] _value	Value of item.
		 *
		 *  \return RET_OPTION_DOESNT_EXISTS
		 */
		virtual returnValue getValue(	int& _value
										) const;

		/** Returns value of item of double type.
		 *
		 *	@param[out] _value	Value of item.
		 *
		 *  \return SUCCESSFUL_RETURN
		 */
		virtual returnValue getValue(	double& _value
										) const;


		/** Sets value of item of integer type.
		 *
		 *	@param[in] _value	New value of item.
		 *
		 *  \return RET_OPTION_DOESNT_EXISTS
		 */
		virtual returnValue setValue(	int _value
										);

		/** Sets value of item of double type.
		 *
		 *	@param[in] _value	New value of item.
		 *
		 *  \return SUCCESSFUL_RETURN
		 */
		virtual returnValue setValue(	double _value
										);


    //
    // DATA MEMBERS:
    //
	protected:
		double value;					/**< Value of item of double type. */
};


CLOSE_NAMESPACE_ACADO


#endif	// ACADO_TOOLKIT_OPTIONS_ITEM_double_HPP


/*
 *	end of file
 */
