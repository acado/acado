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
 *    \file include/acado/user_interaction/options_item.hpp
 *    \author Hans Joachim Ferreau, Boris Houska
 */


#ifndef ACADO_TOOLKIT_OPTIONS_ITEM_HPP
#define ACADO_TOOLKIT_OPTIONS_ITEM_HPP



BEGIN_NAMESPACE_ACADO


/**
 *	\brief Abstract base class for items of a generic OptionsList (for internal use).
 *
 *	\ingroup AuxiliaryFunctionality
 *
 *  The class OptionsItem serves as an abstract base class for items of a generic 
 *	OptionsList that allows to dynamically setup and extend option lists. For each 
 *  option an object of type OptionsItem is appended to a singly linked list. 
 *  For each possible variable type of an option's value, a special variant of
 *  OptionsItem needs to be derived from this abstract base class.
 *
 *	Note that OptionsItems are assumed to be stored in a singly-linked list within 
 *	a OptionsList. Thus, also a pointer to the next options item is stored.
 *
 *	\note OptionsItem and derived classes could have been implemented much cleaner 
 *	      using C++ templates. However, following the ACADO Toolkit programming 
 *	      philosophy to use templates only if they offer a great benefit, it has
 *	      been decided that the extra effort for avoiding templates in this case
 *	      is acceptable.
 *
 *	\author Hans Joachim Ferreau, Boris Houska
 */
class OptionsItem
{
	//
	// PUBLIC MEMBER FUNCTIONS:
	//
	public:

		/** Default constructor. 
		 */
		OptionsItem( );

		/** Constructor which takes the name and internal type
		 *	of the item.
		 *
		 *	@param[in] _name			Name of item.
		 *	@param[in] _type			Iternal type of item.
		 */
		OptionsItem(	OptionsName _name,
						OptionsItemType _type
						);

		/** Copy constructor (deep copy).
		 *
		 *	@param[in] rhs	Right-hand side object.
		 */
		OptionsItem(	const OptionsItem& rhs
						);

		/** Destructor.
		 */
		virtual ~OptionsItem( );

		/** Assignment operator (deep copy).
		 *
		 *	@param[in] rhs	Right-hand side object.
		 */
		OptionsItem& operator=(	const OptionsItem& rhs
								);


		/** Returns value of item of integer type.
		 *
		 *	@param[out] _value	Value of item.
		 *
		 *  \return SUCCESSFUL_RETURN \n
		 *          RET_OPTION_DOESNT_EXISTS
		 */
		virtual returnValue getValue(	int& _value
										) const = 0;

		/** Returns value of item of double type.
		 *
		 *	@param[out] _value	Value of item.
		 *
		 *  \return SUCCESSFUL_RETURN \n
		 *          RET_OPTION_DOESNT_EXISTS
		 */
		virtual returnValue getValue(	double& _value
										) const = 0;


		/** Sets value of item of integer type.
		 *
		 *	@param[in] _value	New value of item.
		 *
		 *  \return SUCCESSFUL_RETURN \n
		 *          RET_OPTION_DOESNT_EXISTS
		 */
		virtual returnValue setValue(	int _value
										) = 0;

		/** Sets value of item of double type.
		 *
		 *	@param[in] _value	New value of item.
		 *
		 *  \return SUCCESSFUL_RETURN \n
		 *          RET_OPTION_DOESNT_EXISTS
		 */
		virtual returnValue setValue(	double _value
										) = 0;


		/** Returns name of item.
		 *
		 *  \return Name of item 
		 */
		inline OptionsName getName( ) const;

		/** Returns type of item.
		 *
		 *  \return Type of item 
		 */
		inline OptionsItemType getType( ) const;


		/** Sets name of item.
		 *
		 *	@param[in] _name	New name of item.
		 *
		 *  \return SUCCESSFUL_RETURN
		 */
		inline returnValue setName(	OptionsName _name
									);

		/** Sets internal type of item.
		 *
		 *	@param[in] _type	New internal type of item.
		 *
		 *  \return SUCCESSFUL_RETURN
		 */
		inline returnValue setType(	OptionsItemType _type
									);


		/** Returns pointer to next OptionsItem within a OptionsList.
		 *
		 *  \return New pointer to next item (or NULL iff item is terminal element). 
		 */
		inline OptionsItem* getNext( ) const;

		/** Assigns pointer to next OptionsItem within a OptionsList.
		 *
		 *	@param[in]  _next	New pointer to next item.
		 *
		 *  \return SUCCESSFUL_RETURN 
		 */
		inline returnValue setNext(	OptionsItem* const _next
									);


	//
	// DATA MEMBERS:
	//
	protected:
		OptionsName     name;			/**< Name of item. */
		OptionsItemType type;			/**< Internal type of item. */

		OptionsItem*    next;			/**< Pointer to next item within a OptionsList. */
};


CLOSE_NAMESPACE_ACADO



#include <acado/user_interaction/options_item.ipp>


#endif	// ACADO_TOOLKIT_OPTIONS_ITEM_HPP


/*
 *	end of file
 */
