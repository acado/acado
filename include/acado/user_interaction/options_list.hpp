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
 *    \file include/acado/user_interaction/options_list.hpp
 *    \author Hans Joachim Ferreau, Boris Houska
 */


#ifndef ACADO_TOOLKIT_OPTIONS_LIST_HPP
#define ACADO_TOOLKIT_OPTIONS_LIST_HPP


#include <acado/utils/acado_utils.hpp>
#include <acado/user_interaction/options_item.hpp>


BEGIN_NAMESPACE_ACADO


/**
 *	\brief Provides a generic list of options (for internal use).
 *
 *	\ingroup AuxiliaryFunctionality
 *
 *  The class OptionsList provides a generic options list that allows to dynamically 
 *  setup and extend option lists. It is intended for internal use only, as all 
 *	user-functionality is encapsulated within the class Options.
 *
 *  For each option, an object of type OptionsItem is appended to a basic 
 *  singly-linked list. For each possible variable type of an option value, a
 *  special variant of OptionsItem needs to be derived from the base class.
 *
 *	\note Parts of the public functionality of the OptionsList class are tunnelled 
 *	via the Options class into the AlgorithmicBase class to be used in derived classes. 
 *	In case public functionality is modified or added to this class, the Options class
 *	as well as the AlgorithmicBase class have to be adapted accordingly.
 *
 *	\author Hans Joachim Ferreau, Boris Houska
 */
class OptionsList
{
	friend class Options;

	//
	// PUBLIC MEMBER FUNCTIONS:
	//
	public:

		/** Default constructor.
		 */
		OptionsList( );

		/** Copy constructor (deep copy).
		 *
		 *	@param[in] rhs	Right-hand side object.
		 */
		OptionsList(	const OptionsList& rhs
						);

		/** Destructor.
		 */
		~OptionsList( );

		/** Assignment operator (deep copy).
		 *
		 *	@param[in] rhs	Right-hand side object.
		 */
		OptionsList& operator=(	const OptionsList& rhs
								);


		/** Add an option item with a given integer default value to the list.
		 *
		 *	@param[in] name		Name of new option item.
		 *	@param[in] value	Default value of new option.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *          RET_OPTION_ALREADY_EXISTS, \n
		 *          RET_OPTIONS_LIST_CORRUPTED
		 */
		returnValue add(	OptionsName name,
							int value
							);

		/** Add an option item with a given double default value to the list.
		 *
		 *	@param[in] name		Name of new option item.
		 *	@param[in] value	Default value of new option.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *          RET_OPTION_ALREADY_EXISTS, \n
		 *          RET_OPTIONS_LIST_CORRUPTED
		 */
		returnValue add(	OptionsName name,
							double value
							);


		/** Returns value of an existing option item of integer type.
		 *
		 *	@param[in]  name	Name of option item.
		 *	@param[out] value	Value of option.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *          RET_OPTION_DOESNT_EXISTS
		 */
		returnValue get(	OptionsName name,
							int& value
							) const;

		/** Returns value of an existing option item of double type.
		 *
		 *	@param[in]  name	Name of option item.
		 *	@param[out] value	Value of option.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *          RET_OPTION_DOESNT_EXISTS
		 */
		returnValue get(	OptionsName name,
							double& value
							) const;


		/** Sets value of an existing option item of integer type to a given value.
		 *
		 *	@param[in] name		Name of option item.
		 *	@param[in] value	New value of option.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *          RET_OPTION_DOESNT_EXISTS, \n
		 *          RET_OPTIONS_LIST_CORRUPTED
		 */
		returnValue set(	OptionsName name,
							int value
							);

		/** Sets value of an existing option item of integer type to a given value.
		 *
		 *	@param[in] name		Name of option item.
		 *	@param[in] value	New value of option.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *          RET_OPTION_DOESNT_EXISTS, \n
		 *          RET_OPTIONS_LIST_CORRUPTED
		 */
		returnValue set(	OptionsName name,
							double value
							);


        /** Assigns a given OptionsList to this object.
		 *
		 *	@param[in] arg		New OptionsList object to be assigned.
		 *
		 *	\note This routine is introduced only for convenience and
         *	      is equivalent to the assignment operator.
		 *
         *  \return SUCCESSFUL_RETURN
         */
        returnValue setOptions(	const OptionsList& arg
								);


		/** Returns total number of option items in list.
		 *
		 *  \return Total number of options in list
		 */
		inline uint getNumber( ) const;


		/** Determines whether a given option exists or not.
		 *
		 *	@param[in] name		Name of option item.
		 *	@param[in] type		Internal type of option item.
		 *
		 *  \return BT_TRUE  iff option item exists, \n
		 *	        BT_FALSE otherwise
		 */
		inline BooleanType hasOption(	OptionsName name,
										OptionsItemType type
										) const;


		/** Determines whether options have been modified.
		 *
		 *	\return BT_TRUE  iff options have been modified, \n
		 *	        BT_FALSE otherwise 
		 */
		inline BooleanType haveOptionsChanged( ) const;

		/** Declares all options to be unchanged.
		 *
		 *  \return SUCCESSFUL_RETURN
		 */
		inline returnValue declareOptionsUnchanged( );


		/** Prints a list of all available options.
		 *
		 *  \return SUCCESSFUL_RETURN
		 */
		returnValue printOptionsList( ) const;



    //
    // PROTECTED MEMBER FUNCTIONS:
    //
    protected:

		/** Returns a pointer to an OptionsItem with given name and internal type.
		 *
		 *	@param[in] name		Name of option item.
		 *	@param[in] type		Internal type of option item.
		 *
		 *  \return Pointer to item or \n
		 *	        NULL if item does not exist
		 */
		OptionsItem* find(	OptionsName name,
							OptionsItemType type
							) const;


    //
    // DATA MEMBERS:
    //
	protected:
		OptionsItem* first;							/**< Pointer to first item of the singly-linked list. */
		OptionsItem* last;							/**< Pointer to first item of the singly-linked list. */

		uint number;								/**< Total number of item within the singly-linked list of the list. */

		BooleanType optionsHaveChanged;				/**< Flag indicating whether the value of at least one option item has been changed. */
};


CLOSE_NAMESPACE_ACADO


#include <acado/user_interaction/options_list.ipp>


#endif	// ACADO_TOOLKIT_OPTIONS_LIST_HPP


/*
 *	end of file
 */
