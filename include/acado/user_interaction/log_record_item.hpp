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
 *    \file include/acado/user_interaction/log_record_item.hpp
 *    \author Hans Joachim Ferreau, Boris Houska
 */


#ifndef ACADO_TOOLKIT_LOG_RECORD_ITEM_HPP
#define ACADO_TOOLKIT_LOG_RECORD_ITEM_HPP


#include <acado/utils/acado_utils.hpp>
#include <acado/matrix_vector/matrix_vector.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>


BEGIN_NAMESPACE_ACADO




/**
 *	\brief Allows to manage single items of user-specified LogRecords of algorithmic information (for internal use).
 *
 *	\ingroup AuxiliaryFunctionality
 *	
 *  The class LogRecordItem allows to manage single items of user-specified 
 *	logging records of algorithmic information to be stored during runtime. 
 *	It is intended for internal use only and is used by the class LogRecord.
 *
 *	All information is internally stored in Matrix format; as information is 
 *	usually not only stored once but at different instants, e.g. at each iteration,
 *	information is stored in a MatrixVariablesGrid. Besides the actual numerical values
 *	of the information, also the output format of these values is stored within this 
 *	class. It describes who the information is to be printed into a string by, e.g., 
 *	defining a label, separators or the decimal precision to be shown.
 *
 *	Note that LogRecordItems are assumed to be stored as a singly-linked list within 
 *	a LogRecord. Thus, also a pointer to the next item is stored.
 *
 *	\author Hans Joachim Ferreau, Boris Houska
 */
class LogRecordItem
{
	//
	// PUBLIC MEMBER FUNCTIONS:
	//
	public:
		/** Default constructor. */
		LogRecordItem( );

		/** Constructor which takes the internal name of the logging item
		 *	along with settings defining the output format of its numerical 
		 *	values. 
		 *
		 *	@param[in] _name			Internal name defined by a LogName.
		 *	@param[in] _label			Label to be printed before the numerical values.
		 *	@param[in] _startString		Prefix before printing the numerical values.
		 *	@param[in] _endString		Suffix after printing the numerical values.
		 *	@param[in] _width			Total number of digits per single numerical value.
		 *	@param[in] _precision		Number of decimals per single numerical value.
		 *	@param[in] _colSeparator	Separator between the columns of the numerical values.
		 *	@param[in] _rowSeparator	Separator between the rows of the numerical values.
		 */
		LogRecordItem(	LogName _name,
						const char* const _label,
						const char* const _startString,
						const char* const _endString,
						uint _width,
						uint _precision,
						const char* const _colSeparator,
						const char* const _rowSeparator
						);

		/** Constructor which takes the internal name of the logging item
		 *	along with settings defining the output format of its numerical 
		 *	values.
		 *
		 *	@param[in] _name			Internal name defined by a symbolic expression (its global index).
		 *	@param[in] _label			Label to be printed before the numerical values.
		 *	@param[in] _startString		Prefix before printing the numerical values.
		 *	@param[in] _endString		Suffix after printing the numerical values.
		 *	@param[in] _width			Total number of digits per single numerical value.
		 *	@param[in] _precision		Number of decimals per single numerical value.
		 *	@param[in] _colSeparator	Separator between the columns of the numerical values.
		 *	@param[in] _rowSeparator	Separator between the rows of the numerical values.
		 */
		LogRecordItem(	const Expression& _name,
						const char* const _label,
						const char* const _startString,
						const char* const _endString,
						uint _width,
						uint _precision,
						const char* const _colSeparator,
						const char* const _rowSeparator
						);


		/** Copy constructor (deep copy).
		 *
		 *	@param[in] rhs	Right-hand side object.
		 */
		LogRecordItem(	const LogRecordItem& rhs
						);

		/** Destructor. */
		~LogRecordItem( );


		/** Assignment operator (deep copy).
		 *
		 *	@param[in] rhs	Right-hand side object.
		 */
		LogRecordItem& operator=(	const LogRecordItem& rhs
									);

									
		/** Returns whether two items are equal.
		 *
		 *	@param[in] rhs	Right-hand side object.
		 *
		 *  \return BT_TRUE  iff they are equal, \n
		 *			BT_FALSE otherwise */
		BooleanType operator==(	const LogRecordItem& rhs
								) const;

		/** Returns whether two items are not equal.
		 *
		 *	@param[in] rhs	Right-hand side object.
		 *
		 *  \return BT_TRUE  iff they are not equal, \n
		 *			BT_FALSE otherwise */
		BooleanType operator!=(	const LogRecordItem& rhs
								) const;


		/** Returns all numerical values of the item.
		 *
		 *  \return Matrix-valued variables grid containing all numerical values 
		 */
		inline const MatrixVariablesGrid& getAllValues( ) const;

		/** Assigns all numerical values of the item. In case the LogFrequency
		 *	is set to LOG_AT_EACH_ITERATION, the full matrix-valued variables grid
		 *	is assigned; otherwise only the first or last matrix is assigned, 
		 *	respectively.
		 *
		 *	@param[in] _frequency	Log frequency defining which values are to be assigned.
		 *	@param[in] _values		New values to be assigned.
		 *
		 *  \return SUCCESSFUL_RETURN
		 */
		returnValue setAllValues(	LogFrequency _frequency,
									const MatrixVariablesGrid& _values
									);


		/** Returns numerical value at given time instant.
		 *
		 *	@param[in] idx	Index of time instant.
		 *
		 *  \return Matrix containing the numerical value 
		 */
		inline Matrix getValue(	uint idx
								) const;

		/** Returns numerical value at first time instant.
		 *
		 *  \return Matrix containing the numerical value 
		 */
		inline Matrix getFirstValue( ) const;

		/** Returns numerical value at last time instant.
		 *
		 *  \return Matrix containing the numerical value 
		 */
		inline Matrix getLastValue( ) const;

		
		/** Assigns the numerical value at current time instant.
		 *
		 *	@param[in] _frequency	Log frequency defining which values are to be assigned.
		 *	@param[in] _value		New value to be assigned.
		 *	@param[in] _time		Time label of the instant.
		 *
		 *  \return SUCCESSFUL_RETURN
		 */
		returnValue setValue(	LogFrequency _frequency,
								const Matrix& _value,
								double _time = -INFTY
								);


		/** Obtains a string containing all numerical values of the item in 
		 *	the pre-defined output format.
		 *
		 *	@param[out] valueString		String containing all numerical values of the item.
		 *
		 *	\note The argument string must not be allocated when calling this 
		 *	      function and has to be de-allocated manually afterwards.
		 *
		 *  \return SUCCESSFUL_RETURN 
		 */
		inline returnValue getValueString(	char** valueString
											) const;

		/** Obtains a string containing the numerical value at a given time
		 *	instant of the item in the pre-defined output format.
		 *
		 *	@param[out] valueString		String containing all numerical values of the item.
		 *	@param[in]  idx				Index of time instant.
		 *
		 *	\note The argument string must not be allocated when calling this 
		 *	      function and has to be de-allocated manually afterwards.
		 *
		 *  \return SUCCESSFUL_RETURN 
		 */
		inline returnValue getValueString(	char** valueString,
											uint idx
											) const;


		/** Returns the length of the string containing all numerical values of the 
		 *	item in the pre-defined output format.
		 *
		 *  \return String length
		 */
		inline uint determineStringLength( ) const;

		/** Returns the length of the containing the numerical value at a given time
		 *	instant of the item in the pre-defined output format.
		 *
		 *	@param[in]  idx				Index of time instant.
		 *
		 *  \return String length 
		 */
		inline uint determineStringLength(	uint idx
											) const;


		/** Returns internal name of item.
		 *
		 *  \return Internal name of item
		 */
		inline int getName( ) const;

		/** Returns internal type of item (LogName enumeration or symbolic expression).
		 *
		 *  \return Internal yype of item
		 */
		inline LogRecordItemType getType( ) const;


		/** Assigns pointer to next LogRecordItem within a LogRecord.
		 *
		 *	@param[in]  _next	New pointer to next item.
		 *
		 *  \return SUCCESSFUL_RETURN 
		 */
		inline returnValue setNext(	LogRecordItem* const _next
									);

		/** Returns pointer to next LogRecordItem within a LogRecord.
		 *
		 *  \return Pointer to next item (or NULL iff item is terminal element). 
		 */
		inline LogRecordItem* getNext( ) const;


		/** Returns number of time instants on which numerical values have been stored.
		 *
		 *  \return Number of time instants.
		 */
		inline uint getNumPoints( ) const;

		/** Returns whether item is empty.
		 *
		 *  \return BT_TRUE  iff no numerical values have been stored, \n
		 *			BT_FALSE otherwise 
		 */
		inline BooleanType isEmpty( ) const;


		/** Enables write-protection of the item (i.e. no values can be written).
		 *
		 *  \return SUCCESSFUL_RETURN 
		 */
		inline returnValue enableWriteProtection( );

		/** Disables write-protection of the item (values can be written again).
		 *
		 *  \return SUCCESSFUL_RETURN 
		 */
		inline returnValue disableWriteProtection( );

		/** Returns whether item is write-protected.
		 *
		 *  \return BT_TRUE  iff item is write-protected, \n
		 *			BT_FALSE otherwise 
		 */
		inline BooleanType isWriteProtected( ) const;


		
		inline uint getNumDoubles( ) const;


	//
	// PROTECTED MEMBER FUNCTIONS:
	//
	protected:

		/** Assigns one digit information to the other.
		 *
		 *	@param[out] toDigit			Reference to digit information to be assigned.
		 *	@param[in]  fromDigit		Digit information to be copied.
		 *	@param[in]  defaultDigit	Default digit information to be copied in case fromDigit is invalid.
		 *
		 *  \return SUCCESSFUL_RETURN 
		 */
		returnValue assignDigits(	uint& toDigit,
									uint fromDigit, 
									uint defaultDigit
									);


	//
	// DATA MEMBERS:
	//
	protected:
		MatrixVariablesGrid values;						/**< The actual numerical values at all time instants. */

		int name;										/**< Internal name defined by a LogName. */
		LogRecordItemType type;							/**< Internal type of item (LogName enumeration or symbolic expression). */

		char* label;									/**< Label to be printed before the numerical values. */
		char* startString;								/**< Prefix before printing the numerical values. */
		char* endString;								/**< Suffix after printing the numerical values. */

		uint width;										/**< Total number of digits per single numerical value. */
		uint precision;									/**< Number of decimals per single numerical value. */

		char* colSeparator;								/**< Separator between the columns of the numerical values. */
		char* rowSeparator;								/**< Separator between the rows of the numerical values. */

		BooleanType writeProtected;						/**< Flag indicating whether item is write-protected (i.e. no values can be written). */

		LogRecordItem* next;							/**< Pointer to next LogRecordItem within a LogRecord. */
};


CLOSE_NAMESPACE_ACADO



#include <acado/user_interaction/log_record_item.ipp>


#endif	// ACADO_TOOLKIT_LOG_RECORD_ITEM_HPP


/*
 *	end of file
 */
