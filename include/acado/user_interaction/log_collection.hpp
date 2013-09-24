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
 *    \file include/acado/user_interaction/log_collection.hpp
 *    \author Hans Joachim Ferreau, Boris Houska
 */


#ifndef ACADO_TOOLKIT_LOG_COLLECTION_HPP
#define ACADO_TOOLKIT_LOG_COLLECTION_HPP


#include <acado/utils/acado_utils.hpp>
#include <acado/matrix_vector/matrix_vector.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/variables_grid/variables_grid.hpp>

#include <acado/user_interaction/log_record.hpp>


BEGIN_NAMESPACE_ACADO


/**
 *	\brief Manages a generic list of log records (for internal use).
 *
 *	\ingroup AuxiliaryFunctionality
 *
 *  The class LogCollection manages a basic singly-linked list of log records 
 *  that allows to store algorithmic information during runtime. It is intended 
 *	for internal use only, as all user-functionality is encapsulated within the 
 *	classes Logging and LogRecord.
 *
 *	Besides managing the singly-linked list, this class tunnels the main 
 *	functionality of the LogRecord class such that this functionality can be 
 *	called on all records within the list simultaneously.
 *
 *	\note Parts of the public functionality of the LogCollection are tunnelled 
 *	by the AlgorithmicBase class to be used in derived developer classes. In case
 *	public functionality is modified or added to this class, the AlgorithmicBase
 *	class has to be adapted accordingly.
 *
 *	\author Hans Joachim Ferreau, Boris Houska
 */
class LogCollection
{
	//
	// PUBLIC MEMBER FUNCTIONS:
	//
	public:
		/** Default constructor. */
		LogCollection( );

		/** Copy constructor (deep copy).
		 *
		 *	@param[in] rhs	Right-hand side object.
		 */
		LogCollection(	const LogCollection& rhs
						);

		/** Destructor. */
		~LogCollection( );

		/** Assignment operator (deep copy).
		 *
		 *	@param[in] rhs	Right-hand side object.
		 */
		LogCollection& operator=(	const LogCollection& rhs
							);


		/** Returns the record of the singly-linked list with given index.
		 *
		 *	@param[in] idx	Index of desired record.
		 *
		 *  \return Record with given index. 
		 */
		inline LogRecord& operator()(	uint idx
										);

		/** Returns the record of the singly-linked list with given index (const version).
		 *
		 *	@param[in] idx	Index of desired record.
		 *
		 *  \return Record with given index. 
		 */
		inline const LogRecord& operator()( uint idx ) const;


		/** Adds a record to the singly-linked list.
		 *
		 *	@param[in] record	Record to be added.
		 *
		 *	\note This function is doing the same as the corresponding 
		 *	      addLogRecord member function and is introduced for syntax reasons only.
		 *
		 *  \return >= 0: index of added record, \n
		 *	        -RET_LOG_COLLECTION_CORRUPTED 
		 */
		int operator<<(	LogRecord& record
						);

		/** Adds a record to the singly-linked list.
		 *
		 *	@param[in] record	Record to be added.
		 *
		 *  \return >= 0: index of added record, \n
		 *	        -RET_LOG_COLLECTION_CORRUPTED 
		 */
		int addLogRecord(	LogRecord& record
							);


		/** Clears all records from the singly-linked list.
		 *
		 *  \return SUCCESSFUL_RETURN
		 */
		returnValue clearAllRecords( );


		/** Gets all numerical values at all time instants of the item
		 *	with given name. If this item exists in more than one record,
		 *	the first one is choosen as they are expected to have identical
		 *	values anyhow.
		 *
		 *	@param[in]  _name	Internal name of item.
		 *	@param[out] values	All numerical values at all time instants of given item.
		 *
		 *	\note All public getAll member functions make use of the <em>protected</em> getAll function.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_LOG_ENTRY_DOESNT_EXIST 
		 */
		inline returnValue getAll(	LogName _name,
									MatrixVariablesGrid& values
									) const;

		/** Gets all numerical values at all time instants of the item
		 *	with given name. If this item exists in more than one record,
		 *	the first one is choosen as they are expected to have identical
		 *	values anyhow.
		 *
		 *	@param[in]  _name	Internal name of item.
		 *	@param[out] values	All numerical values at all time instants of given item.
		 *
		 *	\note All public getAll member functions make use of the <em>protected</em> getAll function.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_LOG_ENTRY_DOESNT_EXIST 
		 */
		inline returnValue getAll(	const Expression& _name,
									MatrixVariablesGrid& values
									) const;


		/** Gets numerical value at first time instant of the item
		 *	with given name. If this item exists in more than one record,
		 *	the first one is choosen as they are expected to have identical
		 *	values anyhow.
		 *
		 *	@param[in]  _name		Internal name of item.
		 *	@param[out] firstValue	Numerical value at first time instant of given item.
		 *
		 *	\note All public getFirst member functions make use of the <em>protected</em> getFirst function.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_LOG_ENTRY_DOESNT_EXIST 
		 */
		inline returnValue getFirst(	LogName _name,
										Matrix& firstValue
										) const;

		/** Gets numerical value at first time instant of the item
		 *	with given name. If this item exists in more than one record,
		 *	the first one is choosen as they are expected to have identical
		 *	values anyhow.
		 *
		 *	@param[in]  _name		Internal name of item.
		 *	@param[out] firstValue	Numerical value at first time instant of given item.
		 *
		 *	\note All public getFirst member functions make use of the <em>protected</em> getFirst function.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_LOG_ENTRY_DOESNT_EXIST 
		 */
		inline returnValue getFirst(	const Expression& _name,
										Matrix& firstValue
										) const;

		/** Gets numerical value at first time instant of the item
		 *	with given name (converts internally used Matrix into VariablesGrid). 
		 *	If this item exists in more than one record, the first one is choosen 
		 *	as they are expected to have identical values anyhow.
		 *
		 *	@param[in]  _name		Internal name of item.
		 *	@param[out] firstValue	Numerical value at first time instant of given item.
		 *
		 *	\note All public getFirst member functions make use of the <em>protected</em> getFirst function.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_LOG_ENTRY_DOESNT_EXIST 
		 */
		inline returnValue getFirst(	LogName _name,
										VariablesGrid& firstValue
										) const;

		/** Gets numerical value at first time instant of the item
		 *	with given name (converts internally used Matrix into VariablesGrid). 
		 *	If this item exists in more than one record, the first one is choosen 
		 *	as they are expected to have identical values anyhow.
		 *
		 *	@param[in]  _name		Internal name of item.
		 *	@param[out] firstValue	Numerical value at first time instant of given item.
		 *
		 *	\note All public getFirst member functions make use of the <em>protected</em> getFirst function.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_LOG_ENTRY_DOESNT_EXIST 
		 */
		inline returnValue getFirst(	const Expression& _name,
										VariablesGrid& firstValue
										) const;


		/** Gets numerical value at last time instant of the item
		 *	with given name. If this item exists in more than one record,
		 *	the first one is choosen as they are expected to have identical
		 *	values anyhow.
		 *
		 *	@param[in]  _name		Internal name of item.
		 *	@param[out] lastValue	Numerical value at last time instant of given item.
		 *
		 *	\note All public getLast member functions make use of the <em>protected</em> getLast function.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_LOG_ENTRY_DOESNT_EXIST 
		 */
		inline returnValue getLast(	LogName _name,
									Matrix& lastValue
									) const;

		/** Gets numerical value at last time instant of the item
		 *	with given name. If this item exists in more than one record,
		 *	the first one is choosen as they are expected to have identical
		 *	values anyhow.
		 *
		 *	@param[in]  _name		Internal name of item.
		 *	@param[out] lastValue	Numerical value at last time instant of given item.
		 *
		 *	\note All public getLast member functions make use of the <em>protected</em> getLast function.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_LOG_ENTRY_DOESNT_EXIST 
		 */
		inline returnValue getLast(	const Expression& _name,	/**< Name of logging entry. */
									Matrix& lastValue				/**< New value of logging entry. */
									) const;

		/** Gets numerical value at last time instant of the item
		 *	with given name (converts internally used Matrix into VariablesGrid). 
		 *	If this item exists in more than one record, the first one is choosen 
		 *	as they are expected to have identical values anyhow.
		 *
		 *	@param[in]  _name		Internal name of item.
		 *	@param[out] lastValue	Numerical value at last time instant of given item.
		 *
		 *	\note All public getLast member functions make use of the <em>protected</em> getLast function.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_LOG_ENTRY_DOESNT_EXIST 
		 */
		inline returnValue getLast(	LogName _name,
									VariablesGrid& lastValue
									) const;

		/** Gets numerical value at last time instant of the item
		 *	with given name (converts internally used Matrix into VariablesGrid). 
		 *	If this item exists in more than one record, the first one is choosen 
		 *	as they are expected to have identical values anyhow.
		 *
		 *	@param[in]  _name		Internal name of item.
		 *	@param[out] lastValue	Numerical value at last time instant of given item.
		 *
		 *	\note All public getLast member functions make use of the <em>protected</em> getLast function.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_LOG_ENTRY_DOESNT_EXIST 
		 */
		inline returnValue getLast(	const Expression& _name,
									VariablesGrid& lastValue
									) const;



		/** Sets all numerical values at all time instants of all items
		 *	with given name within all records.
		 *
		 *	@param[in]  _name	Internal name of item.
		 *	@param[in]  values	All numerical values at all time instants of given item.
		 *
		 *	\note All public setAll member functions make use of the <em>protected</em> setAll function.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_LOG_COLLECTION_CORRUPTED 
		 */
		inline returnValue setAll(	LogName _name,
									const MatrixVariablesGrid& values
									);

		/** Sets all numerical values at all time instants of all items
		 *	with given name within all records.
		 *
		 *	@param[in]  _name	Internal name of item.
		 *	@param[in]  values	All numerical values at all time instants of given item.
		 *
		 *	\note All public setAll member functions make use of the <em>protected</em> setAll function.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_LOG_COLLECTION_CORRUPTED 
		 */
		inline returnValue setAll(	const Expression& _name,
									const MatrixVariablesGrid& values
									);


		/** Sets numerical value at last time instant of all items
		 *	with given name within all records.
		 *
		 *	@param[in]  _name		Internal name of item.
		 *	@param[in]  lastValue	Numerical value at last time instant of given item.
		 *	@param[in]  time		Time label of the instant.
		 *
		 *	\note All public setLast member functions make use of the <em>protected</em> setLast function.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_LOG_ENTRY_DOESNT_EXIST 
		 */
		inline returnValue setLast(	LogName _name,
									int lastValue,
									double time = -INFTY
									);

		/** Sets numerical value at last time instant of all items
		 *	with given name within all records.
		 *
		 *	@param[in]  _name		Internal name of item.
		 *	@param[in]  lastValue	Numerical value at last time instant of given item.
		 *	@param[in]  time		Time label of the instant.
		 *
		 *	\note All public setLast member functions make use of the <em>protected</em> setLast function.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_LOG_ENTRY_DOESNT_EXIST 
		 */
		inline returnValue setLast(	const Expression& _name,
									int lastValue,
									double time = -INFTY
									);

		/** Sets numerical value at last time instant of all items
		 *	with given name within all records.
		 *
		 *	@param[in]  _name		Internal name of item.
		 *	@param[in]  lastValue	Numerical value at last time instant of given item.
		 *	@param[in]  time		Time label of the instant.
		 *
		 *	\note All public setLast member functions make use of the <em>protected</em> setLast function.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_LOG_ENTRY_DOESNT_EXIST 
		 */
		inline returnValue setLast(	LogName _name,
									double lastValue,
									double time = -INFTY
									);

		/** Sets numerical value at last time instant of all items
		 *	with given name within all records.
		 *
		 *	@param[in]  _name		Internal name of item.
		 *	@param[in]  lastValue	Numerical value at last time instant of given item.
		 *	@param[in]  time		Time label of the instant.
		 *
		 *	\note All public setLast member functions make use of the <em>protected</em> setLast function.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_LOG_ENTRY_DOESNT_EXIST 
		 */
		inline returnValue setLast(	const Expression& _name,
									double lastValue,
									double time = -INFTY
									);

		/** Sets numerical value at last time instant of all items
		 *	with given name within all records.
		 *
		 *	@param[in]  _name		Internal name of item.
		 *	@param[in]  lastValue	Numerical value at last time instant of given item.
		 *	@param[in]  time		Time label of the instant.
		 *
		 *	\note All public setLast member functions make use of the <em>protected</em> setLast function.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_LOG_ENTRY_DOESNT_EXIST 
		 */
		inline returnValue setLast(	LogName _name,
									const Vector& lastValue,
									double time = -INFTY
									);

		/** Sets numerical value at last time instant of all items
		 *	with given name within all records.
		 *
		 *	@param[in]  _name		Internal name of item.
		 *	@param[in]  lastValue	Numerical value at last time instant of given item.
		 *	@param[in]  time		Time label of the instant.
		 *
		 *	\note All public setLast member functions make use of the <em>protected</em> setLast function.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_LOG_ENTRY_DOESNT_EXIST 
		 */
		inline returnValue setLast(	const Expression& _name,
									const Vector& value,
									double time = -INFTY
									);

		/** Sets numerical value at last time instant of all items
		 *	with given name within all records.
		 *
		 *	@param[in]  _name		Internal name of item.
		 *	@param[in]  lastValue	Numerical value at last time instant of given item.
		 *	@param[in]  time		Time label of the instant.
		 *
		 *	\note All public setLast member functions make use of the <em>protected</em> setLast function.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_LOG_ENTRY_DOESNT_EXIST 
		 */
		inline returnValue setLast(	LogName _name,
									const Matrix& lastValue,
									double time = -INFTY
									);

		/** Sets numerical value at last time instant of all items
		 *	with given name within all records.
		 *
		 *	@param[in]  _name		Internal name of item.
		 *	@param[in]  lastValue	Numerical value at last time instant of given item.
		 *	@param[in]  time		Time label of the instant.
		 *
		 *	\note All public setLast member functions make use of the <em>protected</em> setLast function.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_LOG_ENTRY_DOESNT_EXIST 
		 */
		inline returnValue setLast(	const Expression& _name,
									const Matrix& lastValue,
									double time = -INFTY
									);

		/** Sets numerical value at last time instant of all items
		 *	with given name within all records.
		 *
		 *	@param[in]  _name		Internal name of item.
		 *	@param[in]  lastValue	Numerical value at last time instant of given item.
		 *	@param[in]  time		Time label of the instant.
		 *
		 *	\note All public setLast member functions make use of the <em>protected</em> setLast function.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_LOG_ENTRY_DOESNT_EXIST 
		 */
		inline returnValue setLast(	LogName _name,
									const VariablesGrid& lastValue,
									double time = -INFTY
									);

		/** Sets numerical value at last time instant of all items
		 *	with given name within all records.
		 *
		 *	@param[in]  _name		Internal name of item.
		 *	@param[in]  lastValue	Numerical value at last time instant of given item.
		 *	@param[in]  time		Time label of the instant.
		 *
		 *	\note All public setLast member functions make use of the <em>protected</em> setLast function.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_LOG_ENTRY_DOESNT_EXIST 
		 */
		inline returnValue setLast(	const Expression& _name,
									const VariablesGrid& lastValue,
									double time = -INFTY
									);


		/** Prints whole collection of log records into the respective, internally specified file;
		 *	all items within the records are printed according to the output format settings.
		 *
		 *	@param[in]  _mode		Print mode: see documentation of LogPrintMode of details.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_INVALID_ARGUMENTS, \n
		 *	        RET_UNKNOWN_BUG
		 */
		returnValue print( LogPrintMode _mode ) const;


		/** Updates all items with the record given as argument. In doing so,
		 *	it is checked for each item whether it is contained within one of 
		 *	of the records of the collection; and if so, the numerical values
		 *	are copied into the argument record.
		 *
		 *	@param[in,out]  _record		Record to be updated
		 *
		 *  \return SUCCESSFUL_RETURN
		 */
		returnValue updateLogRecord(	LogRecord& _record
										) const;


		/** Returns number of records contained in the log collection.
		 *
		 *  \return Number of records
		 */
		inline uint getNumLogRecords( ) const;


		/** Returns whether the collection contains a certain record.
		 *
		 *	@param[in] _record	Record whose existence is to be checked.
		 *
		 *  \return BT_TRUE  iff record exists, \n
		 *	        BT_FALSE otherwise
		 */
		inline BooleanType hasRecord(	const LogRecord& _record
										) const;


		/** Returns whether an (possibly empty) item with given internal name 
		 *	exists within at least one of the records or not.
		 *
		 *	@param[in] _name	Internal name of item.
		 *
		 *  \return BT_TRUE  iff item exists, \n
		 *	        BT_FALSE otherwise
		 */
		inline BooleanType hasItem(	LogName _name
									) const;

		/** Returns whether an (possibly empty) item with given internal name 
		 *	exists within at least one of the records or not.
		 *
		 *	@param[in] _name	Internal name of item.
		 *
		 *  \return BT_TRUE  iff item exists, \n
		 *	        BT_FALSE otherwise
		 */
		inline BooleanType hasItem(	const Expression& _name
									) const;

		/** Returns whether a non-empty item with given internal name exists 
		 *	within at least one of the records or not.
		 *
		 *	@param[in] _name	Internal name of item.
		 *
		 *  \return BT_TRUE  iff non-empty item exists, \n
		 *	        BT_FALSE otherwise
		 */
		inline BooleanType hasNonEmptyItem(	LogName _name
											) const;

		/** Returns whether a non-empty item with given internal name exists 
		 *	within at least one of the records or not.
		 *
		 *	@param[in] _name	Internal name of item.
		 *
		 *  \return BT_TRUE  iff non-empty item exists, \n
		 *	        BT_FALSE otherwise
		 */
		inline BooleanType hasNonEmptyItem(	const Expression& _name
											) const;



		inline uint getNumDoubles( ) const;


    //
    // PROTECTED MEMBER FUNCTIONS:
    //
    protected:

		/** Gets all numerical values at all time instants of the item
		 *	with given internal name and internal type. If this item exists in 
		 *	more than one record, the first one is choosen as they are expected 
		 *	to have identical values anyhow.
		 *
		 *	@param[in]  _name	Internal name of item.
		 *	@param[in]  _type	Internal type of item.
		 *	@param[out] values	All numerical values at all time instants of given item.
		 *
		 *	\note All <em>public</em> getAll member functions make use of this protected function.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_LOG_ENTRY_DOESNT_EXIST 
		 */
		returnValue getAll(	uint _name,
							LogRecordItemType _type,
							MatrixVariablesGrid& values
							) const;

		/** Gets numerical value at first time instant of the item
		 *	with given internal name and internal type. If this item exists in 
		 *	more than one record, the first one is choosen as they are expected 
		 *	to have identical values anyhow.
		 *
		 *	@param[in]  _name		Internal name of item.
		 *	@param[in]  _type		Internal type of item.
		 *	@param[out] firstValue	Numerical value at first time instant of given item.
		 *
		 *	\note All <em>public</em> getFirst member functions make use of this protected function.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_LOG_ENTRY_DOESNT_EXIST 
		 */
		returnValue getFirst(	uint _name,
								LogRecordItemType _type,
								Matrix& lastValue
								) const;

		/** Gets numerical value at last time instant of the item
		 *	with given internal name and internal type. If this item exists in 
		 *	more than one record, the first one is choosen as they are expected 
		 *	to have identical values anyhow.
		 *
		 *	@param[in]  _name		Internal name of item.
		 *	@param[in]  _type		Internal type of item.
		 *	@param[out] lastValue	Numerical value at last time instant of given item.
		 *
		 *	\note All public getLast member functions make use of the <em>protected</em> getLast function.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_LOG_ENTRY_DOESNT_EXIST 
		 */
		returnValue getLast(	uint _name,
								LogRecordItemType _type,
								Matrix& lastValue
								) const;


		/** Sets all numerical values at all time instants of all items
		 *	with given internal name and internal type within all records.
		 *
		 *	@param[in]  _name	Internal name of item.
		 *	@param[in]  _type	Internal type of item.
		 *	@param[in]  values	All numerical values at all time instants of given item.
		 *
		 *	\note All <em>public</em> setAll member functions make use of this protected function.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_LOG_COLLECTION_CORRUPTED 
		 */
		returnValue setAll(	uint _name,
							LogRecordItemType _type,
							const MatrixVariablesGrid& values
							);

		/** Sets numerical value at last time instant of all items
		 *	with given internal name and internal type within all records.
		 *
		 *	@param[in]  _name		Internal name of item.
		 *	@param[in]  _type		Internal type of item.
		 *	@param[in]  lastValue	Numerical value at last time instant of given item.
		 *	@param[in]  time		Time label of the instant.
		 *
		 *	\note All <em>public</em> setLast member functions make use of this protected function.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_LOG_COLLECTION_CORRUPTED 
		 */
		returnValue setLast(	uint _name,
								LogRecordItemType _type,
								const Matrix& value,
								double time = -INFTY
								);


		/** Checks whether the collection contains a certain record and,
		 *	if so, return its index within the collection.
		 *
		 *	@param[in] _record	Record whose index is to be determined.
		 *
		 *  \return >=0: index of record iff it exists, \n
		 *	         -1: record does not exist within collection
		 */
		inline int findRecord(	const LogRecord& _record
								) const;


		/** Returns pointer to first record that contains a item with given internal name.
		 *	
		 *	@param[in] _name	Internal name of item.
		 *	
		 *  \return Pointer to record or \n
		 *	        NULL if record does not exist
		 */
		LogRecord* find(	LogName _name
							) const;

		/** Returns pointer to first record that contains a item with given internal name.
		 *	
		 *	@param[in] _name	Internal name of item.
		 *	
		 *  \return Pointer to record or \n
		 *	        NULL if record does not exist
		 */
		LogRecord* find(	const Expression& _name	/**< Name of logging entry. */
							) const;

		/** Returns pointer to first record that contains a item with given internal name
		 *	and given internal type.
		 *	
		 *	@param[in] _name	Internal name of item.
		 *	@param[in] _type	Internal type of item.
		 *	
		 *  \return Pointer to record or \n
		 *	        NULL if record does not exist
		 */
		LogRecord* find(	uint _name,
							LogRecordItemType _type
							) const;

		/** Returns pointer to first non-empty record that contains a item 
		 *	with given internal name and given internal type.
		 *	
		 *	@param[in] _name	Internal name of item.
		 *	@param[in] _type	Internal type of item.
		 *	
		 *  \return Pointer to non-empty record or \n
		 *	        NULL if record does not exist
		 */
		LogRecord* findNonEmpty(	uint _name,
									LogRecordItemType _type
									) const;


    //
    // DATA MEMBERS:
    //
	protected:
		LogRecord* first;				/**< Pointer to first record of the singly-linked list. */
		LogRecord* last;				/**< Pointer to last record of the singly-linked list. */

		uint number;					/**< Total number of records within the singly-linked list of the collection. */
};


CLOSE_NAMESPACE_ACADO


#include <acado/user_interaction/log_collection.ipp>


#endif	// ACADO_TOOLKIT_LOG_COLLECTION_HPP


/*
 *	end of file
 */
