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
 *    \file include/acado/code_generation/export_variable.hpp
 *    \author Hans Joachim Ferreau, Boris Houska
 */


#ifndef ACADO_TOOLKIT_EXPORT_VARIABLE_HPP
#define ACADO_TOOLKIT_EXPORT_VARIABLE_HPP

#include <acado/utils/acado_utils.hpp>
#include <acado/code_generation/export_argument.hpp>
#include <acado/code_generation/export_index.hpp>



BEGIN_NAMESPACE_ACADO


class ExportArithmeticStatement;


/** 
 *	\brief Defines a matrix-valued variable to be used for exporting code.
 *
 *	\ingroup UserDataStructures
 *
 *	The class ExportVariable defines a matrix-valued variable to be used for exporting
 *	code. Instances of this class can be used similar to usual Matrix objects
 *	but offer additional functionality, e.g. they allow to export arithmetic 
 *	expressions and they can be passed as argument to exported functions. By 
 *	default, all entries of a ExportVariable are undefined, but each of its 
 *	component can be set to a fixed value if known beforehand.
 *
 *	\author Hans Joachim Ferreau, Boris Houska
 */

class ExportVariable : public ExportArgument
{

	//
    // PUBLIC MEMBER FUNCTIONS:
    //
    public:

		/** Default constructor. 
		 */
        ExportVariable( );

		/** Constructor which takes the name, type string
		 *	and dimensions of the variable.
		 *
		 *	@param[in] _name			Name of the argument.
		 *	@param[in] _nRows			Number of rows of the argument.
		 *	@param[in] _nCols			Number of columns of the argument.
		 *	@param[in] _type			Data type of the argument.
		 *	@param[in] _dataStruct		Global data struct to which the argument belongs to (if any).
		 *	@param[in] _callByValue		Flag indicating whether argument it to be called by value.
		 */
		ExportVariable(	const String& _name,
						uint _nRows = 1,
						uint _nCols = 1,
						ExportType _type = REAL,
						ExportStruct _dataStruct = ACADO_LOCAL,
						BooleanType _callItByValue = BT_FALSE
						);

		/** Constructor which takes the name and type string of the variable.
		 *	Moreover, it initializes the variable with the dimensions and the 
		 *	values of the given matrix.
		 *
		 *	@param[in] _name			Name of the argument.
		 *	@param[in] _data			Matrix used for initialization.
		 *	@param[in] _type			Data type of the argument.
		 *	@param[in] _dataStruct		Global data struct to which the argument belongs to (if any).
		 *	@param[in] _callByValue		Flag indicating whether argument it to be called by value.
		 */
		ExportVariable(	const String& _name,
						const Matrix& _data,
						ExportType _type = REAL,
						ExportStruct _dataStruct = ACADO_LOCAL,
						BooleanType _callItByValue = BT_FALSE
						);

		/** Constructor which converts a given matrix into an ExportVariable.
		 *
		 *	@param[in] _data			Matrix used for initialization.
		 */
		ExportVariable(	const Matrix& _data
						);

		/** Copy constructor (deep copy).
		 *
		 *	@param[in] arg		Right-hand side object.
		 */
        ExportVariable(	const ExportVariable& arg
						);

        /** Destructor.
		 */
		virtual ~ExportVariable( );

		/** Assignment operator (deep copy).
		 *
		 *	@param[in] arg		Right-hand side object.
		 */
		ExportVariable& operator=(	const ExportVariable& arg
									);

		/** Assignment operator ...
		 *
		 *	@param[in] arg		Right-hand side object.
		 */
		ExportVariable& operator=(	const Matrix& arg
									);

		/** Clone constructor (deep copy).
		 *
		 *	\return Pointer to cloned object.
		 */
		virtual ExportData* clone( ) const;


		/** Initializes variable with given name, type string
		 *	and dimensions of the variable.
		 *
		 *	@param[in] _name			Name of the argument.
		 *	@param[in] _nRows			Number of rows of the argument.
		 *	@param[in] _nCols			Number of columns of the argument.
		 *	@param[in] _type			Data type of the argument.
		 *	@param[in] _dataStruct		Global data struct to which the argument belongs to (if any).
		 *	@param[in] _callByValue		Flag indicating whether argument it to be called by value.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue init(	const String& _name,
							uint _nRows = 1,
							uint _nCols = 1,
							ExportType _type = REAL,
							ExportStruct _dataStruct = ACADO_LOCAL,
							BooleanType _callItByValue = BT_FALSE
							);

		/** Initializes variable with given name and type string of the variable.
		 *	Moreover, the variable is initialized with the dimensions and the 
		 *	values of the given matrix.
		 *
		 *	@param[in] _name			Name of the argument.
		 *	@param[in] _data			Matrix used for initialization.
		 *	@param[in] _type			Data type of the argument.
		 *	@param[in] _dataStruct		Global data struct to which the argument belongs to (if any).
		 *	@param[in] _callByValue		Flag indicating whether argument it to be called by value.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue init(	const String& _name,
							const Matrix& _data,
							ExportType _type = REAL,
							ExportStruct _dataStruct = ACADO_LOCAL,
							BooleanType _callItByValue = BT_FALSE
							);

		/** Initializes variable with given name, type string
		 *	and dimensions of the variable.
		 *
		 *	@param[in] _name			Name of the argument.
		 *	@param[in] _nRows			Number of rows of the argument.
		 *	@param[in] _nCols			Number of columns of the argument.
		 *	@param[in] _type			Data type of the argument.
		 *	@param[in] _dataStruct		Global data struct to which the argument belongs to (if any).
		 *	@param[in] _callByValue		Flag indicating whether argument it to be called by value.
		 *
		 *	\return Reference to initialized object
		 */
		ExportVariable& setup(	const String& _name,
								uint _nRows = 1,
								uint _nCols = 1,
								ExportType _type = REAL,
								ExportStruct _dataStruct = ACADO_LOCAL,
								BooleanType _callItByValue = BT_FALSE
								);

		/** Initializes variable with given name and type string of the variable.
		 *	Moreover, the variable is initialized with the dimensions and the 
		 *	values of the given matrix.
		 *
		 *	@param[in] _name			Name of the argument.
		 *	@param[in] _data			Matrix used for initialization.
		 *	@param[in] _type			Data type of the argument.
		 *	@param[in] _dataStruct		Global data struct to which the argument belongs to (if any).
		 *	@param[in] _callByValue		Flag indicating whether argument it to be called by value.
		 *
		 *	\return Reference to initialized object
		 */
		ExportVariable& setup(	const String& _name,
								const Matrix& _data,
								ExportType _type = REAL,
								ExportStruct _dataStruct = ACADO_LOCAL,
								BooleanType _callItByValue = BT_FALSE
								);


		/** Returns value of given component.
		 *
		 *	@param[in] rowIdx		Row index of the component to be returned.
		 *	@param[in] colIdx		Column index of the component to be returned.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		double& operator()(	uint rowIdx,
							uint colIdx
							);

		/** Returns value of given component.
		 *
		 *	@param[in] rowIdx		Row index of the component to be returned.
		 *	@param[in] colIdx		Column index of the component to be returned.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		double operator()(	uint rowIdx,
							uint colIdx
							) const;

		/** Returns value of given component.
		 *
		 *	@param[in] totalIdx		Memory location of the component to be returned.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		double& operator()(	uint totalIdx
							);

		/** Returns value of given component.
		 *
		 *	@param[in] totalIdx		Memory location of the component to be returned.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		double operator()(	uint totalIdx
							) const;

		/** Returns a copy of the variable with given name.
		 *
		 *	@param[in] _name		New name of variable copy.
		 *
		 *	\return Copy of the variable with given name
		 */
		ExportVariable operator()(	const String& _name
									) const;


		/** Resets all components of the variable to be undefined.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue resetAll( );

		/** Resets all diagonal components of the square variable to be undefined.
		 *
		 *	\return SUCCESSFUL_RETURN, \n
		 *	        RET_MATRIX_NOT_SQUARE
		 */
		returnValue resetDiagonal( );

		/** Returns whether given component is set to zero.
		 *
		 *	@param[in] rowIdx		Variable row index of the component.
		 *	@param[in] colIdx		Variable column index of the component.
		 *
		 *	\return BT_TRUE  iff given component is set to zero, \n
		 *	        BT_FALSE otherwise
		 */
		BooleanType isZero( const ExportIndex& rowIdx,
							const ExportIndex& colIdx
							) const;

		/** Returns whether given component is set to zero.
		 *
		 *	@param[in] rowIdx		Variable row index of the component.
		 *	@param[in] colIdx		Column index of the component.
		 *
		 *	\return BT_TRUE  iff given component is set to zero, \n
		 *	        BT_FALSE otherwise
		 */
		BooleanType isZero( const ExportIndex& rowIdx,
							uint colIdx
							) const;

		/** Returns whether given component is set to zero.
		 *
		 *	@param[in] rowIdx		Row index of the component.
		 *	@param[in] colIdx		Variable column index of the component.
		 *
		 *	\return BT_TRUE  iff given component is set to zero, \n
		 *	        BT_FALSE otherwise
		 */
		BooleanType isZero( uint rowIdx,
							const ExportIndex& colIdx
							) const;

		/** Returns whether given component is set to zero.
		 *
		 *	@param[in] rowIdx		Row index of the component.
		 *	@param[in] colIdx		Column index of the component.
		 *
		 *	\return BT_TRUE  iff given component is set to zero, \n
		 *	        BT_FALSE otherwise
		 */
		BooleanType isZero( uint rowIdx,
							uint colIdx
							) const;


		/** Returns whether given component is set to one.
		 *
		 *	@param[in] rowIdx		Variable row index of the component.
		 *	@param[in] colIdx		Variable column index of the component.
		 *
		 *	\return BT_TRUE  iff given component is set to one, \n
		 *	        BT_FALSE otherwise
		 */
		BooleanType isOne(	const ExportIndex& rowIdx,
							const ExportIndex& colIdx
							) const;

		/** Returns whether given component is set to one.
		 *
		 *	@param[in] rowIdx		Variable row index of the component.
		 *	@param[in] colIdx		Column index of the component.
		 *
		 *	\return BT_TRUE  iff given component is set to one, \n
		 *	        BT_FALSE otherwise
		 */
		BooleanType isOne(	const ExportIndex& rowIdx,
							uint colIdx
							) const;

		/** Returns whether given component is set to one.
		 *
		 *	@param[in] rowIdx		Row index of the component.
		 *	@param[in] colIdx		Variable column index of the component.
		 *
		 *	\return BT_TRUE  iff given component is set to one, \n
		 *	        BT_FALSE otherwise
		 */
		BooleanType isOne(	uint rowIdx,
							const ExportIndex& colIdx
							) const;

		/** Returns whether given component is set to one.
		 *
		 *	@param[in] rowIdx		Row index of the component.
		 *	@param[in] colIdx		Column index of the component.
		 *
		 *	\return BT_TRUE  iff given component is set to one, \n
		 *	        BT_FALSE otherwise
		 */
		BooleanType isOne(	uint rowIdx,
							uint colIdx
							) const;

		/** Returns whether given component is set to a given value.
		 *
		 *	@param[in] rowIdx		Variable row index of the component.
		 *	@param[in] colIdx		Variable column index of the component.
		 *
		 *	\return BT_TRUE  iff given component is set to a given value, \n
		 *	        BT_FALSE otherwise
		 */
		BooleanType isGiven(	const ExportIndex& rowIdx,
								const ExportIndex& colIdx
								) const;

		/** Returns whether given component is set to a given value.
		 *
		 *	@param[in] rowIdx		Variable row index of the component.
		 *	@param[in] colIdx		Column index of the component.
		 *
		 *	\return BT_TRUE  iff given component is set to a given value, \n
		 *	        BT_FALSE otherwise
		 */
		BooleanType isGiven(	const ExportIndex& rowIdx,
								uint colIdx
								) const;

		/** Returns whether given component is set to a given value.
		 *
		 *	@param[in] rowIdx		Row index of the component.
		 *	@param[in] colIdx		Variable column index of the component.
		 *
		 *	\return BT_TRUE  iff given component is set to a given value, \n
		 *	        BT_FALSE otherwise
		 */
		BooleanType isGiven(	uint rowIdx,
								const ExportIndex& colIdx
								) const;

		/** Returns whether given component is set to a given value.
		 *
		 *	@param[in] rowIdx		Row index of the component.
		 *	@param[in] colIdx		Column index of the component.
		 *
		 *	\return BT_TRUE  iff given component is set to a given value, \n
		 *	        BT_FALSE otherwise
		 */
		BooleanType isGiven(	uint rowIdx,
								uint colIdx
								) const;

		/** Returns whether all components of the variable are set to a given value.
		 *
		 *	\return BT_TRUE  iff all components of the variable are set to a given value, \n
		 *	        BT_FALSE otherwise
		 */
		BooleanType isGiven( ) const;


		/** Returns string containing the value of a given component. If its 
		 *	value is undefined, the string contains the address of the component.
		 *
		 *	@param[in] rowIdx		Variable row index of the component.
		 *	@param[in] colIdx		Variable column index of the component.
		 *
		 *	\return String containing the value of a given component
		 */
		const char* get(	const ExportIndex& rowIdx,
							const ExportIndex& colIdx
							) const;

		/** Returns string containing the value of a given component. If its 
		 *	value is undefined, the string contains the address of the component.
		 *
		 *	@param[in] rowIdx		Row index of the component.
		 *	@param[in] colIdx		Column index of the component.
		 *
		 *	\return String containing the value of a given component
		 */
		const char* get(	uint rowIdx,
							uint colIdx
							) const;


		/** Returns number of rows of the variable.
		 *
		 *	\return Number of rows of the variable
		 */
		virtual uint getNumRows( ) const;

		/** Returns number of columns of the variable.
		 *
		 *	\return Number of columns of the variable
		 */
		virtual uint getNumCols( ) const;

		/** Returns total dimension of the variable.
		 *
		 *	\return Total dimension of the variable
		 */
		virtual uint getDim( ) const;


		/** Operator for adding two ExportVariables.
		 *
		 *	@param[in] arg		Variable to be added.
		 *
		 *	\return Arithmetic statement containing the addition
		 */
		ExportArithmeticStatement operator+(	const ExportVariable& arg
												) const;

		/** Operator for subtracting an ExportVariable from another.
		 *
		 *	@param[in] arg		Variable to be subtracted.
		 *
		 *	\return Arithmetic statement containing the subtraction
		 */
		ExportArithmeticStatement operator-(	const ExportVariable& arg
												) const;

		/** Operator for add-assigning an ExportVariable to another.
		 *
		 *	@param[in] arg		Variable to be add-assigned.
		 *
		 *	\return Arithmetic statement containing the add-assignment
		 */
		ExportArithmeticStatement operator+=(	const ExportVariable& arg
												) const;

		/** Operator for subtract-assigning an ExportVariables from another.
		 *
		 *	@param[in] arg		Variable to be subtract-assigned.
		 *
		 *	\return Arithmetic statement containing the subtract-assignment
		 */
		ExportArithmeticStatement operator-=(	const ExportVariable& arg
												) const;

		/** Operator for multiplying two ExportVariables.
		 *
		 *	@param[in] arg		Variable to be multiplied from the right.
		 *
		 *	\return Arithmetic statement containing the multiplication
		 */
		ExportArithmeticStatement operator*(	const ExportVariable& arg
												) const;

		/** Operator for multiplying an ExportVariable to the transposed on another.
		 *
		 *	@param[in] arg		Variable to be multiplied from the right.
		 *
		 *	\return Arithmetic statement containing the multiplication with left-hand side variable transposed
		 */
		ExportArithmeticStatement operator^(	const ExportVariable& arg
												) const;

		/** Operator for assigning an ExportVariable to another.
		 *
		 *	@param[in] arg		Variable to be assined.
		 *
		 *	\return Arithmetic statement containing the assignment
		 */
		ExportArithmeticStatement operator==(	const ExportVariable& arg
												) const;


		/** Operator for assigning an arithmetic statement to an ExportVariable.
		 *
		 *	@param[in] arg		Arithmetic statement to be assigned.
		 *
		 *	\return Arithmetic statement containing the assignment
		 */
		ExportArithmeticStatement operator==(	ExportArithmeticStatement arg
												) const;

		/** Operator for adding an arithmetic statement to an ExportVariable.
		 *
		 *	@param[in] arg		Arithmetic statement to be added.
		 *
		 *	\return Arithmetic statement containing the addition
		 */
		ExportArithmeticStatement operator+(	ExportArithmeticStatement arg
												) const;

		/** Operator for subtraction an arithmetic statement from an ExportVariable.
		 *
		 *	@param[in] arg		Arithmetic statement to be subtracted.
		 *
		 *	\return Arithmetic statement containing the subtraction
		 */
		ExportArithmeticStatement operator-(	ExportArithmeticStatement arg
												) const;

		/** Operator for add-assigning an arithmetic statement to an ExportVariable.
		 *
		 *	@param[in] arg		Arithmetic statement to be add-assigned.
		 *
		 *	\return Arithmetic statement containing the add-assignment
		 */
		ExportArithmeticStatement operator+=(	ExportArithmeticStatement arg
												) const;

		/** Operator for subtract-assigning an arithmetic statement from an ExportVariable.
		 *
		 *	@param[in] arg		Arithmetic statement to be subtract-assigned.
		 *
		 *	\return Arithmetic statement containing the subtract-assignment
		 */
		ExportArithmeticStatement operator-=(	ExportArithmeticStatement arg
												) const;


		/** Operator for adding a Matrix to an ExportVariable.
		 *
		 *	@param[in] arg		Matrix to be added.
		 *
		 *	\return Arithmetic statement containing the addition
		 */
		ExportArithmeticStatement operator+(	const Matrix& arg
												) const;

		/** Operator for subtracting a Matrix from an ExportVariable.
		 *
		 *	@param[in] arg		Matrix to be subtracted.
		 *
		 *	\return Arithmetic statement containing the subtraction
		 */
		ExportArithmeticStatement operator-(	const Matrix& arg
												) const;

		/** Operator for add-assigning a Matrix to an ExportVariable.
		 *
		 *	@param[in] arg		Matrix to be add-assigned.
		 *
		 *	\return Arithmetic statement containing the add-assignment
		 */
		ExportArithmeticStatement operator+=(	const Matrix& arg
												) const;

		/** Operator for subtract-assigning a Matrix from an ExportVariable.
		 *
		 *	@param[in] arg		Matrix to be subtract-assigned.
		 *
		 *	\return Arithmetic statement containing the subtract-assignment
		 */
		ExportArithmeticStatement operator-=(	const Matrix& arg
												) const;

		/** Operator for multiplying a Matrix to an ExportVariable.
		 *
		 *	@param[in] arg		Matrix to be multiplied from the right.
		 *
		 *	\return Arithmetic statement containing the multiplication
		 */
		ExportArithmeticStatement operator*(	const Matrix& arg
												) const;

		/** Operator for multiplying a Matrix to the transposed of an ExportVariable.
		 *
		 *	@param[in] arg		Matrix to be multiplied from the right.
		 *
		 *	\return Arithmetic statement containing the multiplication with left-hand side variable transposed
		 */
		ExportArithmeticStatement operator^(	const Matrix& arg
												) const;

		/** Operator for assigning a Matrix to an ExportVariable.
		 *
		 *	@param[in] arg		Matrix to be assigned.
		 *
		 *	\return Arithmetic statement containing the assignment
		 */
		ExportArithmeticStatement operator==(	const Matrix& arg
												) const;


		/** Returns a copy of the variable with transposed components.
		 *
		 *	\return Copy of the variable with transposed components
		 */
		ExportVariable getTranspose( ) const;

		/** Returns a copy of the variable whose components are accessed in a transposed manner.
		 *
		 *	\return Copy of the variable whose components are accessed in a transposed manner
		 */
		ExportVariable accessTransposed( );

		/** Returns whether variable is accessed in a transposed manner.
		 *
		 *	\return BT_TRUE  iff variable is accessed in a transposed manner, \n
		 *	        BT_FALSE otherwise
		 */
		BooleanType isAccessedTransposed( ) const;


		/** Returns a new variable containing only the given row of the variable.
		 *
		 *	@param[in] idx			Row index.
		 *
		 *	\return New variable containing only the given row of the variable
		 */
		ExportVariable getRow(	uint idx
								) const;

		/** Returns a new variable containing only the given row of the variable.
		 *
		 *	@param[in] idx			Variable row index.
		 *
		 *	\return New variable containing only the given row of the variable
		 */
		ExportVariable getRow(	const ExportIndex& idx
								) const;

		/** Returns a new variable containing only the given column of the variable.
		 *
		 *	@param[in] idx			Column index.
		 *
		 *	\return New variable containing only the given column of the variable
		 */
		ExportVariable getCol(	uint idx
								) const;

		/** Returns a new variable containing only the given column of the variable.
		 *
		 *	@param[in] idx			Variable column index.
		 *
		 *	\return New variable containing only the given column of the variable
		 */
		ExportVariable getCol(	const ExportIndex& idx
								) const;


		/** Returns a new variable containing only the given rows of the variable.
		 *
		 *	@param[in] idx1			Index of first row of new variable.
		 *	@param[in] idx2			Index following last row of new variable.
		 *
		 *	\return New variable containing only the given rows of the variable
		 */
		ExportVariable getRows(	uint idx1,
								uint idx2
								) const;

		/** Returns a new variable containing only the given rows of the variable.
		 *
		 *	@param[in] idx1			Variable index of first row of new variable.
		 *	@param[in] idx2			Variable index following last row of new variable.
		 *
		 *	\return New variable containing only the given rows of the variable
		 */
		ExportVariable getRows(	const ExportIndex& idx1,
								const ExportIndex& idx2
								) const;

		/** Returns a new variable containing only the given columns of the variable.
		 *
		 *	@param[in] idx1			Index of first column of new variable.
		 *	@param[in] idx2			Index following last column of new variable.
		 *
		 *	\return New variable containing only the given columns of the variable
		 */
		ExportVariable getCols(	uint idx1,
								uint idx2
								) const;

		/** Returns a new variable containing only the given columns of the variable.
		 *
		 *	@param[in] idx1			Variable index of first column of new variable.
		 *	@param[in] idx2			Variable index following last column of new variable.
		 *
		 *	\return New variable containing only the given columns of the variable
		 */
		ExportVariable getCols(	const ExportIndex& idx1,
								const ExportIndex& idx2
								) const;

		/** Returns a new variable containing only the given rows and columns of the variable.
		 *
		 *	@param[in] rowIdx1		Index of first row of new variable.
		 *	@param[in] rowIdx2		Index following last row of new variable.
		 *	@param[in] colIdx1		Index of first column of new variable.
		 *	@param[in] colIdx2		Index following last column of new variable.
		 *
		 *	\return New variable containing only the given sub-matrix of the variable
		 */
		ExportVariable getSubMatrix(	uint rowIdx1,
										uint rowIdx2,
										uint colIdx1,
										uint colIdx2
										) const;

		/** Returns a new variable containing only the given rows and columns of the variable.
		 *
		 *	@param[in] rowIdx1		Variable index of first row of new variable.
		 *	@param[in] rowIdx2		Variable index following last row of new variable.
		 *	@param[in] colIdx1		Index of first column of new variable.
		 *	@param[in] colIdx2		Index following last column of new variable.
		 *
		 *	\return New variable containing only the given sub-matrix of the variable
		 */
		ExportVariable getSubMatrix(	const ExportIndex& rowIdx1,
										const ExportIndex& rowIdx2,
										uint colIdx1,
										uint colIdx2
										) const;

		/** Returns a new variable containing only the given rows and columns of the variable.
		 *
		 *	@param[in] rowIdx1		Index of first row of new variable.
		 *	@param[in] rowIdx2		Index following last row of new variable.
		 *	@param[in] colIdx1		Variable index of first column of new variable.
		 *	@param[in] colIdx2		Variable index following last column of new variable.
		 *
		 *	\return New variable containing only the given sub-matrix of the variable
		 */
		ExportVariable getSubMatrix(	uint rowIdx1,
										uint rowIdx2,
										const ExportIndex& colIdx1,
										const ExportIndex& colIdx2
										) const;

		/** Returns a new variable containing only the given rows and columns of the variable.
		 *
		 *	@param[in] rowIdx1		Variable index of first row of new variable.
		 *	@param[in] rowIdx2		Variable index following last row of new variable.
		 *	@param[in] colIdx1		Variable index of first column of new variable.
		 *	@param[in] colIdx2		Variable index following last column of new variable.
		 *
		 *	\return New variable containing only the given sub-matrix of the variable
		 */
		ExportVariable getSubMatrix(	const ExportIndex& rowIdx1,
										const ExportIndex& rowIdx2,
										const ExportIndex& colIdx1,
										const ExportIndex& colIdx2
										) const;


		/** Returns a copy of the variable that is transformed to a row vector.
		 *
		 *	\return Copy of the variable that is transformed to a row vector
		 */
		ExportVariable makeRowVector( ) const;

		/** Returns a copy of the variable that is transformed to a column vector.
		 *
		 *	\return Copy of the variable that is transformed to a column vector
		 */
		ExportVariable makeColVector( ) const;


		/** Returns whether variable is a vector.
		 *
		 *	\return BT_TRUE  iff variable is a vector, \n
		 *	        BT_FALSE otherwise
		 */
		BooleanType isVector( ) const;


		/** Returns the internal data matrix.
		 *
		 *	\return Internal data matrix
		 */
		Matrix getGivenMatrix( ) const;


		/** Prints contents of variable to screen.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue print( ) const;


	//
    // PROTECTED MEMBER FUNCTIONS:
    //
    protected:

		/** Returns column dimension of the variable.
		 *
		 *	\return Column dimension of the variable
		 */
		virtual uint getColDim( ) const;


		/** Returns total index of given component within memory.
		 *
		 *	@param[in] rowIdx		Row index of the component.
		 *	@param[in] colIdx		Column index of the component.
		 *
		 *	\return Total index of given component
		 */
		virtual uint getTotalIdx(	uint rowIdx,
									uint colIdx
									) const;

		/** Returns total index of given component within memory.
		 *
		 *	@param[in] rowIdx		Row index of the component.
		 *	@param[in] colIdx		Column index of the component.
		 *
		 *	\return Total index of given component
		 */
		virtual ExportIndex	getTotalIdx(	const ExportIndex& rowIdx,
											const ExportIndex& colIdx
											) const;


		/** Assigns offsets and dimensions of a sub-matrix. This function is used to 
		 *	access only a sub-matrix of the variable without copying its values to 
		 *	a new variable.
		 *
		 *	@param[in] _rowOffset		Index of first row of sub-matrix.
		 *	@param[in] _colOffset		Index of first column of sub-matrix.
		 *	@param[in] _colDim			Column dimension of original variable (as only the submatrix data is stored).
		 *	@param[in] _nRows			Number of rows of sub-matrix.
		 *	@param[in] _nCols			Number of columns of sub-matrix.
		 *
		 *	\return SUCCESSFUL_RETURN, \n
		 *	        RET_INVALID_ARGUMENTS
		 */
		returnValue setSubmatrixOffsets(	uint _rowOffset = 0,
											uint _colOffset = 0,
											uint _colDim = 0,
											uint _nRows = 0,
											uint _nCols = 0
											);

		/** Assigns offsets and dimensions of a sub-matrix. This function is used to 
		 *	access only a sub-matrix of the variable without copying its values to 
		 *	a new variable.
		 *
		 *	@param[in] _rowOffset		Variable index of first row of sub-matrix.
		 *	@param[in] _colOffset		Variable index of first column of sub-matrix.
		 *	@param[in] _colDim			Column dimension of variable (as only the submatrix data is stored).
		 *	@param[in] _nRows			Number of rows of sub-matrix.
		 *	@param[in] _nCols			Number of columns of sub-matrix.
		 *
		 *	\return SUCCESSFUL_RETURN, \n
		 *	        RET_INVALID_ARGUMENTS
		 */
		returnValue setSubmatrixOffsets(	const ExportIndex& _rowOffset,
											const ExportIndex& _colOffset,
											uint _colDim = 0,
											uint _nRows = 0,
											uint _nCols = 0
											);


		/** Returns whether given component is set to given value.
		 *
		 *	@param[in] rowIdx		Variable row index of the component.
		 *	@param[in] colIdx		Variable column index of the component.
		 *	@param[in] _value		Value used for comparison.
		 *
		 *	\return BT_TRUE  iff given component is set to given value, \n
		 *	        BT_FALSE otherwise
		 */
		BooleanType hasValue(	const ExportIndex& rowIdx,
								const ExportIndex& colIdx,
								double _value
								) const;

		/** Returns whether given component is set to given value.
		 *
		 *	@param[in] rowIdx		Row index of the component.
		 *	@param[in] colIdx		Variable column index of the component.
		 *	@param[in] _value		Value used for comparison.
		 *
		 *	\return BT_TRUE  iff given component is set to given value, \n
		 *	        BT_FALSE otherwise
		 */
		BooleanType hasValue(	const ExportIndex& rowIdx,
								uint colIdx,
								double _value
								) const;

		/** Returns whether given component is set to given value.
		 *
		 *	@param[in] rowIdx		Variable row index of the component.
		 *	@param[in] colIdx		Column index of the component.
		 *	@param[in] _value		Value used for comparison.
		 *
		 *	\return BT_TRUE  iff given component is set to given value, \n
		 *	        BT_FALSE otherwise
		 */
		BooleanType hasValue(	uint rowIdx,
								const ExportIndex& colIdx,
								double _value
								) const;

		/** Returns whether given component is set to given value.
		 *
		 *	@param[in] rowIdx		Row index of the component.
		 *	@param[in] colIdx		Column index of the component.
		 *	@param[in] _value		Value used for comparison.
		 *
		 *	\return BT_TRUE  iff given component is set to given value, \n
		 *	        BT_FALSE otherwise
		 */
		BooleanType hasValue(	uint rowIdx,
								uint colIdx,
								double _value
								) const;


	protected:

		BooleanType doAccessTransposed;				/**< Flag indicating whether variable is to be accessed in a transposed manner. */

		ExportIndex rowOffset;						/**< Index of first row of a possible sub-matrix of the variable. */
		ExportIndex colOffset;						/**< Index of first column of a possible sub-matrix of the variable. */
		uint colDim;								/**< Column dimension of variable (as only the submatrix data is stored). */
		uint nRows;									/**< Number of rows of a possible sub-matrix of the variable. */
		uint nCols;									/**< Number of columns of a possible sub-matrix of the variable. */
};


static const ExportVariable emptyConstExportVariable;


CLOSE_NAMESPACE_ACADO


#endif  // ACADO_TOOLKIT_EXPORT_VARIABLE_HPP

// end of file.
