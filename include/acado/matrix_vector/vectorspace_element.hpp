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
 *    \file include/acado/matrix_vector/vectorspace_element.hpp
 *    \author Hans Joachim Ferreau, Boris Houska, Milan Vukov
 */

#ifndef ACADO_TOOLKIT_VECTORSPACE_ELEMENT_HPP
#define ACADO_TOOLKIT_VECTORSPACE_ELEMENT_HPP

#include <acado/utils/acado_utils.hpp>

BEGIN_NAMESPACE_ACADO

class Vector;

/**
 *	\brief Basis of the rudimentary dense Matrix and Vector classes.
 *
 *	\ingroup BasicDataStructures
 *
 *  The class VectorspaceElement forms the basis of very rudimentary dense
 *  matrix/vector classes. These classes are only intended to provide a 
 *  convenient way to deal with linear algebra objects and to provide a
 *  wrapper for more efficient implementations. They should not be used 
 *  for efficiency-critical operations, in particular not for large-scale 
 *  or sparse (matrix) objects. \n
 *
 *  The class VectorspaceElement mainly implements (element-wise) relational 
 *  operators and can be used to derive further special variants of vector 
 *  space objects.
 *
 *	\author Hans Joachim Ferreau, Boris Houska, Milan Vukov
 */
class VectorspaceElement
{
    //
    // PUBLIC MEMBER FUNCTIONS:
    //
    public:
        /** Default constructor. */
        VectorspaceElement( );

		/** Constructor which takes dimension of the vectorspace. */
        VectorspaceElement(	uint _dim	/**< Vector space dimension. */
							);

		/** Constructor which takes dimension of the vector space and  
		 *  a double array of appropriate size containing the 
		 *  (initial) values of the vector space element. */
        VectorspaceElement(	uint _dim,					/**< Vector space dimension. */
							const double* const _values	/**< Double array. */
							);

        /** Copy constructor (deep copy). */
        VectorspaceElement(	const VectorspaceElement& rhs	/**< Right-hand side object. */
							);

        /** Copy constructor (deep copy). */
        VectorspaceElement(	const Vector& rhs	/**< Right-hand side object. */
							);

        /** Destructor. */
        virtual ~VectorspaceElement( );

        /** Assignment operator (deep copy). */
        VectorspaceElement& operator=(	const VectorspaceElement& rhs	/**< Right-hand side object. */
										);

        /** Assignment operator (deep copy). */
        VectorspaceElement& operator<<( double *rhs );


        /** Assignment operator (deep copy). */
        friend double* operator<<( double *lhs, VectorspaceElement &rhs );

        inline returnValue convert( double *lhs ) const;


		/** Initializes vector space element with values taken from a double array
		 *  of appropriate size. Previously allocated internal memory is freed.
		 *  \return SUCCESSFUL_RETURN */
		returnValue init(	uint _dim = 0,	/**< Vector space dimension. */
							double* _values = 0
							);


		/** Access operator that return the value of a certain component.
		 *  \return Value of component <idx> */
 		inline double& operator()(	uint idx	/**< Index of the component to be returned. */
									);

		/** Access operator that return the value of a certain component (const variant).
		 *  \return Value of component <idx> */
		inline double operator()(	uint idx	/**< Index of the component to be returned. */
									) const;


		/** Tests for element-wise equality.
		 *  \return BT_TRUE iff both objects are element-wise equal. */
		inline BooleanType operator==(	const VectorspaceElement& arg	/**< Object of comparison. */
										) const;

		/** Tests for non-equality.
		 *  \return BT_TRUE iff both objects differ in at least one component. */
		inline BooleanType operator!=(	const VectorspaceElement& arg	/**< Object of comparison. */
										) const;

		/** Tests for element-wise smaller-than.
		 *  \return BT_TRUE iff left object is element-wise smaller than the right one. */
		inline BooleanType operator<(	const VectorspaceElement& arg	/**< Object of comparison. */
										) const;

		/** Tests for element-wise smaller/equal-than.
		 *  \return BT_TRUE iff left object is element-wise smaller/equal than the right one. */
		inline BooleanType operator<=(	const VectorspaceElement& arg	/**< Object of comparison. */
										) const;

		/** Tests for element-wise greater-than.
		 *  \return BT_TRUE iff left object is element-wise greater than the right one. */
		inline BooleanType operator>(	const VectorspaceElement& arg	/**< Object of comparison. */
										) const;

		/** Tests for element-wise greater/equal-than.
		 *  \return BT_TRUE iff left object is element-wise greater/equal than the right one. */
		inline BooleanType operator>=(	const VectorspaceElement& arg	/**< Object of comparison. */
										) const;

		returnValue append( const VectorspaceElement& arg );

		/** Returns dimension of vector space. */
		inline uint getDim( ) const;

		/** Returns whether the vectorspace element is empty. */
		inline BooleanType isEmpty( ) const;

		/** Returns whether all components are equal to _value. */
		inline BooleanType isEqualTo(	double _value
										) const;

		inline BooleanType isGreaterThan(	double _value
											) const;

		inline BooleanType isSmallerThan(	double _value
											) const;

		inline BooleanType isZero( ) const;

		inline BooleanType isPositive( ) const;

		inline BooleanType isNegative( ) const;
		
		inline BooleanType isFinite( ) const;

		inline BooleanType hasNaN( ) const;

		/** Returns whether all components have same value.
		 *  \return BT_TRUE  iff all components have same value,
		 *			BT_FALSE otherwise. */
		inline BooleanType hasEqualComponents( ) const;

		/** Sets all component to zero.
		 *
		 *  \return SUCCESSFUL_RETURN
		 */
		inline returnValue setZero( );

		/** Sets all component to given value.
		 *
		 *  \return SUCCESSFUL_RETURN
		 */
		inline returnValue setAll(	double _value	/**< Value for all components. */
									);

		/** Returns maximum element.
		 *
		 *  \return Maximum element.
		 */
		inline double getMax( ) const;

		/** Returns minimum element.
		 *  \return minimum element. */
		inline double getMin( ) const;

		/** Returns mean value of all elements.
		 *  \return Mean value of all elements. */
		inline double getMean( ) const;

        /** Returns specified norm of the vectorspace element \n
         *  interpreted as a vector.                          \n
         *                                                    \n
         *  \param norm   the type of norm to be computed.    \n
         *                                                    \n
         *  \return Norm of the vector.                       \n
         */
        double getNorm( VectorNorm norm ) const;

        /** Returns specified norm of the vectorspace element \n
         *  interpreted as a vector (with scaling).           \n
         *                                                    \n
         *  \param norm   the type of norm to be computed.    \n
         *  \param scale  the elementwise scale.              \n
         *                                                    \n
         *  \return Norm of the vector.                       \n
         */
        double getNorm( VectorNorm                norm ,
                        const VectorspaceElement &scale  ) const;

        /** Get a pointer to underlying data structure. */
        double* getDoublePointer( );

		/** Prints object to given file. Various settings can
		 *	be specified defining its output format. 
		 *
		 *	@param[in] stream			Output stream for printing.
		 *	@param[in] name				Name label to be printed before the numerical values.
		 *	@param[in] startString		Prefix before printing the numerical values.
		 *	@param[in] endString		Suffix after printing the numerical values.
		 *	@param[in] width			Total number of digits per single numerical value.
		 *	@param[in] precision		Number of decimals per single numerical value.
		 *	@param[in] colSeparator		Separator between the columns of the numerical values.
		 *	@param[in] rowSeparator		Separator between the rows of the numerical values.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_FILE_CAN_NOT_BE_OPENED, \n
		 *	        RET_UNKNOWN_BUG
		 */
		virtual returnValue print(	std::ostream& stream           = std::cout,
									const char* const name         = DEFAULT_LABEL,
									const char* const startString  = DEFAULT_START_STRING,
									const char* const endString    = DEFAULT_END_STRING,
									uint width                     = DEFAULT_WIDTH,
									uint precision                 = DEFAULT_PRECISION,
									const char* const colSeparator = DEFAULT_COL_SEPARATOR,
									const char* const rowSeparator = DEFAULT_ROW_SEPARATOR
									) const;

		/** Prints object to file with given name. Various settings can
		 *	be specified defining its output format.
		 *
		 *	@param[in] filename			Filename for printing.
		 *	@param[in] name				Name label to be printed before the numerical values.
		 *	@param[in] startString		Prefix before printing the numerical values.
		 *	@param[in] endString		Suffix after printing the numerical values.
		 *	@param[in] width			Total number of digits per single numerical value.
		 *	@param[in] precision		Number of decimals per single numerical value.
		 *	@param[in] colSeparator		Separator between the columns of the numerical values.
		 *	@param[in] rowSeparator		Separator between the rows of the numerical values.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_FILE_CAN_NOT_BE_OPENED, \n
		 *	        RET_UNKNOWN_BUG
		 */
		virtual returnValue print(	const char* const filename,
									const char* const name         = DEFAULT_LABEL,
									const char* const startString  = DEFAULT_START_STRING,
									const char* const endString    = DEFAULT_END_STRING,
									uint width                     = DEFAULT_WIDTH,
									uint precision                 = DEFAULT_PRECISION,
									const char* const colSeparator = DEFAULT_COL_SEPARATOR,
									const char* const rowSeparator = DEFAULT_ROW_SEPARATOR
									) const;

		/** Prints object to given file. Various settings can
		 *	be specified defining its output format. 
		 *
		 *	@param[in] stream			Output stream for printing.
		 *	@param[in] name				Name label to be printed before the numerical values.
		 *	@param[in] printScheme		Print scheme defining the output format of the information.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_FILE_CAN_NOT_BE_OPENED, \n
		 *	        RET_UNKNOWN_BUG
		 */
		virtual returnValue print(	std::ostream& stream,
									const char* const name,
									PrintScheme printScheme
									) const;

		/** Prints object to given file. Various settings can
		 *	be specified defining its output format.
		 *
		 *	@param[in] filename			Filename for printing.
		 *	@param[in] name				Name label to be printed before the numerical values.
		 *	@param[in] printScheme		Print scheme defining the output format of the information.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_FILE_CAN_NOT_BE_OPENED, \n
		 *	        RET_UNKNOWN_BUG
		 */
		virtual returnValue print(	const char* const filename,
									const char* const name,
									PrintScheme printScheme
									) const;

		/** Read data from an input file. */
		virtual returnValue read(	std::istream& stream
									);

		/** Read data from an input file. */
		virtual returnValue read(	const char* const filename
									);

		/** Output streaming operator. */
		friend std::ostream& operator<<(	std::ostream& stream,
											const VectorspaceElement& arg
											);

		/** Input streaming operator. */
		friend std::istream& operator>>(	std::istream& stream,
											VectorspaceElement& arg
											);

    //
    // DATA MEMBERS:
    //
    protected:
        double* element;			/**< Element of vector space. */
		unsigned int dim;			/**< Vector space dimension. */
};

CLOSE_NAMESPACE_ACADO

#include <acado/matrix_vector/vectorspace_element.ipp>

#endif  // ACADO_TOOLKIT_VECTORSPACE_ELEMENT_HPP

/*
 *	end of file
 */
