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
 *    \file include/acado/matrix_vector/vector.hpp
 *    \author Hans Joachim Ferreau, Boris Houska
 */


#ifndef ACADO_TOOLKIT_VECTOR_HPP
#define ACADO_TOOLKIT_VECTOR_HPP


BEGIN_NAMESPACE_ACADO


class Function;
class Matrix;


/**
 *	\brief Implements a rudimentary dense vector class.
 *
 *	\ingroup BasicDataStructures
 *
 *  The class Vector is a rudimentary dense vector class. It is only 
 *  intended to provide a convenient way to deal with linear algebra objects 
 *  and to provide a wrapper for more efficient implementations. It should 
 *  not be used for efficiency-critical operations, in particular not for 
 *  large-scale or sparse (matrix) objects.
 *
 *	 \author Hans Joachim Ferreau, Boris Houska
 */
class Vector : public VectorspaceElement{

    //
    // PUBLIC MEMBER FUNCTIONS:
    //
    public:
        /** Default constructor. */
        Vector( );

		/** Constructor which takes dimension of the vector. */
        Vector(	uint _dim	/**< Vector dimension. */
				);

		/** Constructor which takes dimension of the vector and a double array 
		 *  of appropriate size containing the (initial) values of the vector. */
        Vector(	uint _dim,		/**< Vector dimension. */
				const double* const _values	/**< Double array. */
				);


		/** Constructor which takes a file. */
        Vector(	FILE *file	/**< Vector dimension. */
				);

//	/** Constructor which takes a filename */
//        Vector( const char * filename	/**< Vector dimension. */
//				);

        /** Copy constructor (deep copy). */
        Vector(	const Vector& rhs	/**< Right-hand side object. */
				);

        /** Copy constructor (deep copy). */
        Vector(	const VectorspaceElement& rhs	/**< Right-hand side object. */
				);

        /** Destructor. */
        virtual ~Vector( );


        /** Assignment operator (deep copy). */
		Vector& operator=(	const Vector& rhs	/**< Right-hand side object. */
							);

		Vector& operator=(	FILE *rhs	/**< Right-hand side object. */
							);



		/** Adds (element-wise) two vectors to a temporary object.
		 *  \return Temporary object containing the sum of the vectors. */
		inline Vector operator+(	const Vector& arg	/**< Second summand. */
									) const;

		/** Adds (element-wise) a vector to object.
		 *  \return Reference to object after addition. */
		inline Vector& operator+=(	const Vector& arg	/**< Second summand. */
									);

		/** Subtracts (element-wise) a vector from the object and and stores 
		 *  the result to a temporary object.
		 *  \return Temporary object containing the difference of the vectors. */
		inline Vector operator-(	const Vector& arg	/**< Subtrahend. */
									) const;

		/** Subtracts (element-wise) a vector from the object.
		 *  \return Reference to object after subtraction. */
		inline Vector operator-=(	const Vector& arg	/**< Subtrahend. */
									);

		/** Multiplies each component of the object with a given scalar.
		 *  \return Reference to object after multiplication. */
		inline Vector& operator*=(	double scalar	/**< Scalar factor. */
									);

		/** Divides each component of the object by a given non-zero scalar. 
		 *  \return Reference to object after division. */
		inline Vector& operator/=(	double scalar	/**< Scalar divisor. If it is zero, nothing is done. */
									);

		/** Multiplies the transposed object to a given matrix from the left and
		 *  stores the result to a temporary object.
		 *  \return Temporary object containing result of multiplication. */
		inline Vector operator*(	const Matrix& arg	/**< Matrix factor. */
									) const;

		/** Multiplies the transposed object to a given vector (scalar product).
		 *  \return Scalar product of the two vectors. */
		inline double operator^(	const Vector& arg	/**< Vector factor. */
									) const;

		/** Calculated the dyadic product of the object and a given vector.
		 *  \return Dyadic product of the two vectors. */
		inline Matrix operator%(	const Vector& arg	/**< Vector factor. */
									) const;


		returnValue append( const Vector& arg );


        /** Returns a vector whose components are the absolute
         *  values of the components of this object.
		 *  DOES THE SAME AS GETABSOLUTE AND SHOULD BE ABANDONED SOON! 
         */
        inline Vector absolute();

        /** Returns a vector whose components are the absolute
         *  values of the components of this object. 
         */
        inline Vector getAbsolute() const;


		/** Sets vector to the <idx>th unit vector.
		 *  \return SUCCESSFUL_RETURN */
		inline returnValue setUnitVector(	uint idx	/**< Index. */
											);


    //
    // PROTECTED MEMBER FUNCTIONS:
    //
    protected:



    //
    // DATA MEMBERS:
    //
    protected:
};


CLOSE_NAMESPACE_ACADO



// UNARY MINUS OPERATOR:
// ---------------------

REFER_NAMESPACE_ACADO Vector operator-(const REFER_NAMESPACE_ACADO Vector &arg);





#endif  // ACADO_TOOLKIT_VECTOR_HPP

/*
 *	end of file
 */

