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
 *    \file include/acado/matrix_vector/matrix.hpp
 *    \author Hans Joachim Ferreau, Boris Houska
 */


#ifndef ACADO_TOOLKIT_MATRIX_HPP
#define ACADO_TOOLKIT_MATRIX_HPP

#ifdef _WIN32
    #include <memory>
#else
    #include <tr1/memory>
#endif


BEGIN_NAMESPACE_ACADO


class Vector;
class MatrixVariablesGrid;
class VariablesGrid;
class SparseSolver;


/**
 *  \brief Implements a rudimentary dense matrix class.
 *
 *	\ingroup BasicDataStructures
 *
 *  It is intended to provide a convenient way to deal with linear algebra    \n
 *  objects. The operators that are defined for the Matrix and Vector class   \n
 *  allow a convenient usage of these objects.                                \n
 *                                                                            \n
 *  Example Code:                                                             \n
 *                                                                            \n
 *  - Matrix-Vector multiplication:                                           \n
 *                                                                            \n
 *    \verbatim
         Matrix  A(2,2);               // construct a 2x2 matrix
         Vector  b(2)  ;               // construct a vector with dimension 2
         Vector  c     ;               // the vector c := A*b  to be computed.

         A(0,0) = 1.0;  A(0,1) = 0.5;  // definition of
         A(1,0) = 0.0;  A(1,1) = 1.5;  // a matrix A

         b(0)   = 1.0;  b(1)   = 2.0;  // definition of a vector b

         c = A*b;                      // matrix-vector multiplication

         c.print("c");                 // printing the result.
      \endverbatim                                                            \n
 *                                                                            \n
 *  - Some matrix-matrix computations:                                        \n
 *                                                                            \n
 *    \verbatim
         Matrix  A(2,2);               // construct a 2x2 matrix A
         Matrix  B(2,2);               // construct another matrix B
         Matrix  C(2,2);               // construct a matrix C
         Matrix  D     ;               // the matrix D := A*B + B^T*A + C to be computed.

         A(0,0) = 1.0;  A(0,1) = 0.5;  // definition of
         A(1,0) = 0.0;  A(1,1) = 1.5;  // a matrix A

         b(0)   = 1.0;  b(1)   = 0.0;  // definition of
         b(0)   = 0.0;  b(1)   = 2.0;  // a matrix B

         D = A*B + B^A + C;            // computing the matrix D.

         D.print("D");                 // printing the result.
      \endverbatim                                                            \n
 *                                                                            \n
 *  - Computing the Eigenvalues of a symmetric matrix:                        \n
 *                                                                            \n
 *    \verbatim
         Matrix A(3,3);                // construct and define a 3x3 matrix A.

         A(0,0) = 1.0;  A(0,1) = 0.0;  A(0,2) = 0.0;
         A(1,0) = 0.0;  A(1,1) = 3.0;  A(1,2) = 2.0;
         A(2,0) = 0.0;  A(2,1) = 2.0;  A(2,2) = 3.0;

         A.printEigenvalues();         // print the eigenvalues of the matrix A.
      \endverbatim                                                            \n
 *                                                                            \n
 *
 *  \sa Vector
 *
 *  \author Hans Joachim Ferreau, Boris Houska
 */



class Matrix : public VectorspaceElement{


    //
    // PUBLIC MEMBER FUNCTIONS:
    //
    public:


        /** \brief Default constructor. */
        Matrix( );


        /** \brief Constructor which takes the dimensions of the matrix. \n
         *                                                               \n
         *  @param _nRows Number of rows.                                \n
         *  @param _nCols Number of columns.                             \n
         */
        Matrix( uint _nRows, uint _nCols );


        /** \brief Constructor which takes dimension of the matrix and a double array \n
         *  of appropriate size (_nRows*_nCols) containing the (initial) values       \n
         *  of the matrix.                                                            \n
         *
         *  @param _nRows Number of rows.                                             \n
         *  @param _nCols Number of columns.                                          \n
         *  @param _nCols Double array.                                               \n
         */
        Matrix( uint _nRows, uint _nCols, const double* const _values );


        /** Constructor which takes a file.                                                      \n
         *                                                                                       \n
         *  Example:                                                                             \n
         *                                                                                       \n
         *  \verbatim
            Matrix A = fopen("matrix.dat", "r");  // loading a matrix from the file "matrix.dat"
            A.print("A");                         // printing the matrix.
            \endverbatim                                                                         \n
         *                                                                                       \n
         *  @param file A file from which the matrix should be loaded.                           \n
         *                                                                                       \n
         */
        Matrix( FILE *file );


        /** \brief Constructor which takes an integer to construct a 1x1-matrix. */
        Matrix( int value /**< The value of the matrix element*/ );

        /** \brief Constructor which takes a double to construct a 1x1-matrix. */
        Matrix( double value /**< The value of the matrix element*/ );

        /** \brief Constructor to convert a Vector into a matrix. */
        Matrix(	const Vector& value, /**< The vector containing the elements for the dim-by-1 matrix. */
				BooleanType doTranspose = BT_FALSE
				);

        /** \brief Constructor to convert a VariabledGrid into a matrix. */
        Matrix( const VariablesGrid& value /**< The grid values for the matrix. */ );

        /** \brief Copy constructor (deep copy). */
        Matrix( const Matrix& rhs /**< Right-hand side object. */ );

        /** \brief Destructor. */
        virtual ~Matrix( );


        /** \brief Initializer, which takes the dimensions of the matrix.  \n
         *
         *
         *  @param _nRows Number of rows.                                  \n
         *  @param _nCols Number of columns.                               \n
         */
        returnValue init( uint _nRows, uint _nCols );



        /** \brief Initializer,which takes dimension of the matrix and a double array \n
         *  of appropriate size (_nRows*_nCols) containing the (initial)              \n
         *  values of the matrix.                                                     \n
         *                                                                            \n
         *  @param _nRows Number of rows.                                             \n
         *  @param _nCols Number of columns.                                          \n
         *  @param _nCols Double array.                                               \n
         */
        returnValue init( uint _nRows, uint _nCols, double* _values );



        /** \brief Appends rows at the end of the matrix.
         */
        returnValue appendRows( const Matrix& arg /**< The matrix to be appended.*/ );



        /** \brief Appends columns at the end of the matrix. */
        returnValue appendCols( const Matrix& arg /**< The matrix to be appended.*/ );



        /** \brief Assignment operator (deep copy). */
        Matrix& operator=( const Matrix& rhs /**< The right-hand side object. */ );


        /** \brief Assignment operator, which loads a matrix from a file. */
        Matrix& operator=( FILE *rhs /**< A file containing the matrix data. */ );


        /** \brief Assignment operator, which assigns the elements from a double*. */
        Matrix& operator^=( const double *rhs );


        /** \brief Assignment operator, which assigns the elements from a double*. \n
         *  (friend version)                                                       \n
         */
        friend double* operator^=( double *lhs, Matrix &rhs );



		/** Access operator that return the value of a certain component.
		 *  \return Value of component (<rowIdx>,<colIdx>) */
		inline double& operator()(	uint rowIdx,	/**< Row index of the component to be returned. */
									uint colIdx		/**< Column index of the component to be returned. */
									);

		/** Access operator that return the value of a certain component (const variant).
		 *  \return Value of component (<rowIdx>,<colIdx>) */
		inline double operator()(	uint rowIdx,	/**< Row index of the component to be returned. */
									uint colIdx		/**< Column index of the component to be returned. */
									) const;


		/** Adds (element-wise) two matrices to a temporary object.
		 *  \return Temporary object containing the sum of the matrices. */
		inline Matrix operator+(	const Matrix& arg	/**< Second summand. */
									) const;

		/** Adds (element-wise) a matrix to object.
		 *  \return Reference to object after addition. */
		inline Matrix& operator+=(	const Matrix& arg	/**< Second summand. */
									);
		/**
		*Computes the column-wise sum the Matrix
		* \return sum
                *
                * Example:
                * \code
                * a   |  b
                * c   |  d
                * \endcode
                * returns [a+b;c+d]
		*/
		inline Vector sumCol(const Matrix& arg);

		/**
		* Computes the row-wise sum the Matrix
		* \return sum
                * Example:
                * \code
                * a   |  b
                * c   |  d
                * \endcode
                * returns [a+c|b+d]
		*/
		inline Vector sumRow(const Matrix& arg);

		/** Subtracts (element-wise) a matrix from the object and and stores
		 *  the result to a temporary object.
		 *  \return Temporary object containing the difference of the matrices. */
		inline Matrix operator-(	const Matrix& arg	/**< Subtrahend. */
									) const;

		/** Subtracts (element-wise) a matrix from the object.
		 *  \return Reference to object after subtraction. */
		inline Matrix& operator-=(	const Matrix& arg	/**< Subtrahend. */
									);

		/** Multiplies each component of the object with a given scalar.
		 *  \return Reference to object after multiplication. */
		inline Matrix& operator*=(	double scalar	/**< Scalar factor. */
									);

		/** Divides each component of the object by a given non-zero scalar.
		 *  \return Reference to object after division. */
		inline Matrix& operator/=(	double scalar	/**< Scalar divisor. If it is zero, nothing is done. */
									);


		 /** Tests for equality
		 *	@param[in] rhs	Object of comparison.
		 *
		 *  \return BT_TRUE  iff both objects are equal, \n
		 *	        BT_FALSE otherwise
		 */
		inline BooleanType operator==(	const Matrix& arg
										) const;



		/** Multiplies a matrix from the right to the matrix object and
		 *  stores the result to a temporary object.
		 *  \return Temporary object containing result of multiplication. */
		inline Matrix operator*(	const Matrix& arg	/**< Matrix factor. */
									) const;


		/** Multiplies a matrix from the right to the transposed matrix object and
		 *  stores the result to a temporary object.
		 *  \return Temporary object containing result of multiplication. */
		inline Matrix operator^(	const Matrix& arg	/**< Matrix factor. */
									) const;

		/** Multiplies a vector from the right to the matrix object and
		 *  stores the result to a temporary object.
		 *  \return Temporary object containing result of multiplication. */
		inline Vector operator*(	const Vector& arg	/**< Vector factor. */
									) const;

		/** Multiplies a vector from the right to the transposed matrix object and
		 *  stores the result to a temporary object.
		 *  \return Temporary object containing result of multiplication. */
		inline Vector operator^(	const Vector& arg	/**< Vector factor. */
									) const;


        inline Matrix transpose() const;

        inline Matrix negativeTranspose() const;


		inline Matrix& makeVector( );


		/** Returns number of rows of the matrix object.
		 *  \return Number of rows. */
		inline uint getNumRows( ) const;

		/** Returns number of columns of the matrix object.
		 *  \return Number of columns. */
		inline uint getNumCols( ) const;


		/** Returns a given row of the matrix object.
		 *  \return Temporary object containing the row. */
		inline Vector getRow(	uint idx		/**< Index of the row to be returned. */
								) const;

		/** Returns a given column of the matrix object.
		 *  \return Temporary object containing the column. */
		inline Vector getCol(	uint idx		/**< Index of the column to be returned. */
								) const;

		/** Assigns new values to a given row of the matrix object.
		 *  \return SUCCESSFUL_RETURN */
		inline returnValue setRow(	uint idx,			/**< Row index. */
									const Vector& arg	/**< New values of the row. */
									);

		/** Assigns new values to a given column of the matrix object.
		 *  \return SUCCESSFUL_RETURN */
		inline returnValue setCol(	uint idx,			/**< Column index. */
									const Vector& arg	/**< New values of the column. */
									);


		/** Returns given rows of the matrix object.
		 *  \return Temporary object containing the rows. */
		inline Matrix getRows(	uint idx1,	/**< Start index of the rows to be returned. */
								uint idx2	/**< End index of the rows to be returned. */
								) const;

		/** Returns given columns of the matrix object.
		 *  \return Temporary object containing the columns. */
		inline Matrix getCols(	uint idx1,		/**< Start index of the columns to be returned. */
								uint idx2		/**< End index of the columns to be returned. */
								) const;


		/** Returns a vector containing the diagonal elements of a square matrix.
		 *  \return Vector containing the diagonal elements. */
		inline Vector getDiag( ) const;


		/** Sets object to the identity matrix.
		 *  \return SUCCESSFUL_RETURN */
		inline returnValue setIdentity( );

		/** Tests if object is a square matrix.
		 *  \return BT_TRUE iff matrix object is square. */
		inline BooleanType isSquare( ) const;


		/** Tests if object is a symmetric matrix.
		 *  \return BT_TRUE iff matrix object is symmetric. */
		BooleanType isSymmetric( ) const;
		
		/** Tests if object is a diagonal matrix.
		 *  \return BT_TRUE iff matrix object is diagonal. */
		BooleanType isDiagonal( ) const;

		returnValue symmetrize( );


		/** Tests if object is a positive semi-definite matrix.
		 *	Note that this test involves a Cholesky decomposition and
		 *	thus is computationally expensive for larger matrix dimensions.
		 *  \return BT_TRUE iff matrix object is positive definite. */
		BooleanType isPositiveSemiDefinite( ) const;
		
		/** Tests if object is a (strictly) positive definite matrix.
		 *	Note that this test involves a Cholesky decomposition and
		 *	thus is computationally expensive for larger matrix dimensions.
		 *  \return BT_TRUE iff matrix object is (strictly) positive definite. */
		BooleanType isPositiveDefinite( ) const;

        /** Returns the a matrix whose components are the absolute
         *  values of the components of this object.
         */
        inline Matrix absolute();


        /** Returns the a matrix whose components are equal to
         *  the components of this object, if they are positive or zero,
         *  but zero otherwise.
         */
        inline Matrix positive();


        /** Returns the a matrix whose components are equal to
         *  the components of this object, if they are negative or zero,
         *  but zero otherwise.
         */
        inline Matrix negative();


        /** Switches the sign of all components.
         */
        inline Matrix minus();


		/** Returns specified norm of the matrix.
		 *  \return Norm of the matrix. */
		double getNorm(	MatrixNorm norm
						) const;

		/** Returns trace of the matrix.
		 *  \return Trace of the matrix. */
		double getTrace( ) const;


        /** Returns a vector containing the eigenvalues of the matrix.    \n
         *  Note that this routine requires that the matrix is symmetric. \n
         *                                                                \n
         *  \return The eigenvalues of the matrix.
         */
        Vector getEigenvalues( ) const;


        /** Returns a vector containing the eigenvalues of the matrix as  \n
         *  well as the corresponding eigenvectors. The eigenvectors are  \n
         *  stored in the matrix Q (orthogonal matrix).                   \n
         *  Note that this routine requires that the matrix is symmetric. \n
         *                                                                \n
         *  \return The eigenvalues of the matrix.
         */
        Vector getEigenvalues( Matrix &Q ) const;


        /**  Prints the eigenvalues of the standard output stream.        \n
         *                                                                \n
         *  \return SUCCESSFUL_RETURN.
         */
        returnValue printEigenvalues( ) const;



        /**  Computes the Singular Value Decomposition of the matrix and  \n
         *   stores the result in the form                                \n
         *                                                                \n
         *             A = U D V^T ,                                      \n
         *                                                                \n
         *   where the matrices U and V are orthogonal. Note that         \n
         *   diagonal matrix D will be stored as a vector containing the  \n
         *   diagonal elements.                                           \n
         *                                                                \n
         *   \return SUCCESSFUL_RETURN                                    \n
         */
        returnValue getSingularValueDecomposition( Matrix &U, Vector &D, Matrix &V ) const;



        /**  Computes the inverse matrix. \n
         *
         *   \return The inverse A^{-1}.  \n
         */
        Matrix getInverse() const;



        /**  Computes the Cholesky Decomposition of the matrix            \n
         *                                                                \n
         *            A = L L^T                                           \n
         *                                                                \n
         *   \return The lower triangular matrix L.                       \n
         */
        Matrix getCholeskyDecomposition() const;



        /**  Computes the Cholesky Decomposition of the matrix            \n
         *                                                                \n
         *            A = L D L^T                                         \n
         *                                                                \n
         *   \return The lower triangular matrix L.                       \n
         */
        Matrix getCholeskyDecomposition( Vector &D ) const;



        /**  Computes the inverse of a positive definite matrix based on  \n
         *   the Cholesky decomposition.                                  \n
         *                                                                \n
         *   \return  The matrix  A^{-1} = L^{-T}L^{-1}                   \n
         */
        Matrix getCholeskyInverse() const;



        /**  Computes the QR decomposition of the matrix.                 \n
         *   The result of the decomposition will be strored in this      \n
         *   matrix which will be increased by one column. The matrix     \n
         *   can then be used via the routine "solveQR".                  \n
         *                                                                \n
         *   \return  The matrix decomposition  A = QR  in an efficient   \n
         *            storage format.                                     \n
         */
        returnValue computeQRdecomposition();


        /**  Solves the system  A x = b provided that the routine         \n
         *   computeQRdecomposition() has been used before.               \n
         *                                                                \n
         *   \return  The solution x.                                     \n
         */
        Vector solveQR( const Vector &b ) const;


        /**  Solves the system  A^T x = b provided that the routine       \n
         *   computeQRdecomposition() has been used before.               \n
         *                                                                \n
         *   \return  The solution x.                                     \n
         */
        Vector solveTransposeQR( const Vector &b ) const;



        /**  Computes the sparse LU decomposition of the matrix.          \n
         *                                                                \n
         *   \return  The matrix decomposition  A = LU  in an efficient   \n
         *            sparse storage format.                              \n
         */
        returnValue computeSparseLUdecomposition();


        /**  Solves the system  A x = b provided that the routine         \n
         *   computeQRdecomposition() has been used before.               \n
         *                                                                \n
         *   \return  The solution x.                                     \n
         */
        Vector solveSparseLU( const Vector &b ) const;


        /**  Solves the system  A^T x = b provided that the routine       \n
         *   computeQRdecomposition() has been used before.               \n
         *                                                                \n
         *   \return  The solution x.                                     \n
         */
        Vector solveTransposeSparseLU( const Vector &b ) const;


        /** Prints the matrix into a file. \n
         *
         *  \return SUCCESSFUL_RETURN            \n
         *          RET_CAN_NOT_WRITE_INTO_FILE  \n
         */
        friend returnValue operator<<(	FILE          *file, /**< the file to print to */
										Matrix        &arg   /**< the matrix to print  */
										);

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
		virtual returnValue printToFile(	const char* const filename,
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
		 *	@param[in] file				File for printing.
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
		virtual returnValue printToFile(	FILE* file,
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
		 *	@param[in] printScheme		Print scheme defining the output format of the information.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_FILE_CAN_NOT_BE_OPENED, \n
		 *	        RET_UNKNOWN_BUG
		 */
		virtual returnValue printToFile(	const char* const filename,		/**< Filename for printing */
											const char* const name,
											PrintScheme printScheme
											) const;

		/** Prints object to given file. Various settings can
		 *	be specified defining its output format. 
		 *
		 *	@param[in] filen			File for printing.
		 *	@param[in] name				Name label to be printed before the numerical values.
		 *	@param[in] printScheme		Print scheme defining the output format of the information.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_FILE_CAN_NOT_BE_OPENED, \n
		 *	        RET_UNKNOWN_BUG
		 */
		virtual returnValue printToFile(	FILE* file,
											const char* const name,
											PrintScheme printScheme
											) const;

		/** Prints object to given string. Various settings can
		 *	be specified defining its output format. 
		 *
		 *	@param[in,out] string			File for printing.
		 *	@param[in]     name				Name label to be printed before the numerical values.
		 *	@param[in]     startString		Prefix before printing the numerical values.
		 *	@param[in]     endString		Suffix after printing the numerical values.
		 *	@param[in]     width			Total number of digits per single numerical value.
		 *	@param[in]     precision		Number of decimals per single numerical value.
		 *	@param[in]     colSeparator		Separator between the columns of the numerical values.
		 *	@param[in]     rowSeparator		Separator between the rows of the numerical values.
		 *	@param[in]     allocateMemory	Flag indicating whether memory for string shall be allocated.
		 *
		 *	\note If 'allocateMemory' flag is set to BT_TRUE, it is assumed that no
		 *	      memory is allocated to the 'string' pointer. Otherwise, it is assumed
		 *	      that sufficient memory has been allocated, e.g. by using the determineStringLength()
		 *	      member function.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_FILE_CAN_NOT_BE_OPENED, \n
		 *	        RET_UNKNOWN_BUG
		 */
		virtual returnValue printToString(	char** string,
											const char* const name         = DEFAULT_LABEL,
											const char* const startString  = DEFAULT_START_STRING,
											const char* const endString    = DEFAULT_END_STRING,
											uint width                     = DEFAULT_WIDTH,
											uint precision                 = DEFAULT_PRECISION,
											const char* const colSeparator = DEFAULT_COL_SEPARATOR,
											const char* const rowSeparator = DEFAULT_ROW_SEPARATOR,
											BooleanType allocateMemory     = BT_TRUE
											) const;

		/** Prints object to given string. Various settings can
		 *	be specified defining its output format. 
		 *
		 *	@param[in,out] string			File for printing.
		 *	@param[in]     name				Name label to be printed before the numerical values.
		 *	@param[in]     printScheme		Print scheme defining the output format of the information.
		 *	@param[in]     allocateMemory	Flag indicating whether memory for string shall be allocated.
		 *
		 *	\note If 'allocateMemory' flag is set to BT_TRUE, it is assumed that no
		 *	      memory is allocated to the 'string' pointer. Otherwise, it is assumed
		 *	      that sufficient memory has been allocated, e.g. by using the determineStringLength()
		 *	      member function.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_FILE_CAN_NOT_BE_OPENED, \n
		 *	        RET_UNKNOWN_BUG
		 */
		virtual returnValue printToString(	char** string,
											const char* const name,
											PrintScheme printScheme,
											BooleanType allocateMemory = BT_TRUE
											) const;


		/** Determines length of string required for printing object with 
		 *	given settings defining its output format. 
		 *
		 *	@param[in] name				Name label to be printed before the numerical values.
		 *	@param[in] startString		Prefix before printing the numerical values.
		 *	@param[in] endString		Suffix after printing the numerical values.
		 *	@param[in] width			Total number of digits per single numerical value.
		 *	@param[in] precision		Number of decimals per single numerical value.
		 *	@param[in] colSeparator		Separator between the columns of the numerical values.
		 *	@param[in] rowSeparator		Separator between the rows of the numerical values.
		 *
		 *  \return Length of string required for printing object
		 */
		virtual uint determineStringLength(	const char* const name,
											const char* const startString,
											const char* const endString,
											uint width,
											uint precision,
											const char* const colSeparator,
											const char* const rowSeparator
											) const;



    //
    // PROTECTED MEMBER FUNCTIONS:
    //
    protected:

        void householdersReductionToBidiagonalForm( double  *A            ,
                                                    int      nrows        ,
                                                    int      ncols        ,
                                                    double  *U            ,
                                                    double  *V            ,
                                                    double  *diagonal     ,
                                                    double  *superdiagonal  ) const;


        int givensReductionToDiagonalForm( int nrows,
                                           int ncols,
                                           double* U,
                                           double* V,
                                           double* diagonal,
                                           double* superdiagonal ) const;


        void sortByDecreasingSingularValues( int     nrows         ,
                                             int     ncols         ,
                                             double *singular_value,
                                             double *U             ,
                                             double *V               ) const;


        void lowerTriangularInverse( double *L, int n ) const;



    //
    // DATA MEMBERS:
    //
    protected:

		uint nRows;				/**< Number of rows.        */
		uint nCols;				/**< Number of columns.     */
        SparseSolver *solver;   /**< Optional sparse solver */

};

/** Shared pointer of the Matrix class. */
typedef std::tr1::shared_ptr< Matrix > matrixPtr;


CLOSE_NAMESPACE_ACADO


// UNARY MINUS OPERATOR:
// ---------------------

REFER_NAMESPACE_ACADO Matrix operator-(const REFER_NAMESPACE_ACADO Matrix &arg);



#endif  // ACADO_TOOLKIT_MATRIX_HPP

/*
 *	end of file
 */
