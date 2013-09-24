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
 *    \file include/acado/symbolic_expression/expression.hpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */


#ifndef ACADO_TOOLKIT_EXPRESSION_HPP
#define ACADO_TOOLKIT_EXPRESSION_HPP

#include <acado/utils/acado_utils.hpp>
#include <acado/matrix_vector/matrix_vector.hpp>
#include <acado/variables_grid/variables_grid.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>
#include <iostream>

BEGIN_NAMESPACE_ACADO


class ConstraintComponent;


/**
 *  \brief Base class for all variables within the symbolic expressions family.
 *
 *	\ingroup BasicDataStructures
 *
 *  The class Expression serves as a base class for all
 *  symbolic variables within the symbolic expressions family.
 *  Moreover, the Expression class defines all kind of matrix
 *  and vector operations on a symbolic level.
 *
 *  \author Boris Houska, Hans Joachim Ferreau
 */


class Expression{


    friend class COperator;
    friend class CFunction;
    friend class FunctionEvaluationTree;

    //
    // PUBLIC MEMBER FUNCTIONS:
    //

    public:

        /** Default constructor. */
        Expression( );

        /** Casting constructor. */
        Expression( const Operator &tree_ );

        /** Casting constructor. */
        Expression( const String &name_ );


        /** Constructor which takes the arguments. */
        explicit Expression( int          nRows_                     ,  /**< number of rows          */
                             int          nCols_         = 1         ,  /**< number of columns       */
                             VariableType variableType_  = VT_UNKNOWN,  /**< the variable type       */
                             int          globalTypeID   = 0         ,  /**< the global type ID      */
                             String       name_          = ""           /**< the name                */  );

        /** Constructor which takes the arguments. */
        explicit Expression( uint         nRows_                     ,  /**< number of rows          */
                             uint         nCols_         = 1         ,  /**< number of columns       */
                             VariableType variableType_  = VT_UNKNOWN,  /**< the variable type       */
                             uint         globalTypeID   = 0         ,  /**< the global type ID      */
                             String       name_          = ""           /**< the name                */  );


        /** Copy constructor (deep copy). */
        Expression( const double      & rhs );
        Expression( const Vector      & rhs );
        Expression( const Matrix      & rhs );

        Expression( const Expression  & rhs );


        /** Destructor. */
        virtual ~Expression( );


        /** Assignment Operator.                                             \n
         *                                                                   \n
         *  \param arg  the double value to be assigned to the expression.   \n
         */
        Expression& operator=( const double& arg );


        /** Assignment Operator.                                       \n
         *                                                             \n
         *  \param arg  the vector to be assigned to the expression.   \n
         */
        Expression& operator=( const Vector& arg );


        /** Assignment Operator.                                       \n
         *                                                             \n
         *  \param arg  the matrix to be assigned to the expression.   \n
         */
        Expression& operator=( const Matrix& arg );


        /** Assignment Operator.                         \n
         *                                               \n
         *  \param arg  the Expression to be assigned.   \n
         */
        Expression& operator=( const Expression& arg );


        Expression& operator<<( const double      & arg );
        Expression& operator<<( const Vector      & arg );
        Expression& operator<<( const Matrix      & arg );
        Expression& operator<<( const Expression  & arg );

	/**
	Appends an Expression matrix (n x m)
	with an argument matrix (s x m) in the row direction,
	such that the result is ( (n + s) x m).
	
	As a special case, when applied on an empty Expression,
	the Expression will be assigned the argument.
	
	
	*/
	Expression&  appendRows(const Expression& arg);
	
	/**
	Appends an Expression matrix (n x m)
	with an argument matrix (n x s) in the column direction,
	such that the result is ( n x (m + s) ).
	
	As a special case, when applied on an empty Expression,
	the Expression will be assigned the argument.
	
	*/
	Expression&  appendCols(const Expression& arg);

        Expression operator()( uint idx                 ) const;
        Expression operator()( uint rowIdx, uint colIdx ) const;

        Operator& operator()( uint idx                 );
        Operator& operator()( uint rowIdx, uint colIdx );


        Expression operator+( const double      & arg ) const;
        Expression operator+( const Vector      & arg ) const;
        Expression operator+( const Matrix      & arg ) const;
        Expression operator+( const Expression  & arg ) const;

        friend Expression operator+( const double       & arg1, const Expression& arg2 );
        friend Expression operator+( const Vector       & arg1, const Expression& arg2 );
        friend Expression operator+( const Matrix       & arg1, const Expression& arg2 );


        Expression operator-( const double      & arg ) const;
        Expression operator-( const Vector      & arg ) const;
        Expression operator-( const Matrix      & arg ) const;
        Expression operator-( const Expression  & arg ) const;

        Expression operator-( ) const;

        friend Expression operator-( const double       & arg1, const Expression& arg2 );
        friend Expression operator-( const Vector       & arg1, const Expression& arg2 );
        friend Expression operator-( const Matrix       & arg1, const Expression& arg2 );



        Expression operator*( const double      & arg ) const;
        Expression operator*( const Vector      & arg ) const;
        Expression operator*( const Matrix      & arg ) const;
        Expression operator*( const Expression  & arg ) const;

        friend Expression operator*( const double       & arg1, const Expression& arg2 );
        friend Expression operator*( const Vector       & arg1, const Expression& arg2 );
        friend Expression operator*( const Matrix       & arg1, const Expression& arg2 );


        Expression operator/( const double      & arg ) const;
        Expression operator/( const Expression  & arg ) const;

        friend Expression operator/( const double       & arg1, const Expression& arg2 );
        friend Expression operator/( const Vector       & arg1, const Expression& arg2 );
        friend Expression operator/( const Matrix       & arg1, const Expression& arg2 );


        Stream print( Stream &stream ) const;
        friend Stream operator<<( Stream &stream, const Expression &arg );


        /** Returns the symbolic inverse of a matrix (only for square matrices) */
        Expression getInverse( ) const;

        Expression getRow( const uint& rowIdx ) const;
        Expression getCol( const uint& colIdx ) const;
	
	
        /**
	* When operated on an n x 1 Expression, returns an m x n Matrix.
	* The element (i,j) of this matrix is zero when this(i) does not depend on arg(j)
	* \param arg m x 1 Expression
	*/
        Matrix getDependencyPattern( const Expression& arg ) const;

        Expression getSin    ( ) const;
        Expression getCos    ( ) const;
        Expression getTan    ( ) const;
        Expression getAsin   ( ) const;
        Expression getAcos   ( ) const;
        Expression getAtan   ( ) const;
        Expression getExp    ( ) const;
        Expression getSqrt   ( ) const;
        Expression getLn     ( ) const;

        Expression getPow   ( const Expression &arg ) const;
        Expression getPowInt( const int        &arg ) const;

        Expression getSumSquare    ( ) const;
        Expression getLogSumExp    ( ) const;
        Expression getEuclideanNorm( ) const;
        Expression getEntropy      ( ) const;

        Expression getDot          ( ) const;
        Expression getNext         ( ) const;


        Expression ADforward ( const Expression &arg ) const;
        Expression ADforward ( const VariableType &varType_, const int *arg, int nV ) const;
        Expression ADbackward( const Expression &arg ) const;

		Expression getODEexpansion( const int &order, const int *arg ) const;

        Expression ADforward ( const Expression &arg, const Expression &seed ) const;
        Expression ADforward ( const VariableType &varType_, const int *arg, const Expression &seed ) const;
        Expression ADforward ( const VariableType *varType_, const int *arg, const Expression &seed ) const;
        Expression ADbackward( const Expression &arg, const Expression &seed ) const;


        ConstraintComponent operator<=( const double& ub ) const;
        ConstraintComponent operator>=( const double& lb ) const;
        ConstraintComponent operator==( const double&  b ) const;

        ConstraintComponent operator<=( const Vector& ub ) const;
        ConstraintComponent operator>=( const Vector& lb ) const;
        ConstraintComponent operator==( const Vector&  b ) const;

        ConstraintComponent operator<=( const VariablesGrid& ub ) const;
        ConstraintComponent operator>=( const VariablesGrid& lb ) const;
        ConstraintComponent operator==( const VariablesGrid&  b ) const;

        friend ConstraintComponent operator<=( double lb, const Expression &arg );
        friend ConstraintComponent operator==( double  b, const Expression &arg );
        friend ConstraintComponent operator>=( double ub, const Expression &arg );

        friend ConstraintComponent operator<=( Vector lb, const Expression &arg );
        friend ConstraintComponent operator==( Vector  b, const Expression &arg );
        friend ConstraintComponent operator>=( Vector ub, const Expression &arg );

        friend ConstraintComponent operator<=( VariablesGrid lb, const Expression &arg );
        friend ConstraintComponent operator==( VariablesGrid  b, const Expression &arg );
        friend ConstraintComponent operator>=( VariablesGrid ub, const Expression &arg );



        /** Returns the transpose of this expression.
         *  \return The transposed expression. */
         Expression transpose( ) const;



        /** Returns dimension of vector space.
         *  \return Dimension of vector space. */
         inline uint getDim( ) const;


        /** Returns the number of rows.
         *  \return The number of rows. */
         inline uint getNumRows( ) const;


        /** Returns the number of columns.
         *  \return The number of columns. */
         inline uint getNumCols( ) const;


        /** Returns the global type idea of the idx-component.
         *  \return The global type ID. */
         inline uint getComponent( const unsigned int idx ) const;


        /** Returns the number of columns.
         *  \return The number of columns. */
         inline BooleanType isVariable( ) const;


        /** Returns a clone of the operator with index idx.
         *  \return A clone of the requested operator. */
         inline Operator* getOperatorClone( uint idx ) const;



        /** Returns the variable type
         *  \return The the variable type. */
         inline VariableType getVariableType( ) const;


         BooleanType isDependingOn( VariableType type ) const;
         BooleanType isDependingOn(const  Expression &e ) const;


        /** Substitutes a given variable with an expression.
         *  \return SUCCESSFUL_RETURN
         */
        returnValue substitute( int idx, const Expression &arg ) const;


        Expression convert( const double      & arg ) const;
        Expression convert( const Vector      & arg ) const;
        Expression convert( const Matrix      & arg ) const;



        /** Returns a tree projection with respect to the specified index.
         */
        inline TreeProjection getTreeProjection( const uint &idx, String name_="" ) const;



    // PROTECTED FUNCTIONS:
    // ---------------------------------------------------------------
    protected:


        /** Generic constructor (protected, only for internal use).
         */
        void construct( VariableType  variableType_,  /**< The variable type.     */
                        uint          globalTypeID_,  /**< the global type ID     */
                        uint                 nRows_,  /**< The number of rows.    */
                        uint                 nCols_,  /**< The number of columns. */
                        const String         &name_   /**< The name               */ );


        /** Generic copy routine (protected, only for internal use).
         */
        void copy( const Expression &rhs );


        /** Generic destructor (protected, only for internal use).
         */
        void deleteAll( );


        /** Generic copy routine (protected, only for internal use).
         */
        Expression& assignmentSetup( const Expression &arg );


        /** Internal product routine (protected, only for internal use). */
        Operator* product( const Operator *a, const Operator *b ) const;


    // PROTECTED DATA MEMBERS:
    // ---------------------------------------------------------------
    protected:

        Operator**         element     ;   /**< Element of vector space.   */
        uint               dim         ;   /**< Vector space dimension.    */
        uint               nRows, nCols;   /**< Matrix dimension.          */
        VariableType       variableType;   /**< Variable type.             */
        uint               component   ;   /**< The expression component   */
        String             name        ;   /**< The name of the expression */
};


CLOSE_NAMESPACE_ACADO


#include <acado/symbolic_expression/expression.ipp>

#endif  // ACADO_TOOLKIT_EXPRESSION_HPP

/*
 *   end of file.
 */
