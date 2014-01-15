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
 *    \author Boris Houska, Hans Joachim Ferreau, Milan Vukov
 *
 */


#ifndef ACADO_TOOLKIT_EXPRESSION_HPP
#define ACADO_TOOLKIT_EXPRESSION_HPP

#include <acado/utils/acado_utils.hpp>
#include <acado/matrix_vector/matrix_vector.hpp>
#include <acado/variables_grid/variables_grid.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>

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
 *  \author Boris Houska, Hans Joachim Ferreau, Milan Vukov
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
        explicit Expression(	const std::string& name_
        						);

        /** Constructor which takes the arguments. */
        explicit Expression(	const std::string&  name_,							/**< the name                */
        						uint          		nRows_,							/**< number of rows          */
        						uint          		nCols_,							/**< number of columns       */
        						VariableType  		variableType_  = VT_UNKNOWN,	/**< the variable type       */
        						uint          		globalTypeID   = 0				/**< the global type ID      */
        						);

        /** Constructor which takes the arguments. */
        explicit Expression(	int          nRows_                     ,  /**< number of rows          */
        						int          nCols_         = 1         ,  /**< number of columns       */
        						VariableType variableType_  = VT_UNKNOWN,  /**< the variable type       */
        						int          globalTypeID   = 0            /**< the global type ID      */
        						);

        /** Constructor which takes the arguments. */
        explicit Expression(	uint         nRows_                     ,  /**< number of rows          */
        						uint         nCols_         = 1         ,  /**< number of columns       */
        						VariableType variableType_  = VT_UNKNOWN,  /**< the variable type       */
        						uint         globalTypeID   = 0            /**< the global type ID      */
        						);

        /** Copy constructor (deep copy). */
        Expression( const double      & rhs );
        Expression( const DVector      & rhs );
        Expression( const DMatrix      & rhs );

        Expression( const Expression  & rhs );


        /** Destructor. */
        virtual ~Expression( );

        /** Function for cloning. */
        virtual Expression* clone() const
        { return new Expression( *this ); }

        /** Assignment Operator.                                             \n
         *                                                                   \n
         *  \param arg  the double value to be assigned to the expression.   \n
         */
        Expression& operator=( const double& arg );


        /** Assignment Operator.                                       \n
         *                                                             \n
         *  \param arg  the vector to be assigned to the expression.   \n
         */
        Expression& operator=( const DVector& arg );


        /** Assignment Operator.                                       \n
         *                                                             \n
         *  \param arg  the matrix to be assigned to the expression.   \n
         */
        Expression& operator=( const DMatrix& arg );


        /** Assignment Operator.                         \n
         *                                               \n
         *  \param arg  the Expression to be assigned.   \n
         */
        Expression& operator=( const Expression& arg );

        Expression& operator<<( const double      & arg );
        Expression& operator<<( const DVector      & arg );
        Expression& operator<<( const DMatrix      & arg );
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
        Expression operator+( const DVector      & arg ) const;
        Expression operator+( const DMatrix      & arg ) const;
        Expression operator+( const Expression  & arg ) const;

        friend Expression operator+( const double       & arg1, const Expression& arg2 );
        friend Expression operator+( const DVector       & arg1, const Expression& arg2 );
        friend Expression operator+( const DMatrix       & arg1, const Expression& arg2 );


        Expression operator-( const double      & arg ) const;
        Expression operator-( const DVector      & arg ) const;
        Expression operator-( const DMatrix      & arg ) const;
        Expression operator-( const Expression  & arg ) const;

        Expression operator-( ) const;

        friend Expression operator-( const double       & arg1, const Expression& arg2 );
        friend Expression operator-( const DVector       & arg1, const Expression& arg2 );
        friend Expression operator-( const DMatrix       & arg1, const Expression& arg2 );



        Expression operator*( const double      & arg ) const;
        Expression operator*( const DVector      & arg ) const;
        Expression operator*( const DMatrix      & arg ) const;
        Expression operator*( const Expression  & arg ) const;

        friend Expression operator*( const double       & arg1, const Expression& arg2 );
        friend Expression operator*( const DVector       & arg1, const Expression& arg2 );
        friend Expression operator*( const DMatrix       & arg1, const Expression& arg2 );


        Expression operator/( const double      & arg ) const;
        Expression operator/( const Expression  & arg ) const;

        friend Expression operator/( const double       & arg1, const Expression& arg2 );
        friend Expression operator/( const DVector       & arg1, const Expression& arg2 );
        friend Expression operator/( const DMatrix       & arg1, const Expression& arg2 );


        std::ostream& print( std::ostream &stream ) const;
        friend std::ostream& operator<<( std::ostream& stream, const Expression &arg );


        /** Returns the symbolic inverse of a matrix (only for square matrices) */
        Expression getInverse( ) const;

        Expression getRow( const uint& rowIdx ) const;
        Expression getCol( const uint& colIdx ) const;
	
	
        /**
	* When operated on an n x 1 Expression, returns an m x n DMatrix.
	* The element (i,j) of this matrix is zero when this(i) does not depend on arg(j)
	* \param arg m x 1 Expression
	*/
        DMatrix getDependencyPattern( const Expression& arg ) const;

        Expression getSign   ( ) const;
        Expression getAbs    ( ) const;
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

        ConstraintComponent operator<=( const DVector& ub ) const;
        ConstraintComponent operator>=( const DVector& lb ) const;
        ConstraintComponent operator==( const DVector&  b ) const;

        ConstraintComponent operator<=( const VariablesGrid& ub ) const;
        ConstraintComponent operator>=( const VariablesGrid& lb ) const;
        ConstraintComponent operator==( const VariablesGrid&  b ) const;

        friend ConstraintComponent operator<=( double lb, const Expression &arg );
        friend ConstraintComponent operator==( double  b, const Expression &arg );
        friend ConstraintComponent operator>=( double ub, const Expression &arg );

        friend ConstraintComponent operator<=( DVector lb, const Expression &arg );
        friend ConstraintComponent operator==( DVector  b, const Expression &arg );
        friend ConstraintComponent operator>=( DVector ub, const Expression &arg );

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
        Expression convert( const DVector      & arg ) const;
        Expression convert( const DMatrix      & arg ) const;



        /** Returns a tree projection with respect to the specified index. */
        inline TreeProjection getTreeProjection( const uint &idx, const std::string& name_ ) const;



    // PROTECTED FUNCTIONS:
    // ---------------------------------------------------------------
    protected:


        /** Generic constructor (protected, only for internal use).
         */
        void construct( VariableType  variableType_,  /**< The variable type.     */
                        uint          globalTypeID_,  /**< the global type ID     */
                        uint                 nRows_,  /**< The number of rows.    */
                        uint                 nCols_,  /**< The number of columns. */
                        const std::string&   name_    /**< The name               */ );


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
        uint               dim         ;   /**< DVector space dimension.    */
        uint               nRows, nCols;   /**< DMatrix dimension.          */
        VariableType       variableType;   /**< Variable type.             */
        uint               component   ;   /**< The expression component   */
        std::string             name   ;   /**< The name of the expression */
};

CLOSE_NAMESPACE_ACADO

#include <acado/symbolic_expression/expression.ipp>

BEGIN_NAMESPACE_ACADO

/** A helper class implementing the CRTP design pattern.
 *
 *  This class gives object counting and clone capability to a derived
 *  class via static polymorphism.
 *
 *  \tparam Derived      The derived class.
 *  \tparam Type         The expression type. \sa VariableType
 *  \tparam AllowCounter Allow object instance counting.
 *
 *  \note Unfortunately the derived classes have to implement all necessary
 *        ctors. In C++11, this can be done in a much simpler way. One only
 *        needs to say: using Base::Base.
 *
 */
template<class Derived, VariableType Type, bool AllowCounter = true>
class ExpressionType : public Expression
{
public:

	/** Default constructor. */
	ExpressionType()
		: Expression("", 1, 1, Type, AllowCounter ? count : 0)
	{
		if (AllowCounter == true)
			count++;
	}

	/** The constructor with arguments. */
	ExpressionType(const std::string& _name, unsigned _nRows, unsigned _nCols)
		: Expression(_name, _nRows, _nCols, Type, AllowCounter ? count : 0)
	{
		if (AllowCounter == true)
			count += _nRows * _nCols;
	}

	/** The constructor from an expression. */
	ExpressionType(const Expression& _expression, unsigned _componentIdx = 0)
		: Expression( _expression )
	{
		variableType = Type;
		component += _componentIdx;
		if (AllowCounter == true)
			count++;
	}

	/** Destructor. */
	virtual ~ExpressionType() {}

	/** Function for cloning. */
	virtual Expression* clone() const
	{ return new Derived( static_cast< Derived const& >( *this ) ); }

	/** A function for resetting of the istance counter. */
	returnValue clearStaticCounters()
	{ count = 0; return SUCCESSFUL_RETURN; }

private:
	static unsigned count;
};

template<class Derived, VariableType Type, bool AllowCounter>
unsigned ExpressionType<Derived, Type, AllowCounter>::count( 0 );

CLOSE_NAMESPACE_ACADO

#endif  // ACADO_TOOLKIT_EXPRESSION_HPP

/*
 *   end of file.
 */
