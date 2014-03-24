/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2014 by Boris Houska, Hans Joachim Ferreau,
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

class Expression;
class ScalarExpression;

/**
 *  \brief Base class for all variables within the symbolic expressions family.
 *
 *	\ingroup BasicDataStructures
 *
 *  The class Expression serves as a base class for all
 *  symbolic variables within the symbolic expressions family.
 *
 *  \author Boris Houska, Hans Joachim Ferreau, Milan Vukov
 */


class ScalarExpression{

    //
    // PUBLIC MEMBER FUNCTIONS:
    //

    public:

        /** Default constructor. */
        ScalarExpression( );

        /** Casting constructor. */
        ScalarExpression( const double &element_ );

        /** Casting constructor. */
        ScalarExpression( const SharedOperator &element_ );

        /** Explicit copy constructor. */
        ScalarExpression( const ScalarExpression &rhs );

        /** Destructor. */
        ~ScalarExpression();

        ScalarExpression& operator= ( const Expression& arg );
        ScalarExpression& operator+=( const Expression& arg );
        ScalarExpression& operator-=( const Expression& arg );
        ScalarExpression& operator*=( const Expression& arg );
        ScalarExpression& operator/=( const Expression& arg );

        ScalarExpression& operator= ( const ScalarExpression& arg );
        ScalarExpression& operator+=( const ScalarExpression& arg );
        ScalarExpression& operator-=( const ScalarExpression& arg );
        ScalarExpression& operator*=( const ScalarExpression& arg );
        ScalarExpression& operator/=( const ScalarExpression& arg );

        ScalarExpression& operator= ( const double& arg );
        ScalarExpression& operator+=( const double& arg );
        ScalarExpression& operator-=( const double& arg );
        ScalarExpression& operator*=( const double& arg );
        ScalarExpression& operator/=( const double& arg );

        const ScalarExpression operator-( ) const;

        const ScalarExpression operator+( const ScalarExpression  & arg ) const;
        const ScalarExpression operator-( const ScalarExpression  & arg ) const;
        const ScalarExpression operator*( const ScalarExpression  & arg ) const;
        const ScalarExpression operator/( const ScalarExpression  & arg ) const;

        friend const ScalarExpression operator+( const double &arg1, const ScalarExpression& arg2 );
        friend const ScalarExpression operator-( const double &arg1, const ScalarExpression& arg2 );
        friend const ScalarExpression operator*( const double &arg1, const ScalarExpression& arg2 );
        friend const ScalarExpression operator/( const double &arg1, const ScalarExpression& arg2 );


        bool operator==( const ScalarExpression& arg ) const;
        bool operator!=( const ScalarExpression& arg ) const;
        bool operator>=( const ScalarExpression& arg ) const;
        bool operator<=( const ScalarExpression& arg ) const;
        bool operator< ( const ScalarExpression& arg ) const;
        bool operator> ( const ScalarExpression& arg ) const;
	
//         std::ostream& print( std::ostream &stream ) const;
//         friend std::ostream& operator<<( std::ostream& stream, const ScalarExpression &arg );
	
//         /**
// 	* When operated on an n x 1 ScalarExpression, returns an m x n DMatrix.
// 	* The element (i,j) of this matrix is zero when this(i) does not depend on arg(j)
// 	* \param arg m x 1 ScalarExpression
// 	*/
//         DMatrix getDependencyPattern( const ScalarExpression& arg ) const;

        const ScalarExpression getSin    ( ) const;
        const ScalarExpression getCos    ( ) const;
        const ScalarExpression getTan    ( ) const;
        const ScalarExpression getAsin   ( ) const;
        const ScalarExpression getAcos   ( ) const;
        const ScalarExpression getAtan   ( ) const;
        const ScalarExpression getExp    ( ) const;
        const ScalarExpression getSqrt   ( ) const;
        const ScalarExpression getLn     ( ) const;

        const ScalarExpression getPow   ( const ScalarExpression &arg ) const;
        const ScalarExpression getPowInt( const int              &arg ) const;

	/** Second order symmetric AD routine returning \n
	 *  l^T*f''  with f'' being the second          \n
	 * order derivative of the current expression.  \n
	 * The he vector l can be interpreted as a      \n
	 * backward seed. Optionally, this routine also \n
	 * returns expressions for the first order      \n
	 * order terms f'  and  l^T*f' computed by      \n
	 * first order forward and first order backward \n
	 * automatic differentiation, respectively.     \n
	 * Caution: this routine is tailored for        \n
	 * full Hessian computation exploiting symmetry.\n
	 * If only single elements of the Hessian are   \n
	 * needed, forward-over-adjoint or ajoint-over- \n
	 * forward differentiation may be more          \n
	 * efficient.                                   \n
	 */
        SharedOperatorMap2 ADsymmetric( const ScalarExpression &l  , /** backward seed */
                                        SharedOperatorMap      &dfS, /** first order forward  result */
                                        SharedOperatorMap      &ldf  /** first order backward result */ ) const;

        const ScalarExpression AD_forward ( const Expression &arg, const Expression &seed ) const;

        SharedOperatorMap AD_backward( const ScalarExpression &seed ) const;

        /** Returns whether the expression is a variable.
         *  \return true if the expression is a variable, false otherwise. */
        BooleanType isVariable( ) const;


    // DATA MEMBERS:
    // ---------------------------------------------------------------
        SharedOperator element;   /**< The element of the expression. */
};

CLOSE_NAMESPACE_ACADO

namespace Eigen {
template<> struct NumTraits<ACADO::ScalarExpression>
: NumTraits<double> // permits to get the epsilon, dummy_precision, lowest, highest functions
{
typedef ACADO::ScalarExpression Real;
typedef ACADO::ScalarExpression NonInteger;
typedef ACADO::ScalarExpression Nested;
enum {
IsComplex = 0,
IsInteger = 0,
IsSigned = 1,
RequireInitialization = 1,
ReadCost = 1,
AddCost = 3,
MulCost = 3
};
};
}

namespace ACADO {
inline ScalarExpression sqrt(const ScalarExpression& x) { return x.getSqrt(); }
inline const ScalarExpression& conj(const ScalarExpression& x) { return x; }
inline const ScalarExpression& real(const ScalarExpression& x) { return x; }
inline ScalarExpression imag(const ScalarExpression&) { return 0.; }
inline ScalarExpression abs(const ScalarExpression& x) { return sqrt(x*x); }
inline ScalarExpression abs2(const ScalarExpression& x) { return x*x; }
}


BEGIN_NAMESPACE_ACADO

class Expression : public Eigen::Matrix<ScalarExpression,Eigen::Dynamic,Eigen::Dynamic>{
  
public:
  
  typedef Eigen::Matrix<ScalarExpression,Eigen::Dynamic,Eigen::Dynamic> Base;
  
  Expression( const int &nRows = 1, const int &nCols = 1 );
  
  Expression( const ScalarExpression &arg );
  
  Expression( const double &arg );
  
  Expression( const DMatrix &arg );
  
  template<typename OtherDerived>
  inline Expression(const Eigen::MatrixBase<OtherDerived>& other) : Base(other){ }
  using Base::operator=;
  
  template<typename OtherDerived>
  inline Expression(const Eigen::ReturnByValue<OtherDerived>& other) : Base( other ) {}

  template<typename OtherDerived>
  inline Expression(const Eigen::EigenBase<OtherDerived>& other) : Base( other ) {}
  
  Expression operator,( const Expression &arg ) const;
  
  const Expression operator+( const double& arg ) const{ return Base::operator+(Expression(arg)); }
  const Expression operator-( const double& arg ) const{ return Base::operator-(Expression(arg)); }
  const Expression operator*( const double& arg ) const{ return Base::operator*(Expression(arg)); }
  const Expression operator/( const double& arg ) const{ return Base::operator/(ScalarExpression(arg)); }

  const Expression operator+( const Expression& arg ) const{ return Base::operator+(arg); }
  const Expression operator-( const Expression& arg ) const{ return Base::operator-(arg); }
  const Expression operator*( const Expression& arg ) const{ return Base::operator*(arg); }
  const Expression operator/( const Expression& arg ) const{ ASSERT(arg.size()==1); return Base::operator/(arg(0)); }
  
  friend const Expression operator+( const double &arg1, const Expression& arg2 ){ return Expression(arg1)+arg2; }
  friend const Expression operator-( const double &arg1, const Expression& arg2 ){ return Expression(arg1)-arg2; }
  friend const Expression operator*( const double &arg1, const Expression& arg2 ){ return Expression(arg1)*arg2; }
  friend const Expression operator/( const double &arg1, const Expression& arg2 ){ return Expression(arg1)/arg2; }
  
  Expression& appendRows( const Expression &arg );
  
  Expression& appendCols( const Expression &arg );
  
  Expression AD_backward( const Expression &arg, const Expression &seed ) const;
  
  Expression AD_symmetric( const Expression &arg, const Expression &seed,
                           Expression *dfS = 0  , Expression *ldf = 0 ) const;
  
  /** Checks whether the expression is a variable */
  BooleanType isVariable() const;
  
  /** Checks componentwise whether the expression is one or zero */
  std::vector<NeutralElement> isOneOrZero() const;
  
  /** Checks whether the expression is smooth */
  BooleanType isSmooth() const;

};


CLOSE_NAMESPACE_ACADO

#endif  // ACADO_TOOLKIT_EXPRESSION_HPP

/*
 *   end of file.
 */
