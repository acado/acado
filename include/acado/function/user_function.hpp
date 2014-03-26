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
*    \file include/acado/function/user_function.hpp
*    \author Boris Houska
*    \date 2014
*/


#ifndef ACADO_TOOLKIT_USER_FUNCTION_HPP
#define ACADO_TOOLKIT_USER_FUNCTION_HPP


#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/function/function_fwd.hpp>

BEGIN_NAMESPACE_ACADO


/**
 *  \brief The class UserDefinedOperator is an auxiliary class to use C-Functions within a function evaluation tree.
 *
 *	\ingroup BasicDataStructures
 *
 *  The class UserDefinedOperator is designed to be an auxiliary class to use
 *  user defined functions within a symbolic evaluation tree.
 *
 *  \author Boris Houska, Hans Joachim Ferreau
 */

template <typename T, class Derived>
class UserDefinedOperator : public SmoothOperator{

    typedef UserFunction<T,Derived>                    FcnName;
    typedef std::tr1::shared_ptr<FcnName>   SharedUserFunction;
    
public:
  
    /** Default constructor. */
    UserDefinedOperator();

    /** Default constructor. */
    UserDefinedOperator( const SharedUserFunction &fcn_      ,
                         const Expression         &argument_ ,
                         const int                &component_  );

    /** Copy constructor (deep copy). */
    UserDefinedOperator( const UserDefinedOperator &arg );

    /** Default destructor. */
    ~UserDefinedOperator();

    /** Evaluates the expression (templated version) */
    virtual returnValue evaluate( EvaluationBase *x );


    /** Automatic Differentiation in forward mode on the symbolic \n
     *  level. This function generates an expression for a        \n
     *  forward derivative                                        \n
     *  \return SUCCESSFUL_RETURN                                 \n
     */
     virtual SharedOperator AD_forward( SharedOperatorMap &seed /**< the forward seed   */ );



    /** Automatic Differentiation in backward mode on the symbolic \n
     *  level. This function generates an expression for a         \n
     *  backward derivative                                        \n
     *  \return SUCCESSFUL_RETURN                                  \n
     */
    virtual returnValue AD_backward( SharedOperator     &seed, /**< the backward seed  */
                                     SharedOperatorMap  &df  , /**< the result         */
                                     SharedOperatorMap2 &IS    /**< the new IS-pointer */ );
    
    
    /** Automatic Differentiation in symmetric mode on the symbolic \n
     *  level. This function generates an expression for a          \n
     *  second order derivative.                                    \n
     *  \return SUCCESSFUL_RETURN                                   \n
     */
     virtual returnValue AD_symmetric( SharedOperator     &l  ,
                                       SharedOperatorMap  &ldf,
                                       SharedOperatorMap  &df ,
                                       SharedOperatorMap2 &H  ,
                                       SharedOperatorMap2 &LIS,
                                       SharedOperatorMap2 &SIS,
                                       SharedOperatorMap3 &HIS  );


    /** Substitutes key with the expression sub. \n
     *  \return The substituted expression.      \n
     *
     */
     virtual SharedOperator substitute( SharedOperatorMap &sub /**< the substitution */ );


    /** Checks whether the expression is zero or one              \n
     *  \return NE_ZERO                                           \n
     *          NE_ONE                                            \n
     *          NE_NEITHER_ONE_NOR_ZERO                           \n
     *
     */
     virtual NeutralElement isOneOrZero() const;


    /** Prints the expression into a stream. \n
     *  \return SUCCESFUL_RETURN             \n
     */
     virtual std::ostream& print( std::ostream &stream, StringMap &name ) const;


     virtual returnValue getArgumentList( DependencyMap &exists,
                                          SharedOperatorVector &list );

    /** Asks whether all elements are purely symbolic.                \n
      *                                                               \n
      * \return BT_TRUE  if the complete tree is symbolic.            \n
      *         BT_FALSE otherwise (e.g. if C functions are linked).  \n
      */
    virtual BooleanType isSymbolic() const;
    
    
    void setID( const SharedOperatorVector &ID_ );
    
    
//
//  PROTECTED MEMBERS:
//

protected:

    SharedUserFunction   userFcn;
    Expression          argument;
    int                component;
    SharedOperatorVector      ID;
};



template <typename T, class Derived> UserDefinedOperator<T,Derived>::UserDefinedOperator(){ }

template <typename T, class Derived> UserDefinedOperator<T,Derived>::UserDefinedOperator( const SharedUserFunction  &fcn_      ,
                                                                           const Expression          &argument_ ,
                                                                           const int                 &component_  ){
  
    userFcn    = fcn_      ;
    argument   = argument_ ;
    component  = component_;
}

template <typename T, class Derived> UserDefinedOperator<T,Derived>::UserDefinedOperator( const UserDefinedOperator &arg ){
  
    userFcn    = arg.userFcn   ;
    argument   = arg.argument  ;
    component  = arg.component ;
    ID         = arg.ID        ;
}

template <typename T, class Derived> UserDefinedOperator<T,Derived>::~UserDefinedOperator(){}


template <typename T, class Derived> returnValue UserDefinedOperator<T,Derived>::evaluate( EvaluationBase *x ){
  
  typedef EvaluationTemplate<T> EvalType; 
  
  EvalType *xx = dynamic_cast<EvalType*>(x);
  if( xx == 0 ) return ACADOERROR( RET_INVALID_ARGUMENTS );
    
  if( (xx->map)->count(this) == 0 ){
    
      std::vector<T> input ( argument.size() );  // input
      std::vector<T> result( userFcn->size() );  // output
      
      for( int i=0; i<argument.size(); ++i ){
           xx->project(argument(i).element.get());
           input[i] = xx->res;
      }
      userFcn->evaluate( input, result );
      
      for( int i=0; i<argument.size(); ++i )
           (xx->map)->operator[](ID[i].get()) = result[i];
  }
  xx->res = (xx->map)->operator[](this);
  
  return SUCCESSFUL_RETURN;
}



template <typename T, class Derived> SharedOperator UserDefinedOperator<T,Derived>::AD_forward( SharedOperatorMap &seed ){
  
    return SharedOperator( );
}


template <typename T, class Derived> returnValue UserDefinedOperator<T,Derived>::AD_backward(
                                    SharedOperator     &seed,
                                    SharedOperatorMap  &df  ,
                                    SharedOperatorMap2 &IS    ){

    return SUCCESSFUL_RETURN;
}
    
    

template <typename T, class Derived> returnValue UserDefinedOperator<T,Derived>::AD_symmetric( SharedOperator     &l  ,
                                     SharedOperatorMap  &ldf,
                                     SharedOperatorMap  &df ,
                                     SharedOperatorMap2 &H  ,
                                     SharedOperatorMap2 &LIS,
                                     SharedOperatorMap2 &SIS,
                                     SharedOperatorMap3 &HIS  ){

    return SUCCESSFUL_RETURN;
}



template <typename T, class Derived> SharedOperator UserDefinedOperator<T,Derived>::substitute( SharedOperatorMap &sub ){
 
    return SharedOperator();
}


template <typename T, class Derived> NeutralElement UserDefinedOperator<T,Derived>::isOneOrZero() const{ return NE_NEITHER_ONE_NOR_ZERO; }

template <typename T, class Derived> std::ostream& UserDefinedOperator<T,Derived>::print( std::ostream &stream, StringMap &name ) const{
    
    
   return stream;
}
    
template <typename T, class Derived> returnValue UserDefinedOperator<T,Derived>::getArgumentList( DependencyMap &exists, SharedOperatorVector &list ){
  
    if( exists[ID[0].get()] != true ){
         for( int i=0; i<argument.size(); ++i ){
             (argument(i)).element->getArgumentList(exists,list);
             list.push_back((argument(i)).element);
         }
         exists[ID[0].get()] = true;
    }
    return SUCCESSFUL_RETURN;
}


template <typename T, class Derived> BooleanType UserDefinedOperator<T,Derived>::isSymbolic() const{ return BT_FALSE; }

template <typename T, class Derived> void UserDefinedOperator<T,Derived>::setID( const SharedOperatorVector &ID_ ){ ID = ID_; }





/**
 *  \brief The class UserFunction is a base class for all user defined functions.
 *
 *	\ingroup BasicDataStructures
 *
 *  The class UserFunction is designed to allow an easy setup
 *  of user defined functions within a symbolic evaluation tree.
 *
 *  \author Boris Houska
 *  \date 2014
 * 
 */



template <typename T, class Derived>
class UserFunction{

    //
    // PUBLIC MEMBER FUNCTIONS:
    //

    public:

    /** Default constructor. */
    UserFunction( int dim_ = 1 ){ dim = dim_; }

    /** Copy constructor. */
    UserFunction( const UserFunction& rhs ){ dim = rhs.dim; }

    /** Destructor. */
    virtual ~UserFunction( ){ }

    /** Evaluates the expression */
    virtual Derived* clone() const = 0;
    
    /** Assignment operator. */
    UserFunction& operator=( const UserFunction& rhs ){ return *this; }

    /** Loading Expressions. */
    virtual Expression operator()( const Expression &arg );

    /** Returns the output dimension of the user function. */
    virtual uint size() const{ return dim; }

    /** Evaluates the expression */
    virtual void evaluate( const std::vector<T> &input, std::vector<T> &output ) = 0;

//
//  PROTECTED MEMBERS:
//

     protected:

     uint dim;    /**< size of the function (output) */
};


template<typename T, class Derived> Expression UserFunction<T,Derived>::operator()( const Expression &arg ){

    typedef UserFunction<T,Derived>                   FcnName;
    typedef std::tr1::shared_ptr<FcnName>  SharedUserFunction;
    typedef UserDefinedOperator<T,Derived>                UDO;
    
    Expression tmp((int) dim,1);
    
    SharedUserFunction fcn(clone());
    
    std::vector<UDO*> temp(dim); 
    SharedOperatorVector ID(dim);
    
    for( uint i=0; i<dim; ++i ){
         temp[i] = new UDO( fcn, arg, i );
         ID[i] = SharedOperator(temp[i]);
    }
    for( uint i=0; i<dim; ++i ){
         temp[i]->setID(ID);
	 tmp(i) = ScalarExpression(ID[i]);
    }
    return tmp;
}


CLOSE_NAMESPACE_ACADO

#endif

