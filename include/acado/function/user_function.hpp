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

template <typename T>
class UserDefinedOperator : public SmoothOperator{

    typedef std::vector<T>                             TVector;
    typedef UserFunction<T>                            FcnName;
    typedef std::tr1::shared_ptr<TVector>         SharedResult;
    typedef std::tr1::shared_ptr<FcnName>   SharedUserFunction;
    
public:
  
    /** Default constructor. */
    UserDefinedOperator();

    /** Default constructor. */
    UserDefinedOperator( const SharedUserFunction &fcn_      ,
                         const Expression         &argument_ ,
                         const SharedResult       &result_   ,
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
    
    
    void setID( const SharedOperator &ID_ );
    
    
//
//  PROTECTED MEMBERS:
//

protected:

    SharedUserFunction   userFcn;
    Expression          argument;
    SharedResult          result;
    int                component;
    
    SharedOperator            ID;
    BooleanType            first;
};



template <typename T> UserDefinedOperator<T>::UserDefinedOperator(){ }

template <typename T> UserDefinedOperator<T>::UserDefinedOperator( const SharedUserFunction  &fcn_      ,
                                                                   const Expression          &argument_ ,
                                                                   const SharedResult        &result_   ,
                                                                   const int                 &component_  ){
  
    userFcn    = fcn_      ;
    argument   = argument_ ;
    result     = result_   ;
    component  = component_;
}

template <typename T> UserDefinedOperator<T>::UserDefinedOperator( const UserDefinedOperator &arg ){
  
    userFcn    = arg.userFcn   ;
    argument   = arg.argument  ;
    result     = arg.result    ;
    component  = arg.component ;
    ID         = arg.ID        ;
    first      = arg.first     ;
}

template <typename T> UserDefinedOperator<T>::~UserDefinedOperator(){}


template <typename T> returnValue UserDefinedOperator<T>::evaluate( EvaluationBase *x ){
  
  typedef EvaluationTemplate<T> EvalType; 
  
  EvalType *xx = dynamic_cast<EvalType*>(x);
  if( xx == 0 ) return ACADOERROR( RET_INVALID_ARGUMENTS );
  
  if( first == true ){
    
      TVector input(argument.size());
      
      for( int i=0; i<argument.size(); ++i ){
           xx->project(argument(i).element.get());
           input[i] = xx->res;
      }
      result->resize(userFcn->size());
      userFcn->evaluate( input, *result );
  }
  
  xx->res = result->operator[](component);
  
  return SUCCESSFUL_RETURN;
}



template <typename T> SharedOperator UserDefinedOperator<T>::AD_forward( SharedOperatorMap &seed ){

    return SharedOperator(); 
}

template <typename T> returnValue UserDefinedOperator<T>::AD_backward( SharedOperator     &seed,
                                    SharedOperatorMap  &df  ,
                                    SharedOperatorMap2 &IS    ){

    return SUCCESSFUL_RETURN;
}
    
    

template <typename T> returnValue UserDefinedOperator<T>::AD_symmetric( SharedOperator     &l  ,
                                     SharedOperatorMap  &ldf,
                                     SharedOperatorMap  &df ,
                                     SharedOperatorMap2 &H  ,
                                     SharedOperatorMap2 &LIS,
                                     SharedOperatorMap2 &SIS,
                                     SharedOperatorMap3 &HIS  ){

    return SUCCESSFUL_RETURN;
}



template <typename T> SharedOperator UserDefinedOperator<T>::substitute( SharedOperatorMap &sub ){
 
    return SharedOperator();
}


template <typename T> NeutralElement UserDefinedOperator<T>::isOneOrZero() const{ return NE_NEITHER_ONE_NOR_ZERO; }

template <typename T> std::ostream& UserDefinedOperator<T>::print( std::ostream &stream, StringMap &name ) const{ return stream; }
    
template <typename T> returnValue UserDefinedOperator<T>::getArgumentList( DependencyMap &exists, SharedOperatorVector &list ){

    first = false;
  
    if( exists[ID.get()] != true ){
      
         for( int i=0; i<argument.size(); ++i ){
             (argument(i)).element->getArgumentList(exists,list);
             list.push_back((argument(i)).element);
         }
 
         exists[ID.get()] = true;
         first            = true;
    }
    return SUCCESSFUL_RETURN;
}


template <typename T> BooleanType UserDefinedOperator<T>::isSymbolic() const{ return BT_FALSE; }

template <typename T> void UserDefinedOperator<T>::setID( const SharedOperator &ID_ ){ ID = ID_; }





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



template <typename T>
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

    /** Assignment operator. */
    UserFunction& operator=( const UserFunction& rhs ){ return *this; }

    /** Loading Expressions. */
    virtual Expression operator()( const Expression &arg );

    /** Returns the output dimension of the user function. */
    virtual uint size() const{ return dim; }

    /** Evaluates the expression */
    virtual void evaluate( const std::vector<T> &input, std::vector<T> &output ){ }

//
//  PROTECTED MEMBERS:
//

     protected:

     uint dim;    /**< size of the function (output) */
};


template<typename T> Expression UserFunction<T>::operator()( const Expression &arg ){
 
    typedef std::vector<T>                                 TVector;
    typedef UserFunction<T>                                FcnName;
    typedef std::tr1::shared_ptr<TVector>             SharedResult;
    typedef std::tr1::shared_ptr<FcnName>       SharedUserFunction;
    
    Expression tmp((int) dim,1);
    
    SharedUserFunction     fcn( new FcnName(*this)      );
    SharedResult        result( new TVector(arg.size()) );
    
    SharedOperator ID;
    
    for( uint i=0; i<dim; ++i ){
         UserDefinedOperator<T> *temp = new UserDefinedOperator<T>( fcn, arg, result, i );
         SharedOperator component = SharedOperator( temp );
         if( i == 0 ) ID = component;
         temp->setID(ID);
	 tmp(i) = ScalarExpression(component);
    }
    return tmp;
}



CLOSE_NAMESPACE_ACADO

#endif

