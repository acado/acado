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
 *    \file include/acado/function/function_.hpp
 *    \author Boris Houska, Hans Joachim Ferreau
 */


#ifndef ACADO_TOOLKIT_FUNCTION__HPP
#define ACADO_TOOLKIT_FUNCTION__HPP


#include <acado/symbolic_expression/expression.hpp>


BEGIN_NAMESPACE_ACADO

/** 
 *	\brief Allows to setup and evaluate a general function based on SymbolicExpressions.
 *
 *	\ingroup BasicDataStructures
 *
 *  The class Function allows to setup and evaluate general functions
 *	based on SymbolicExpressions.
 *
 *	\author Boris Houska, Hans Joachim Ferreau
 */

class Function{

//
// PUBLIC MEMBER FUNCTIONS:
//

public:

    /** Default constructor. */
    Function();

    /** Copy constructor (deep copy). */
    Function( const Function& rhs );

    /** Destructor. */
    virtual ~Function( );

    /** Assignment operator (deep copy). */
    Function& operator=( const Function& rhs );

    /** Loading Expressions (deep copy). */
    Function& operator<<( const Expression &arg );

    /** Returns the i-th component of this function.*/
    Function operator()( uint idx ) const;
    
    /** Returns the dimension of the function */
    uint size() const;
    
    /** Defines the order of the input. */
    returnValue setInput( const Expression &input );    
    
    /** Evaluates the function. */
    template <typename T> std::vector<T> evaluate( const std::vector<T> &x );
    
    
    /** Evaluates the function based on a template map.*/
    template <typename T> std::vector<T> evaluate( std::map<Operator*,T>  &x );


     /** Checks whether the function is a constant. */
     BooleanType isConstant() const;


    /** Prints the function into a stream. */
    friend std::ostream& operator<<( std::ostream& stream, const Function &arg);

    /** Prints the function in form of plain C-code into a file. The integer             \n
     *  "precision" must be in [1,16] and defines the number of internal decimal places  \n
     *  which occur in "double" - valued parts of the expression tree.                   \n
     *                                                                                   \n
     *  \param file      The file to which the expression should be printed.             \n
     *  \param fcnName   The name of the generated function (default: "ACADOfcn").       \n
     *  \param precision The number of internal dec. places to be printed (default: 16). \n
     *                                                                                   \n
     *  \return SUCCESFUL_RETURN                                                         \n
     */
     returnValue print(	std::ostream& stream,
						const char *fcnName = "ACADOfcn",
						const char *realString = "double"
						) const;

     returnValue exportForwardDeclarations(	std::ostream& stream,
											const char *fcnName = "ACADOfcn",
											const char *realString = "double"
											) const;

     returnValue exportCode(	std::ostream& stream,
								const char *fcnName = "ACADOfcn",
								const char *realString = "double",
								bool       allocateMemory = true,
								bool       staticMemory   = false
								) const;

     /** Returns whether the function is symbolic or not. */
     BooleanType isSymbolic() const;

     /** Set name of the variable that holds intermediate values. */
     returnValue setGlobalExportVariableName(const std::string& var);

     /** Get name of the variable that holds intermediate values. */
     std::string getGlobalExportVariableName( ) const;

     /** Get size of the variable that holds intermediate values. */
     unsigned getGlobalExportVariableSize( ) const;


// PROTECTED FUNCTIONS:
// --------------------
     
   void copy( const Function &arg );
     
     
// PROTECTED MEMBERS:
// --------------------

protected:

     SharedOperatorVector     f;   /**< The right-hand side expressions */
     SharedOperatorVector   sub;   /**< The intermediate expressions    */
     DependencyMap          dep;   /**< The dependency map              */
     SharedOperatorVector    in;   /**< The order of the inputs         */
     
     std::string  globalExportVariableName;   /** Name of the variable that holds intermediate expressions. */
};

template <typename T> std::vector<T> Function::evaluate( std::map<Operator*,T> &x ){
      
    std::vector<T> result(size());
    EvaluationTemplate<T> y(&x);
    
    for( uint i=0; i<sub.size(); ++i ){
         sub[i]->evaluate(&y);
         x[sub[i].get()] = y.res;
    }
    
    for( uint i=0; i<size(); ++i ){
         f[i]->evaluate(&y);
         result[i] = y.res;
    }
    return result;
}

template <typename T> std::vector<T> Function::evaluate( const std::vector<T> &x ){
  
    ASSERT( (int) in.size()== x.size());
    
    std::map<Operator*,T> xMap;
    
    for( uint i=0; i<x.size(); ++i ){
        xMap[in[i].get()] = x[i];
    }    
    return evaluate(xMap);  
}


CLOSE_NAMESPACE_ACADO

#endif  // ACADO_TOOLKIT_FUNCTION__HPP

/*
 *   end of file
 */
