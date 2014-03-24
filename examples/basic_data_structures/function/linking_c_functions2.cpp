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
 *    \file examples/integrator/linking_c_functions2.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */

#include <time.h>

#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/function/function.hpp>


USING_NAMESPACE_ACADO

// =================================================
//  DEFINE A SYMBLOIC FUNCTION CALLED "MyFunction":
// =================================================

class MyFunction : public UserFunction<double>{   // inherit form UserFunction<double>,
                                                  // the template parameter "double"
                                                  // specifies that the evaluation is
                                                  // based on double.
                                           
public:
  
    MyFunction() : UserFunction<double>(2){ }  // we may set the output dimension
                                               // in the constructor as it
                                               // is not allowed to change anyhow.
  
    MyFunction( const MyFunction &rhs ):UserFunction<double>(rhs){ }
  
    //  Overwrite the evaluate routine!
    //  Notice the input and output vector are of type double
    //  in this example, since we inherit from "UserFunction<double>".
    // ----------------------------------------------------------------
    virtual void evaluate( const std::vector<double> &input,
                                 std::vector<double> &output ){

       // An example how to access the input dimension.
       // --------------------------------------------------------------------
       std::cout << "The input dimension is : " << input.size()  << std::endl;
       
       output[0] = input[0]*input[0] + input[1]; // this is standard C-code,
       output[1] = sin(input[0]);                // define what-ever you like!
    }
};

// =================================================
// ==================================================================


int main( ){
  
    Expression y,z;
    Expression x(2);
    Expression m;
    
    Function f;
    
    MyFunction map;    // constructs an instance of the class MyFunction
    
    f.setInput((y,z));
    
    x(0) = y;
    x(1) = 2*z+1;

    printf("main: here 1 \n");
    
    m = map(x);     // The use function "map" can be use like
                    // any other symbolic function
                    // the user-definition is now part
                    // of the evaluation tree.

    
    printf("main: here 2 \n");
    
    f << m(0);
    f << m(1);        // m is an expression like any other.

// ==================================================================
    
    std::vector<double> xx(2);

    xx[0] = 1.;
    xx[1] = 2.;
    
    std::vector<double> result = f.evaluate(xx); // Notice: the evaluation has to be based on
                                                 // double, since MyFunction inherits from
                                                 // UserFunction<double>. If the function should
                                                 // be evaluated based on other types, the
                                                 // class MyFunction should be defined accordingly.
    
    std::cout << "f = [ " << result[0] << "," << result[1] << "]" << std::endl;  // print the result.

    return 0;
}



