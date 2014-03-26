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
 *    \file examples/integrator/loops.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */

#include <time.h>

#include <acado/utils/acado_utils.hpp>
#include <acado/user_interaction/user_interaction.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/function/function.hpp>



int main( ){

    USING_NAMESPACE_ACADO

       double t = -acadoGetTime();
    
       Expression x,y,z;

       z = 1.0;

       for( int i=0; i<5000; ++i )
            z += sin( z + x*y );

       Function f((x,y),z);

       t += acadoGetTime();
       
       printf("time for loading = %.6e \n", t );
       
       t = -acadoGetTime();
       
       std::vector<double> xx(2);
       xx[0] = 2.0;
       xx[1] = 0.0;
       
       std::vector<double> result = f.evaluate(xx);
       
       t += acadoGetTime();
       
       printf("time for evaluation = %.6e \n", t );
       
       std::cout << "f = " << result[0] << std::endl;
       
       

    return 0;
}



