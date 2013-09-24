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
 *    \file include/acado/symbolic_expression/symbolic_expression.hpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */


#ifndef ACADO_TOOLKIT_SYMBOLIC_EXPRESSION_HPP
#define ACADO_TOOLKIT_SYMBOLIC_EXPRESSION_HPP


    // COLLECTION OF ALL EXPRESSION-HEADER FILES:
    // -------------------------------------------------------

    #include <acado/symbolic_expression/expression.hpp>
    #include <acado/symbolic_expression/acado_syntax.hpp>

    #include <acado/symbolic_expression/differential_state.hpp>
    #include <acado/symbolic_expression/differential_state_derivative.hpp>
    #include <acado/symbolic_expression/algebraic_state.hpp>
    #include <acado/symbolic_expression/parameter.hpp>
    #include <acado/symbolic_expression/control.hpp>
    #include <acado/symbolic_expression/disturbance.hpp>
    #include <acado/symbolic_expression/integer_control.hpp>
    #include <acado/symbolic_expression/integer_parameter.hpp>
    #include <acado/symbolic_expression/vt_time.hpp>
    #include <acado/symbolic_expression/intermediate_state.hpp>
    #include <acado/symbolic_expression/output.hpp>
    #include <acado/symbolic_expression/lyapunov.hpp>

    #include <acado/symbolic_expression/constraint_component.hpp>


// 	BEGIN_NAMESPACE_ACADO
// 
// 	returnValue clearAllStaticCounters( )
// 	{
// 		DifferentialState           dummy1;
// 		DifferentialStateDerivative dummy2;
// 		AlgebraicState              dummy3;
// 		IntermediateState           dummy4;
// 		Parameter                   dummy5;
// 		Control                     dummy6;
// 		Disturbance                 dummy7;
// 
// 		dummy1.clearStaticCounters( );
// 		dummy2.clearStaticCounters( );
// 		dummy3.clearStaticCounters( );
// 		dummy4.clearStaticCounters( );
// 		dummy5.clearStaticCounters( );
// 		dummy6.clearStaticCounters( );
// 		dummy7.clearStaticCounters( );
// 		
// 		return SUCCESSFUL_RETURN;
// 	}
// 	
// 	CLOSE_NAMESPACE_ACADO


#endif // ACADO_TOOLKIT_SYMBOLIC_EXPRESSION_HPP
