/*
*    This file is part of ACADO Toolkit.
*
*    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
*    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
*    Developed within the Optimization in Engineering Center (OPTEC) under
*    supervision of Moritz Diehl. All rights reserved.
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
*    Author Rien Quirynen
*    Date 2012
*    http://www.acadotoolkit.org
*/

#include <acado_code_generation.hpp>

int main() {
	USING_NAMESPACE_ACADO
	
	Vector Meas(1);
	Meas(0) = 5;
 
    SIMexport sim( 1, 0.1 );
    
    sim.set( INTEGRATOR_TYPE, INT_IRK_RIIA3 );
    sim.set( NUM_INTEGRATOR_STEPS, 4 );
    sim.set( MEASUREMENT_GRID, EQUIDISTANT_GRID );
    
    sim.setModel( "model", "rhs", "rhs_jac" );
    sim.setDimensions( 6, 6, 5, 1 );
    
    sim.addOutput( "out", "out_jac", 2 );
    sim.setMeasurements( Meas );
    
    acadoPrintf( "-----------------------------------------\n  Using an externally defined Pendulum DAE model':\n-----------------------------------------\n" );
	
    sim.exportAndRun( "externModel_export", "init_externModel.txt", "controls_externModel.txt" );
    
	return 0;
} 

