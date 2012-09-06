function set(obj, name, value)
%Sets value of an option.
%
%  Usage:
%    >> OptimizationAlgorithm.set(name, value)
%
%  Parameters:
%    name  Possible values:     [STRING]
%         OPT_UNKNOWN 	
%         MAX_NUM_STEPS 	
%         INTEGRATOR_TOLERANCE 	
%         ABSOLUTE_TOLERANCE 	
%         INITIAL_STEPSIZE 	
%         MIN_STEPSIZE 	
%         MAX_STEPSIZE 	
%         STEPSIZE_TUNING 	
%         CORRECTOR_TOLERANCE 	
%         INTEGRATOR_PRINTLEVEL 	
%         LINEAR_ALGEBRA_SOLVER 	
%         ALGEBRAIC_RELAXATION 	
%         RELAXATION_PARAMETER 	
%         FEASIBILITY_CHECK 	
%         MAX_NUM_ITERATIONS 	
%         KKT_TOLERANCE 	
%         KKT_TOLERANCE_SAFEGUARD 	
%         LEVENBERG_MARQUARDT 	
%         PRINTLEVEL 	
%         PRINT_COPYRIGHT 	
%         HESSIAN_APPROXIMATION 	
%         DYNAMIC_SENSITIVITY 	
%         OBJECTIVE_SENSITIVITY 	
%         CONSTRAINT_SENSITIVITY 	
%         DISCRETIZATION_TYPE 	
%         LINESEARCH_TOLERANCE 	
%         MIN_LINESEARCH_PARAMETER 	
%         MAX_NUM_QP_ITERATIONS 	
%         HOTSTART_QP 	
%         INFEASIBLE_QP_RELAXATION 	
%         INFEASIBLE_QP_HANDLING 	
%         USE_REALTIME_ITERATIONS 	
%         INTEGRATOR_TYPE 	
%         SAMPLING_TIME 	
%         SIMULATE_COMPUTATIONAL_DELAY 	
%         PARETO_FRONT_DISCRETIZATION 	
%         PARETO_FRONT_GENERATION 	
%         PARETO_FRONT_HOTSTART 
%    value  Assigned value to option    [NUMERIC/HessianApproximationMode/..]
%
%  Example:
%    >> ocp = acado.OCP(0.0, 1.0, 20);
%    >> algo = acado.OptimizationAlgorithm(ocp);   
%    >> algo.set('INTEGRATOR_TOLERANCE', 1e-0 );
%
%  Licence:
%    This file is part of ACADO Toolkit  - (http://www.acadotoolkit.org/)
%
%    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
%    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
%    Developed within the Optimization in Engineering Center (OPTEC) under
%    supervision of Moritz Diehl. All rights reserved.
%
%    ACADO Toolkit is free software; you can redistribute it and/or
%    modify it under the terms of the GNU Lesser General Public
%    License as published by the Free Software Foundation; either
%    version 3 of the License, or (at your option) any later version.
%
%    ACADO Toolkit is distributed in the hope that it will be useful,
%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
%    Lesser General Public License for more details.
%
%    You should have received a copy of the GNU Lesser General Public
%    License along with ACADO Toolkit; if not, write to the Free Software
%    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
%
%    Author: David Ariens
%    Date: 2009
% 
   
if (isa(value, 'acado.MexInput'))
    error('You cant use an acado.MexInput for setting optimization algorithm options.');
end


    obj.set_n{end+1} = name;
    obj.set_v{end+1} = value;
    
end