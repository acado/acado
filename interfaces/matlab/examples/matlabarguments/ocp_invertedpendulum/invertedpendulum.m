% This example shows how to use acado.MexInput and variants.
%
% Use acado.MexInput to set a variable numeric value
% Use acado.MexInputVector to set variable vectors
% Use acado.MexInputMatrix to set variable matrices
%
% Compile your file. 
% You can now run it multiple times with different parameters without
% needing to recompile again. Eg: 
% invertedpendulum_RUN(3, [0 -0.00001], 15, 0.210)
%
% REMARK: notice the sequence of the parameters, it's the same sequence as
% how they were defined: RUN(input1, input2, input3, input4)

clear;

BEGIN_ACADO;
    
    acadoSet('problemname', 'invertedpendulum'); 

    DifferentialState x alpha xdot alphadot;
    Control F;
    Parameter T;
    
    input1 = acado.MexInput;            % used in subjectTo
    input2 = acado.MexInputMatrix;      % initializeControls. Initializations are always matrices, also when they contain only one row
    input3 = acado.MexInput;            % acado.OCP number of intervals
    input4 = acado.MexInput;            % used in the differential equation

    
    % Set default objects
    f = acado.DifferentialEquation(0.0, T);
    
    f.ODE(-dot(x) + xdot);
    f.ODE(-dot(alpha) + alphadot);
    f.ODE(-dot(xdot) + ( F - input4*cos(alpha)*9.81*sin(alpha) + 0.210*0.305*alphadot*alphadot*sin(alpha) ) / (0.455 + 0.210*sin(alpha)*sin(alpha)));
    f.ODE(-dot(alphadot) + (1.0/0.305)*(9.81*sin(alpha) - (( F - 0.210*cos(alpha)*9.81*sin(alpha) + 0.210*0.305*alphadot*alphadot*sin(alpha) ) / (0.455 + 0.210*sin(alpha)*sin(alpha)))*cos(alpha)));


    ocp = acado.OCP(0.0, T, input3);
    
    ocp.minimizeMayerTerm(T);

    ocp.subjectTo( f );
    ocp.subjectTo( 'AT_START', x ==  0.0 );     
    ocp.subjectTo( 'AT_START', alpha ==  0.0);    
    ocp.subjectTo( 'AT_START', xdot ==  0.0 );    
    ocp.subjectTo( 'AT_START', alphadot ==  0.0 ); 

    ocp.subjectTo( 'AT_END', x == input1 ); 
    ocp.subjectTo( 'AT_END', alpha == 0.0 );    
    ocp.subjectTo( 'AT_END', xdot ==  0.0 );    
    ocp.subjectTo( 'AT_END', alphadot ==  0.0 ); 

    ocp.subjectTo( -100 <= F <=  100 );   
    ocp.subjectTo( 1 <= T <= 15 );

    
    
    algo = acado.OptimizationAlgorithm(ocp);
    algo.set('KKT_TOLERANCE', 1e-4); 
    algo.initializeControls(input2);

    
END_ACADO;


out = invertedpendulum_RUN(3,    [0 -0.00001], 15,     0.210);
                        % input1 input2        input3  input4
draw

