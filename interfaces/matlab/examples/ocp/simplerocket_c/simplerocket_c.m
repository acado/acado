clear;

BEGIN_ACADO;                                % Always start with "BEGIN_ACADO". 
    
    acadoSet('problemname', 'simplerocket_c'); 
    
    DifferentialState v s m L;
    Control u;
    
    % Set default objects
    f = acado.DifferentialEquation();
    f.linkCFunction('cfunction.cpp', 'myAcadoDifferentialEquation');
    
    
    ocp = acado.OCP(0.0, 10, 40);
    
    ocp.minimizeMayerTerm(L);  % minimizeLagrange is not yet implemented for matlab ode calls!
                               % but you can define another differential
                               % state to get the same effect (L)
    
    ocp.subjectTo( f );
    ocp.subjectTo( 'AT_START', s ==  0.0 );
    ocp.subjectTo( 'AT_START', v ==  0.0 );
    ocp.subjectTo( 'AT_START', L ==  0.0 );
    ocp.subjectTo( 'AT_START', m ==  1.0 );
    ocp.subjectTo( 'AT_END'  , s == 10.0 );
    ocp.subjectTo( 'AT_END'  , v ==  0.0 );
    ocp.subjectTo( -0.01 <= v <= 1.3 );
    
    
    algo = acado.OptimizationAlgorithm(ocp);
    % !!
    % algo.set( 'HESSIAN_APPROXIMATION', 'EXACT_HESSIAN' );    
    % DO NOT USE EXACT HESSIAN WHEN LINKING TO MATLAB ODE
    % !!
    
    algo.set( 'KKT_TOLERANCE', 1e-5 );

    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.

% Run the test
out = simplerocket_c_RUN();

draw;