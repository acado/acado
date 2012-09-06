clear;

BEGIN_ACADO;                                % Always start with "BEGIN_ACADO". 
    
    acadoSet('problemname', 'active_damping');     % Set your problemname. If you 
                                                   % skip this, all files will
                                                   % be named "myAcadoProblem"
    
    
    DifferentialState xB xW vB vW;          % Differential States:
                                            % xBody, xWheel, vBody, vWheel
                                            
    Control F;                              % Control: 
                                            % dampingForce
                                            
    Disturbance R;                          % Disturbance: 
                                            % roadExcitation

    
    %% Differential Equation
    f = acado.DifferentialEquation();       % Set the differential equation object
    
    f.linkMatlabODE('ode');                 % Link to a Matlab ODE
    
    
    %% Optimal Control Problem

    ocp = acado.OCP(0.0, 1.0, 50);          % Set up the Optimal Control Problem (OCP)
                                            % Start at 0s, control in 50
                                            % intervals upto 1s

    
    h={xB, xW, vB, vW};                     % the LSQ-Function

    Q = eye(4);                             % The weighting matrix
    Q(1,1) = 10;
    Q(2,2) = 10;
    
    r = zeros(1,4);                         % The reference
    
    ocp.minimizeLSQ( Q, h, r );             % Minimize this Least Squares Term
    
    ocp.subjectTo( f );                     % Your OCP is always subject to your 
                                            % differential equation
                                            
    ocp.subjectTo( 'AT_START', xB == 0.01); % Initial conditions on states
	ocp.subjectTo( 'AT_START', xW == 0.0);
	ocp.subjectTo( 'AT_START', vB == 0.0 );
	ocp.subjectTo( 'AT_START', vW == 0.0 );
    
    ocp.subjectTo( -500.0 <= F <= 500.0 );  % Bounds
    ocp.subjectTo( R == 0.0 );

  
    
    %% Optimization Algorithm
    algo = acado.OptimizationAlgorithm(ocp); % Set up the optimization algorithm
    
    algo.set('INTEGRATOR_TYPE', 'INT_RK45');
    algo.set( 'INTEGRATOR_TOLERANCE',   1e-6);    
    algo.set( 'ABSOLUTE_TOLERANCE',     1e-4 );

    algo.set( 'HESSIAN_APPROXIMATION', 'GAUSS_NEWTON' );  % Example setting hessian approximation
    %algo.set( 'HESSIAN_APPROXIMATION', 'CONSTANT_HESSIAN' );  % Other possible settings
    %algo.set( 'HESSIAN_APPROXIMATION', 'FULL_BFGS_UPDATE' );
    %algo.set( 'HESSIAN_APPROXIMATION', 'BLOCK_BFGS_UPDATE' );
    %algo.set( 'HESSIAN_APPROXIMATION', 'GAUSS_NEWTON_WITH_BLOCK_BFGS' );
    
    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. 
                     % You can run the file problemname_ACADO.m as
                     % many times as you want without having to compile again.


out = active_damping_RUN();                 % Run the test. The name of the RUN file
                                            % is problemname_RUN, so in
                                            % this case active_damping_RUN

draw;
