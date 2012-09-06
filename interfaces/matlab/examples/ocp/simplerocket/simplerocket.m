clear;

BEGIN_ACADO;                                % Always start with "BEGIN_ACADO". 
    
    acadoSet('problemname', 'simplerocket');% Set your problemname. If you 
                                            % skip this, all files will
                                            % be named "myAcadoProblem"

    DifferentialState v;                    % Velocity 
    DifferentialState s;                    % Distance
    DifferentialState m;                    % Mass
    DifferentialState L;                    % Dummy state
    
    Control u;                              % Control input
    
    Disturbance test;
    
    %% Diferential Equation
    f = acado.DifferentialEquation();       % Set the differential equation object
   
    f.add(dot(s) == v);                     % Write down your ODE. 
    f.add(dot(v) == (u-test*v*v)/(m));      %
    f.add(dot(m) == -0.01*u*u);             %
    f.add(dot(L) == u*u);                   % Dummy equation to integrate used power
    
    %f.differentialList{1}.toString         % Print an equation to the screen 
    
    
    %% Optimal Control Problem
    ocp = acado.OCP(0.0, 10.0, 20);         % Set up the Optimal Control Problem (OCP)
                                            % Start at 0s, control in 20
                                            % intervals upto 10s
                                            
    ocp.minimizeMayerTerm(L);               % Minimize the consumed energy
    
    ocp.subjectTo( f );                     % Optimize with respect to your differential equation
    ocp.subjectTo( 'AT_START', s ==  0.0 ); % s(0) = 0
    ocp.subjectTo( 'AT_START', v ==  0.0 ); % v(0) = 0
    ocp.subjectTo( 'AT_START', L ==  0.0 ); % L(0) = 0
    ocp.subjectTo( 'AT_START', m ==  1.0 ); % m(0) = 1
    ocp.subjectTo( 'AT_END'  , s == 10.0 ); % s(10) = 10 fly in 10 seconds to position 10 with minimum energy
    ocp.subjectTo( 'AT_END'  , v ==  0.0 ); % v(10) = 0  speed at the end should be zero 
    ocp.subjectTo( -0.01 <= v <= 1.3 );     % path constraint on speed
    ocp.subjectTo( -1.1 <= u <= 1.1 );
    ocp.subjectTo( test == [0 0.021
        1 0.02
        2 0.01] );
    
    %% Optimization Algorithm
    algo =acado.OptimizationAlgorithm(ocp); % Set up the optimization algorithm, link it to your OCP
    algo.set( 'KKT_TOLERANCE', 1e-6 );      % Set a custom KKT tolerance
    
    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.

% Run the test
out = simplerocket_RUN();

draw;