clear;

BEGIN_ACADO;                                % Always start with "BEGIN_ACADO". 
    
    acadoSet('problemname', 'discreterocket');% Set your problemname. If you 
                                            % skip this, all files will
                                            % be named "myAcadoProblem"

    DifferentialState v;                    % Velocity 
    DifferentialState s;                    % Distance
    DifferentialState m;                    % Mass
    
    Control u;                              % Control input
    
    h = 0.01;
    
    %% Diferential Equation
    f = acado.DiscretizedDifferentialEquation(h); % Set the differential equation object
                                                  % 0.01 is the step length
   
    f.add(next(s) == s + h*v);                    % Write down your discrete ODE. 
    f.add(next(v) == v + h*(u-0.02*v*v)/m);       %
    f.add(next(m) == m - h*0.01*u*u);             %

    
    %% Optimal Control Problem
    ocp = acado.OCP(0.0, 10.0, 50);         % Set up the Optimal Control Problem (OCP)
                                            % Start at 0s, control in 50
                                            % intervals upto 10s
                                            
    ocp.minimizeLagrangeTerm( u*u );        % Minimize the consumed energy
    
    ocp.subjectTo( f );                     % Optimize with respect to your differential equation
    ocp.subjectTo( 'AT_START', s ==  0.0 ); % s(0) = 0
    ocp.subjectTo( 'AT_START', v ==  0.0 ); % v(0) = 0
    ocp.subjectTo( 'AT_START', m ==  1.0 ); % m(0) = 1
    ocp.subjectTo( 'AT_END'  , s == 10.0 ); % s(10) = 10 fly in 10 seconds to position 10 with minimum energy
    ocp.subjectTo( 'AT_END'  , v ==  0.0 ); % v(10) = 0  speed at the end should be zero 
    ocp.subjectTo( -0.01 <= v <= 1.3 );     % path constraint on speed

    
    %% Optimization Algorithm
    algo =acado.OptimizationAlgorithm(ocp); % Set up the optimization algorithm, link it to your OCP
    algo.set( 'HESSIAN_APPROXIMATION', 'EXACT_HESSIAN' );
    algo.set( 'KKT_TOLERANCE', 1e-10 );     % Set a custom KKT tolerance

    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.

% Run the test
out = discreterocket_RUN();

draw;