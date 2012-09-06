clear;

BEGIN_ACADO;                                % Always start with "BEGIN_ACADO". 
    
    acadoSet('problemname', 'time_optimal_rocket');% Set your problemname. If you 
                                            % skip this, all files will
                                            % be named "myAcadoProblem"

    DifferentialState v;                    % Velocity 
    DifferentialState s;                    % Distance
    DifferentialState m;                    % Mass
    
    Parameter T;                            % We would like to minize the T, so T is a parameter
    
    Control u;                              % Control input
    
    
    %% Diferential Equation
    f = acado.DifferentialEquation(0, T);   % Set the differential equation object. We would like to minize the time.
   
    f.add(dot(s) == v);                     % Write down your ODE. 
    f.add(dot(v) == (u-0.02*v*v)/m);        %
    f.add(dot(m) == -0.01*u*u);             %
    
    
    %% Optimal Control Problem
    ocp = acado.OCP(0.0, T);                % Set up the Optimal Control Problem (OCP)
                                            % Start at 0 and go to T.
                                            
    ocp.minimizeMayerTerm(T);               % Minimize the consumed energy
    
    ocp.subjectTo( f );                     % Optimize with respect to your differential equation
    
    ocp.subjectTo( 'AT_START', s ==  0.0 ); % s(0) = 0
    ocp.subjectTo( 'AT_START', v ==  0.0 ); % v(0) = 0
    ocp.subjectTo( 'AT_START', m ==  1.0 ); % m(0) = 1
    
    ocp.subjectTo( 'AT_END'  , s == 10.0 ); % s(T) = 10 fly in 10 seconds to position 10 with minimum energy
    ocp.subjectTo( 'AT_END'  , v ==  0.0 ); % v(T) = 0  speed at the end should be zero 
    
    ocp.subjectTo( -0.1 <= v <= 1.7 );      % path constraint on speed
    ocp.subjectTo( -1.1 <= u <= 1.1 );      % Restrict the control, since otherwise it would take very large values in order to minimize T
    ocp.subjectTo(    5 <= T <= 15  );      % Restrict the time horizon
    
    
    %% Optimization Algorithm
    algo =acado.OptimizationAlgorithm(ocp); % Set up the optimization algorithm, link it to your OCP
    algo.set( 'KKT_TOLERANCE', 1e-10 );     % Set a custom KKT tolerance
    
    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.

% Run the test
out = time_optimal_rocket_RUN();

draw;