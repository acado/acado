%clear;

BEGIN_ACADO;                                % Always start with "BEGIN_ACADO". 
    
    acadoSet('problemname', 'getting_started');     % Set your problemname. If you 
                                                    % skip this, all files will
                                                    % be named "myAcadoProblem"
    
    
    DifferentialState       x;              % The differential states
    Control                 u;              % The controls
    Disturbance             w;              % The disturbances
    Parameter               p q;            % The free parameters
    
    
    %% Differential Equation
    f = acado.DifferentialEquation();       % Set the differential equation object
    
    f.add(dot(x) == -x*x + p + u*u + w);    % Write down your ODE. You can add 
                                            % multiple ODE's with f.add() when you 
                                            % have multiple states. To print an ODE 
                                            % to the screen, use
                                            % f.differentialList{1}.toString

    
    %% Optimal Control Problem
    ocp = acado.OCP(0.0, 1.0, 20);          % Set up the Optimal Control Problem (OCP)
                                            % Start at 0s, control in 20
                                            % intervals upto 1s
                                            
    ocp.minimizeMayerTerm(x - p*p + q^2);   % Minimize a Mayer term
    
    ocp.subjectTo( f );                     % Your OCP is always subject to your 
                                            % differential equation
                                            
    ocp.subjectTo( 'AT_START', x == 1.0 );  % Initial condition
    ocp.subjectTo(  0.1 <= p <= 2.0 );      % Bounds
    ocp.subjectTo(  0.1 <= u <= 2.0 );
    ocp.subjectTo( -0.1 <= w <= 2.1 );
    
    
    %% Optimization Algorithm
    algo = acado.OptimizationAlgorithm(ocp); % Set up the optimization algorithm
    
    algo.set('INTEGRATOR_TOLERANCE', 1e-5 ); % Set some parameters for the algorithm
    
    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.



out = getting_started_RUN();                % Run the test. The name of the RUN file
                                            % is problemname_RUN, so in
                                            % this case getting_started_RUN
                                            
draw;