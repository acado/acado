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

    

    mB = 350.0;                             % Some static parameters
	mW = 50.0;
	kS = 20000.0;
	kT = 200000.0;
    
    %% Differential Equation
    f = acado.DifferentialEquation();       % Set the differential equation object
    
    f.linkMatlabODE('ode');                 % Link a Matlab ODE
    
    
    %% Optimal Control Problem
    ocp = acado.OCP(0.0, 1.0, 50);          % Set up the Optimal Control Problem (OCP)
                                            % Start at 0s, control in 20
                                            % intervals upto 1s

    h={xB, xW, vB, vW};                     % the LSQ-Function

    Q = eye(4);                             % The weighting matrix
    Q(1,1) = 10;
    Q(2,2) = 10;
    
    r = zeros(1,4);                         % The reference
        
    ocp.minimizeLSQ( Q, h, r );             % Minimize this Least Squares Term
    %ocp.minimizeLSQ( h, r );               % (Other possibility)
    %ocp.minimizeLSQ( h );                  % (Other possibility)
    
    ocp.subjectTo( f );                     % Your OCP is always subject to your 
                                            % differential equation

    ocp.subjectTo( -500.0 <= F <= 500.0 ); % Bounds
    ocp.subjectTo(  R == 0.0 ); 

   
    
    %% SETTING UP THE (SIMULATED) PROCESS
    identity = acado.OutputFcn();
    dynamicSystem = acado.DynamicSystem(f, identity);    % Set up a dynamic system with
                                                         % the differential equation and an
                                                         % output function
                                                         
    process = acado.Process(dynamicSystem, 'INT_RK45');  % Simulates the process to be controlled 
                                                         % based on a dynamic model.
                                                         % The class Process is one of the two main 
                                                         % building-blocks within the SimulationEnvironment 
                                                         % and complements the Controller. It simulates the 
                                                         % process to be controlled based on a dynamic model.
                                                         
    disturbance = [                                      % The process disturbance matrix consists of one
        0.0       0.00                                   % column with timepoints and one for each disturbance.
        0.5       0.01
        0.55      0.01
        0.6       0.00
        3.0       0.00];
    process.setProcessDisturbance(disturbance);         % Set the process disturbances
    
    
    %% SETTING UP THE MPC CONTROLLER:
    algo = acado.RealTimeAlgorithm(ocp, 0.02);          % The class RealTimeAlgorithm serves as a user-interface 
                                                        % to formulate and solve model predictive control problems.
                                                        
    algo.set('MAX_NUM_ITERATIONS', 2 );                 % Set some algorithm parameters

    algo.set('INTEGRATOR_TYPE', 'INT_RK45');
    algo.set( 'INTEGRATOR_TOLERANCE',   1e-5);    
    algo.set( 'ABSOLUTE_TOLERANCE',     1e-4 );
    
    zeroReference = acado.StaticReferenceTrajectory();  % Allows to define a static reference trajectory that 
                                                        % the ControlLaw aims to track. 
  
    
    controller = acado.Controller( algo,zeroReference ); % The controller complements a Process. 
                                                         % It contains an online control law for
                                                         % obtaining the control inputs of a process
    
    
    %% SETTING UP THE SIMULATION ENVIRONMENT,  RUN THE EXAMPLE..
    sim = acado.SimulationEnvironment( 0.0,3.0,process,controller ); % Setup the closed-loop simulations of dynamic systems. 
                                                                     % Simulate from 0 to 3 sec
    
    r = zeros(1,4);                                     % Initilize the states
    sim.init( r );

    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.
                     
    
% Run the test
out = active_damping_RUN();

draw;
