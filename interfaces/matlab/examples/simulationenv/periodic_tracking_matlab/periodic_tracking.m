clear;

BEGIN_ACADO;
    
    acadoSet('problemname', 'periodic_tracking');

    DifferentialState x;
    Control           u;
    Disturbance       w;
    

    % Differential Equation
    f = acado.DifferentialEquation();
    f.linkMatlabODE('ode1');
    
    f2 = acado.DifferentialEquation();      % Use a different differential equation for simulation vs optimization
    f2.linkMatlabODE('ode2');
    

    h={x u};
    Q = eye(2);
    r = zeros(1,2);
    
    ocp = acado.OCP(0.0, 7.0, 14);
    ocp.minimizeLSQ( Q, h, r );
    
    ocp.subjectTo( f );
    ocp.subjectTo( -1.0 <= u <= 2.0 );
    %ocp.subjectTo( w == 0.0 );
     
    % SETTING UP THE (SIMULATED) PROCESS:
    identity = acado.OutputFcn();
    dynamicSystem = acado.DynamicSystem(f2, identity);    
    process = acado.Process(dynamicSystem, 'INT_RK45');
    disturbance = [
        0.0       0.00
        0.5       0.00
        1.0       0.00
        1.5       1.00
        2.0       1.00
        2.5       1.00
        3.0       0.00
        3.5       0.00
        4.0       0.00
        15.0      0.00
        30.0      0.00];
    process.setProcessDisturbance(disturbance);
    
    
    % SETUP OF THE ALGORITHM AND THE TUNING OPTIONS:
    algo = acado.RealTimeAlgorithm(ocp, 0.5);
    algo.set( 'HESSIAN_APPROXIMATION', 'GAUSS_NEWTON' );
    algo.set('MAX_NUM_ITERATIONS', 2 );
    
    
    % SETTING UP THE NMPC CONTROLLER:
    ref = [0.0       0.00       0.00            % Set up a given reference trajectory
        0.5       0.00       0.00               % This trajectory is PERIODIC!
        1.0       0.00       0.00
        1.25      0.00       0.00
        1.5       0.00       0.00
        1.75      0.00       0.00
        2.0       0.00       0.00
        2.5       0.00       0.00
        3.0       0.00       -0.50
        3.5       0.00       -0.50
        4.0       0.00       0.00];
    %   TIME      X_REF      U_REF
    
    reference = acado.PeriodicReferenceTrajectory(ref);    
    controller = acado.Controller( algo,reference );
    
    
    % SETTING UP THE SIMULATION ENVIRONMENT,  RUN THE EXAMPLE..
    sim = acado.SimulationEnvironment( 0.0,15.0,process,controller );
    
    r = zeros(1,1);
    r(1,1) = 1;
    sim.init( r );

    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.

% Run the test
out = periodic_tracking_RUN();

draw;