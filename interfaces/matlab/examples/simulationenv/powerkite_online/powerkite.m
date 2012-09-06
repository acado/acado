clear;

BEGIN_ACADO;
    
    acadoSet('problemname', 'powerkite');
    
    %% Symbolics
    % DIFFERENTIAL STATES :
    DifferentialState      r;      %  the length r of the cable
    DifferentialState    phi;      %  the angle phi
    DifferentialState  theta;      %  the angle theta
    DifferentialState     dr;      %  first  derivative of r0    with respect to t
    DifferentialState   dphi;      %  first  derivative of phi   with respect to t
    DifferentialState dtheta;      %  first  derivative of theta with respect to t
    DifferentialState      n;      %  winding number
    DifferentialState    Psi;      %  the roll angle Psi
    DifferentialState     CL;      %  the aerodynamic lift coefficient
    DifferentialState      W;      %  integral over the power at the generator
                                   %  ( = ENERGY )
    % CONTROL :
    Control             ddr0;      %  second derivative of r0    with respect to t
    Control             dPsi;      %  first  derivative of Psi   with respect to t
    Control              dCL;      %  first  derivative of CL    with respect to t
    
    % DISTURBANCE:
    Disturbance      w_extra;
    

    %% Differential equation
    f = acado.DifferentialEquation();
    f.linkMatlabODE('ode');
    
    
    %% OCP
    model_response={r phi theta dr dphi dtheta ddr0 dPsi dCL };

	x_scal(1) =   60.0; 
	x_scal(2) =   1.0e-1;
	x_scal(3) =   1.0e-1;
	x_scal(4) =   40.0; 
	x_scal(5) =   1.0e-1;
	x_scal(6) =   1.0e-1;
	x_scal(7) =   60.0; 
	x_scal(8) =   1.5e-1;
	x_scal(9) =   2.5;  
    
    Q = eye(9,9);
    Q_end = eye(9,9);            

    for i=1:6 
	   Q(i,i) = (1.0e-1/x_scal(i))*(1.0e-1/x_scal(i));
       Q_end(i,i) = (5.0e-1/x_scal(i))*(5.0e-1/x_scal(i));            
    end
   
    for i=7:9 
	   Q(i,i) = (1.0e-1/x_scal(i))*(1.0e-1/x_scal(i));
       Q_end(i,i) = (5.0e-1/x_scal(i))*(5.0e-1/x_scal(i));            
    end    
    
    measurements = zeros(1,9);
    
    ocp = acado.OCP(0.0, 10.0, 10);
    ocp.minimizeLSQ( Q, model_response, measurements );
    ocp.minimizeLSQEndTerm( Q_end, model_response, measurements );
    
    
    ocp.subjectTo( f );
    ocp.subjectTo( -0.34   <= phi   <= 0.34   );
    ocp.subjectTo(  0.85   <= theta <= 1.45   );
    ocp.subjectTo( -40.0   <= dr    <= 10.0   );
    ocp.subjectTo( -0.29   <= Psi   <= 0.29   );
    ocp.subjectTo(  0.1    <= CL    <= 1.50   );
    ocp.subjectTo( -0.7    <= n     <= 0.90   );
    ocp.subjectTo( -25.0   <= ddr0  <= 25.0   );
    ocp.subjectTo( -0.065  <= dPsi  <= 0.065  );
    ocp.subjectTo( -3.5    <= dCL   <= 3.5    );
    ocp.subjectTo( -60.0   <= cos(theta)*r    );
    ocp.subjectTo( w_extra == 0.0             );
    
    

    
    %% SETTING UP THE (SIMULATED) PROCESS:
    identity = acado.OutputFcn();
    dynamicSystem = acado.DynamicSystem(f, identity);    
    process = acado.Process(dynamicSystem, 'INT_RK45');
    disturbance = [
        0.0       0.00
        0.5       0.00
        1.0       0.00
        1.5       0.00
        2.0       0.00
        2.5       0.00
        3.0       0.00
        3.5       0.00
        50.0      0.00
        51.0      10.00
        57.0      10.0
        58.0      0.0
        100.0     0.0];
    process.setProcessDisturbance(disturbance);
    
    
    
    %% SETUP OF THE ALGORITHM AND THE TUNING OPTIONS:
    algo = acado.RealTimeAlgorithm(ocp, 1.0);
    algo.set('HESSIAN_APPROXIMATION', 'GAUSS_NEWTON' );
    algo.set('MAX_NUM_ITERATIONS', 3 );
    algo.set('KKT_TOLERANCE', 1e-2 );
    algo.set('INTEGRATOR_TOLERANCE', 1e-5 );
    
    algo.initializeDifferentialStates(powerkite_states);
    algo.initializeControls          (powerkite_controls);
    
    
    %% SETTING UP THE NMPC CONTROLLER:
    reference = acado.PeriodicReferenceTrajectory(powerkite_reference);    
    controller = acado.Controller( algo,reference );
    
    
    
    %% SETTING UP THE SIMULATION ENVIRONMENT,  RUN THE EXAMPLE..
    sim = acado.SimulationEnvironment( 0.0,90.0,process,controller );
    x_0 = [1.8264164528775887e+03 -5.1770453309520573e-03 1.2706440287266794e+00 2.1977888424944396e+00 3.1840786108641383e-03 -3.8281200674676448e-02 0.0000000000000000e+00 -1.0372313936413566e-02 1.4999999999999616e+00 0.0000000000000000e+00];
    sim.init( x_0 );
    

    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.
                     

% Run the test
out = powerkite_RUN();

draw;