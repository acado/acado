clear;
global counter;
global timecounter;
counter = 0;
timecounter = 0;

BEGIN_ACADO;                                % Always start with "BEGIN_ACADO". 

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
    
   
   
    %% Differential equation
    f = acado.DifferentialEquation();
    f.linkMatlabODE('ode');
    
    
    %% OCP
    ocp = acado.OCP(0.0, 18.0, 18);
    ocp.minimizeMayerTerm(W);

    ocp.subjectTo( f );
    
    % INITIAL VALUE CONSTRAINTS:
    ocp.subjectTo( 'AT_START', n == 0.0 );
    ocp.subjectTo( 'AT_START', W == 0.0 );


    % PERIODIC BOUNDARY CONSTRAINTS:
    ocp.subjectTo( 0.0, r     , -r     , 0.0 );
    ocp.subjectTo( 0.0, phi   , -phi   , 0.0 );
    ocp.subjectTo( 0.0, theta , -theta , 0.0 );
    ocp.subjectTo( 0.0, dr    , -dr    , 0.0 );
    ocp.subjectTo( 0.0, dphi  , -dphi  , 0.0 );
    ocp.subjectTo( 0.0, dtheta, -dtheta, 0.0 );
    ocp.subjectTo( 0.0, Psi   , -Psi   , 0.0 );
    ocp.subjectTo( 0.0, CL    , -CL    , 0.0 );

    ocp.subjectTo( -0.34   <= phi   <= 0.34   );
    ocp.subjectTo(  0.85   <= theta <= 1.45   );
    ocp.subjectTo( -40.0   <= dr    <= 10.0   );
    ocp.subjectTo( -0.29   <= Psi   <= 0.29   );
    ocp.subjectTo(  0.1    <= CL    <= 1.50   );
    ocp.subjectTo( -0.7    <= n     <= 0.90   );
    ocp.subjectTo( -25.0   <= ddr0  <= 25.0   );
    ocp.subjectTo( -0.065  <= dPsi  <= 0.065  );
    ocp.subjectTo( -3.5    <= dCL   <= 3.5    );
    
    
    %% Optimization Algorithm
    algo = acado.OptimizationAlgorithm(ocp);   
    algo.set('KKT_TOLERANCE', 1e-2);
    algo.set('MAX_NUM_ITERATIONS', 100);
    
    algo.initializeDifferentialStates(powerkite_states);
    algo.initializeControls          (powerkite_controls);
    
    
    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.


% Run the test
out = powerkite_RUN();


fprintf('TOTAL EVALS:    %20.0f \n', counter);
fprintf('TOTAL ODE TIME: %20.5f ms\n', timecounter*1000);
fprintf('TOTAL ODE TIME/EVAL: %20.10f ms\n', (timecounter/counter)*1000);

draw;