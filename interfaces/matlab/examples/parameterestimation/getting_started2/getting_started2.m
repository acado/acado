clear;

BEGIN_ACADO;                                % Always start with "BEGIN_ACADO". 
    
    acadoSet('problemname', 'getting_started');     % Set your problemname. If you 
                                                    % skip this, all files will
                                                    % be named
                                                    % "myAcadoProblem"

    %% INTRODUCE THE VARIABLES:
    DifferentialState      phi;    % the angle phi
    DifferentialState     dphi;    % the first derivative of phi w.r.t. time

    Parameter                l;    % the length of the pendulum
    Parameter            alpha;    % frictional constant
    Parameter                g;    % the gravitational constant

    Control                  F;    % force acting on the pendulum
                                   % (control input)
        
    
    %% Differential Equation
    f = acado.DifferentialEquation();       % Set the differential equation object
        
    % possibility 1: link a Matlab ODE
    % f.linkMatlabODE('myode');
    
    % possibility 2: write down the ODE directly in ACADO syntax
    m = 1.0;
    f.ODE(dot(phi ) == dphi); 
    f.ODE(dot(dphi) == -(g/l)*sin(phi) - alpha*dphi + F/m);
    
    
    
    %% Optimal Control Problem
    % MEASUREMENT DATA. First column are time points, second column is phi (the measured state).
    M = [0.00000e+00    1.00000e+00
        2.72321e-01    7.33213e-01
        3.72821e-01    5.75146e-01
        7.25752e-01   -5.91794e-02
        9.06107e-01   -3.54347e-01
        1.23651e+00   -3.03056e-01
        1.42619e+00   -6.23527e-01
        1.59469e+00   -9.64208e-02
        1.72029e+00   -1.97671e-02
        2.00000e+00    9.35138e-02];

    
    % DEFINE A MEASUREMENT FUNCTION
    h={phi};   %The state phi is being measured.
    
    % DEFINE THE INVERSE OF THE VARIANCE-COVARIANCE MATRIX OF THE MEASUREMENTS:
    S = eye(1);
    S(1,1) = 1/(0.1)^2; % (1 over the variance of the measurement)  HERE: the standard deviation of the measurement is assumed to be 0.1, thus S = 1/(0.1)^2.
    

    ocp = acado.OCP(M(:,1)');               % The OCP should be evaluated on a grid
                                            % equal to the timepoints in
                                            % the measurement matrix M.
                                            
    ocp.minimizeLSQ( S, h, M );             % We want to minimize a least squares problem
                                            % with S, h and the
                                            % measurements
                                            
    ocp.subjectTo( f );                     % Your OCP is always subject to your 
                                            % differential equation
                                            
    ocp.subjectTo( 0.0 <= alpha <= 4.0  ); % bounds
    ocp.subjectTo( 0.0 <=   l   <= 2.0  );
    ocp.subjectTo( F == 0.1   );
    ocp.subjectTo( g == 9.81 );

    
    
    %% Optimization Algorithm
    algo = acado.ParameterEstimationAlgorithm(ocp);  % Setup the parameter estimation algorithm
    algo.initializeDifferentialStates(M);   % Initialize the differential states

    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.
                     

% Run the test
out = getting_started_RUN();                % Run the test. The name of the RUN file
                                            % is problemname_RUN, so in
                                            % this case getting_started_RUN

out.PARAMETERS