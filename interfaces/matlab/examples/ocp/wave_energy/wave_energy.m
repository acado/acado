clear;

% Maximize produced wave energy
% The buoy moves using the wave energy (represented by a sin in hw)

BEGIN_ACADO;                                % Always start with "BEGIN_ACADO". 
    
    acadoSet('problemname', 'wave_energy'); % Set your problemname. If you 
                                            % skip this, all files will
                                            % be named "myAcadoProblem"
    
    
    DifferentialState       h;              % Position of the buoy
    DifferentialState       v;              % Velocity of the buoy
    DifferentialState       w;              % Produced wave energy
    Control                 u;              % The control (free parameter), can be used to adjust a force at the generator
    TIME                    t;              % the TIME (can be defined only once)

    h_hw = 10;                              % water level
    A_hw = 1.0;                             % amplitude of the waves
    T_hw = 5.0;                             % duration of a wave
    h_b  = 3.0;                             % height of the buoy
    rho  = 1000;                            % density of water
    A    = 1.0;                             % bottom area of the buoy
    m    = 100;                             % mass of the buoy
    g    = 9.81;                            % gravitational constant
    
    %% Intermediate state
    hw = h_hw + A_hw*sin(2*pi*t/T_hw);      % Height of the wave
  
    
    %% Differential Equation
    f = acado.DifferentialEquation();       % Set the differential equation object
    
    f.add(dot(h) ==  v);                    % Write down your ODE. You can add 
    f.add(dot(v) ==  rho*A*(hw-h)/m-g-u);   % multiple ODE's with f.add() when you 
    f.add(dot(w) ==  u*v);                  % have multiple differential states. 

    
    %% Optimal Control Problem
    ocp = acado.OCP(0.0, 15.0, 50);        % Set up the Optimal Control Problem (OCP) Start at 0s, control in 50 intervals to 15s
                                            
    ocp.maximizeMayerTerm( w );             % Maximize the energy
    
    ocp.subjectTo( f );                     % Your OCP is always subject to your 
                                            % differential equation
                                            
    ocp.subjectTo( 'AT_START', h - (h_hw-A_hw) ==  0.0 );
    ocp.subjectTo( 'AT_START', v ==  0.0 );
    ocp.subjectTo( 'AT_START', w ==  0.0 );
  
    ocp.subjectTo( -h_b <= h-hw <= 0.0 );
    ocp.subjectTo( 0.0 <= u <= 100.0 );
    
    
    %% Optimization Algorithm
    algo = acado.OptimizationAlgorithm(ocp); % Set up the optimization algorithm
    
    algo.set('MAX_NUM_ITERATIONS', 100 );    % Set some parameters for the algorithm
    algo.set('HESSIAN_APPROXIMATION','EXACT_HESSIAN' );
    
    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.



out = wave_energy_RUN();                    % Run the test. The name of the RUN file
                                            % is problemname_RUN, so in
                                            % this case wave_energy_RUN
                                            
draw;