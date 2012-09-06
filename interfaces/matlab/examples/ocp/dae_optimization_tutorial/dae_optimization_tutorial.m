clear;

BEGIN_ACADO;                                % Always start with "BEGIN_ACADO". 
    
    acadoSet('problemname', 'dae_optimization_tutorial'); % Set your problemname. If you 
                                                          % skip this, all files will
                                                          % be named "myAcadoProblem"
    
    
    DifferentialState       x;              % The differential states
    DifferentialState       l;
    AlgebraicState          z;              % The algebraic state
    Control                 u;              % The control

    
    %% Differential Equation
    f = acado.DifferentialEquation();       % Set the differential equation object
    
    f.add(dot(x) == -x + 0.5*x*x + u + 0.5*z);  % Write down your equations. You can add 
    f.add(dot(l) ==  x*x + 3.0*u*u          );  % multiple ODE's and DAE's with f.add()   
    f.add(     0 ==  z + exp(z) - 1.0 + x   );  % when you have multiple states.
    

    
    %% Optimal Control Problem
    ocp = acado.OCP(0.0, 5.0, 10);          % Set up the Optimal Control Problem (OCP)
                                            % Start at 0s, control in 10
                                            % intervals upto 5s
                                            
    ocp.minimizeMayerTerm(l);               % Minimize a Mayer term
    
    ocp.subjectTo( f );                     % Your OCP is always subject to your 
                                            % differential equation
                                            
    ocp.subjectTo( 'AT_START', x == 1.0 );  % Initial condition
    ocp.subjectTo( 'AT_START', l == 0.0 );   
    
    
    
    %% Optimization Algorithm
    algo = acado.OptimizationAlgorithm(ocp); % Set up the optimization algorithm
    
    algo.set('KKT_TOLERANCE', 1e-5 );        % Set some parameters for the algorithm
    algo.set('RELAXATION_PARAMETER', 1.5 );
    
    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.



out = dae_optimization_tutorial_RUN();      % Run the test. The name of the RUN file
                                            % is problemname_RUN, so in
                                            % this case
                                            % dae_optimization_tutorial_RUN
                                            
draw;