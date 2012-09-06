clear;

BEGIN_ACADO;                                % Always start with "BEGIN_ACADO". 
    
    acadoSet('problemname', 'dae_optimization_tutorial2');% Set your problemname. If you 
                                                          % skip this, all files will
                                                          % be named "myAcadoProblem"
    
    
    DifferentialState       x;              % The differential state
    AlgebraicState          z;              % The algebraic state
    Control                 u;              % The control
    Parameter               p;              % The parameter
    
    
    %% Differential Equation
    f = acado.DifferentialEquation();       % Set the differential equation object
    
    f.add(dot(x) == -0.5*x-z+u*u     );     % Write down your equations. You can add 
    f.add(     0 ==  z+exp(z)+x-1.0+u);     % multiple ODE's and DAE's with f.add()   
                                            % when you have multiple states.
    

    
    %% Optimal Control Problem
    ocp = acado.OCP(0.0, 4.0);              % Set up the Optimal Control Problem (OCP)
                                            % Start at 0s, control in 10
                                            % intervals upto 5s
                                            
    ocp.minimizeMayerTerm(x*x + p*p);       % Minimize a Mayer term
    
    ocp.subjectTo( f );                     % Your OCP is always subject to your 
                                            % differential equation
                                            
    ocp.subjectTo( 'AT_START', x == 1.0 );  % Initial condition
    ocp.subjectTo( 'AT_END', x + p == 1.0 );% Terminal constraint  
    
    ocp.subjectTo( -1.0 <= x*u <= 1.0 );    % a path constraint
    
    
    
    %% Optimization Algorithm
    algo = acado.OptimizationAlgorithm(ocp); % Set up the optimization algorithm
    
    algo.set('KKT_TOLERANCE', 1e-5 );        % Set some parameters for the algorithm

    
    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.



out = dae_optimization_tutorial2_RUN();     % Run the test. The name of the RUN file
                                            % is problemname_RUN, so in
                                            % this case
                                            % dae_optimization_tutorial2_RUN
                                            
draw;