% This is test problem. 
% Please refer to the other examples.

clear;

BEGIN_ACADO; 
    
    acadoSet('problemname', 'dev_dcmotor');
    
    DifferentialState x;                      
    Control u;

    t_start	= 0.0;
    t_end	= 1.0;
    alphaD	= 0.8;	% alpha Desired
    
    %% Differential Equation
    f = acado.DifferentialEquation();      
    
    f.add(dot(x) == -43.548913059153811*x +   1.0*u);
    
    alpha = 27.062532550322111*x + 0.377532234341255*u;
    
    
    %% Optimal Control Problem
    ocp = acado.OCP(t_start, t_end, 10 );         

    h={alpha};                          

    Q = eye(1,1);          
    Q(1,1) = 10;
    
    r = zeros(1,1);
    r(1) = alphaD;

    
    ocp.minimizeLSQ( Q, h, r );
    
    ocp.subjectTo( f );                     
    ocp.subjectTo( 0 <= u <= 1.0 ); 
    ocp.subjectTo( 0.0 <= alpha <= 1.0 ); 


    %% SETTING UP THE MPC CONTROLLER:
    algo = acado.RealTimeAlgorithm(ocp, 0.05);  
    
    algo.set('MAX_NUM_ITERATIONS', 3 );
    algo.set('HESSIAN_APPROXIMATION', 'GAUSS_NEWTON' );
    
    
    %% CONTROLLER
    x0 = zeros(1,1); 
    ref = [ 0.0    0.0
            1.0    0.2
            3.0    0.8
            5.0    0.5];
    reference = acado.PeriodicReferenceTrajectory(ref);    
                                                        
    controller = acado.Controller( algo,reference ); 
    
    controller.init(0, x0); 
    controller.step(0, x0);
 
    
END_ACADO;    

% Run the test 
out = dev_dcmotor_RUN()      % RUN THE EXAMPLE

