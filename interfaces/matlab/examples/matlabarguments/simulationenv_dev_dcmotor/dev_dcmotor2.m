% This example shows how to use acado.MexInput and variants.
%
% Use acado.MexInput to set a variable numeric value
% Use acado.MexInputVector to set variable vectors
% Use acado.MexInputMatrix to set variable matrices
%
% Compile your file. 
% You can now run it multiple times with different parameters without
% needing to recompile again. Eg:  dev_dcmotor2_RUN(0, [0])
%
% REMARK: notice the sequence of the parameters, it's the same sequence as
% how they were defined: 
% dev_dcmotor2_RUN(input1_startime, input1_x0)

clear;

BEGIN_ACADO; 
    
    acadoSet('problemname', 'dev_dcmotor2');
    
    DifferentialState x;                   
    Control u;
    
    input1_startime = acado.MexInput;       % Startime is free
    input2_x0 = acado.MexInputVector;       % x0 is free

    
    t_start	= 0.0;
    t_end	= 10.0;
    alphaD	= 0.8;	% alpha Desired
    
    %% Differential Equation
    f = acado.DifferentialEquation();      
    
    f.ODE(dot(x) == -43.548913059153811*x +   1.0*u);
    alpha = 27.062532550322111*x + 0.377532234341255*u;
    
    
    %% Optimal Control Problem
    ocp = acado.OCP(t_start, t_end, 10 );         

    h={alpha};                          
    
    r = zeros(1,1);
    r(1) = alphaD;

    
    ocp.minimizeLSQ( [10], h, r );
    
    ocp.subjectTo( f );                     
    ocp.subjectTo( 0 <= u <= 1.0 ); 
    ocp.subjectTo( 0.0 <= alpha <= 1.0 ); 


    %% SETTING UP THE MPC CONTROLLER:
    algo = acado.RealTimeAlgorithm(ocp, 0.05);  
    
    algo.set('MAX_NUM_ITERATIONS', 3 );
    algo.set('HESSIAN_APPROXIMATION', 'GAUSS_NEWTON' );
    
    
    %% CONTROLLER
    ref = [ 0.0    0.0
            1.0    0.2
            3.0    0.8
            5.0    0.5];

    reference = acado.PeriodicReferenceTrajectory(ref);    
                                                        
    controller = acado.Controller( algo,reference ); 
    
    controller.init(input1_startime, input2_x0);     % USE THE FREE VARIABLES
    controller.step(input1_startime, input2_x0);
 
    
END_ACADO;    



% EXECUTE MULTIPLE STEPS
optimcontrol = [];
for i=1:10
    
    x0 = [0 + 0.01*i];   % some perturbations
    out = dev_dcmotor2_RUN(i, x0);   % changing time
    optimcontrol(i) = out.U(1);
    
end


for i=1:length(optimcontrol)
   fprintf(sprintf('Current optimal control after step %d: %f \n', i, optimcontrol(i)));
end