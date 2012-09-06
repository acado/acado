% This example shows how to use acado.MexInput and variants.
%
% Use acado.MexInput to set a variable numeric value
% Use acado.MexInputVector to set variable vectors
% Use acado.MexInputMatrix to set variable matrices
%
% Compile your file. 
% You can now run it multiple times with different parameters without
% needing to recompile again. Eg: 
% getting_started_RUN(M(:,1)', M);
%
% REMARK: notice the sequence of the parameters, it's the same sequence as
% how they were defined: getting_started_RUN(timepoints, measurements)

clear;

BEGIN_ACADO;  
    
    acadoSet('problemname', 'getting_started');  
    
    DifferentialState       phi;            % the angle phi
    DifferentialState       dphi;           % the first derivative of phi w.r.t. time
    Parameter               l;              % the length of the pendulum
    Parameter               alpha;          % frictional constant
    Parameter               g;              % the gravitational constant    
    
    % GRID AND MEASUREMENTS ARE FREE
    timepoints = acado.MexInputVector;
    measurements = acado.MexInputMatrix;
    % GRID AND MEASUREMENTS ARE FREE
    
    
    %% Differential Equation
    f = acado.DifferentialEquation();
    f.linkMatlabODE('myode');

    
    %% Optimal Control Problem
    h={phi};  
    S = eye(1);
    S(1,1) = 1/(0.1)^2;    

    ocp = acado.OCP(timepoints);    % FREE VARIABLE MexInputVector                                  
    ocp.minimizeLSQ( S, h, measurements );  % FREE VARIABLE MexInputMatrix                                    
    ocp.subjectTo( f );                                                         
    ocp.subjectTo( 0.0 <= alpha <= 4.0  ); 
    ocp.subjectTo( 0.0 <=   l   <= 2.0  );
    ocp.subjectTo( g == 9.81 );
    
    
    %% Optimization Algorithm
    algo = acado.ParameterEstimationAlgorithm(ocp);  
    %algo.initializeDifferentialStates(measurements);  % FREE VARIABLE MexInputMatrix (optional)
    
END_ACADO;                     



%% Run the test

% MEASUREMENT DATA. First column are time points, second column is phi.
M = [0.00000e+00    1.00000e+00
    3.72821e-01    5.75146e-01
    7.25752e-01   -5.91794e-02
    9.06107e-01   -3.54347e-01
    1.23651e+00   -3.03056e-01
    1.59469e+00   -9.64208e-02
    1.72029e+00   -1.97671e-02
    2.00000e+00    9.35138e-02];

guess_l     = [];
guess_alpha = [];
guess_g     = [];

% We simulate an improving measurement matrix over time.
% Everytime we get a new measurement, a new parameter estimation is ran.
% At least two measurements are needed !!
for i=2:8
    out = getting_started_RUN(M(1:i,1)',            M(1:i,:));
                            % timepoints (vector),  matrix (small in the beginning, big in the end)
    
    guess_l(i)      = out.PARAMETERS(1);       % Parameter results are located in the first row, starting at column index 2 
    guess_alpha(i)  = out.PARAMETERS(2);
    guess_g(i)      = out.PARAMETERS(3);
    
end

hold on

plot(guess_l(2:end), 'b')
plot(guess_alpha(2:end), 'r')
plot(guess_g(2:end), 'g')

xlabel('Result after x measurements');
ylabel('Value of parameter');

hold off

legend('Guess L', 'Guess alpha', 'Guess G');
