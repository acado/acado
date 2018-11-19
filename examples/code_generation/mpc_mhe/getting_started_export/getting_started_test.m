%% getting_started_test.m

close all;
clear all;
clc;

%% Make the solver
%  Copy code_generation_getting_started(.exe) to this directory
if isunix()
    !./code_generation_getting_started
else
    !code_generation_getting_started.exe
end
cd getting_started_export
make_acado_solver;
cd ..

%% Create the structures and set dimensions
%  Those dimensions must match ones defined in the c++ code generator.
N   = 10;
NX  = 4;
NU  = 1;
NY  = 5;
NYN = 4;

% Always use integration from the last node for shifting
mpcInput.shifting.strategy = 2;
% Initialization by a forward simulation.
mpcInput.initialization = 1;

mpcInput.x = zeros(N + 1, NX);
mpcInput.u = zeros(N, NU);
mpcInput.y = zeros(N, NY);
mpcInput.yN = zeros(1, NYN);

% In case weighting matrices are not defined, you can use:
% mpcInput.W = eye( NY );
% mpcInput.WN = eye( NYN ) * 5;

mpcInput.x0 = [2, 0, 0, 0];

%% Run the simulation

nSteps = 10;

% Run the simulation and plot horizons
hf = figure;
for kk = 1: nSteps
    % Call the solver
    mpcOutput = acado_solver( mpcInput );
    
    % Plot results
    figure( hf );
    subplot(5, 1, 1);
        stairs(mpcOutput.x(:, 1), 'b');
        legend('p');
    subplot(5, 1, 2);
        stairs(mpcOutput.x(:, 2), 'b');
        legend('v');
    subplot(5, 1, 3);
        stairs(mpcOutput.x(:, 3), 'b');
        legend('phi');
    subplot(5, 1, 4);
        stairs(mpcOutput.x(:, 4), 'b');
        legend('omega');
    subplot(5, 1, 5);
        stairs(mpcOutput.u(:, 1), 'r');
        legend('a');
    pause;
    
    % Prepare for the next step
    mpcInput.x = mpcOutput.x;
    mpcInput.u = mpcOutput.u;
    mpcInput.x0 = mpcOutput.x(2, :);
end;