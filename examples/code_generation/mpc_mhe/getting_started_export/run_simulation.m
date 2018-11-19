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
make_acado_solver_sfunction;
cd ..

addpath('getting_started_export')
sim('getting_started_test_sfunction')
open getting_started_test_sfunction