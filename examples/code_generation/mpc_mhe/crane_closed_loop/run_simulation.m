close all;
clear all;
clc;

%% Make S-functions
%  NOTE: You can safely close, without saving, the two "Untitled" windows
%        which are going to show up after compilation
!./crane_cl_mhe
cd crane_cl_mhe_export/
make_mhe_solver_sfunction
cd ..

!./crane_cl_nmpc
cd crane_cl_nmpc_export/
make_nmpc_solver_sfunction
cd ..

%% Open the Simulink model
addpath('crane_cl_mhe_export/', 'crane_cl_nmpc_export/')
open closed_loop_simulation
