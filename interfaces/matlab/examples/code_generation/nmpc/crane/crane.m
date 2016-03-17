clc;
clear all;
close all;

Ts = 0.1;
EXPORT = 1;

DifferentialState xC vC xL vL uC uL theta omega;
Control duC duL;

n_XD = length(diffStates);
n_U = length(controls);

tau1 = 0.012790605943772;   a1   = 0.047418203070092;
tau2 = 0.024695192379264;   a2   = 0.034087337273386;
g = 9.81;
m = 1318.0;
c = 0;

%% Differential Equation
aC = is(-1.0/tau1*vC + a1/tau1*uC);
aL = is(-1.0/tau2*vL + a2/tau2*uL);

M1 = eye(6);
A1 = zeros(6,6);
B1 = zeros(6,2);
A1(1,2) = 1;
A1(2,2) = -1.0/tau1;
A1(2,5) = a1/tau1;
A1(3,4) = 1.0;
A1(4,4) = -1.0/tau2;
A1(4,6) = a2/tau2;

B1(5,1) = 1;
B1(6,2) = 1;

f = dot([theta; omega]) == [ omega; ...
    1.0/xL*(-g*sin(theta) - aC*cos(theta) - 2*vL*omega - c*omega/(m*xL)) ];

h = [diffStates; controls];
hN = [diffStates];

%% SIMexport
acadoSet('problemname', 'sim');

numSteps = 5;
sim = acado.SIMexport( Ts );
sim.setLinearInput(M1,A1,B1);
sim.setModel(f);
sim.set( 'INTEGRATOR_TYPE',             'INT_IRK_RIIA5' );
sim.set( 'NUM_INTEGRATOR_STEPS',        numSteps        );

if EXPORT
    sim.exportCode( 'export_SIM' );
    
    cd export_SIM
    make_acado_integrator('../integrate_crane')
    cd ..
end

%% MPCexport
acadoSet('problemname', 'mpc');

N = 10;
ocp = acado.OCP( 0.0, N*Ts, N );

W_mat = eye(n_XD+n_U,n_XD+n_U);
WN_mat = eye(n_XD,n_XD);
W = acado.BMatrix(W_mat);
WN = acado.BMatrix(WN_mat);

ocp.minimizeLSQ( W, h );
ocp.minimizeLSQEndTerm( WN, hN );

Slx = [0 0 0 0 0 0 0.01 0];
Slu = [0 0];
ocp.minimizeLSQLinearTerms( Slx, Slu );

ocp.subjectTo( -10.0 <= [uC;uL] <= 10.0 );
ocp.subjectTo( -100.0 <= [duC;duL] <= 100.0 );
ocp.subjectTo( -0.3 <= vC <= 0.3 );
ocp.setLinearInput(M1,A1,B1);
ocp.setModel(f);

mpc = acado.OCPexport( ocp );
mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
mpc.set( 'SPARSE_QP_SOLUTION',          'FULL_CONDENSING_N2');
mpc.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL4'       );
mpc.set( 'NUM_INTEGRATOR_STEPS',        2*N                 );
mpc.set( 'QP_SOLVER',                   'QP_QPOASES'    	);
mpc.set( 'HOTSTART_QP',                 'NO'             	);
mpc.set( 'LEVENBERG_MARQUARDT', 		 1e-10				);
% mpc.set( 'GENERATE_SIMULINK_INTERFACE', 'YES'               );

if EXPORT
    mpc.exportCode( 'export_MPC' );
    copyfile('../../../../../../external_packages/qpoases', 'export_MPC/qpoases', 'f')
    
    cd export_MPC
    make_acado_solver('../acado_MPCstep')
    cd ..
end

%% PARAMETERS SIMULATION
X0 = [0.0 0 0.8 0 0 0 0 0];
Xref = [0.5 0 0.4 0 0 0 0 0];
input.x = repmat(Xref,N+1,1);
Xref = repmat(Xref,N,1);
input.od = [];

Uref = zeros(N,n_U);
input.u = Uref;

input.y = [Xref(1:N,:) Uref];
input.yN = Xref(N,:);

input.W = diag([1 2e-2 1 2e-2 4e-4 3e-4 1e-3 1e-1 4e-5 3e-5]);
input.WN = diag([1 2e-2 1 2e-2 4e-4 3e-4 1e-3 1e-1]);

%% SIMULATION LOOP
display('------------------------------------------------------------------')
display('               Simulation Loop'                                    )
display('------------------------------------------------------------------')

iter = 0; time = 0;
Tf = 5;
KKT_MPC = []; INFO_MPC = [];
controls_MPC = [];
state_sim = X0;

visualize;
while time(end) < Tf
    tic
    % Solve NMPC OCP
    input.x0 = state_sim(end,:);
    output = acado_MPCstep(input);
    
    % Save the MPC step
    INFO_MPC = [INFO_MPC; output.info];
    KKT_MPC = [KKT_MPC; output.info.kktValue];
    controls_MPC = [controls_MPC; output.u(1,:)];
    input.x = output.x;
    input.u = output.u;
    
    % Simulate system
    sim_input.x = state_sim(end,:).';
    sim_input.u = output.u(1,:).';
    states = integrate_crane(sim_input);
    state_sim = [state_sim; states.value'];
    
    iter = iter+1;
    nextTime = iter*Ts; 
    disp(['current time: ' num2str(nextTime) '   ' char(9) ' (RTI step: ' num2str(output.info.cpuTime*1e6) ' µs)'])
    time = [time nextTime];
    
    visualize; pause(abs(Ts-toc));
end


