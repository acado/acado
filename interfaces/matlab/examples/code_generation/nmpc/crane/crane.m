clc;
clear all;
close all;

Ts = 0.05;
EXPORT = 1;

DifferentialState xC vC xL vL theta omega uC uL;
Control duC duL;

n_XD = length(diffStates);
n_U = length(controls);

tau1 = 0.012790605943772;   a1   = 0.047418203070092;
tau2 = 0.024695192379264;   a2   = 0.034087337273386;
g = 9.81;
m = 1318.0;

%% Differential Equation
aC = is(-1.0/tau1*vC + a1/tau1*uC);
aL = is(-1.0/tau2*vL + a2/tau2*uL);

f = [ vC; ...
    aC; ...
    vL; ...
    aL; ...
    omega; ...
    1.0/xL*(-g*sin(theta) - aC*cos(theta) - 2*vL*omega); ...
    duC; ...
    duL ];

%% SIMexport
acadoSet('problemname', 'sim');

numSteps = 5;
sim = acado.SIMexport( Ts );
sim.setModel(f);
sim.set( 'INTEGRATOR_TYPE',             'INT_IRK_RIIA5' );
sim.set( 'NUM_INTEGRATOR_STEPS',        numSteps        );
% sim.set( 'OPERATING_SYSTEM', 'OS_WINDOWS'               );

if EXPORT
    sim.exportCode('export_SIM');
end

%% MPCexport
acadoSet('problemname', 'mpc');

N = 20;
ocp = acado.OCP( 0.0, N*Ts, N );
ExportVariable QQ(n_XD,n_XD) RR(n_U,n_U) QT(n_XD,n_XD)
ocp.minimizeLSQ( QQ,RR );
ocp.minimizeLSQEndTerm( QT );

ocp.subjectTo( -10.0 <= [uC;uL] <= 10.0 );
ocp.subjectTo( -100.0 <= [duC;duL] <= 100.0 );
ocp.setModel( f );

mpc = acado.MPCexport( ocp );
mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
mpc.set( 'SPARSE_QP_SOLUTION',          'FULL_CONDENSING'   );
mpc.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL4'       );
mpc.set( 'NUM_INTEGRATOR_STEPS',        N                   );
mpc.set( 'MEX_VERBOSE',                 1                   );
% mpc.set( 'OPERATING_SYSTEM', 'OS_WINDOWS'                   );

if EXPORT
    mpc.exportCode( 'export_MPC' );
end

%% PARAMETERS SIMULATION
X0 = [0.0 0 0.8 0 0 0 0 0];
Xref = [0.5 0 0.4 0 0 0 0 0];
X = repmat(Xref,N+1,1);
Xref = repmat(Xref,N,1);

Uref = zeros(N,n_U);
U = Uref;

Q = diag([6e-1, 1.5e-1, 5e-2, 1e-3, 3e-3, 1e-1, 1e-6, 1e-6]); S = Q;
R = diag([1e-6, 1e-6]);

%% SIMULATION LOOP
display('------------------------------------------------------------------')
display('               Simulation Loop'                                    )
display('------------------------------------------------------------------')

time = 0;
Tf = 10;
KKT_MPC = []; INFO_MPC = [];
controls_MPC = [];
state_sim = X0;

visualize; pause
while time(end) < Tf
    tic
    % Solve NMPC OCP
    X0 = state_sim(end,:);
    [U_mpc,X_mpc,info_mpc] = MPCstep(X0,X,U,Xref,Uref,Q,R,S);
    
    % Save the MPC step
    INFO_MPC = [INFO_MPC; info_mpc];
    KKT_MPC = [KKT_MPC; info_mpc.KKTvalue];
    controls_MPC = [controls_MPC; U_mpc(1,:)];
    X = X_mpc;
    U = U_mpc;
    
    % Simulate system
    states = integrate(state_sim(end,:), U_mpc(1,:));
    state_sim = [state_sim; states.value'];
    
    nextTime = time(end)+Ts; disp(['current time: ' num2str(nextTime)])
    time = [time nextTime];
    
    visualize; pause(abs(Ts-toc));
end


