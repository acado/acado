clc;
clear all;
close all;

EXPORT = 1;

if EXPORT
    cd mpc_export
    make_acado_solver('../acado_MPCstep')
    cd ..
    cd sim_export
    make_acado_integrator('../acado_simulate')
    cd ..
end

%% PARAMETERS SIMULATION
N = 18;
Ts = 18/N;
nx = 10;
nu = 3;

phi1 = -0.34;   phi2 = 0.34; 
theta1 = 0.85;  theta2 = 1.45;
dr1 = -40;      dr2 = 10;
n1 = -0.7;      n2 = 0.9;
Psi1 = -0.29;   Psi2 = 0.29;
CL1 = 0.1;      CL2 = 1.5;

ddr01 = -25;    ddr02 = 25;
dPsi1 = -0.065; dPsi2 = 0.065;
dCL1 = -3.5;    dCL2 = 3.5;

init_states = textread('powerkite_states.txt', '');
init_controls = textread('powerkite_controls.txt', '');
initX = []; initU = [];
for i = 1:N
   initX = [initX; init_states(1+2*(i-1),:)];
   initU = [initU; init_controls(1+2*(i-1),:)];
end
initX = [initX; init_states(end,:)];

init_states = initX;
init_controls = initU;

% X0 = init_states(1,1:nx);
X0 = [1230; -0.07; 1.35; 4.5; -0.06; -0.03; 0; 0.06; 1.5; 0].';
Xref = zeros(1,nx);
input.x = init_states(:,1:nx);
Xref = repmat(Xref,N,1);

input.p = [];
input.mu = ones(N,length(X0));

Uref = zeros(N,nu);
input.u = init_controls(:,1:nu);

input.y = [Xref(1:N,:) Uref];
input.yN = Xref(N,:).';

Q = 0*eye(nx); S = Q;
R = 0*eye(nu);
W = blkdiag(Q,R);

input.W = repmat(W,N,1); 
input.WN = Q;

input.lbValues = repmat([ddr01;dPsi1;dCL1], N, 1);
input.ubValues = repmat([ddr02;dPsi2;dCL2], N, 1);

stateLB = repmat([phi1;theta1;dr1;n1;Psi1;CL1], N-1, 1);
stateUB = repmat([phi2;theta2;dr2;n2;Psi2;CL2], N-1, 1);
input.lbAValues = [stateLB; zeros(nx,1)];
input.ubAValues = [stateUB; zeros(nx,1)];


%% SIMULATION LOOP
display('------------------------------------------------------------------')
display('               Simulation Loop'                                    )
display('------------------------------------------------------------------')

Tf = 360;

time = 0;
KKT_MPC = []; INFO_MPC = [];
controls_MPC = [];
state_sim = X0;
state_sim2 = X0;

totalCPU = 0;
iter = 0;

pause
while time(end) < Tf
    tic
    % Solve NMPC OCP
    input.x0 = [state_sim(end,1:6).'; 0; state_sim(end,8:9).'; 0];
    input.lbAValues = [stateLB; state_sim(end,1:6).'; n1; state_sim(end,8:9).'; -1e8];
    input.ubAValues = [stateUB; state_sim(end,1:6).'; n2; state_sim(end,8:9).'; 1e8];
    
    output = acado_MPCstep(input);
    
    input.x = output.x;
    input.u = output.u;
    input.mu = output.mu;
    
    % Save the MPC step
    INFO_MPC = [INFO_MPC; output.info];
    KKT_MPC = [KKT_MPC; output.info.kktValue];
    controls_MPC = [controls_MPC; output.u(1,:)];
    
    % SHIFTING !!
    input.x = [input.x(2:end,:); input.x(2,:)];
    input.u = [input.u(2:end,:); input.u(1,:)];
    input.mu = [input.mu(2:end,:); input.mu(2,:)];
    
    % Simulate system
    for i = 1:5
        input_sim.x = state_sim2(end,:).';
        input_sim.u = output.u(1,:).';
        states = acado_simulate(input_sim);
        state_sim2 = [state_sim2; states.value'];
    end
    
    state_sim = [state_sim; states.value'];
    
    nextTime = iter*Ts; 
    disp(['current time      : ' num2str(nextTime)])
    disp(['RTI step KKT value: ' num2str(output.info.kktValue)]);
    disp(['RTI step CPU time : ' num2str(round(1e6*output.info.cpuTime)) ' μs']);
    disp('----------------------------------------------');
    time = [time nextTime];
    iter = iter+1;
    
    totalCPU = totalCPU + output.info.cpuTime;
end

disp('----------------------------------------------');
disp('----------------------------------------------');
disp(['AVERAGE RTI step CPU time    : ' num2str(round(1e6*totalCPU/iter)) ' μs']);
disp('----------------------------------------------');


%% Plot

Fontsize = 18;
set(0,'DefaultAxesFontSize',Fontsize)

X = state_sim.';
X2 = state_sim2.';
U = controls_MPC.';

TIME = time;

figure(2); clf;
plot(X2(2,:),X2(3,:),'--k', 'linewidth',1)
hold on
plot(X(2,:),X(3,:),'xk', 'linewidth',1.6, 'MarkerSize', 8)
plot(X0(2),X0(3),'ok', 'MarkerFaceColor', 'k', 'MarkerSize', 11)
xlabel('phi')
ylabel('theta')
    

