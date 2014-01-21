clc;
clear all;
close all;

h = 0.1;
N = 5;

acadoSet('problemname', 'crane');

DifferentialState xT vT xL vL phi omega uT uL;
Control duT duL;

tau1 = 0.012790605943772;
a1   = 0.047418203070092;
tau2 = 0.024695192379264;
a2   = 0.034087337273386;
g = 9.81;
c = 0.0;
m = 1318.0;

aT = -1.0/tau1*vT + a1/tau1*uT;
aL = -1.0/tau2*vL + a2/tau2*uL;

%% Differential Equation
f = dot(diffStates) == [ vT; ...
    aT; ...
    vL; ...
    aL; ...
    omega; ...
    1.0/xL*(-g*sin(phi) - aT*cos(phi) - 2*vL*omega - c*omega/(m*xL)); ...
    duT; ...
    duL ];

numSteps = 2;

%% SIMexport
sim = acado.SIMexport( h );

sim.setModel(f);
sim.addOutput([xT; vT; xL; vL], 3);

sim.set( 'INTEGRATOR_TYPE',             'INT_IRK_RIIA3' );
sim.set( 'NUM_INTEGRATOR_STEPS',        numSteps        );
sim.set( 'GENERATE_MATLAB_INTERFACE',   1               );
sim.set( 'MEASUREMENT_GRID',       'ONLINE_GRID'        );

sim.exportCode('crane_export')

cd crane_export
make_acado_integrator('../integrate_crane')
cd ..


%% SIMexport: a more accurate integrator as a reference
sim.set( 'NUM_INTEGRATOR_STEPS',        2*numSteps       );
sim.exportCode('crane_export')

cd crane_export
make_acado_integrator('../integrate_crane2')
cd ..

%% accuracy states wrt ode45:
grid = [1/3 2/3 3/3];

x = [0.5; 0.1; 0.7; -0.1; 0.5; -0.07; 0.2; -0.3];
u = zeros(2,1);
xs = x; xs2 = x; N = 5;
input.u = u;
input.grid1 = grid;
for i = 1:N
    input.x = xs2(:,end);
    [states out] = integrate_crane(input);
    xs(:,end+1) = states.value;
    
    input.x = xs2(:,end);
    [states out] = integrate_crane2(input);
    xs2(:,end+1) = states.value;
end
mean_error = mean(abs(xs(:,2:end)-xs2(:,2:end))./abs(xs2(:,2:end)))

%% some timing results:
Nt = 50000;
tic
for i = 1:Nt
    input.x = x;
    [statesOr outOr] = integrate_crane(input);
end
time = toc/Nt;
disp(['average time per integration: ' num2str(round(time*10^6)) ' μs'])

