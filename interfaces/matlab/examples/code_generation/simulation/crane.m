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
integrationGrid = [0:h/numSteps:h];

%% SIMexport
sim = acado.SIMexport( h );
sim.setModel(f);
sim.setIntegrationGrid( integrationGrid );
sim.addOutput([xT; vT; xL; vL]);
sim.setMeasurements(3);
sim.set( 'INTEGRATOR_TYPE',             'INT_IRK_RIIA5' );
sim.set( 'GENERATE_MATLAB_INTERFACE',   1               );
sim.set( 'MEASUREMENT_GRID',       'ONLINE_GRID'        );

sim.exportCode('crane_export')

%% accuracy states wrt ode45:
grid = [1/3 2/3 3/3];

x = [0.5; 0.1; 0.7; -0.1; 0.5; -0.07; 0.2; -0.3];
u = zeros(2,1);
xs = x; N = 5;
for i = 1:N
    [states out] = integrate(xs(:,end),u,grid);
    xs(:,end+1) = states.value;
end

options = odeset('RelTol',1e-12,'AbsTol',1e-12);
% use of an anonymous function in matlab: http://stackoverflow.com/questions/2256229/matlab-how-do-i-pass-a-parameter-to-a-function
[tout exact] = ode45(@(t, y) rhs(t, y, u),[0:h:N*h],x,options);
exact = exact';
format long e
mean_error = mean(abs(xs(:,2:end)-exact(:,2:end))./abs(exact(:,2:end)))

%% some timing results:
Nt = 50000;
tic
for i = 1:Nt
    [statesOr outOr] = integrate(x,u,grid);
end
time = toc/Nt;
disp(['average time per integration: ' num2str(round(time*10^6)) ' μs'])

