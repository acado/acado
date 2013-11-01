clc;
clear all;
close all;

h = 0.1;
N = 10;

acadoSet('problemname', 'getting_started');

DifferentialState a b
Control u
Parameter mu

x = [a; b];
f = [   x(2) + u*(mu+(1-mu)*x(1)); ...
        x(1) + u*(mu-4*(1-mu)*x(2))     ];

%% SIMexport
sim = acado.SIMexport( h );

sim.setModel(f);

sim.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL4'   );
sim.set( 'NUM_INTEGRATOR_STEPS',        10              );
sim.set( 'GENERATE_MATLAB_INTERFACE',   1               );

sim.exportCode('getting_started_export');

cd getting_started_export
make_acado_integrator('../integrate_getting_started')
make_acado_model('../rhs_getting_started')
cd ..

%% simulation (test results):
mu = 0.5;
x = [-0.683; -0.864]; xs = x;
u = 2;
for i = 1:N
    states = integrate_getting_started(xs(:,end),u,mu);
    xs(:,i+1) = states.value;
end

options = odeset('RelTol',1e-12,'AbsTol',1e-12);
[tout exact] = ode45(@(t, y) rhs_getting_started(t, y, u, mu),[0:h:N*h],x,options);
exact = exact';

format long e
mean_error = mean(mean(abs(xs(:,2:end)-exact(:,2:end))./abs(exact(:,2:end))))

figure(1);
subplot(2,1,1);
plot(h*[0:N], xs(1,:), '--b');
ylabel('x1')
xlabel('t')
title('State 1')

subplot(2,1,2);
plot(h*[0:N], xs(2,:), '--b');
ylabel('x2')
xlabel('t')
title('State 2')

%% some timing results:
Nt = 500000;
tic
for i = 1:Nt
    statesOr = integrate_getting_started(x,u,mu);
end
time = toc/Nt;
disp(['average time per integration: ' num2str(round(time*10^6)) ' μs'])

