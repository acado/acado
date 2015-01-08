clc;
clear all;
close all;

h = 0.1;
N = 200;                            
    
acadoSet('problemname', 'pendulum');

DifferentialState x y alpha dx dy dalpha;
AlgebraicState ddalpha Fx Fy;
Control u;

m = 1;
I = 1;
g = 9.81;
L = 1;
c = 0.7;

%% Differential Equation
f = [ dx; ...
    dy; ...
    dalpha; ...
    1/m*(Fx+u); ...
    1/m*Fy-g; ...
    ddalpha; ...
    sin(alpha)*L - x; ...
    -y - cos(alpha)*L; ...
    I*ddalpha + sin(alpha)*L*Fy + cos(alpha)*L*Fx + c*dalpha];

%% SIMexport
numMeas = 6;
sim = acado.SIMexport( h );

sim.setModel(f);
sim.addOutput([Fx; Fy], numMeas);

sim.set( 'INTEGRATOR_TYPE',             'INT_IRK_RIIA5'         );
sim.set( 'NUM_INTEGRATOR_STEPS',        10                      );
sim.set( 'GENERATE_MATLAB_INTERFACE',   1                       );

sim.exportCode('pendulum_export');

cd pendulum_export
make_acado_integrator('../integrate_pendulum')
cd ..

%% test simulation with visualization:
x = [zeros(6,1); ones(3,1)];       
x(2) = 1; 
x(3) = pi;
xs = x;
u = 0.01;
input.u = u;
for i = 1:N
    input.x = xs(:,end);
    [states out] = integrate_pendulum(input);
    xs(:,end+1) = states.value;
end

figure(1);
for i = 1:N
    plot(0, 0, 'rs', 'MarkerSize', 15);
    hold on;
    plot(xs(1,i), xs(2,i), 'bo', 'MarkerSize', 12);
    plot([0; xs(1,i)], [0; xs(2,i)], ':k');
    xlim([-1.1 1.1]);
    ylim([-1.1 1.1]);
    axis square
    title('Falling pendulum')
    pause(0.05);
end

%% define the pendulum model now using an external C-function:
disp('-----------------------------------------------------------')
disp('Let us use an external pendulum model, defined in C-code...')
    
sim = acado.SIMexport( h );

sim.setModel('model', 'rhs', 'rhs_jac');
sim.setDimensions(6,0,3,1,0,0);
sim.addOutput('out', 'out_jac', 2, numMeas);

sim.set( 'INTEGRATOR_TYPE',             'INT_IRK_RIIA5'         );
sim.set( 'NUM_INTEGRATOR_STEPS',        10                      );
sim.set( 'GENERATE_MATLAB_INTERFACE',   1                       );

sim.exportCode('pendulum_export');

cd pendulum_export
make_acado_integrator('../integrate_pendulum2', 'model.c')
cd ..

input.x = xs(:,end);
[states out] = integrate_pendulum(input);
[states2 out2] = integrate_pendulum2(input);

err = max(abs(states.value-states2.value)) + max(max(abs(states.sensX-states2.sensX))) + ...
    max(max(abs(states.sensU-states2.sensU))) + max(max(abs(out.value-out2.value))) + ...
    max(max(abs(out.sensX-out2.sensX))) + max(max(abs(out.sensU-out2.sensU)));

if( err > 1e-6 )
    error('Unusual mismatch between ACADO and externally defined model !')
else
    disp('Small, numerically justifiable mismatch between ACADO and externally defined model !')
end

