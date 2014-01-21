clc;
clear all;
close all;

% Define variables, functions and constants:
DifferentialState dT(4) T(4) W(4) q(4) Omega(3) V(3) P(3) IP(3)
Control U(4)

EXPORT = 1;

h = 0.1;
rho = 1.23;
A = 0.1;
Cl = 0.25;
Cd = 0.3*Cl;
m = 1;
g = 9.81;

L  = 0.5;
Jp = 1e-2;
xi = 1e-2;

J1 = 0.025;J2 = 0.025;J3 = 0.1;

gain = 1e-4;
alpha = 1;

%% Define the quadcopter ODE model in fully nonlinear form:
f(1) = U1*gain; 
f(2) = U2*gain; 
f(3) = U3*gain; 
f(4) = U4*gain; 
f(5) = dT1; 
f(6) = dT2; 
f(7) = dT3; 
f(8) = dT4; 
f(9) = (T1 - W1*xi)/Jp; 
f(10) = (T2 - W2*xi)/Jp; 
f(11) = (T3 - W3*xi)/Jp; 
f(12) = (T4 - W4*xi)/Jp; 
f(13) = - (Omega1*q2)/2 - (Omega2*q3)/2 - (Omega3*q4)/2 - (alpha*q1*(q1*q1 + q2*q2 + q3*q3 + q4*q4 - 1))/(q1*q1 + q2*q2 + q3*q3 + q4*q4);
f(14) = (Omega1*q1)/2 - (Omega3*q3)/2 + (Omega2*q4)/2 - (alpha*q2*(q1*q1 + q2*q2 + q3*q3 + q4*q4 - 1))/(q1*q1 + q2*q2 + q3*q3 + q4*q4);
f(15) = (Omega2*q1)/2 + (Omega3*q2)/2 - (Omega1*q4)/2 - (alpha*q3*(q1*q1 + q2*q2 + q3*q3 + q4*q4 - 1))/(q1*q1 + q2*q2 + q3*q3 + q4*q4);
f(16) = (Omega3*q1)/2 - (Omega2*q2)/2 + (Omega1*q3)/2 - (alpha*q4*(q1*q1 + q2*q2 + q3*q3 + q4*q4 - 1))/(q1*q1 + q2*q2 + q3*q3 + q4*q4);
f(17) = (J3*Omega2*Omega3 - J2*Omega2*Omega3 + (A*Cl*L*rho*(W2*W2 - W4*W4))/2)/J1; 
f(18) = -(J3*Omega1*Omega3 - J1*Omega1*Omega3 + (A*Cl*L*rho*(W1*W1 - W3*W3))/2)/J2; 
f(19) = (J2*Omega1*Omega2 - J1*Omega1*Omega2 + (A*Cd*rho*(W1*W1 - W2*W2 + W3*W3 - W4*W4))/2)/J3; 
f(20) = (A*Cl*rho*(2*q1*q3 + 2*q2*q4)*(W1*W1 + W2*W2 + W3*W3 + W4*W4))/(2*m); 
f(21) = -(A*Cl*rho*(2*q1*q2 - 2*q3*q4)*(W1*W1 + W2*W2 + W3*W3 + W4*W4))/(2*m); 
f(22) = (A*Cl*rho*(W1*W1 + W2*W2 + W3*W3 + W4*W4)*(q1*q1 - q2*q2 - q3*q3 + q4*q4))/(2*m) - g; 
f(23) = V1; 
f(24) = V2; 
f(25) = V3; 
f(26) = P1; 
f(27) = P2; 
f(28) = P3; 

sim = acado.SIMexport( h );

sim.setModel(f);

sim.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL4'   );
sim.set( 'NUM_INTEGRATOR_STEPS',        5               );
if EXPORT
    sim.exportCode( 'quadcopter_export' );
    
    cd quadcopter_export
    make_acado_integrator('../integrate_quadcopter')
    cd ..
end

%% Define the quadcopter ODE model in 3-stage format:

% LINEAR INPUT SYSTEM (STAGE 1):
M1 = eye(12);
A1 = zeros(12,12);
B1 = zeros(12,4);
	
A1(5,1) = 1.0;
A1(6,2) = 1.0;
A1(7,3) = 1.0;
A1(8,4) = 1.0;
A1(9,5) = 1.0/Jp;	A1(9,9) = -xi/Jp;
A1(10,6) = 1.0/Jp;	A1(10,10) = -xi/Jp;
A1(11,7) = 1.0/Jp;	A1(11,11) = -xi/Jp;
A1(12,8) = 1.0/Jp;	A1(12,12) = -xi/Jp;

B1(1,1) = gain;
B1(2,2) = gain;
B1(3,3) = gain;
B1(4,4) = gain;
	
% NONLINEAR SYSTEM (STAGE 2):
f2(1) = - (Omega1*q2)/2 - (Omega2*q3)/2 - (Omega3*q4)/2 - (alpha*q1*(q1*q1 + q2*q2 + q3*q3 + q4*q4 - 1))/(q1*q1 + q2*q2 + q3*q3 + q4*q4);
f2(2) = (Omega1*q1)/2 - (Omega3*q3)/2 + (Omega2*q4)/2 - (alpha*q2*(q1*q1 + q2*q2 + q3*q3 + q4*q4 - 1))/(q1*q1 + q2*q2 + q3*q3 + q4*q4);
f2(3) = (Omega2*q1)/2 + (Omega3*q2)/2 - (Omega1*q4)/2 - (alpha*q3*(q1*q1 + q2*q2 + q3*q3 + q4*q4 - 1))/(q1*q1 + q2*q2 + q3*q3 + q4*q4);
f2(4) = (Omega3*q1)/2 - (Omega2*q2)/2 + (Omega1*q3)/2 - (alpha*q4*(q1*q1 + q2*q2 + q3*q3 + q4*q4 - 1))/(q1*q1 + q2*q2 + q3*q3 + q4*q4);
f2(5) = (J3*Omega2*Omega3 - J2*Omega2*Omega3 + (A*Cl*L*rho*(W2*W2 - W4*W4))/2)/J1;
f2(6) = -(J3*Omega1*Omega3 - J1*Omega1*Omega3 + (A*Cl*L*rho*(W1*W1 - W3*W3))/2)/J2;
f2(7) = (J2*Omega1*Omega2 - J1*Omega1*Omega2 + (A*Cd*rho*(W1*W1 - W2*W2 + W3*W3 - W4*W4))/2)/J3;
f2(8) = (A*Cl*rho*(2*q1*q3 + 2*q2*q4)*(W1*W1 + W2*W2 + W3*W3 + W4*W4))/(2*m);
f2(9) = -(A*Cl*rho*(2*q1*q2 - 2*q3*q4)*(W1*W1 + W2*W2 + W3*W3 + W4*W4))/(2*m);
f2(10) = (A*Cl*rho*(W1*W1 + W2*W2 + W3*W3 + W4*W4)*(q1*q1 - q2*q2 - q3*q3 + q4*q4))/(2*m) - g;
	
%LINEAR OUTPUT SYSTEM (STAGE 3):
M3 = eye(6);
A3 = zeros(6,6);

A3(4,1) = 1.0;
A3(5,2) = 1.0;
A3(6,3) = 1.0;

f3 = [V1; V2; V3; zeros(3,1)];
    
sim2 = acado.SIMexport( h );

sim2.setLinearInput(M1,A1,B1);
sim2.setModel(f2);
sim2.setLinearOutput(M3,A3,f3);

sim2.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL4'   );
sim2.set( 'NUM_INTEGRATOR_STEPS',        5               );
if EXPORT
    sim2.exportCode( 'quadcopter_export' );
    
    cd quadcopter_export
    make_acado_integrator('../integrate_quadcopter2')
    cd ..
end


%% Timing results:
load controller_quadcopter.mat P X0
T = 500; U = -P.K*(X0 - P.Xref);
input.u = U;
input.x = X0;
tic
for i = 1:T
    states = integrate_quadcopter(input);
end
time = toc/T;
tic
for i = 1:T
    states2 = integrate_quadcopter2(input);
end
time2 = toc/T;

states = integrate_quadcopter(input);

states2 = integrate_quadcopter2(input);

err_int = max(abs(states.value-states2.value)) + max(max(abs(states.sensX-states2.sensX))) + ...
    max(max(abs(states.sensU-states2.sensU)));

if( err_int > 1e-6 )
    error('Unusual mismatch between nonlinear and 3-stage model !')
else
    disp('Small, numerically justifiable mismatch between nonlinear and 3-stage model !')
end
disp('------------------------------------------');
disp('Timing results:');
disp(['average time per integration: ' num2str(round(time*10^6)) ' μs  --> ' num2str(round(time2*10^6)) ' μs'])


%% Simulation results:

N = 100;
xs = X0'; controls = [];
for i = 1:N
    state = xs(end,:)';
    U = -P.K*(state - P.Xref);
    
    input.x = state;
    input.u = U;
    states = integrate_quadcopter(input);
    
    xs = [xs; states.value'];
    controls = [controls; U'];
end

set(0,'DefaultAxesFontSize',20)

time = h*[0:N];
states = xs;
figure(1);
for k = 1:length(time)
    
    figure(1);
    clf
    x      = states(k,23);
    y      = states(k,24);
    z      = states(k,25);
    
    q1     = states(k,13);
    q2     = states(k,14);
    q3     = states(k,15);
    q4     = states(k,16);

    R(1,1) =q1*q1 + q2*q2 - q3*q3 - q4*q4; 
    R(1,2) =2*q2*q3 - 2*q1*q4; 
    R(1,3) =2*q1*q3 + 2*q2*q4; 
    R(2,1) =2*q1*q4 + 2*q2*q3; 
    R(2,2) =q1*q1 - q2*q2 + q3*q3 - q4*q4; 
    R(2,3) =2*q3*q4 - 2*q1*q2; 
    R(3,1) =2*q2*q4 - 2*q1*q3; 
    R(3,2) =2*q1*q2 + 2*q3*q4; 
    R(3,3) =q1*q1 - q2*q2 - q3*q3 + q4*q4; 
    
    widths = [1.8 1.8 1];
    lengths = 0.3.*[1 1 0.3];
    for vec = 1:3
        L2 = lengths(vec)*L;
        line([x-L2*R(vec,1) x+L2*R(vec,1)],[y-L2*R(vec,2) y+L2*R(vec,2)],[z-L2*R(vec,3) z+L2*R(vec,3)],'color','b','linewidth',widths(vec));hold on
    end
    plot3(x,y,z,'ks','MarkerFaceColor','k','MarkerSize',8)
    plot3(0,0,0,'r+','MarkerFaceColor','r','MarkerSize',10)
    plot3(0,0,0,'ro','MarkerSize',12)
    view(40,20)
    plot3(states(:,23),states(:,24),states(:,25),':k','linewidth',2)
    xlabel('x'); ylabel('y'); zlabel('z');
    
    text(0.5,1.4,1.2,['current time: ' num2str(time(k)) 's'],'FontSize',15);
    
    axis([-0.15 1.15 -0.15 1.15 -0.15 1.15])
    grid
    
    pause(0.1/2)
end



