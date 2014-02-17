clear all;
close all;
clc;

RECOMPILE = 1;

% generate and compile the ACADO NMPC controller
if RECOMPILE
    generate_code
end

Ts = 0.001;
Tf = 10;

numSteps = 0;
S = diag([ 2e-5; 1e-5; 1e-1; 5; 5; 5e-7; 2e-7 ]);
SN = 2*S(1:5,1:5);

% compute the reference trajectory for the controller
s = 0.2;
time_ref = [ 0:Ts:Tf ].';
N = length(time_ref);
x = 0.3*sin(time_ref/s);
y = 0.8*sin(time_ref/(2*s));
v = ((9*cos(time_ref/s).^2)/(100*s.^2) + (4*cos(time_ref/(2*s)).^2)/(25*s.^2)).^(1/2);

theta = atan((4*cos(time_ref./(2*s)))./(3*cos(time_ref./s)));
for i = 1:length(theta)-1
    while(abs(theta(i)-theta(i+1)) > pi/2)
       if(theta(i+1) > theta(i))
           theta(i+1) = theta(i+1) - pi;
       else
           theta(i+1) = theta(i+1) + pi;
       end
    end
end
omega = -((2*sin(time_ref./(2*s)))./(3*s*cos(time_ref./s)) - (4*cos(time_ref./(2*s)).*sin(time_ref./s))./(3*s*cos(time_ref./s).^2))./((16*cos(time_ref./(2*s)).^2)./(9*cos(time_ref./s).^2) + 1);
ref = [ v omega theta x y zeros(N,2) ];
    
xInit = ref(1:5);
uInit = ref(6:7);

lbV = -30*ones(2,1);      ubV = 30*ones(2,1);
lbAV = [-2.5; -6];         ubAV = [2.5; 6];

% closed loop simulation with the ACADO generated NMPC controller
open('NMPC_model');
sim('NMPC_model');

statesV = states_sim.signals.values;
controlsV = controls_sim.signals.values;

time = 0:Ts:Tf;
figure(1);
subplot(3,6,1:2);
stairs(time_ref, ref(:,4), '--r');
hold on;
plot(time, statesV(:,4), 'b');
xlabel('t')
ylabel('x')
title('x position')

subplot(3,6,3:4);
stairs(time_ref, ref(:,5), '--r');
hold on;
plot(time, statesV(:,5), 'b');
xlabel('t')
ylabel('y')
title('y position')

subplot(3,6,5:6);
stairs(time_ref, ref(:,3), '--r');
hold on;
plot(time, statesV(:,3), 'b');
xlabel('t')
ylabel('theta')
title('angle')

subplot(3,6,7:9);
stairs(time_ref, ref(:,1), '--r');
hold on;
stairs(time, statesV(:,1), 'b');
xlabel('t')
ylabel('v')
title('linear velocity')

subplot(3,6,10:12);
stairs(time_ref, ref(:,2), '--r');
hold on;
stairs(time, statesV(:,2), 'b');
xlabel('t')
ylabel('omega')
title('angular velocity')

subplot(3,6,13:15);
stairs(time_ref, ref(:,6), '--r');
hold on;
stairs(time, controlsV(:,1), 'g');
xlabel('t')
ylabel('a')
title('control input 1')

subplot(3,6,16:18);
stairs(time_ref, ref(:,7), '--r');
hold on;
stairs(time, controlsV(:,2), 'g');
xlabel('t')
ylabel('alpha')
title('control input 2')

xs = statesV.';
figure(2);
whitebg([1.0 1.0 1.0])
set(gcf,'Color',[1 1 1])
pause
step = 40; cur_time = 0;
ref_iter = 1; cur_ref = ref(1,:);

for i = 1:step:size(xs,2)
   clf
   tic
   plot(xs(4,i), xs(5,i), 'bo', 'MarkerSize', 45, 'Linewidth', 5); hold on
   plot(xs(4,i), xs(5,i), 'kx', 'MarkerSize', 15, 'Linewidth', 2);
   theta = xs(3,i);
   plot([xs(4,i) xs(4,i)+0.08*cos(theta)], [xs(5,i) xs(5,i)+0.08*sin(theta)], 'r', 'Linewidth', 2.5);
   
   text(0.6,0.9,['current time: ' num2str(cur_time) 's'],'FontSize',15);
   
   plot(ref(:,4), ref(:,5), '--g', 'Linewidth', 0.2);
   plot(ref(i,4), ref(i,5), 'ko', 'MarkerSize', 4, 'Linewidth', 10);
   
   xlabel('x'); ylabel('y');
   xlim([-1 1]); ylim([-1 1]);
   grid on;
   drawnow
   cur_time = cur_time + step*Ts;
   
   timing = toc;
   pause(step*Ts-timing);
end


