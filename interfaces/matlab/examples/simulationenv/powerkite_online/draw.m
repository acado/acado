figure;

subplot(2,3,1)
plot(out.STATES_SAMPLED(:,1), out.STATES_SAMPLED(:,2), 'r')
title('CABLE LENGTH  r [m]');

subplot(2,3,2)
plot(out.STATES_SAMPLED(:,1), out.STATES_SAMPLED(:,3), 'r')
title('POSITION ANGLE  phi [rad]');

subplot(2,3,3)
plot(out.STATES_SAMPLED(:,1), out.STATES_SAMPLED(:,4), 'r')
title('POSITION ANGLE theta [rad]');

subplot(2,3,4)
plot(out.STATES_SAMPLED(:,1), out.STATES_SAMPLED(:,10), 'r')
title('LIFT COEFFICIENT  CL');

subplot(2,3,5)
plot(out.STATES_SAMPLED(:,1), out.STATES_SAMPLED(:,11), 'r')
title('ENERGY  W [MJ]');

subplot(2,3,6)
plot(out.STATES_SAMPLED(:,4), out.STATES_SAMPLED(:,3), 'r')
title('Kite Orbit');
xlabel('theta [rad]');
ylabel('phi [rad]');


figure;

subplot(1,3,1)
plot(out.CONTROLS(:,1), out.CONTROLS(:,2), 'r')
title('CONTROL 1 DDR0');

subplot(1,3,2)
plot(out.CONTROLS(:,1), out.CONTROLS(:,3), 'r')
title('CONTROL 2 DPSI');

subplot(1,3,3)
plot(out.CONTROLS(:,1), out.CONTROLS(:,4), 'r')
title('CONTROL 3 DCL');
