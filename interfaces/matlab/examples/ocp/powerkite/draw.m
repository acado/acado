figure;

subplot(2,3,1)
plot(out.STATES(:,1), out.STATES(:,2), 'r')
title('CABLE LENGTH  r [m]');

subplot(2,3,2)
plot(out.STATES(:,1), out.STATES(:,3), 'r')
title('POSITION ANGLE  phi [rad]');

subplot(2,3,3)
plot(out.STATES(:,1), out.STATES(:,4), 'r')
title('POSITION ANGLE theta [rad]');

subplot(2,3,4)
plot(out.STATES(:,1), out.STATES(:,10), 'r')
title('LIFT COEFFICIENT  CL');

subplot(2,3,5)
plot(out.STATES(:,1), out.STATES(:,11), 'r')
title('ENERGY  W [MJ]');

subplot(2,3,6)
plot(out.STATES(:,4), out.STATES(:,3), 'r')
title('Kite Orbit');
xlabel('theta [rad]');
ylabel('phi [rad]');
