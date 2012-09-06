figure;

subplot(2,3,1)
plot(out.STATES_SAMPLED(:,1), out.STATES_SAMPLED(:,2), 'r')
title('Body Position [m]');

subplot(2,3,2)
plot(out.STATES_SAMPLED(:,1), out.STATES_SAMPLED(:,3), 'r')
title('Wheel Position [m]');

subplot(2,3,3)
plot(out.STATES_SAMPLED(:,1), out.STATES_SAMPLED(:,4), 'r')
title('Body Velocity [m/s]');

subplot(2,3,4)
plot(out.STATES_SAMPLED(:,1), out.STATES_SAMPLED(:,5), 'r')
title('Wheel Velocity [m/s]');

subplot(2,3,5)
plot(out.CONTROLS(:,1), out.CONTROLS(:,2), 'r')
title('Damping Force [N]');

subplot(2,3,6)
plot(disturbance(:,1), disturbance(:,2), 'r')
title('Road Excitation [m]');