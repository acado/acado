figure;

subplot(2,3,1)
plot(out.STATES(:,1), out.STATES(:,2), 'r')
title('Body Position [m]');

subplot(2,3,2)
plot(out.STATES(:,1), out.STATES(:,3), 'r')
title('Wheel Position [m]');

subplot(2,3,3)
plot(out.STATES(:,1), out.STATES(:,4), 'r')
title('Body Velocity [m/s]');

subplot(2,3,4)
plot(out.STATES(:,1), out.STATES(:,5), 'r')
title('Wheel Velocity [m/s]');

subplot(2,3,5)
plot(out.CONTROLS(:,1), out.CONTROLS(:,2), 'r')
title('Damping Force [N]');

subplot(2,3,6)
plot(out.DISTURBANCES(:,1), out.DISTURBANCES(:,2), 'r')
title('Road Excitation [m]');