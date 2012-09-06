
figure;

subplot(2,2,1)
plot(out.STATES(:,1), out.STATES(:,2), 'r')
title('v');

subplot(2,2,2)
plot(out.STATES(:,1), out.STATES(:,3), 'r')
title('s');

subplot(2,2,3)
plot(out.STATES(:,1), out.STATES(:,4), 'r')
title('m');

subplot(2,2,4)
stairs(out.STATES(:,1), out.CONTROLS(:,2), 'r')
title('u');


figure;
plot(out.STATES(:,1), 0.5.*out.STATES(:,4).*out.STATES(:,2).*out.STATES(:,2), 'r')
title('Kinetic Engery');