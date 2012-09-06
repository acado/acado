figure;

subplot(2,3,1)
plot(out.STATES(:,1), out.STATES(:,2), 'r')
title('Height of buoy (h)');

subplot(2,3,2)
plot(out.STATES(:,1), out.STATES(:,3), 'r')
title('Velocity of buoy (v)');

subplot(2,3,3)
plot(out.STATES(:,1), out.STATES(:,4), 'r')
title('Objective function (produced wave energy, w)');

subplot(2,3,4)
plot(out.CONTROLS(:,1), out.CONTROLS(:,2), 'r')
title('Resistance (u)');

subplot(2,3,5)
plot(out.STATES(:,1), 10 + 1.*sin(2.*pi.*out.STATES(:,1)./5), 'r')
title('Wave height (hw)');


