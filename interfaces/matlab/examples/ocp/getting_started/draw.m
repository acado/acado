figure;

subplot(2,2,1)
plot(out.STATES(:,1), out.STATES(:,2), 'r')
% The first column contains the time points
% The second column contains the state 'x'
title('x');

subplot(2,2,2)
plot(out.STATES(:,1), out.CONTROLS(:,2), 'r')
title('u');

subplot(2,2,3)
plot(out.STATES(:,1), out.PARAMETERS(:,2), 'r')
title('p');

subplot(2,2,4)
plot(out.STATES(:,1), out.DISTURBANCES(:,2), 'r')
title('w');
