figure;

subplot(2,2,1)
plot(out.STATES(:,1), out.STATES(:,2), 'r')
title('DIFFERENTIAL STATE  x');

subplot(2,2,2)
plot(out.PARAMETERS(:,1), out.PARAMETERS(:,2), 'r')
title('PARAMETER p');

subplot(2,2,3)
plot(out.CONTROLS(:,1), out.CONTROLS(:,2), 'r')
title('CONTROL u');

subplot(2,2,4)
plot(out.ALGEBRAICSTATES(:,1), out.ALGEBRAICSTATES(:,2), 'r')
title('ALGEBRAIC STATE  z');
