figure;
hold on;

plot(out.STATES(:,1), out.STATES(:,2), 'r')
plot(out.STATES(:,1), out.STATES(:,3), 'b')
hold off;

legend('x', 'alpha');
