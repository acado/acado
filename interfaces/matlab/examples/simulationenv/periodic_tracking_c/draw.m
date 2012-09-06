figure;

% Periodic repeated reference
refEXT = [0.0       0.00       0.00          
        0.5       0.00       0.00          
        1.0       0.00       0.00
        1.25      0.00       0.00
        1.5       0.00       0.00
        1.75      0.00       0.00
        2.0       0.00       0.00
        2.5       0.00       0.00
        3.0       0.00       -0.50
        3.5       0.00       -0.50
        4.0       0.00       0.00
        4.0       0.00       0.00          
        4.5       0.00       0.00          
        5.0       0.00       0.00
        5.25      0.00       0.00
        5.5       0.00       0.00
        5.75      0.00       0.00
        6.0       0.00       0.00
        6.5       0.00       0.00
        7.0       0.00       -0.50
        7.5       0.00       -0.50
        8.0       0.00       0.00
        8.0       0.00       0.00          
        8.5       0.00       0.00          
        9.0       0.00       0.00
        9.25      0.00       0.00
        9.5       0.00       0.00
        9.75      0.00       0.00
        10.0       0.00       0.00
        10.5       0.00       0.00
        11.0       0.00       -0.50
        11.5       0.00       -0.50
        12.0       0.00       0.00
        15.0        0           0];
    
    
subplot(1,3,1)
hold on
plot(out.CONTROLS(:,1), out.CONTROLS(:,2), 'r')
stairs(refEXT(:,1), refEXT(:,3), 'b+')
hold off
xlim([0 15]);
title('Control (reference in blue)');

subplot(1,3,2)
hold on
plot(out.STATES_SAMPLED(:,1), out.STATES_SAMPLED(:,2), 'r')
stairs(refEXT(:,1), refEXT(:,2), 'b+')
hold off
xlim([0 15]);
title('Differential State (reference in blue)');

subplot(1,3,3)
stairs(disturbance(:,1), disturbance(:,2), 'r')
xlim([0 15]);
title('Disturbance');
    