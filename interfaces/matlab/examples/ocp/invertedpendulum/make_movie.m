% AUTHOR: Maarten Breckpot

close all

time = out.STATES(:,1);
x = out.STATES(:,2);
alpha = out.STATES(:,3);

% alpha = rand(10,1)
scrsz = get(0,'ScreenSize');
l = 0.3;

for i=1:length(alpha)
    fig = figure('Position',[1 1 600 1200]);
    subplot(2,1,1)
    plot([x(i), x(i)+l*cos(alpha(i)+pi/2)],[0 l*sin(alpha(i)+pi/2)],'b','LineWidth',2);
    hold on
    plot(x(i)+l*cos(alpha(i)+pi/2),l*sin(alpha(i)+pi/2),'o','LineWidth',2,'MarkerFaceColor','b');
    plot(x(i)-0.05,0,'sg','LineWidth',6,'MarkerFaceColor','g');
    plot(x(i)+0.05,0,'sg','LineWidth',6,'MarkerFaceColor','g');
    plot(x(i)-0.06,-0.04,'ok','LineWidth',1,'MarkerFaceColor','k');
    plot(x(i)+0.06,-0.04,'ok','LineWidth',1,'MarkerFaceColor','k');
    xlabel('x position')
    ylabel('y position')
    axis([min(x)-1.1 max(x)+1.1 -0.5 +0.8])
    subplot(2,1,2)
    plot(time(1:i),x(1:i))
    axis([0 time(end) -4 1])
    xlabel('time [s]')
    ylabel('position')
    F(i) = getframe(fig);
    disp('Hit enter to see the next frame');
    pause
    close(fig)
    
end
for k=i+1:i+4
    F(k) = F(i);
end
movie2avi(F,'test5.avi')
