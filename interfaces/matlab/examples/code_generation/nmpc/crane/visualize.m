H = 0.9;

figure(10); clf
whitebg([1.0 1.0 1.0])
set(gcf,'Color',[1 1 1])

x_r = Xref(1,1);
y_r = H-Xref(1,3);

subplot(3,1,[1 2]);
line([-0.1 1.0], [H H], 'color', 'k', 'Linewidth',1.5); hold on;
plot(x_r, y_r, 'gx', 'MarkerSize', 16, 'Linewidth', 2);
text(H-0.08,H+0.045,['current time: ' num2str(time(end)) 's'],'FontSize',15);
plot(state_sim(end,1),H,'ks','MarkerSize',30,'Linewidth',3);

alpha = state_sim(end,7);
xB = state_sim(end,1)+state_sim(end,3)*sin(alpha);
yB = H-state_sim(end,3)*cos(alpha);

line([state_sim(end,1) xB], [H yB], 'Linewidth',2);
plot(xB,yB,'ro','MarkerSize',25,'Linewidth',3);

grid on;
xlim([-0.1 1.0]);
ylim([0.0 1.0]);

subplot(3,1,3);
semilogy(time(1:end-1),KKT_MPC,'linewidth',1.5,'color','k','linestyle','--','marker','.');hold on

grid on;
xlabel('t [s]','FontSize',13);    ylabel('MPC KKT','FontSize',13)