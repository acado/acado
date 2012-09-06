function [] = plotAllKiteControls( )

clear;
load 'kite_u.mat';
load 'kite_uRef.mat';

u(1,2) = 20.505;
u(1,3) = -0.1;

plotKiteControl(u(:,1),[u(:,2),uRef(:,2)],'pitch control',[18.75 21.25]);
plotKiteControl(u(:,1),[u(:,3),uRef(:,3)],'roll control',[-0.25 0.25]);