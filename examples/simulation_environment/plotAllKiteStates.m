function [] = plotAllKiteStates( )

clear;
load 'kite_x.mat';
load 'kite_xRef.mat';

plotKiteState(x(:,1),[x(:,2),xRef(:,2)],'phi [rad]');
plotKiteState(x(:,1),[x(:,3),xRef(:,3)],'theta [rad]');
plotKiteState(x(:,1),[x(:,4),xRef(:,4)],'dphi [rad/s]');
plotKiteState(x(:,1),[x(:,5),xRef(:,5)],'dtheta [rad/s]');

