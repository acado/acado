function [ dx ] = matlab_integrator_ode_pendulum( t,x,u,p,w )

%   THIS IS THE DEFINED SEQUENCE:
%     DifferentialState phi, dphi;
%     Control u;
%     Parameter p;
%     TIME t;
%
%   Thus x(1) = phi, x(2) = dphi
%   Since only one control is set is u(1) = u.
    
    phi = x(1);       % the angle phi
    dphi = x(2);      % the first derivative of phi w.r.t time
    F = u;            % a force acting on the pendulum
    l = p;            % the length of the pendulum

    m     = 1.0  ;    % the mass of the pendulum
    g     = 9.81 ;    % the gravitational constant
    alpha = 2.0  ;    % frictional constant
    
    
    dx(1) = dphi;
    dx(2) = -(m*g/l)*sin(phi) - alpha*dphi + F/(m*l);
    
end
