function [ dx ] = rocketode( t,x,u,p,w )

%   THIS IS THE DEFINED SEQUENCE:
%   DifferentialState v s m L;
%   Control u;
%
%   Thus x(1) = v, x(2) = s, x(3) = m and x(4) = L
%   Since only one control is set is u(1) = u.
    
    v = x(1);
    m = x(3);
 
    dx(1) = (u-0.02*v*v)/m;
	dx(2) = v;
    dx(3) = -0.01*u*u;
    dx(4) = u*u;
    
end
