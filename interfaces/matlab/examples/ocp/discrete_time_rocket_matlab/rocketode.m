function [ dx ] = rocketode( t,x,u,p,w )

%   THIS IS THE DEFINED SEQUENCE:
%   DifferentialState v s m;
%   Control u;
%
%   Thus x(1) = v, x(2) = s, x(3) = m and x(4) = L
%   Since only one control is set is u(1) = u.

    h = 0.01;
    
    v = x(1);
    s = x(2);
    m = x(3);
 
    dx(1) =  s + h*v;
	dx(2) =  v + h*(u-0.02*v*v)/m;
    dx(3) =  m - h*0.01*u*u;
    
end
