function [ dx ] = ode2( t,x,u,p,w )

	dx(1) = -2.0*x(1) + u(1) + 0.1*w(1);

end
