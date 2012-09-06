function [ dx ] = ode1( t,x,u,p,w )

	dx(1) = -2.0*x(1) + u(1);

end
