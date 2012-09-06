function [ dx ] = myode( t,x,u,p,w )

    % Just for ease of use we write down all variables. This is not needed. 
    phi    = x(1);
    omega  = x(2);
    l      = p(1);
    alpha  = p(2);
    g      = p(3);
    
    % Differential equation.
    % The states are located in x. Dot(state) is located in dx. The index
    % refers to the corresponding state. Remark that this is the same
    % sequence how your states, parameters... are defined in your problem
	dx(1) = omega;
    dx(2) = -(g/l)*sin(phi) - alpha*omega;
    
	
    % Debug code
%      disp('====== EVAL ODE ======');
%      disp(t)
%      disp(x)
%      disp(u)
%      disp(p)
%      disp(w)
end
