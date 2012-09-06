function [ dx ] = ode( t,x,u,p,w )

    % Just for ease of use we write down all variables. This is not really needed. 
    xB     = x(1);
    xW     = x(2);
    vB     = x(3);
    vW     = x(4);
    F      = u(1);
    R      = w(1);
  

    % constants
    mB = 350.0;
	mW = 50.0;
	kS = 20000.0;
	kT = 200000.0;
    
    
    % Differential equation.
    % The states are located in x. Dot(state) is located in dx. The index
    % refers to the corresponding state. Remark that this is the same
    % sequence as defined in algo.setVariables({},{xB xW vB vW},{F},{},{R});
	dx(1) = vB;
    dx(2) = vW;
    dx(3) = ( -kS*xB + kS*xW + F ) / mB;
    dx(4) = ( -kT*xB - (kT+kS)*xW + kT*R - F ) / mW;

    
    
%     % Debug code
%     disp('====== EVAL ODE ======');
%     disp(t)
%     disp(x)
%     disp(u)
%     disp(p)
%     disp(w)
end
