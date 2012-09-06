function [ J ] = myjacobian( t,x,u,p,w )

	J = [-2*x , 2*u , 1 , 0 , 1];
        % dx    du    dp  dq  dw
        
%     disp('====== EVAL JAC ======');

end
