function [ dx ] = myode( t,x,u,p,w )
    
    dx(1) = -x*x + p(1) + u*u + w;
        
%     disp('====== EVAL ODE ======');
%      disp(t)
%      disp(x)
%      disp(u)
%      disp(p)
%      disp(w)

end
