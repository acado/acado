function [ f ] = dae( t,x,xa,u,p,w ) 
%Blackbox DAE example file. 
%
%  Every DAE f should have the header
%  [ f ] = FILE_NAME( t,x,xa,u,p,w,dx )
%  with  t: time
%       x: differential states
%       xa: algebraic states (if any)
%       u: controls (if any)
%       p: parameters (if any)
%       w: disturbances (if any)
%
%  Write first all differential states (dot(x) == ...)
%  afterwards all algebraic states (0 == ...)
%


    f(1) = -x(1) + 0.5*x(1)*x(1) + u + 0.5*xa; 
    f(2) =  x(1)*x(1) + 3.0*u*u           ;  
    f(3) =  xa + exp(xa) - 1.0 + x(1)  ;
   
    
end
