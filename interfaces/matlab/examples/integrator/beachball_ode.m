%%
%%    This file is part of ACADO Toolkit.
%%
%%    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
%%    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
%%Developed within the Optimization in Engineering Center (OPTEC) under
%%    supervision of Moritz Diehl. All rights reserved.
%%
%%    ACADO Toolkit is free software; you can redistribute it and/or
%%    modify it under the terms of the GNU Lesser General Public
%%    License as published by the Free Software Foundation; either
%%    version 3 of the License, or (at your option) any later version.
%%
%%    ACADO Toolkit is distributed in the hope that it will be useful,
%%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
%%    Lesser General Public License for more details.
%%
%%    You should have received a copy of the GNU Lesser General Public
%%    License along with ACADO Toolkit; if not, write to the Free Software
%%    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
%%
%%

%%
%%    \file interfaces/matlab/examples/integrator/beachball_ode.m
%%    \author David Ariens
%%    \date 2009
%%

% Every ODE should have the header
% [ dx ] = FILE_NAME( t,x,u,p,w )
% with  t: time
%       x: differential states
%       u: controls (if any)
%       p: parameters (if any)
%       w: disturbances (if any)

function [ dx ] = beachball_ode( t,x,u,p,w )

	dx(1) = x(2);
	dx(2) = -9.81 - .02*x(2);

        

% fprintf('ODE call        t=%5e \n', t);
%     disp(t)
%     disp(x)
end
