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
%%    \file interfaces/matlab/models/glycemia_matlab2.m
%%    \author Niels Haverbeke, Boris Houska, Hans Joachim Ferreau
%%    \date 2008
%%


function [ dx ] = glycemia_matlab2( t,x,u,p,w )

	s = 0.1;
	z = exp(x(4)/s);

	dx(1) = (-p(2) - x(3)) * x(1)   +   p(2) * p(5)  +  p(1)/p(6) + w(1);
	dx(2) = p(11) * s *log(1 + z) - p(10)*(x(2)-p(8))  +  u(1)/p(7) + w(2);
	dx(3) = -p(3) * x(3) + p(4) * 0.001 * (x(2)-p(8)) + w(3);
	dx(4) = p(12) * (x(1) - p(9)) - p(10) * (x(4)) + w(4);

end
