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
%%    \file interfaces/matlab/models/glycemia_matlab.m
%%    \author Niels Haverbeke, Boris Houska, Hans Joachim Ferreau
%%    \date 2008
%%


function [ dx ] = glycemia_matlab( t,x,u,p,w )

	G  = x(1);		%% blood glucose concentration
	I  = x(2);		%% blood insulin concentration
	X  = x(3);		%% effect of insulin on net glucose disappearance
	I2 = x(4);		%% effect of endogenous insulin

	FI = u(1);		%% exogenous insulin flow

	w0 = w(1);
	w1 = w(2);
	w2 = w(3);
	w3 = w(4);

	FG = p(1);		%% carbohydrate calories flow
	P1 = p(2);
	P2 = p(3);
	P3 = p(4);
	C1 = p(5);
	C2 = p(6);
	C3 = p(7);
	C4 = p(8);
	C5 = p(9);
	n = p(10);
	alpha = p(11);
	gamma = p(12);

	s = 0.1;
	z = exp(I2/s);

	dx(1) = (-P1 - X) * G   +   P1 * C1  +  FG/C2 + w0;
	dx(2) = alpha * s *log(1 + z) - n*(I-C4)  +  FI/C3 + w1;
	dx(3) = -P2 * X + P3 * 0.001 * (I-C4) + w2;
	dx(4) = gamma * (G - C5) - n * (I2) + w3;

end
