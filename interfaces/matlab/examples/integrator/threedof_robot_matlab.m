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
%%    \file interfaces/matlab/models/threedof_robot_matlab.m
%%    \author Niels Haverbeke, Boris Houska, Hans Joachim Ferreau
%%    \date 2008
%%


function [ y ] = threedof_robot_matlab( t,x,u,p,w )

	%% differential states
    q1  = x(1);
    q2  = x(2);
    q3  = x(3);
    dq1 = x(4);
    dq2 = x(5);
    dq3 = x(6);

	%% differential states derivatives
    d_q1  = dx(1);    %% the 1st derivative of q1    w.r.t  t
    d_q2  = dx(2);    %% the 1st derivative of q2    w.r.t  t
    d_q3  = dx(3);    %% the 1st derivative of q3    w.r.t  t
    d_dq1 = dx(4);    %% the 1st derivative of dq1    w.r.t  t
    d_dq2 = dx(5);    %% the 1st derivative of dq2    w.r.t  t
    d_dq3 = dx(6);    %% the 1st derivative of dq3    w.r.t  t

	% controls
    tau1 = u(1);
    tau2 = u(2);
    tau3 = u(3);

	% disturbances
    w1 = w(1);
    w2 = w(2);
    w3 = w(3);
    w4 = w(4);
    w5 = w(5);
    w6 = w(6);

	% parameters
    y0  = p(1);
    y1  = p(2);
    y2  = p(3);
    y3  = p(4);
    y4  = p(5);
    y5  = p(6);
    y6  = p(7);
    y7  = p(8);
    y8  = p(9);
    y9  = p(10);
    y10 = p(11);
    y11 = p(12);
    y12 = p(13);
    y13 = p(14);
    y14 = p(15);
    y15 = p(16);
    y16 = p(17);
    y17 = p(18);
    y18 = p(19);
    y19 = p(20);
    y20 = p(21);
    y21 = p(22);
    y22 = p(23);
    y23 = p(24);
    y24 = p(25);
    y25 = p(26);

	% constants
    D13 = 0.48;
    g1 = 9.81;           %% gravitational constant
    r3 = 51.44118;
    l = 0.488;
    r = 0.1;
    dqm = 0.001;

	%% intermediate states
	t1 = cos(q3);
	t2 = D13 * t1;
	t3 = sin(q3);
	t4 = D13 * t3;
	t5 = q2 + q3;
	t6 = sin(t5);
	t7 = cos(t5);
	t8 = t6 * t7;

	t11 = t6 * t6;
	t12 = t7 * t7;
	t13 = t11 - t12;

	t20 = cos(q2);
	t21 = t20 * t20;
	t22 = sin(q2);
	t23 = t22 * t22;
	t34 = t6 * y9;
	t35 = t7 * y10;	
	t40 = t2 * y6;
	t41 = t4 * y5;

	M44 = (t2 - 2 * t4 * t8 + t2 * t13) * y6 + t13 * y7 + 2 * t8 * y8 + y0 + (t21 - t23) * y1 - 2 * t20 * t22 * y2 + (t4 - t4 * t13 - 2 * t2 * t8) * y5;
	M45 = t34 + t35 + t20 * y3 - t22 * y4;
	M46 = t34 + t35;

	M55 = 2 * t41 + 2 * t40 + y13 + y14 + y15 / 1000;
	M56 = t41 + t40 + y14 + r3 * y15 / 1000;

	M66 = y14 + r3 * r3 * y15 / 1000;

	sdq1 = (2/(1+exp(-3*dq1/dqm))-1);
	sdq2 = (2/(1+exp(-3*dq2/dqm))-1);
	sdq3 = (2/(1+exp(-3*dq3/dqm))-1);	

	t9 = y2 * dq1;
	t10 = y8 * dq1;

	t24 = dq3 * t6;

	t26 = y5 * dq1;
	t27 = t26 * dq2;

	t29 = D13 * t22;
	t30 = t29 * t6;
	t36 = y6 * dq1;
	t37 = t36 * dq2;
	t38 = D13 * t20;
	t39 = t38 * t6;
	t42 = t29 * t10;
	t45 = y7 * dq1;
	t50 = t38 * t10;
	t58 = -4 * t9 * dq2 * t21 + 4 * t10 * dq2 * t12 + 4 * t10 * dq3 * t12 + 2 * y9 * dq2 * dq3 * t10 - 2 * y10 * dq2 * t24 + 2 * t27 * t30 + 2 * t26 * dq3 * t30 + 2 * t37 * t39 + 2 * t37 * t42 + 4 * t45 * dq2 * t6 * t10 + y23 - 2 * t27 * t50 - 4 * y1 * dq1 * dq2 * t20 * t22;
	t73 = dq2 * dq2;
	t80 = dq3 * dq3;
	t87 = dq1 * y17 + sdq1 * y18 + 2 * t36 * dq3 * t42 + 4 * t45 * t24 * t10 + 2 * t9 * dq2 - 2 * t10 * dq2 - 2 * t10 * dq3 - y3 * t73 * t22 - y4 * t73 * t20 + y9 * t73 * t10 + y9 * t80 * t10 - y10 * t73 * t6 - y10 * t80 * t6;

	t88 = dq1 * dq1;
	t89 = t88 * y8;

	t96 = sqrt(l * l + r * r - 2 * l * r * t20);
	t101 = t88 * y2;
	t111 = t88 * t6;
	t112 = t10 * y7;
	t116 = g1 * t20;
	t125 = y5 * t96;
	t126 = t125 * t88;
	t129 = -t89 * t96 + 2 * t89 * t96 * t12 + t101 * t96 - 2 * t101 * t96 * t21 - 2 * t88 * t20 * t22 * y1 * t96 + 2 * t111 * t112 * t96 + t116 * y12 * t96 - dq2 * y19 * t96 - sdq2 * y20 * t96 - t22 * y11 * t96 - t126 * t50 + t126 * t30;
	t131 = dq3 * D13;

	t136 = t80 * D13;
	t141 = g1 * t22;

	t145 = y6 * t96;
	t146 = t145 * t88;
	t163 = -2 * t125 * dq2 * t131 * t9 - t125 * t136 * t9 - t125 * t116 * t9 + t125 * t141 * t21 + t146 * t39 + t146 * t42 + 2 * t145 * dq2 * t131 * t21 + t145 * t136 * t21 + t145 * t116 * t21 + t145 * t141 * t9 + 1000 * l * r * t22 * y16 - y24 * t96;

	F4 = tau1 - (t58 + t87);	
	F5 = tau2 +(t129 + t163) / t96;
	F6 = tau3 - (y6 * t73 * D13 * t21 - y5 * t73 * D13 * t9 - 2 * t89 * t12 + sdq3 * y22 - 2 * t111 * t112 - y5 * t88 * t30 + t89 - y6 * g1 * t6 + y5 * g1 * t10 - y6 * t88 * t42 + dq3 * y21 + y25);


	%% ODE system
	y(1) = d_q1 - dq1 + w1;
	y(2) = d_q2 - dq2 + w2;
	y(3) = d_q3 - dq3 + w3;
	y(4) = M44 * d_dq1 + M45 * d_dq2 + M46 * d_dq3 - F4 + w4;
	y(5) = M45 * d_dq1 + M55 * d_dq2 + M56 * d_dq3 - F5 + w5;
	y(6) = M46 * d_dq1 + M56 * d_dq2 + M66 * d_dq3 - F6 + w6;

end
