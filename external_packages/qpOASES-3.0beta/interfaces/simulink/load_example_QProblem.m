%%
%%	This file is part of qpOASES.
%%
%%	qpOASES -- An Implementation of the Online Active Set Strategy.
%%	Copyright (C) 2007-2011 by Hans Joachim Ferreau, Andreas Potschka,
%%	Christian Kirches et al. All rights reserved.
%%
%%	qpOASES is free software; you can redistribute it and/or
%%	modify it under the terms of the GNU Lesser General Public
%%	License as published by the Free Software Foundation; either
%%	version 2.1 of the License, or (at your option) any later version.
%%
%%	qpOASES is distributed in the hope that it will be useful,
%%	but WITHOUT ANY WARRANTY; without even the implied warranty of
%%	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
%%	See the GNU Lesser General Public License for more details.
%%
%%	You should have received a copy of the GNU Lesser General Public
%%	License along with qpOASES; if not, write to the Free Software
%%	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
%%



%%
%%	Filename:  interfaces/simulink/load_example_QProblem.m
%%	Author:    Aude Perrin, Hans Joachim Ferreau
%%	Version:   3.0beta
%%	Date:      2007-2011
%%



clear all;


%% setup QP data
simulationTime  =  [0;0.1];

H.time = simulationTime;
data1 = [1.0,0.0,0.0,0.5];
data2 = [1.0,0.0,0.0,0.5];
H.signals.values = [data1; data2];
H.signals.dimensions = 4;

g.time = simulationTime;
data1 = [1.5,1.0];
data2 = [1.0,1.5];
g.signals.values = [data1; data2];
g.signals.dimensions = 2;

A.time = simulationTime;
data1 = [1.0,1.0];
data2 = [1.0,1.0];
A.signals.values = [data1; data2];
A.signals.dimensions = 2;

lb.time = simulationTime;
data1 = [0.5,-2.0];
data2 = [0.0,-1.0];
lb.signals.values = [data1; data2];
lb.signals.dimensions = 2;

ub.time = simulationTime;
data1 = [5.0,2.0];
data2 = [5.0,-0.5];
ub.signals.values = [data1; data2];
ub.signals.dimensions = 2;

lbA.time = simulationTime;
data1 = [-1.0];
data2 = [-2.0];
lbA.signals.values = [data1; data2];
lbA.signals.dimensions = 1;

ubA.time = simulationTime;
data1 = [2.0];
data2 = [1.0];
ubA.signals.values = [data1; data2];
ubA.signals.dimensions = 1;


clear simulationTime data1 data2


%% open corresponding simulink example
open( 'example_QProblem.mdl' );



%%
%%	end of file
%%
