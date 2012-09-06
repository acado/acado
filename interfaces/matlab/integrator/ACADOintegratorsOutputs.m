%ACADOintegrators is a collection of different integrators for solving ODE and 
%DAE systems including efficient sensitivity generation.
%ACADOintegratorsOutputs returns an outputs struct as returned after calling 
%ACADOintegrators comprising the following items:
%
%    Status                 --  0 on successful return of the integrator,
%                               otherwise -1
%    NumberOfSteps          --  Number of actually performed integrator steps
%    NumberOfRejectedSteps  --  Number of rejected integrator steps
%
%    xTrajectory            --  Matrix, first column are the time points,
%                               next columns contain values for all 
%                               differential states
%    xaTrajectory           --  Matrix, first column are the time points,
%                               next columns contain values for all 
%                               algebraic states
%
%    J    --  Forward mode Jacobian
%    Jx   --  Backward mode Jacobian w.r.t. to differential states x
%    Ju   --  Backward mode Jacobian w.r.t. to controls u
%    Jp   --  Backward mode Jacobian w.r.t. to parameters p
%    Jw   --  Backward mode Jacobian w.r.t. to disturbances w
%    J2   --  Forward mode Jacobian (2nd order)
%    J2x  --  Backward mode Jacobian w.r.t. to differential states x (2nd order)
%    J2u  --  Backward mode Jacobian w.r.t. to controls u (2nd order)
%    J2p  --  Backward mode Jacobian w.r.t. to parameters p (2nd order)
%    J2w  --  Backward mode Jacobian w.r.t. to disturbances w (2nd order)
%
%On successful return (i.e. Status == 0), the number of steps, the mesh and the
%trajectory information are written. Sensitivity output is only written, if 
%SensitivityMode had not been empty and corresponding seeds had been written 
%within the ACADOintegratorsSettings struct.
%
%If Status is -1, all other components remain empty.
%
%
%See also ACADOINTEGRATORS, ACADOINTEGRATORSSETTINGS
%
%
%For additional information see the ACADOintegrators Tutorial and User's Manual 
%or visit http://www.acadotoolkit.org.
%
%Please send remarks and questions to support@acadotoolkit.org!
%
%
%  Licence:
%    This file is part of ACADO Toolkit  - (http://www.acadotoolkit.org/)
%
%    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
%    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
%    Developed within the Optimization in Engineering Center (OPTEC) under
%    supervision of Moritz Diehl. All rights reserved.
%
%    ACADO Toolkit is free software; you can redistribute it and/or
%    modify it under the terms of the GNU Lesser General Public
%    License as published by the Free Software Foundation; either
%    version 3 of the License, or (at your option) any later version.
%
%    ACADO Toolkit is distributed in the hope that it will be useful,
%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
%    Lesser General Public License for more details.
%
%    You should have received a copy of the GNU Lesser General Public
%    License along with ACADO Toolkit; if not, write to the Free Software
%    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
%
%    Author: Niels Haverbeke, Boris Houska, Hans Joachim Ferreau, David Ariens
%    Date: 2008-2010
% 


function [ outputs ] = ACADOintegratorsOutputs( )

	outputs = struct(	'Status',[],...
						'NumberOfSteps',[],...
						'NumberOfRejectedSteps',[],...
						'xTrajectory',[],...
						'xaTrajectory',[],...
						'J',[],...
						'Jx',[],...
						'Ju',[],...
						'Jp',[],...
						'Jw',[],...
						'J2',[],...
						'J2x',[],...
						'J2u',[],...
						'J2p',[],...
						'J2w' );

end
