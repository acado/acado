function initializeDisturbances(obj, varargin)
%Initialization of the disturbances. Initialize disturbances on different time points. 
% The matrix M can contain multiple rows for different initializations of 
% the disturbances. Note that these are only initializations, the optimization 
% routine is not forced to use these values exactly. 
% If values should be fixed, use "ocp.subjectTo".
%
%  Usage:
%    >> algo.initializeDisturbances(M)
%
%  Parameters:
%    M 	  m x (n_z + 1) Matrix with initial values
%         where m   = number of different initializations on different times
%               n_z = number of disturbances
%         the first column represents the time points
%         the second column represents the first disturbance
%         the thirth column represents the second disturbance
%         ...
%
%    A warning will be shown when the number of columns is not equal to the
%    number of disturbances + 1 (the time);
%
%  Example:
%    Example 1: Initialize initial (on t=0) values of 2 disturbances.
%    Initial values are: w1(0) = 5 and w2(0) = 2.5. In this case the matrix
%    M will have one row, the first element is '0' (the time), the second
%    and thirth element are 5 and 2.5 (note that the sequence should be the
%    same as how the disturbances are defined)
%     >> Disturbance w1 w2;
%     >> M = [0  5  2.5];
%     >> algo.initializeDisturbances(M);
%
%    Example 2: Initialize on t=0 and t=1 (for example we know what these
%    values will be due to other calculations). Initial values are: 
%    w1(0) = 5, w2(0) = 2.5 and w1(1) = 5.5, w2(1) = -1
%     >> Disturbance w1 w2;
%     >> M = [0  5    2.5
%             1  5.5  -1];
%     >> algo.initializeDisturbances(M);
%
%    Example 3: Although we have two disturbances, we only want to 
%    initialize the first: w1(0) = 5. In this case the matrix should only
%    contain one extra column next to the time reference. Note that a
%    warning will be displayed to indicate a possible problem. Ignore this
%    warning.
%     >> Disturbance w1 w2;
%     >> M = [0  5];
%     >> algo.initializeDisturbances(M);
%
%    Example 4: We only want to initialize the second disturbance: w2(0) = 2.5. 
%    In this case you should reverse the order how the disturbances are defined!
%     >> Disturbance w2 w1;
%     >> M = [0  2.5];
%     >> algo.initializeDisturbances(M);
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
%    Author: David Ariens
%    Date: 2010
% 

global ACADO_;

if (length(varargin) == 1) 

    [m n] = size(varargin{1});
    if (length(ACADO_.helper.w) == n)
        warning('ACADO:initialize', 'Possible problem in initializeDisturbances. You have %d disturbances and the number of columns in the initialisation matrix is also %d. This is probably not correct since the first column should contain a time reference. This matrix should thus have %d columns. See also <a href="matlab: help acado.OptimizationAlgorithm.initializeDisturbances">help acado.OptimizationAlgorithm.initializeDisturbances</a>', length(ACADO_.helper.w), n, length(ACADO_.helper.w)+1);
    elseif (length(ACADO_.helper.w) ~= (n-1))
        warning('ACADO:initialize', 'Possible problem in initializeDisturbances. You have %d disturbances and the number of columns in the initialisation matrix is %d. First column should be a time reference, the next columns should be initialisations for all disturbances. This matrix should thus have %d columns. See also <a href="matlab: help acado.OptimizationAlgorithm.initializeDisturbances">help acado.OptimizationAlgorithm.initializeDisturbances</a>', length(ACADO_.helper.w), n, length(ACADO_.helper.w)+1);
    end
    
    obj.initdisturbances = acado.Matrix(varargin{1});
    
else %error
    
   error('ERROR: Invalid initializeDisturbances call. <a href="matlab: help acado.OptimizationAlgorithm.initializeDisturbances">help acado.OptimizationAlgorithm.initializeDisturbances</a>'); 

end


end