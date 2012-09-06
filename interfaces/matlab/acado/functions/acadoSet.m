%Set ACADO main settings
%
%  Usage:
%    >> acadoSet(property, value);
%
%  Possible settings:
%    'problemname',         a valid variable name, default 'myAcadoProblem'
%       Set the output name of your problem. This value will be used to name
%       all files related to your problem
%
%    'results_to_file',     [LOGICAL], default false
%       The result of your problem is written to an output struct which you
%       can store by running your problem as "out = myAcadoProblem_RUN();".
%       Optionaly ACADO can also write your results to a matrix in a file.
%
%
%  Example:
%    >> acadoSet('problemname', 'my_demo');
%    >> acadoSet('results_to_file', true);
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
function acadoSet( property, value )
    
    global ACADO_;
    
    checkActiveModel;
    
    if (strcmp(property, 'problemname') && isvarname(value))
        
        ACADO_.problemname = value;
    
    elseif (strcmp(property, 'results_to_file') && isa(value, 'logical'))
        
        ACADO_.results_to_file = value;
        
    else
        
        error('ERROR: Invalid setting. <a href="matlab: help acadoSet">help acadoSet</a>'); 
        
    end
    
end
