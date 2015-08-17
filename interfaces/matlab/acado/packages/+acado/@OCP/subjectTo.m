function subjectTo(obj, varargin)
%Add constraints to an OCP. Link differential equation to an OCP.
%
%  Usage:
%    * ocp.subjectTo(acado.DifferentialEquation)
%      OCP w.r.t. a differential equation. This line should always be added
%      in the OCP formulation! All other subjectto's are optional.
%      >> ocp.subjectTo(f) 
%
%    * ocp.subjectTo(acado.DifferentialEquation, n)
%      OCP w.r.t. a differential equation. "n" is the number of control
%      intervals (use this in a multi stage OCP)
%      >> ocp.subjectTo(f, 10) 
%
%    * ocp.subjectTo( 'AT_START', expression );
%      Add an Initial constraint for expression. This constraint should
%      only be satisfied in the beginning (eg fix initial values)
%      >> ocp.subjectTo( 'AT_START', x == 1.0 ); 
%      >> ocp.subjectTo( 'AT_START', x+y == sin(pi) );
%   
%    * ocp.subjectTo( 'AT_END', expression );
%      Add a terminal constraint for expression. This constraint should
%      only be satisfied at the end (eg fix a terminal value)
%      >> ocp.subjectTo( 'AT_END', x == 10.0 );
%
%    * ocp.subjectTo(  expression ); 
%      Add a path constraint (should always be satisfield). 
%      >> ocp.subjectTo(  0.1 <= p <= 2.0 );  
%         (P should be between certain bounds)
%      >> ocp.subjectTo(  0.1 == p );
%         (P should always be fixed to 0.1 over the entire interval. Use
%         this to fix for example a parameters)
%      >> ocp.subjectTo(  0.1 == (p + cos(m))/2 );
%         (More difficult expresssions are also allowed)
%      >> ocp.subjectTo(  p == MATRIX )
%         (Matrix contains as first column time points and as second column
%         reference values for p on these specific time points. Use this to
%         set a trajectory for values over time. You can for example use
%         this for time varying paramters. When you have a certain input
%         trajectory which is fixed, define a control and use this notation
%         to fix the input)
%
%    * ocp.subjectTo(  0.0, r , -r , 0.0 );   
%      Adds a constraint of the form lb_ <= arg1(0) + arg_2(T) <= ub with 
%      constant lower and upper bounds. 
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
%    Date: 2009-2010
% 
    % Fix because of all the calls to the "toString" function
    global ACADO_;
    ACADO_.generatingCode = 1;

    if (nargin == 2 && isa(varargin{1}, 'acado.DifferentialEquation'))
        % ocp.subjectTo( f ); 
        obj.subjectoItems{end+1} = sprintf('%s', varargin{1}.name);
        
    elseif (nargin == 3 && isa(varargin{1}, 'acado.DifferentialEquation') && isa(varargin{2}, 'numeric'))
        % ocp.subjectTo( f, 10 ); 
        double1 = acado.DoubleConstant(varargin{2});
        obj.subjectoItems{end+1} = sprintf('%s, %s', varargin{1}.name, double1.toString());
          
    elseif(nargin == 3 && isa(varargin{1}, 'char') && isa(varargin{2}, 'acado.Expression'))
        % ocp.subjectTo( 'AT_START', x == 1.0 );
        % ocp.subjectTo( 'AT_END', x == 1.0 );
        for i = 1:length(varargin{2})
            obj.subjectoItems{end+1} = sprintf('%s, %s', varargin{1}, varargin{2}(i).toString());
        end
          
    elseif(nargin == 3 && isnumeric(varargin{1}) && isa(varargin{2}, 'acado.Expression'))
        % ocp.subjectTo( 5, x == 1.0 );
        stage = acado.DoubleConstant(varargin{1});
        for i = 1:length(varargin{2})
            obj.subjectoItems{end+1} = sprintf('%s, %s', stage.toString(), varargin{2}(i).toString());
        end
       
    elseif(nargin == 2 && isa(varargin{1}, 'acado.Expression'))
        % ocp.subjectTo(  0.1 <= p <= 2.0 );
        % ocp.subjectTo(  0.1 == p );
        for i = 1:length(varargin{1})
            obj.subjectoItems{end+1} = varargin{1}(i).toString();
        end
    
    elseif(nargin == 5 && isa(varargin{1}, 'numeric') && isa(varargin{2}, 'acado.Expression') && isa(varargin{3}, 'acado.Expression') && isa(varargin{4}, 'numeric'))
        % ocp.subjectTo( 0.0, r , -r , 0.0 );
        
        double1 = acado.DoubleConstant(varargin{1});
        double2 = acado.DoubleConstant(varargin{4});
        
        obj.subjectoItems{end+1} = sprintf('%s, %s, %s, %s',double1.toString(), varargin{2}.toString(), varargin{3}.toString(), double2.toString());
     
    else
       error('ERROR: Invalid subjectTo. <a href="matlab: help acado.OCP.subjectTo">help acado.OCP.subjectTo</a>'); 
        
    end
    % Important to set the value to zero again
    ACADO_.generatingCode = 0;
end