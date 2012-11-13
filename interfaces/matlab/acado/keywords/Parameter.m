%Shorthand for acado.Parameter
%
%  Example:
%    >> Parameter p;
%    >> Parameter p1 p2 p3 p4;
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
function  Parameter( varargin )

checkActiveModel;

if ~iscellstr( varargin ),
    error( 'Syntax is: Parameter x' );
    
else
    
    for k = 1 : nargin,
        [name N M] = readVariable(varargin{k});
        
        for i = 1:N
            for j = 1:M
                if N > 1
                    VAR_NAME = strcat(name,num2str(i));
                else
                    VAR_NAME = name;
                end
                if M > 1
                    VAR_NAME = strcat(VAR_NAME,num2str(j));
                end
                VAR_ASSIGN = acado.Parameter(VAR_NAME);
                var(i,j) = VAR_ASSIGN;
                
                assignin( 'caller', VAR_NAME, VAR_ASSIGN );
            end
        end
        assignin( 'caller', name, var );
        var = VAR_ASSIGN;
    end
    
end

end

