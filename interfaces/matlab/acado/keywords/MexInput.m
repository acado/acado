%Shorthand for acado.MexInput
%
%  Example:
%    >> MexInput x;
%    >> MexInput x1 x2 x3 x4;
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
%    Author: Rien Quirynen
%    Date: 2012
%
function MexInput( varargin )

checkActiveModel;

if ~iscellstr( varargin ),
    error( 'Syntax is: MexInput x' );
    
else
    
    for k = 1 : nargin,
        [name N M] = readVariable(varargin{k});
        
        if N == 0 && M == 0
            global ACADO_;
            ACADO_.helper.clearIn;
        else
            VAR_NAME = name;
            if N == 1 && M == 1
                VAR_ASSIGN = acado.MexInput(VAR_NAME);
            elseif N == 1 || M == 1
                VAR_ASSIGN = acado.MexInputVector(VAR_NAME);
            else
                VAR_ASSIGN = acado.MexInputMatrix(VAR_NAME);
            end
            
            assignin( 'caller', VAR_NAME, VAR_ASSIGN );
        end
    end
    
end

end

