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

function [name N M] = readVariable(in)

    if ~isa(in, 'char')
       error('Unsupported use of the readVariable function.'); 
    end
    l = strfind(in,'(');
    r = strfind(in,')');
    if ~isempty(l) && ~isempty(r)
        c = strfind(in,',');
        name = in(1:l-1);
        if ~isempty(c)
            [N,OK] = str2num(in(l+1:c-1));
            if ~OK
               N = evalin('base', in(l+1:c-1)); 
            end
            [M,OK] = str2num(in(c+1:r-1));
            if ~OK
               M = evalin('base', in(c+1:r-1)); 
            end
        else
            M = 1;
            [N,OK] = str2num(in(l+1:r-1));
            if ~OK
               N = evalin('base', in(l+1:r-1)); 
            end
        end
    else
        name = in;
        N = 1;
        M = 1;
    end
    if strcmp(name, '0') || strcmp(name, 'NULL')
        N = 0;
        M = 0;
    end

end