function getInstructions(obj, cppobj, get)
%Used to generate CPP file
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


if (get == 'FB')

    fprintf(cppobj.fileMEX,sprintf('    double *%s_temp = NULL; \n', obj.name));
    fprintf(cppobj.fileMEX,sprintf('    if( !mxIsDouble(prhs[%d]) || mxIsComplex(prhs[%d]) || !(mxGetM(prhs[%d])==1 && mxGetN(prhs[%d])==1) ) { \n', obj.counter, obj.counter, obj.counter, obj.counter));
    fprintf(cppobj.fileMEX,sprintf('      mexErrMsgTxt("Input %d must be a noncomplex scalar double.");\n', obj.counter));
    fprintf(cppobj.fileMEX,sprintf('    } \n'));
    fprintf(cppobj.fileMEX,sprintf('    %s_temp = mxGetPr(prhs[%d]); \n', obj.name, obj.counter));
    fprintf(cppobj.fileMEX,sprintf('    double %s = *%s_temp; \n', obj.name, obj.name));

    fprintf(cppobj.fileMEX,'\n');
end

end
