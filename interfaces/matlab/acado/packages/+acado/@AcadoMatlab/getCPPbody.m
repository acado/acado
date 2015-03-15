function getCPPbody(obj)
%Generate CPP file body
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
%    Author: David Ariens, Rien Quirynen
%    Date: 2010-2013
% 
    
fprintf(obj.fileMEX,'#include <mex.h>\n');
fprintf(obj.fileMEX,'\n\n');
fprintf(obj.fileMEX,'void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) \n ');
fprintf(obj.fileMEX,'{ \n \n');
fprintf(obj.fileMEX,'    MatlabConsoleStreamBuf mybuf;\n');
fprintf(obj.fileMEX,'    RedirectStream redirect(std::cout, mybuf);\n');
fprintf(obj.fileMEX,'    clearAllStaticCounters( ); \n \n');

fprintf(obj.fileMEX,'    mexPrintf("\\nACADO Toolkit for Matlab - Developed by David Ariens and Rien Quirynen, 2009-2013 \\n"); \n');
fprintf(obj.fileMEX,'    mexPrintf("Support available at http://www.acadotoolkit.org/matlab \\n \\n"); \n\n');

fprintf(obj.fileMEX,sprintf('    if (nrhs != %d){ \n', length(obj.in)));
fprintf(obj.fileMEX,sprintf('      mexErrMsgTxt("This problem expects %d right hand side argument(s) since you have defined %d MexInput(s)");\n', length(obj.in), length(obj.in)));
fprintf(obj.fileMEX,'    } \n \n');      

end