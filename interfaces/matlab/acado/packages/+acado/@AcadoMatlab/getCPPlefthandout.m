function getCPPlefthandout(obj, nameB, name, out)
%Generate CPP code for a left hand out matrix
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



    fprintf(obj.fileMEX,sprintf('    mxArray *%s = NULL;\n', nameB));
    fprintf(obj.fileMEX,sprintf('    double  *%s = NULL;\n', name));
    fprintf(obj.fileMEX,sprintf('    %s = mxCreateDoubleMatrix( %s.getNumPoints(),1+%s.getNumValues(),mxREAL ); \n', nameB, out, out));
    fprintf(obj.fileMEX,sprintf('    %s = mxGetPr( %s );\n', name, nameB));

    fprintf(obj.fileMEX,sprintf('    for( int i=0; i<%s.getNumPoints(); ++i ){ \n', out));
    fprintf(obj.fileMEX,sprintf('      %s[0*%s.getNumPoints() + i] = %s.getTime(i); \n', name, out, out));
    fprintf(obj.fileMEX,sprintf('      for( int j=0; j<%s.getNumValues(); ++j ){ \n', out));
    fprintf(obj.fileMEX,sprintf('        %s[(1+j)*%s.getNumPoints() + i] = %s(i, j); \n', name, out, out));
    fprintf(obj.fileMEX,'       } \n');
    fprintf(obj.fileMEX,'    } \n\n');
    
end