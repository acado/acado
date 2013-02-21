function getCPPheader(obj)
%Generate CPP file header
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
    
    fprintf(obj.fileMEX,'/*\n');
    fprintf(obj.fileMEX,'*    This file is part of ACADO Toolkit.\n');
    fprintf(obj.fileMEX,'*\n');
    fprintf(obj.fileMEX,'*    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.\n');
    fprintf(obj.fileMEX,'*    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.\n');
    fprintf(obj.fileMEX,'*    Developed within the Optimization in Engineering Center (OPTEC) under\n');
    fprintf(obj.fileMEX,'*    supervision of Moritz Diehl. All rights reserved.\n');
    fprintf(obj.fileMEX,'*\n');
    fprintf(obj.fileMEX,'*    ACADO Toolkit is free software; you can redistribute it and/or\n');
    fprintf(obj.fileMEX,'*    modify it under the terms of the GNU Lesser General Public\n');
    fprintf(obj.fileMEX,'*    License as published by the Free Software Foundation; either\n');
    fprintf(obj.fileMEX,'*    version 3 of the License, or (at your option) any later version.\n');
    fprintf(obj.fileMEX,'*\n');
    fprintf(obj.fileMEX,'*    ACADO Toolkit is distributed in the hope that it will be useful,\n');
    fprintf(obj.fileMEX,'*    but WITHOUT ANY WARRANTY; without even the implied warranty of\n');
    fprintf(obj.fileMEX,'*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU\n');
    fprintf(obj.fileMEX,'*    Lesser General Public License for more details.\n');
    fprintf(obj.fileMEX,'*\n');
    fprintf(obj.fileMEX,'*    You should have received a copy of the GNU Lesser General Public\n');
    fprintf(obj.fileMEX,'*    License along with ACADO Toolkit; if not, write to the Free Software\n');
    fprintf(obj.fileMEX,'*    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA\n');
    fprintf(obj.fileMEX,'*\n');
    fprintf(obj.fileMEX,'*/\n');
    fprintf(obj.fileMEX,'\n');
    fprintf(obj.fileMEX,'\n');
    fprintf(obj.fileMEX,'/**\n');
    fprintf(obj.fileMEX,'*    Author David Ariens, Rien Quirynen\n');
    fprintf(obj.fileMEX,'*    Date 2009-2013\n');
    fprintf(obj.fileMEX,'*    http://www.acadotoolkit.org/matlab \n');    
    fprintf(obj.fileMEX,'*/\n');
    fprintf(obj.fileMEX,'\n');
    fprintf(obj.fileMEX,'#include <acado_optimal_control.hpp>\n');
    fprintf(obj.fileMEX,'#include <acado_toolkit.hpp>\n');    
    fprintf(obj.fileMEX,'#include <acado/utils/matlab_acado_utils.hpp>\n');
    fprintf(obj.fileMEX,'\n');
    fprintf(obj.fileMEX,'USING_NAMESPACE_ACADO\n');
    fprintf(obj.fileMEX,'\n');
    
end