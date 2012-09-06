function [ ] = automatic_model_detection_integrator( )
% This function generates the integrator/model_include.hpp file based on integrator/MODEL_INCLUDE.m
%
%  Usage:
%    This file is used in make.m.
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
%    \author Boris Houska, Niels Haverbeke, Hans Joachim Ferreau
%    \date 2008
%
%    \author David Ariens
%    \date 2009-2010
% 

	INCLUDELIST    = MODEL_INCLUDE();

	file = fopen('integrator/model_include.hpp','w');

    
    
	fprintf(file,'/*\n');
    fprintf(file,' *    !! DO NOT CHANGE THIS FILE. ALL CHANGES MADE WILL BE ERRASED WHEN COMPILING !! \n');
    fprintf(file,' *               ADD NEW FILES TO COMPILE TO matlab/MODEL_INCLUDE.m  \n');
    fprintf(file,' *\n');
    fprintf(file,' *\n');
	fprintf(file,' *    This file is part of ACADO Toolkit.\n');
	fprintf(file,' *\n');
	fprintf(file,' *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.\n');
	fprintf(file,' *    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau.\n');
	fprintf(file,' *    All rights reserved.\n');
	fprintf(file,' *\n');
	fprintf(file,' *    ACADO Toolkit is free software; you can redistribute it and/or\n');
	fprintf(file,' *    modify it under the terms of the GNU Lesser General Public\n');
	fprintf(file,' *    License as published by the Free Software Foundation; either\n');
	fprintf(file,' *    version 3 of the License, or (at your option) any later version.\n');
	fprintf(file,' *\n');
	fprintf(file,' *    ACADO Toolkit is distributed in the hope that it will be useful,\n');
	fprintf(file,' *    but WITHOUT ANY WARRANTY; without even the implied warranty of\n');
	fprintf(file,' *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU\n');
	fprintf(file,' *    Lesser General Public License for more details.\n');
	fprintf(file,' *\n');
	fprintf(file,' *    You should have received a copy of the GNU Lesser General Public\n');
	fprintf(file,' *    License along with ACADO Toolkit; if not, write to the Free Software\n');
	fprintf(file,' *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA\n');
	fprintf(file,' *\n');
	fprintf(file,' */\n');
	fprintf(file,'\n');
	fprintf(file,'\n');
	fprintf(file,'/**\n');
	fprintf(file,' *   \\file interfaces/matlab/integrator/model_include.hpp\n');
	fprintf(file,' *   \\author Boris Houska, Niels Haverbeke, Hans Joachim Ferreau, David Ariens\n');
	fprintf(file,' *   \\date 2008-2010\n');
	fprintf(file,' */\n');
	fprintf(file,'\n');
	fprintf(file,'\n');
	fprintf(file,'\n');
	fprintf(file,'// LIST OF ALL MODELS (CPP FILES):\n');
	fprintf(file,'// -------------------------------\n');

    
	numberOfModels = length(INCLUDELIST.MODELFILES1);
    numberOfOtherFiles = length(INCLUDELIST.OTHERFILES);

    
    for i=1:numberOfOtherFiles
	    fprintf(file,'   #include "%s"\n',INCLUDELIST.OTHERFILES{i});
    end
    
	for i=1:numberOfModels
	    fprintf(file,'   #include "%s"\n',INCLUDELIST.MODELFILES1{i});
    end


	fprintf(file,'\n');
	fprintf(file,'\n');
	fprintf(file,'// FUNCTION POINTER ALLOCATION:\n');
	fprintf(file,'// ----------------------------\n');
	fprintf(file,'\n');
	fprintf(file,'\n');
	fprintf(file,'void (*allocateFunctionPointer( char *name ))(DifferentialEquation*){\n');
	fprintf(file,'\n');
	fprintf(file,'    const int numberOfModels = %d;\n',numberOfModels);
	fprintf(file,'\n');
	fprintf(file,'    const char *modelList[numberOfModels];\n');
	fprintf(file,'    void (*modelFcn[numberOfModels])( DifferentialEquation* );\n');
	fprintf(file,'\n');

	for i=1:numberOfModels

	    fprintf(file,'    modelList[%d] = "%s";\n', i-1, INCLUDELIST.MODELFILES2{i});
	    fprintf(file,'    modelFcn [%d] = &%s ;\n', i-1, INCLUDELIST.MODELFILES2{i});
	end

	fprintf(file,'\n');
	fprintf(file,'    int run1 = 0;\n');
	fprintf(file,'    while( run1 < numberOfModels ){\n');
	fprintf(file,'\n');
	fprintf(file,'         if( strcmp(name,modelList[run1]) == 0 ){\n');
	fprintf(file,'             return modelFcn[run1];\n');
	fprintf(file,'         }\n');
	fprintf(file,'        run1++;\n');
	fprintf(file,'    }\n');
	fprintf(file,'    return 0;\n');
	fprintf(file,'}\n');
	fprintf(file,'\n');
	fprintf(file,'\n');

	fclose(file);

end
