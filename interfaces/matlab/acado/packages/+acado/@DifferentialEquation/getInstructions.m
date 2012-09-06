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


%% ODE + DAE together  (shared code)
    if (~isempty(obj.matlabODE_fcnHandle) || ~isempty(obj.matlabDAE_fcnHandle))
        if (get == 'H')
            fprintf(cppobj.fileMEX,sprintf('mxArray* ModelFcn_%d_f = NULL;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('mxArray* ModelFcn_%d_jac = NULL;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('mxArray* ModelFcn_%dT  = NULL;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('mxArray* ModelFcn_%dX  = NULL;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('mxArray* ModelFcn_%dXA = NULL;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('mxArray* ModelFcn_%dU  = NULL;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('mxArray* ModelFcn_%dP  = NULL;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('mxArray* ModelFcn_%dW  = NULL;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('mxArray* ModelFcn_%dDX = NULL;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('unsigned int ModelFcn_%dNT  = 0;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('unsigned int ModelFcn_%dNX  = 0;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('unsigned int ModelFcn_%dNXA = 0;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('unsigned int ModelFcn_%dNU  = 0;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('unsigned int ModelFcn_%dNP  = 0;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('unsigned int ModelFcn_%dNW  = 0;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('unsigned int ModelFcn_%dNDX = 0;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('unsigned int jacobianNumber_%d = -1;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('double* f_store_%d             = NULL;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('double* J_store_%d             = NULL;\n\n', obj.matlablinkcount));

            fprintf(cppobj.fileMEX,sprintf('void clearAllGlobals%d( ){ \n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('    if ( f_store_%d != NULL ){\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        f_store_%d = NULL;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('    }\n\n'));
            fprintf(cppobj.fileMEX,sprintf('    if ( J_store_%d != NULL ){\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        J_store_%d = NULL;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('    }\n\n'));
            fprintf(cppobj.fileMEX,sprintf('    if ( ModelFcn_%d_f != NULL ){\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        mxDestroyArray( ModelFcn_%d_f );\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        ModelFcn_%d_f = NULL;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('    }\n\n'));
            fprintf(cppobj.fileMEX,sprintf('    if ( ModelFcn_%dT != NULL ){\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        mxDestroyArray( ModelFcn_%dT );\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        ModelFcn_%dT = NULL;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('    }\n\n'));
            fprintf(cppobj.fileMEX,sprintf('    if ( ModelFcn_%dX != NULL ){\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        mxDestroyArray( ModelFcn_%dX );\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        ModelFcn_%dX = NULL;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('    }\n\n'));
            fprintf(cppobj.fileMEX,sprintf('    if ( ModelFcn_%dXA != NULL ){\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        mxDestroyArray( ModelFcn_%dXA );\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        ModelFcn_%dXA = NULL;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('    }\n\n'));
            fprintf(cppobj.fileMEX,sprintf('    if ( ModelFcn_%dU != NULL ){\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        mxDestroyArray( ModelFcn_%dU );\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        ModelFcn_%dU = NULL;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('    }\n\n'));
            fprintf(cppobj.fileMEX,sprintf('    if ( ModelFcn_%dP != NULL ){\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        mxDestroyArray( ModelFcn_%dP );\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        ModelFcn_%dP = NULL;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('    }\n\n'));
            fprintf(cppobj.fileMEX,sprintf('    if ( ModelFcn_%dW != NULL ){\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        mxDestroyArray( ModelFcn_%dW );\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        ModelFcn_%dW = NULL;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('    }\n\n'));
            fprintf(cppobj.fileMEX,sprintf('    if ( ModelFcn_%dDX != NULL ){\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        mxDestroyArray( ModelFcn_%dDX );\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        ModelFcn_%dDX = NULL;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('    }\n\n'));
            fprintf(cppobj.fileMEX,sprintf('    if ( ModelFcn_%d_jac != NULL ){\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        mxDestroyArray( ModelFcn_%d_jac );\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        ModelFcn_%d_jac = NULL;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('    }\n\n'));
            fprintf(cppobj.fileMEX,sprintf('    ModelFcn_%dNT  = 0;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('    ModelFcn_%dNX  = 0;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('    ModelFcn_%dNXA = 0;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('    ModelFcn_%dNU  = 0;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('    ModelFcn_%dNP  = 0;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('    ModelFcn_%dNW  = 0;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('    ModelFcn_%dNDX = 0;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('    jacobianNumber_%d = -1;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('}\n\n'));
            
        end
        
        % BODY
        if (get == 'B')
            fprintf(cppobj.fileMEX,sprintf('    ModelFcn_%dT  = mxCreateDoubleMatrix( %d, 1,mxREAL );\n', obj.matlablinkcount, length(cppobj.t)));
            fprintf(cppobj.fileMEX,sprintf('    ModelFcn_%dX  = mxCreateDoubleMatrix( %d, 1,mxREAL );\n', obj.matlablinkcount, length(cppobj.x)));
            fprintf(cppobj.fileMEX,sprintf('    ModelFcn_%dXA = mxCreateDoubleMatrix( %d, 1,mxREAL );\n', obj.matlablinkcount, length(cppobj.z)));
            fprintf(cppobj.fileMEX,sprintf('    ModelFcn_%dDX = mxCreateDoubleMatrix( %d, 1,mxREAL );\n', obj.matlablinkcount, length(cppobj.x)));
            fprintf(cppobj.fileMEX,sprintf('    ModelFcn_%dU  = mxCreateDoubleMatrix( %d, 1,mxREAL );\n', obj.matlablinkcount, length(cppobj.u)));
            fprintf(cppobj.fileMEX,sprintf('    ModelFcn_%dP  = mxCreateDoubleMatrix( %d, 1,mxREAL );\n', obj.matlablinkcount, length(cppobj.p)));
            fprintf(cppobj.fileMEX,sprintf('    ModelFcn_%dW  = mxCreateDoubleMatrix( %d, 1,mxREAL );\n', obj.matlablinkcount, length(cppobj.w)));

            fprintf(cppobj.fileMEX,sprintf('    ModelFcn_%dNT  = %d;\n', obj.matlablinkcount, length(cppobj.t)));
            fprintf(cppobj.fileMEX,sprintf('    ModelFcn_%dNX  = %d;\n', obj.matlablinkcount, length(cppobj.x)));
            fprintf(cppobj.fileMEX,sprintf('    ModelFcn_%dNXA = %d;\n', obj.matlablinkcount, length(cppobj.z)));
            fprintf(cppobj.fileMEX,sprintf('    ModelFcn_%dNDX = %d;\n', obj.matlablinkcount, length(cppobj.x)));
            fprintf(cppobj.fileMEX,sprintf('    ModelFcn_%dNP  = %d;\n', obj.matlablinkcount, length(cppobj.p)));
            fprintf(cppobj.fileMEX,sprintf('    ModelFcn_%dNU  = %d;\n', obj.matlablinkcount, length(cppobj.u)));
            fprintf(cppobj.fileMEX,sprintf('    ModelFcn_%dNW  = %d;\n', obj.matlablinkcount, length(cppobj.w)));

            fprintf(cppobj.fileMEX,sprintf('    %s;\n',  obj.getHeader));
            
            if (~isempty(obj.matlabODE_fcnHandle))
                fprintf(cppobj.fileMEX,sprintf('    ModelFcn_%d_f = mxCreateString("%s");\n', obj.matlablinkcount, obj.matlabODE_fcnHandle));
            else
                fprintf(cppobj.fileMEX,sprintf('    ModelFcn_%d_f = mxCreateString("%s");\n', obj.matlablinkcount, obj.matlabDAE_fcnHandle));
            end
               
            fprintf(cppobj.fileMEX,sprintf('    IntermediateState setc_is_%d(%d);\n', obj.matlablinkcount, length(cppobj.t)+length(cppobj.x)+length(cppobj.z)+length(cppobj.u)+length(cppobj.p)+length(cppobj.w)));

            count_is = 0;
            fprintf(cppobj.fileMEX,sprintf('    setc_is_%d(%d) = %s;\n', obj.matlablinkcount, count_is, cppobj.t{1}.name)); count_is = count_is+1;

            for i=1:length(cppobj.x)
                fprintf(cppobj.fileMEX,sprintf('    setc_is_%d(%d) = %s;\n', obj.matlablinkcount, count_is, cppobj.x{i}.name)); count_is = count_is+1;
            end
            for i=1:length(cppobj.z)
                fprintf(cppobj.fileMEX,sprintf('    setc_is_%d(%d) = %s;\n', obj.matlablinkcount, count_is, cppobj.z{i}.name)); count_is = count_is+1;
            end
            for i=1:length(cppobj.u)
                fprintf(cppobj.fileMEX,sprintf('    setc_is_%d(%d) = %s;\n', obj.matlablinkcount, count_is, cppobj.u{i}.name)); count_is = count_is+1;
            end
            for i=1:length(cppobj.p)
                fprintf(cppobj.fileMEX,sprintf('    setc_is_%d(%d) = %s;\n', obj.matlablinkcount, count_is, cppobj.p{i}.name)); count_is = count_is+1;
            end
            for i=1:length(cppobj.w)
                fprintf(cppobj.fileMEX,sprintf('    setc_is_%d(%d) = %s;\n', obj.matlablinkcount, count_is, cppobj.w{i}.name)); count_is = count_is+1;
            end

            
            if (~isempty(obj.matlabODE_fcnHandle))
                if (isempty(obj.matlabJacobian_fcnHandle))
                    fprintf(cppobj.fileMEX,sprintf('    ModelFcn_%d_jac = %s;\n', obj.matlablinkcount, 'NULL'));
                    fprintf(cppobj.fileMEX,sprintf('    CFunction cLinkModel_%d( ModelFcn_%dNX, genericODE%d ); \n', obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount));     

                else
                    fprintf(cppobj.fileMEX,sprintf('    ModelFcn_%d_jac = mxCreateString("%s");\n', obj.matlablinkcount, obj.matlabJacobian_fcnHandle));
                    fprintf(cppobj.fileMEX,sprintf('    CFunction cLinkModel_%d( ModelFcn_%dNX, genericODE%d, genericJacobian%d, genericJacobian%d ); \n', obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount));                

                end               
                
            elseif (~isempty(obj.matlabDAE_fcnHandle))
                fprintf(cppobj.fileMEX,sprintf('    CFunction cLinkModel_%d( ModelFcn_%dNX+ModelFcn_%dNXA, genericDAE%d ); \n', obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount));                

            end

            fprintf(cppobj.fileMEX,sprintf('    %s << cLinkModel_%d(setc_is_%d); \n\n', obj.name, obj.matlablinkcount, obj.matlablinkcount));
        end    
        
        
        %FOOTER
        if (get == 'F')
            fprintf(cppobj.fileMEX,sprintf('    clearAllGlobals%d( ); \n', obj.matlablinkcount));
        end

    end

%% ODE
    if (~isempty(obj.matlabODE_fcnHandle))
        % MATLAB LINK 
        
        % HEADER
        if (get == 'H')
            fprintf(cppobj.fileMEX,sprintf('void genericODE%d( double* x, double* f, void *userData ){\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('    unsigned int i;\n'));
            fprintf(cppobj.fileMEX,sprintf('    double* tt = mxGetPr( ModelFcn_%dT );\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('    tt[0] = x[0];\n'));
            fprintf(cppobj.fileMEX,sprintf('    double* xx = mxGetPr( ModelFcn_%dX );\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('    for( i=0; i<ModelFcn_%dNX; ++i )\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        xx[i] = x[i+1];\n'));
            fprintf(cppobj.fileMEX,sprintf('    double* uu = mxGetPr( ModelFcn_%dU );\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('    for( i=0; i<ModelFcn_%dNU; ++i )\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        uu[i] = x[i+1+ModelFcn_%dNX];\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('    double* pp = mxGetPr( ModelFcn_%dP );\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('    for( i=0; i<ModelFcn_%dNP; ++i )\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        pp[i] = x[i+1+ModelFcn_%dNX+ModelFcn_%dNU];\n', obj.matlablinkcount, obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('    double* ww = mxGetPr( ModelFcn_%dW );\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('    for( i=0; i<ModelFcn_%dNW; ++i )\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        ww[i] = x[i+1+ModelFcn_%dNX+ModelFcn_%dNU+ModelFcn_%dNP];\n', obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('    mxArray* FF = NULL;\n'));
            fprintf(cppobj.fileMEX,sprintf('    mxArray* argIn[]  = { ModelFcn_%d_f,ModelFcn_%dT,ModelFcn_%dX,ModelFcn_%dU,ModelFcn_%dP,ModelFcn_%dW };\n', obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('    mxArray* argOut[] = { FF };\n\n'));
            fprintf(cppobj.fileMEX,sprintf('    mexCallMATLAB( 1,argOut, 6,argIn,"generic_ode" );\n'));
            fprintf(cppobj.fileMEX,sprintf('    double* ff = mxGetPr( *argOut );\n'));
            fprintf(cppobj.fileMEX,sprintf('    for( i=0; i<ModelFcn_%dNX; ++i ){\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        f[i] = ff[i];\n'));
            fprintf(cppobj.fileMEX,sprintf('    }\n'));
            fprintf(cppobj.fileMEX,sprintf('    mxDestroyArray( *argOut );\n'));
            fprintf(cppobj.fileMEX,sprintf('}\n\n'));

            fprintf(cppobj.fileMEX,sprintf('void genericJacobian%d( int number, double* x, double* seed, double* f, double* df, void *userData  ){\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('    unsigned int i, j;\n'));
            fprintf(cppobj.fileMEX,sprintf('    double* ff;\n'));
            fprintf(cppobj.fileMEX,sprintf('    double* J;\n'));
            fprintf(cppobj.fileMEX,sprintf('    if (J_store_%d == NULL){\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        J_store_%d = (double*) calloc ((ModelFcn_%dNX+ModelFcn_%dNU+ModelFcn_%dNP+ModelFcn_%dNW)*(ModelFcn_%dNX),sizeof(double));\n', obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        f_store_%d = (double*) calloc (ModelFcn_%dNX,sizeof(double));\n', obj.matlablinkcount, obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('    }\n'));
            fprintf(cppobj.fileMEX,sprintf('    if ( (int) jacobianNumber_%d == number){\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        J = J_store_%d;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        ff = f_store_%d;\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        for( i=0; i<ModelFcn_%dNX; ++i ) {\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('            df[i] = 0;\n'));
            fprintf(cppobj.fileMEX,sprintf('            f[i] = 0;\n'));
            fprintf(cppobj.fileMEX,sprintf('            for (j=0; j < ModelFcn_%dNX+ModelFcn_%dNU+ModelFcn_%dNP+ModelFcn_%dNW; ++j){\n', obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('                df[i] += J[(j*(ModelFcn_%dNX))+i]*seed[j+1]; \n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('            }\n'));
            fprintf(cppobj.fileMEX,sprintf('        }\n'));
            fprintf(cppobj.fileMEX,sprintf('        for( i=0; i<ModelFcn_%dNX; ++i ){\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('            f[i] = ff[i];\n'));
            fprintf(cppobj.fileMEX,sprintf('        }\n'));
            fprintf(cppobj.fileMEX,sprintf('    }else{\n'));
            fprintf(cppobj.fileMEX,sprintf('        jacobianNumber_%d = number; \n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        double* tt = mxGetPr( ModelFcn_%dT );\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        tt[0] = x[0];\n'));
            fprintf(cppobj.fileMEX,sprintf('        double* xx = mxGetPr( ModelFcn_%dX );\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        for( i=0; i<ModelFcn_%dNX; ++i )\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('            xx[i] = x[i+1];\n'));
            fprintf(cppobj.fileMEX,sprintf('        double* uu = mxGetPr( ModelFcn_%dU );\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        for( i=0; i<ModelFcn_%dNU; ++i )\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('            uu[i] = x[i+1+ModelFcn_%dNX];\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        double* pp = mxGetPr( ModelFcn_%dP );\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        for( i=0; i<ModelFcn_%dNP; ++i )\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('            pp[i] = x[i+1+ModelFcn_%dNX+ModelFcn_%dNU];\n', obj.matlablinkcount, obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        double* ww = mxGetPr( ModelFcn_%dW );\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('            for( i=0; i<ModelFcn_%dNW; ++i )\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        ww[i] = x[i+1+ModelFcn_%dNX+ModelFcn_%dNU+ModelFcn_%dNP];\n', obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        mxArray* FF = NULL;\n'));
            fprintf(cppobj.fileMEX,sprintf('        mxArray* argIn[]  = { ModelFcn_%d_jac,ModelFcn_%dT,ModelFcn_%dX,ModelFcn_%dU,ModelFcn_%dP,ModelFcn_%dW };\n', obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        mxArray* argOut[] = { FF };\n'));
            fprintf(cppobj.fileMEX,sprintf('        mexCallMATLAB( 1,argOut, 6,argIn,"generic_jacobian" );\n'));
            fprintf(cppobj.fileMEX,sprintf('        unsigned int rowLen = mxGetM(*argOut);\n'));
            fprintf(cppobj.fileMEX,sprintf('        unsigned int colLen = mxGetN(*argOut);\n'));
            fprintf(cppobj.fileMEX,sprintf('        if (rowLen != ModelFcn_%dNX){\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('            mexErrMsgTxt( "ERROR: Jacobian matrix rows do not match (should be ModelFcn_%dNX). " );\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        }\n'));
            fprintf(cppobj.fileMEX,sprintf('        if (colLen != ModelFcn_%dNX+ModelFcn_%dNU+ModelFcn_%dNP+ModelFcn_%dNW){\n', obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('            mexErrMsgTxt( "ERROR: Jacobian matrix columns do not match (should be ModelFcn_%dNX+ModelFcn_%dNU+ModelFcn_%dNP+ModelFcn_%dNW). " );\n', obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        }\n'));
            fprintf(cppobj.fileMEX,sprintf('        J = mxGetPr( *argOut );\n'));
            fprintf(cppobj.fileMEX,sprintf('        memcpy(J_store_%d, J, (ModelFcn_%dNX+ModelFcn_%dNU+ModelFcn_%dNP+ModelFcn_%dNW)*(ModelFcn_%dNX) * sizeof ( double ));\n', obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        for( i=0; i<ModelFcn_%dNX; ++i ) {\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('            df[i] = 0;\n'));
            fprintf(cppobj.fileMEX,sprintf('            f[i] = 0;\n'));
            fprintf(cppobj.fileMEX,sprintf('            for (j=0; j < ModelFcn_%dNX+ModelFcn_%dNU+ModelFcn_%dNP+ModelFcn_%dNW; ++j){\n', obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('                df[i] += J[(j*(ModelFcn_%dNX))+i]*seed[j+1];\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('            }\n'));
            fprintf(cppobj.fileMEX,sprintf('        }\n'));
            fprintf(cppobj.fileMEX,sprintf('        mxArray* FF2 = NULL;\n'));
            fprintf(cppobj.fileMEX,sprintf('        mxArray* argIn2[]  = { ModelFcn_%d_f,ModelFcn_%dT,ModelFcn_%dX,ModelFcn_%dU,ModelFcn_%dP,ModelFcn_%dW };\n', obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        mxArray* argOut2[] = { FF2 };\n'));
            fprintf(cppobj.fileMEX,sprintf('        mexCallMATLAB( 1,argOut2, 6,argIn2,"generic_ode" );\n'));
            fprintf(cppobj.fileMEX,sprintf('        ff = mxGetPr( *argOut2 );\n'));
            fprintf(cppobj.fileMEX,sprintf('        memcpy(f_store_%d, ff, (ModelFcn_%dNX) * sizeof ( double ));\n', obj.matlablinkcount, obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('        for( i=0; i<ModelFcn_%dNX; ++i ){\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('            f[i] = ff[i];\n'));
            fprintf(cppobj.fileMEX,sprintf('        }\n'));
            fprintf(cppobj.fileMEX,sprintf('        mxDestroyArray( *argOut );\n'));
            fprintf(cppobj.fileMEX,sprintf('        mxDestroyArray( *argOut2 );\n'));
            fprintf(cppobj.fileMEX,sprintf('    }\n'));
            fprintf(cppobj.fileMEX,sprintf('}\n'));
        end
    

    end     

%% DAE
    if (~isempty(obj.matlabDAE_fcnHandle))
        % MATLAB LINK 
        
        % HEADER
        if (get == 'H')
            fprintf(cppobj.fileMEX,sprintf('void genericDAE%d( double* x, double* f, void *userData ){\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('	unsigned int i;\n\n'));
            fprintf(cppobj.fileMEX,sprintf('	double* tt = mxGetPr( ModelFcn_%dT );\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('	tt[0] = x[0];\n\n'));
            fprintf(cppobj.fileMEX,sprintf('	double* xx = mxGetPr( ModelFcn_%dX );\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('	for( i=0; i<ModelFcn_%dNX; ++i )\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('		xx[i] = x[i+1];\n\n'));
            fprintf(cppobj.fileMEX,sprintf('	double* xxa = mxGetPr( ModelFcn_%dXA );\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('	for( i=0; i<ModelFcn_%dNXA; ++i )\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('		xxa[i] = x[i+1+ModelFcn_%dNX];\n\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('	double* uu = mxGetPr( ModelFcn_%dU );\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('	for( i=0; i<ModelFcn_%dNU; ++i )\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('		uu[i] = x[i+1+ModelFcn_%dNX+ModelFcn_%dNXA];\n\n', obj.matlablinkcount, obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('	double* pp = mxGetPr( ModelFcn_%dP );\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('	for( i=0; i<ModelFcn_%dNP; ++i )\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('		pp[i] = x[i+1+ModelFcn_%dNX+ModelFcn_%dNXA+ModelFcn_%dNU];\n\n', obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('	double* ww = mxGetPr( ModelFcn_%dW );\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('	for( i=0; i<ModelFcn_%dNW; ++i )\n', obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('		ww[i] = x[i+1+ModelFcn_%dNX+ModelFcn_%dNXA+ModelFcn_%dNU+ModelFcn_%dNP];\n\n', obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('	mxArray* argIn_f[]  = { ModelFcn_%d_f,ModelFcn_%dT,ModelFcn_%dX,ModelFcn_%dXA,ModelFcn_%dU,ModelFcn_%dP,ModelFcn_%dW }; \n\n', obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount, obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('	mxArray* FF = NULL;\n'));
            fprintf(cppobj.fileMEX,sprintf('	double*  ff = NULL;\n'));
            fprintf(cppobj.fileMEX,sprintf('	mxArray* argOut[] = { FF };\n'));
            fprintf(cppobj.fileMEX,sprintf('    mexCallMATLAB( 1,argOut, 7,argIn_f,"generic_dae" );\n'));
            fprintf(cppobj.fileMEX,sprintf('	ff = mxGetPr( *argOut );\n'));
            fprintf(cppobj.fileMEX,sprintf('	for( i=0; i<ModelFcn_%dNX+ModelFcn_%dNXA; ++i )\n', obj.matlablinkcount, obj.matlablinkcount));
            fprintf(cppobj.fileMEX,sprintf('		f[i] = ff[i];\n'));
            fprintf(cppobj.fileMEX,sprintf('	mxDestroyArray( *argOut );\n'));
            fprintf(cppobj.fileMEX,sprintf('}\n\n'));
        end
        
    end   
        
%% C FILE
    if (~isempty(obj.cfunction_file))
        % C LINK 
        
        if (get == 'H')
            fprintf(cppobj.fileMEX,sprintf('#include "%s"\n', obj.cfunction_file));
        end
               
        if (get == 'B')
            fprintf(cppobj.fileMEX,sprintf('    %s;\n', obj.getHeader));
            
            fprintf(cppobj.fileMEX,sprintf('    IntermediateState setc_is_%d(%d);\n', obj.matlablinkcount, length(cppobj.t)+length(cppobj.x)+length(cppobj.u)+length(cppobj.z)+length(cppobj.p)+length(cppobj.w)));

            count_is = 0;
            fprintf(cppobj.fileMEX,sprintf('    setc_is_%d(%d) = %s;\n', obj.matlablinkcount, count_is, cppobj.t{1}.name)); count_is = count_is+1;

            for i=1:length(cppobj.x)
                fprintf(cppobj.fileMEX,sprintf('    setc_is_%d(%d) = %s;\n', obj.matlablinkcount, count_is, cppobj.x{i}.name)); count_is = count_is+1;
            end
            for i=1:length(cppobj.z)
                fprintf(cppobj.fileMEX,sprintf('    setc_is_%d(%d) = %s;\n', obj.matlablinkcount, count_is, cppobj.z{i}.name)); count_is = count_is+1;
            end
            for i=1:length(cppobj.u)
                fprintf(cppobj.fileMEX,sprintf('    setc_is_%d(%d) = %s;\n', obj.matlablinkcount, count_is, cppobj.u{i}.name)); count_is = count_is+1;
            end
            for i=1:length(cppobj.p)
                fprintf(cppobj.fileMEX,sprintf('    setc_is_%d(%d) = %s;\n', obj.matlablinkcount, count_is, cppobj.p{i}.name)); count_is = count_is+1;
            end
            for i=1:length(cppobj.w)
                fprintf(cppobj.fileMEX,sprintf('    setc_is_%d(%d) = %s;\n', obj.matlablinkcount, count_is, cppobj.w{i}.name)); count_is = count_is+1;
            end      
            
            
            fprintf(cppobj.fileMEX,sprintf('    CFunction cLinkModel_%d( %d, %s ); \n', obj.matlablinkcount, length(cppobj.x)+length(cppobj.z), obj.cfunction_function));
            
            fprintf(cppobj.fileMEX,sprintf('    %s << cLinkModel_%d(setc_is_%d); \n\n', obj.name, obj.matlablinkcount, obj.matlablinkcount));
            
        end
    end
    
%% ACADO FUNCTION
    if (~isempty(obj.differentialList))
       % FUNCTION DEFINED
       if (get == 'B')

            fprintf(cppobj.fileMEX,sprintf('    %s;\n', obj.getHeader));


            for i=1:length(obj.differentialList)
                fprintf(cppobj.fileMEX,sprintf('    %s << %s;\n', obj.name, obj.differentialList{i}.toString()));
            end

            fprintf(cppobj.fileMEX,'\n');
       end
    end
    
        
%% ERROR
    if (isempty(obj.differentialList) && isempty(obj.cfunction_file) && isempty(obj.matlabODE_fcnHandle) && isempty(obj.matlabDAE_fcnHandle))
        error('No differential equation found');
    end
    
end