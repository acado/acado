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


global ACADO_;

if (get == 'B')
     
    fprintf(cppobj.fileMEX,sprintf('    SimulationEnvironment %s(%s, %s, %s, %s);\n', obj.name, stringIntDouble(obj.startTime), stringIntDouble(obj.endTime), obj.process.name, obj.controller.name));
    
    if (~isempty(obj.initvector))
        fprintf(cppobj.fileMEX,sprintf('     %s.init(%s);\n', obj.name, obj.initvector.name));
    end
    
    fprintf(cppobj.fileMEX,sprintf('    returnValue returnvalue = %s.run();\n', obj.name));
    
    fprintf(cppobj.fileMEX,'\n');
    fprintf(cppobj.fileMEX,'\n');

    
    fprintf(cppobj.fileMEX,'    VariablesGrid out_processout; \n');
    fprintf(cppobj.fileMEX,'    VariablesGrid out_feedbackcontrol; \n');
    fprintf(cppobj.fileMEX,'    VariablesGrid out_feedbackparameter; \n');
    fprintf(cppobj.fileMEX,'    VariablesGrid out_states; \n');
    fprintf(cppobj.fileMEX,'    VariablesGrid out_algstates; \n');
    
    fprintf(cppobj.fileMEX,sprintf('    %s.getSampledProcessOutput(out_processout);\n', obj.name));
    fprintf(cppobj.fileMEX,sprintf('    %s.getProcessDifferentialStates(out_states);\n', obj.name));
    if (~isempty(cppobj.u))
        fprintf(cppobj.fileMEX,sprintf('    %s.getFeedbackControl(out_feedbackcontrol);\n', obj.name));
    end
    if (~isempty(cppobj.p))
        fprintf(cppobj.fileMEX,sprintf('    %s.getFeedbackParameter(out_feedbackparameter);\n', obj.name));
    end
    if (~isempty(cppobj.z))
        fprintf(cppobj.fileMEX,sprintf('    %s.getProcessAlgebraicStates(out_algstates);\n', obj.name));
    end
    

    if (ACADO_.results_to_file == true)
        % Write output to files
        fprintf(cppobj.fileMEX,sprintf('    out_processout.print( "%s_OUT_states_sampled.m","STATES_SAMPLED",PS_MATLAB ); \n', cppobj.problemname));
        fprintf(cppobj.fileMEX,sprintf('    out_feedbackcontrol.print( "%s_OUT_controls.m","CONTROLS",PS_MATLAB ); \n', cppobj.problemname));
        fprintf(cppobj.fileMEX,sprintf('    out_feedbackparameter.print( "%s_OUT_parameters.m","PARAMETERS",PS_MATLAB ); \n', cppobj.problemname));
        fprintf(cppobj.fileMEX,sprintf('    out_states.print( "%s_OUT_states.m","STATES",PS_MATLAB ); \n', cppobj.problemname));
        fprintf(cppobj.fileMEX,sprintf('    out_algstates.print( "%s_OUT_algebraicstates.m","ALGEBRAICSTATES",PS_MATLAB ); \n', cppobj.problemname));
    end
    

    fprintf(cppobj.fileMEX,'    const char* outputFieldNames[] = {"STATES_SAMPLED", "CONTROLS", "PARAMETERS", "STATES", "ALGEBRAICSTATES", "CONVERGENCE_ACHIEVED"}; \n');
    fprintf(cppobj.fileMEX,'    plhs[0] = mxCreateStructMatrix( 1,1,6,outputFieldNames ); \n');
    
    cppobj.getCPPlefthandout('OutSS', 'outSS', 'out_processout')
    fprintf(cppobj.fileMEX,'    mxSetField( plhs[0],0,"STATES_SAMPLED",OutSS );\n');
    
    cppobj.getCPPlefthandout('OutS', 'outS', 'out_states')
    fprintf(cppobj.fileMEX,'    mxSetField( plhs[0],0,"STATES",OutS );\n');
    
    cppobj.getCPPlefthandout('OutC', 'outC', 'out_feedbackcontrol')
    fprintf(cppobj.fileMEX,'    mxSetField( plhs[0],0,"CONTROLS",OutC );\n');
    
    cppobj.getCPPlefthandout('OutP', 'outP', 'out_feedbackparameter')
    fprintf(cppobj.fileMEX,'    mxSetField( plhs[0],0,"PARAMETERS",OutP );\n');
 
    cppobj.getCPPlefthandout('OutZ', 'outZ', 'out_algstates')
    fprintf(cppobj.fileMEX,'    mxSetField( plhs[0],0,"ALGEBRAICSTATES",OutZ );\n');

    fprintf(cppobj.fileMEX,'    mxArray *OutConv = NULL;\n');
    fprintf(cppobj.fileMEX,'    if ( returnvalue == SUCCESSFUL_RETURN ) { OutConv = mxCreateDoubleScalar( 1 ); }else{ OutConv = mxCreateDoubleScalar( 0 ); } \n');
    fprintf(cppobj.fileMEX,'    mxSetField( plhs[0],0,"CONVERGENCE_ACHIEVED",OutConv );\n');	
    
    fprintf(cppobj.fileMEX,'\n');
   
end

end