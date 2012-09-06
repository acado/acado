function [ ] = makeocp( varargin )
%  Make ACADO OCP. Run the command 'makeocp' directly within the directory <ACADOtoolkit-inst-dir>/interfaces/matlab/
%
%  Usage:
%   >>makeocp                    Make changed files only, no debugging, no cleaning
%   >>makeocp clean              Clean all object and mex files
%   >>makeocp debug              Make changed files only in debug mode
%   >>makeocp all                Force all files to be maked again
%   >>makeocp all debug          Force all files to be maked again in debug mode
%   >>makeocp clean all          First clean, then make all files
%   >>makeocp clean all debug    First clean, then make all files in debug mode
%  
%  All information regarding installation available at http://www.acadotoolkit.org/matlab
%
%  You need to setup mex before you can run this make script. Run 
%   >>mex -setup
%  in your console if you haven't used mex yet. 
%
%  When using Windows, we encourage you to install Microsoft Visual C++
%  (express) compiler first. After installation, run "mex -setup" and
%  select the Microsoft Visual C++ compiler as your default compiler. This
%  compiler can be download free of charge on
%  http://www.microsoft.com/Express/VC/.
%
%  When running Linux/Mac OS, select the GCC compiler. 
%
%
%  After having run makeocp, all ACADO files are compiled and stored in the bin
%  directory. Your paths are also altered to reference the ACADO folders.
%  If you close are reopen Matlab you need to rerun this file because your
%  path settings will be lost. As long as you do not delete the content of
%  the bin folder, this should only take a few seconds because all files
%  are unchanged. If you prefer not to rerun this file. Run 
%    >>savepath
%  in your console. This will store the current path for future sessions.
%
%
%  Example:
%   >>makeocp; 
%   <hit enter and wait for the compilation to end. This will take a few minutes>
%
%  see also make, makeintegrators, makesimulation, makemex
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
%    \author David Ariens
%    \date 2009
% 

    if verLessThan('matlab', '7.6.0')
        fprintf('\n--------------------------------------------------------\nWARNING: You are using a non supported Matlab version.\n-------------------------------------------------------- \n\nThe optimization interface (OCP, MPC, parameter estimation...) \nonly supports R2008a or newer. \nThe integrators can be ran on earlier versions. \nUse makeintegrators instead.\n\n');
        error('Stopping make...');
    end

    
if (nargin ~= 0)
    makehelper(2, {}, varargin);
else
    makehelper(2, {});
end

end