function [ ] = makemex( mexfile, outputname, outputdir )
%Compile a MEX file containing ACADO code. Run the command 'makemex'
% directly within the directory <ACADOtoolkit-inst-dir>/interfaces/matlab/.
% See also the examples folder: examples/mexfiles
%
%  Usage:
%   >> makemex( mexfile, outputname, outputdir )
%  
%  Parameters:
%   mexfile     Full path to a CPP file, relative to the directory
%               <ACADOtoolkit-inst-dir>/interfaces/matlab/
%   outputdir   Full path to the directory where to store the compiled MEX-file
%               relative to: <ACADOtoolkit-inst-dir>/interfaces/matlab/
%   outputname  Name of the MEX-file without file extention. This
%               extention depends on the platform you are using and will be
%               added automatically
%
%  Example:
%    >> makemex('integrator/ACADOintegrators.cpp', 'ACADOintegrators', 'integrator/')
%    This call is used to generate the ACADO integrators MEX-file.
%
%    >> makemex('examples/mexfiles/empty.cpp', 'empty', 'examples/mexfiles/')
%    Compile the example file. Run it by going to the directory
%    "examples/mexfiles/" and by calling "empty()" in the command line.
%
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
%  see also make, makeintegrators, makeocp, makesimulation
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
%    \date 2010
% 

if verLessThan('matlab', '7.6.0')
    fprintf('\n--------------------------------------------------------\nWARNING: You are using a non supported Matlab version.\n-------------------------------------------------------- \n\nThe optimization interface (OCP, MPC, parameter estimation...) \nonly supports R2008a or newer. \nThe integrators can be ran on earlier versions. \nUse makeintegrators instead.\n\n');
    error('Stopping make...');
end

if (~exist(mexfile))
    error('The file %s does not exist or cannot be found. Check your path.', mexfile);
end

if (~exist(outputdir))
    error('The folder %s does not exist or cannot be found. Check your path.', outputdir);
end

optmake.mexfile    = mexfile;
optmake.outputdir  = outputdir;
optmake.outputname = outputname;

makehelper(0, optmake);


end