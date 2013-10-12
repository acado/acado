% Licence:
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
% 
function buildInterface

global ACADO_
if ~isempty(ACADO_)
    h = ACADO_.helper;
    num = length(h.mexList);
    
    disp(' ');
    disp('COMPILATION OF MEX FILES...');
%     d = dir('.');
%     isub = [d(:).isdir];
%     nameFolds = {d(isub).name}';
%     nameFolds(ismember(nameFolds,{'.','..'})) = []
    
    num_files = 0;
    for i = 1:num
        dir = h.mexList{i};
        string = [char(dir) '/make_*.m'];
        [files, objs] = getFilesAndObjectNames( '.', {string} );
        for j = 1:length(files)
            disp([char(files(j)) '...']);
            run([char(dir) '/' char(files(j))]);
            num_files = num_files + 1;
        end
    end
        
    disp(['COMPILATION DONE (' num2str(num_files) ' files)!']);
    disp(' ');
    h.clearMEX;
end

end
