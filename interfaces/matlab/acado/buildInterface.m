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
    if num ~= length(h.mainList)
        error('Internal error: contact developer ACADO Matlab interface for info.')
    end
    if num > 0
        disp(' ');
        disp('COMPILATION OF MEX FILES...');
    end
    for i = 1:num
        m = h.mexList{i};
        out = h.mexOutList{i};
        dirName = m{1};
        [files, objs] = getFilesAndObjectNames( dirName, {'*.c', '*.cpp'}, {'EXAMPLES'} );
        k = 1;
        while k <= length(files)
            found = 0;
            for j = 1:length(m)-1
                mexF = fullfile(dirName, m{1+j});
                if strcmp(files(k), mexF)
                    files = [files(1:k-1); files(k+1:end)];
                    objs = [objs(1:k-1) objs(k+1:end)];
                    found = 1;
                    break;
                end
            end
            if ~found
                main = h.mainList{i};
                for j = 1:length(main)-1
                    mainF = fullfile(dirName, main{1+j});
                    if strcmp(files(k), mainF)
                        files = [files(1:k-1); files(k+1:end)];
                        objs = [objs(1:k-1) objs(k+1:end)];
                        found = 1;
                        break;
                    end
                end
            end
            k = k + (~found);
        end
        
        dirData = dir(dirName);
        dirIndex = [dirData.isdir];
        subDirs = {dirData(dirIndex).name};
        validIndex = ~ismember(subDirs, {'.', '..', '.svn'});
        subDirs = subDirs(validIndex);
        for j = 1:length(subDirs)
           subDirs{j} = fullfile(dirName, subDirs{j}); 
        end
        newDirs = subDirs;
        while ~isempty(newDirs)
            newSubs = {};
            for j = 1:length(newDirs)
                dirData = dir(newDirs{j});
                dirIndex = [dirData.isdir];
                tmp = {dirData(dirIndex).name};
                validIndex = ~ismember(tmp, {'.', '..', '.svn'});
                tmp = tmp(validIndex);
                for k = 1:length(tmp)
                    tmp{k} = fullfile(newDirs{j}, tmp{k});
                end
                newSubs = {newSubs{:}, tmp{:}};
            end
            subDirs = {subDirs{:}, newSubs{:}};
            newDirs = newSubs;
        end
        
        for j = 1:length(m)-1
            p = strfind(m{1+j},'.');
            if isempty(out)
                output = [m{1+j}(1:p-1) '.' mexext];
            else
                output = [out{j} '.' mexext];
            end
            mexF = fullfile(dirName, m{1+j});
            cmd = sprintf('mex COPTIMFLAGS=''-DNDEBUG -O3'' -output %s %s', output, mexF);
            for k = 1:length(subDirs)
                cmd = [cmd sprintf(' -I./%s', subDirs{k})];
            end
            for k = 1:length(files)
                cmd = [cmd sprintf(' %s', files{k})];
            end
            disp(cmd)
            eval(cmd)
        end
    end
    disp(' ');
    h.clearMEX;
    h.clearMain;
end

end
