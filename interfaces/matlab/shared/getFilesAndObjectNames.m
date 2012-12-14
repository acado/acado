function [fileList, objList] = getFilesAndObjectNames( dirName, varargin )

    exclude = {};
    if nargin == 1
        search = {'*.cpp'};
    else
        search = varargin{1};
        if nargin > 2
           exclude = varargin{2};
        end
    end
	% Get the data for the current directory
	dirData = dir(dirName);
	% Find the index for directories
	dirIndex = [dirData.isdir];
    
	% Get a list of the files
    fileList = {};
    for i = 1:length(search)
        fileListTemp = dir(fullfile(dirName, search{i}));
        fileList = { fileList{:}, fileListTemp( : ).name }';
    end
	
	objList = {};
	for i = 1: length( fileList )
		[~, name, ~] = fileparts( fileList{ i } );
		objList{ i } = name; 
	end;
  
	if ~isempty( fileList )
		% Prepend path to files
		[fileList] = cellfun(@( x ) fullfile(dirName, x), ...  
						fileList, 'UniformOutput', false);
	end;
	
	% Get a list of the subdirectories
	subDirs = { dirData(dirIndex).name };
	% Find index of subdirectories that are not '.' or '..'
	validIndex = ~ismember(subDirs, {'.', '..', '.svn', exclude{:}});  
                                               
 	% Loop over valid subdirectories
 	for iDir = find( validIndex )
 		% Get the subdirectory path
		nextDir = fullfile(dirName, subDirs{ iDir });
		% Recursively call getFilesAndObjectNames
		[fl, ol] = getFilesAndObjectNames( nextDir, search, exclude ); 
		fileList = [fileList; fl];
		objList  = [objList  ol];
	end;
	
end