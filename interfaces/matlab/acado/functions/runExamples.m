clc;
clear all;

delete('log_examples.txt')
exceptions = {'_RUN', 'export', 'OUT'};
broken = {'liebouds_system.m', 'threedof_robot.m'};

fileList = getFilesAndObjectNames( '../../examples', {'*.m'} );

for i = 1:length(exceptions)
    I = find( cellfun( @numel, regexp( fileList, exceptions{i} ) ) == 0 );
    fileList = fileList(I);
end
for i = 1:length(broken)
    I = find( cellfun( @numel, regexp( fileList, broken{i} ) ) == 0 );
    fileList = fileList(I);
end

diary('log_examples.txt')
fprintf(['-----------------------------------------------------------------------------------------\n STARTING EXAMPLE RUN: ' num2str(length(fileList)) ' files found.. \n-----------------------------------------------------------------------------------------\n']);
numS = 0; numF = 0; numO = 0;
for i = 1:length(fileList)
    fprintf(['-----------------------------------------------------------------------------------------\n EXAMPLE: ' fileList{i} '...\n']);
    try
        save('tmp')
        clc
        eval(['run ' fileList{i}])
        
        clear all
        load('tmp')
        numS = numS+1;
        save('tmp')
        
        fprintf('... ran succesfully!\n');
    catch exception
        if strcmp(exception.identifier, 'MATLAB:minrhs') || strcmp(exception.identifier, 'MATLAB:undefinedVarOrClass')
            clear all
            load('tmp')
            numF = numF+1;
            save('tmp')
            
            disp('This file is not a script.')
        else
            exception
            clear all
            load('tmp')
            numO = numO+1;
            save('tmp')
        end
    end
    clear all
    close all
    load('tmp')
    fprintf(['-----------------------------------------------------------------------------------------\n']);
end
fprintf(['-----------------------------------------------------------------------------------------\n END EXAMPLE RUN: ' num2str(numS) ' out of ' num2str(length(fileList)) ' files were run succesfully!\n']);
disp([' number of example scripts: ' num2str(numS+numO)])
disp([' number of failed calls   : ' num2str(numO)])
disp([' number of functions found: ' num2str(numF)])
disp('-----------------------------------------------------------------------------------------')
diary off
