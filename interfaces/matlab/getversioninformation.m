CURRENT_VERSION = 2022;


fprintf (1, sprintf('\nVERSION INFORMATION ACADO FOR MATLAB: \n\n')) ;

fprintf (1, sprintf('ACADO VERSION:\n==================\n')) ;
CURRENT_VERSION

fprintf (1, sprintf('MEX:\n==================\n')) ;
mex.getCompilerConfigurations()

fprintf (1, sprintf('\n\nMATLAB:\n==================\n')) ;
ver matlab

fprintf (1, sprintf('\n\nCOMPUTER INFO:\n==================\n')) ;
computer