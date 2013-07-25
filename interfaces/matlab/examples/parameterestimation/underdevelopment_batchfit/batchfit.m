clear;

BEGIN_ACADO;                                % Always start with "BEGIN_ACADO". 
    
    %% WARNING ------------------------------------------------------------
    error('This example should only be used to understand the syntax. The content as such is still under devellopment and does not converge.');
    %% --------------------------------------------------------------------
    
    acadoSet('problemname', 'batchfit');
    
    %Author: Julian Bonilla, Astrid Cappuyns, Hans Joachim Ferreau, Boris Houska, Filip Logist
    
    %% INTRODUCE THE VARIABLES:
    DifferentialState      cx;    % concentration 1
    DifferentialState      cs;    % concentration 2
    Parameter           mumax;    % the three
    Parameter             Yxs;    % parameters to be
    Parameter              km;    % estimated.
        
    
    %% DEFINE A DIFFERENTIAL EQUATION:
    f = acado.DifferentialEquation();  
    f.add(dot(cx) == (mumax*cs/(cs+km))*cx); 
    f.add(dot(cs) == -(mumax*cs/(cs+km))/Yxs*cx);
    
    
    %% MEASUREMENT DATA
    measurementdata;

    
    %% DEFINE A MEASUREMENT FUNCTION
    h={cx cs};   %first and second concentration are being measured.

    
    %% DEFINE A PARAMETER ESTIMATION PROBLEM:
    ocp = acado.OCP(M(:,1)');
    ocp.minimizeLSQ( h, M );
    ocp.subjectTo( f );
    ocp.subjectTo( -0.001 <=  cs    <= 3.0 );
    ocp.subjectTo( -0.001 <=  cx    <= 3.0 );
    ocp.subjectTo(  0.0 <=  km    <= 5.0 );
    ocp.subjectTo(  0.0 <=  Yxs   <= 5.0 );
    ocp.subjectTo(  0.1 <=  mumax <= 5.0 );
    
    
    %% DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
    algo = acado.ParameterEstimationAlgorithm(ocp);
    algo.initializeDifferentialStates(M);
    algo.set( 'INTEGRATOR_TYPE', 'INT_BDF' );
    algo.set( 'KKT_TOLERANCE', 1e-3 );
    algo.set( 'ABSOLUTE_TOLERANCE', 1e-5 );
    algo.set( 'INTEGRATOR_TOLERANCE', 1e-7 );

    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.
                     

% Run
out = batchfit_RUN();
