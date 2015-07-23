function [] = generate_code

Ts = 1e-3;   % 1 ms
EXPORT = 1;

base = 0.1;

DifferentialState v omega theta xC yC
Control aL aR
    
A1 = zeros(3,3);
A1(3,2) = 1.0;

B1 = zeros(3,2);
B1(1,1) = 0.5;
B1(1,2) = 0.5;
B1(2,1) = 1.0/base;
B1(2,2) = -1.0/base;

f = [ dot(xC) == v*cos(theta); dot(yC) == v*sin(theta) ];

%% SIMexport
fprintf('----------------------------\n         SIMexport         \n----------------------------\n');
sim = acado.SIMexport( Ts );
sim.setLinearInput( A1, B1 );
sim.setModel( f );
sim.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL4'   );
sim.set( 'NUM_INTEGRATOR_STEPS',        1               );

if EXPORT
    sim.exportCode('export_sim');
    
    cd export_sim
    make_acado_integrator('../integrate_robot')
    cd ..
end

x = [0; 0; pi/2; 0; 0];
xs = x; xs2 = x;
u = [10; -30]; 
N = 80;
t1 = 0; t2 = 0;
sim_input.u = u;
for i = 1:N
    sim_input.x = xs(:,end);
    tic
    states = integrate_robot(sim_input);
    t1 = t1+toc;
    xs(:,i+1) = states.value;
end

%% MPCexport
fprintf('----------------------------\n         MPCexport         \n----------------------------\n');
acadoSet('problemname', 'mpc');

N = 25;
n_XD = length(diffStates);
n_U = length(controls);
ocp = acado.OCP( 0.0, N*Ts, N );

W_mat = eye(n_XD+n_U,n_XD+n_U);
WN_mat = eye(n_XD,n_XD);
W = acado.BMatrix(W_mat);
WN = acado.BMatrix(WN_mat);

h = [diffStates; controls];
hN = [diffStates];

ocp.minimizeLSQ( W, h );
ocp.minimizeLSQEndTerm( WN, hN );

ocp.subjectTo( -100 <= controls <= 100 );
ocp.subjectTo( 0.0 <= v <= 0.3 );
ocp.subjectTo( -0.5 <= omega <= 0.5 );

ocp.setLinearInput( A1,B1 );
ocp.setModel( f );

mpc = acado.OCPexport( ocp );
mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
mpc.set( 'QP_SOLVER',                   'QP_QPOASES'    	);
mpc.set( 'HOTSTART_QP',                 'NO'             	);
mpc.set( 'LEVENBERG_MARQUARDT', 		 1e-10				);
mpc.set( 'CG_HARDCODE_CONSTRAINT_VALUES','NO' 				);
mpc.set( 'SPARSE_QP_SOLUTION',          'FULL_CONDENSING'   );
mpc.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL2'       );
mpc.set( 'NUM_INTEGRATOR_STEPS',        N                   );

if EXPORT
    mpc.exportCode( 'export_mpc' );
    copyfile('../../../../../../external_packages/qpoases', 'export_mpc/qpoases')
    
    cd export_mpc
    make_sfunction('../sfunction_robot')
    cd ..
end

end
