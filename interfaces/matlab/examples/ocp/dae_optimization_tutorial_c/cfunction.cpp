void myAcadoDifferentialEquation( double *xx, double *f, void *user_data ){

    // x[0] -> time t
    // x[1] -> v
    // x[2] -> s
    // x[3] -> m
    // x[4] -> L
    // x[5] -> u
    
    double t = xx[0];
    double x = xx[1];
    double l = xx[2];
    double z = xx[3];
    double u = xx[4];

    
    
    f[0] = -x + 0.5*x*x + u + 0.5*z; 
    f[1] =  x*x + 3.0*u*u           ;  
    f[2] =  z + exp(z) - 1.0 + x  ;
    
    

}
