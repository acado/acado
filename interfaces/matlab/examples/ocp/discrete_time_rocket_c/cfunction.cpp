void myAcadoDifferentialEquation( double *x, double *f, void *user_data ){

    // x[0] -> time t
    // x[1] -> v
    // x[2] -> s
    // x[3] -> m
    // x[4] -> u
    
    double h = 0.01;
    
    double t = x[0];
    double v = x[1];
    double s = x[2];
    double m = x[3];
    double u = x[4];

    f[0] =  s + h*v;
	f[0] =  v + h*(u-0.02*v*v)/m;
    f[0] =  m - h*0.01*u*u;

}
