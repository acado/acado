void myAcadoDifferentialEquation1( double *x, double *f, void *user_data ){

    double t = x[0];
    double xx = x[1];
    double u = x[2];
    double w = x[3];

    f[0] = -2.0*xx + u;

    
}
