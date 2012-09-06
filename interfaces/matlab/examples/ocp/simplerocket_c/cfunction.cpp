void myAcadoDifferentialEquation( double *x, double *f, void *user_data ){

    // x[0] -> time t
    // x[1] -> v
    // x[2] -> s
    // x[3] -> m
    // x[4] -> L
    // x[5] -> u
    
    double t = x[0];
    double v = x[1];
    double s = x[2];
    double m = x[3];
    double L = x[4];
    double u = x[5];

    f[0] =  (u-0.02*v*v)/(m);       //dot(v)
    f[1] =  v;                      //dot(s)
    f[2] =  -0.01*u*u;              //dot(m)
    f[3] =  u*u;                    //dot(L)

}
