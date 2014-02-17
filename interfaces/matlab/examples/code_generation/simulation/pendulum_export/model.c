
#include "acado_common.h"


void rhs( const real_t *x, real_t *f ){
const real_t* xd = x;
const real_t* xa = x + 6;
const real_t* u  = x + 9;

f[0] = xd[3];
f[1] = xd[4];
f[2] = xd[5];
f[3] = (xa[1]+u[0]);
f[4] = ((real_t)(-9.81)+xa[2]);
f[5] = xa[0];
f[6] = ((sin(xd[2]))-xd[0]);
f[7] = (((real_t)(0.)-(cos(xd[2])))-xd[1]);
f[8] = (((((real_t)(0.7)*xd[5])+(xa[1]*(cos(xd[2]))))+(xa[2]*(sin(xd[2]))))+xa[0]);
}



void rhs_jac( const real_t *x, real_t *f ){
const real_t* xd = x;
const real_t* xa = x + 6;

f[0] = (real_t)(0.);
f[1] = (real_t)(0.);
f[2] = (real_t)(0.);
f[3] = (real_t)(1.);
f[4] = (real_t)(0.);
f[5] = (real_t)(0.);
f[6] = (real_t)(0.);
f[7] = (real_t)(0.);
f[8] = (real_t)(0.);
f[9] = (real_t)(0.);
f[10] = (real_t)(0.);
f[11] = (real_t)(0.);
f[12] = (real_t)(0.);
f[13] = (real_t)(0.);
f[14] = (real_t)(1.);
f[15] = (real_t)(0.);
f[16] = (real_t)(0.);
f[17] = (real_t)(0.);
f[18] = (real_t)(0.);
f[19] = (real_t)(0.);
f[20] = (real_t)(0.);
f[21] = (real_t)(0.);
f[22] = (real_t)(0.);
f[23] = (real_t)(0.);
f[24] = (real_t)(0.);
f[25] = (real_t)(1.);
f[26] = (real_t)(0.);
f[27] = (real_t)(0.);
f[28] = (real_t)(0.);
f[29] = (real_t)(0.);
f[30] = (real_t)(0.);
f[31] = (real_t)(0.);
f[32] = (real_t)(0.);
f[33] = (real_t)(0.);
f[34] = (real_t)(0.);
f[35] = (real_t)(0.);
f[36] = (real_t)(0.);
f[37] = (real_t)(1.);
f[38] = (real_t)(0.);
f[39] = (real_t)(1.);
f[40] = (real_t)(0.);
f[41] = (real_t)(0.);
f[42] = (real_t)(0.);
f[43] = (real_t)(0.);
f[44] = (real_t)(0.);
f[45] = (real_t)(0.);
f[46] = (real_t)(0.);
f[47] = (real_t)(0.);
f[48] = (real_t)(1.);
f[49] = (real_t)(0.);
f[50] = (real_t)(0.);
f[51] = (real_t)(0.);
f[52] = (real_t)(0.);
f[53] = (real_t)(0.);
f[54] = (real_t)(0.);
f[55] = (real_t)(0.);
f[56] = (real_t)(1.);
f[57] = (real_t)(0.);
f[58] = (real_t)(0.);
f[59] = (real_t)(0.);
f[60] = ((real_t)(0.)-(real_t)(1.));
f[61] = (real_t)(0.);
f[62] = (cos(xd[2]));
f[63] = (real_t)(0.);
f[64] = (real_t)(0.);
f[65] = (real_t)(0.);
f[66] = (real_t)(0.);
f[67] = (real_t)(0.);
f[68] = (real_t)(0.);
f[69] = (real_t)(0.);
f[70] = (real_t)(0.);
f[71] = ((real_t)(0.)-(real_t)(1.));
f[72] = ((real_t)(0.)-((real_t)(-1.)*(sin(xd[2]))));
f[73] = (real_t)(0.);
f[74] = (real_t)(0.);
f[75] = (real_t)(0.);
f[76] = (real_t)(0.);
f[77] = (real_t)(0.);
f[78] = (real_t)(0.);
f[79] = (real_t)(0.);
f[80] = (real_t)(0.);
f[81] = (real_t)(0.);
f[82] = ((xa[1]*((real_t)(-1.)*(sin(xd[2]))))+(xa[2]*(cos(xd[2]))));
f[83] = (real_t)(0.);
f[84] = (real_t)(0.);
f[85] = (real_t)(0.7);
f[86] = (real_t)(1.);
f[87] = (cos(xd[2]));
f[88] = (sin(xd[2]));
f[89] = (real_t)(0.);
}



void out( const real_t *x, real_t *f ){
const real_t* xd = x;
const real_t* xa = x + 6;

f[0] = xa[1];
f[1] = xa[2];
}



void out_jac( const real_t *x, real_t *f ){
const real_t* xd = x;
const real_t* xa = x + 6;

f[0] = (real_t)(0.);
f[1] = (real_t)(0.);
f[2] = (real_t)(0.);
f[3] = (real_t)(0.);
f[4] = (real_t)(0.);
f[5] = (real_t)(0.);
f[6] = (real_t)(0.);
f[7] = (real_t)(1.);
f[8] = (real_t)(0.);
f[9] = (real_t)(0.);
f[10] = (real_t)(0.);
f[11] = (real_t)(0.);
f[12] = (real_t)(0.);
f[13] = (real_t)(0.);
f[14] = (real_t)(0.);
f[15] = (real_t)(0.);
f[16] = (real_t)(0.);
f[17] = (real_t)(0.);
f[18] = (real_t)(1.);
f[19] = (real_t)(0.);
}
