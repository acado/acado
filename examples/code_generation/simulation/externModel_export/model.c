
#include "acado_common.h"


void rhs( const real_t *x, real_t *f ){
const real_t *xd = x;
const real_t *xa = x + 6;
const real_t *u  = x + 11;
const real_t *dx = x + 12;

/* COMPUTE INTERMEDIATE STATES: */
/* ---------------------------- */

/* COMPUTE OUTPUT: */
/* --------------- */
f[0] = (dx[0]-xd[3]);
f[1] = (dx[1]-xd[4]);
f[2] = (((real_t)(0.)-xd[5])+dx[2]);
f[3] = (((real_t)(0.)-xa[0])+dx[3]);
f[4] = (((real_t)(0.)-xa[1])+dx[4]);
f[5] = (((real_t)(0.)-xa[2])+dx[5]);
f[6] = ((((real_t)(2.)*xa[0])-xa[3])-u[0]);
f[7] = ((((real_t)(19.62)+((real_t)(2.)*xa[1]))-xa[4])-u[0]);
f[8] = ((((((real_t)(0.)-(xa[3]+u[0]))*xd[1])+((xa[4]+u[0])*xd[0]))+((real_t)(0.1)*xa[2]))-(real_t)(3.5));
f[9] = (((xd[5]*xd[4])+(xa[2]*xd[1]))+xa[0]);
f[10] = (((((real_t)(0.)-xd[5])*xd[3])-(xa[2]*xd[0]))+xa[1]);
}



void rhs_jac( const real_t *x, real_t *f ){
const real_t *xd = x;
const real_t *xa = x + 6;
const real_t *u  = x + 11;
const real_t *dx = x + 12;

/* COMPUTE INTERMEDIATE STATES: */
/* ---------------------------- */

/* COMPUTE OUTPUT: */
/* --------------- */
f[0] = (real_t)(0.);
f[1] = (real_t)(0.);
f[2] = (real_t)(0.);
f[3] = ((real_t)(0.)-(real_t)(1.));
f[4] = (real_t)(0.);
f[5] = (real_t)(0.);
f[6] = (real_t)(0.);
f[7] = (real_t)(0.);
f[8] = (real_t)(0.);
f[9] = (real_t)(0.);
f[10] = (real_t)(0.);
f[11] = (real_t)(0.);
f[12] = (real_t)(1.);
f[13] = (real_t)(0.);
f[14] = (real_t)(0.);
f[15] = (real_t)(0.);
f[16] = (real_t)(0.);
f[17] = (real_t)(0.);
f[18] = (real_t)(0.);
f[19] = (real_t)(0.);
f[20] = (real_t)(0.);
f[21] = (real_t)(0.);
f[22] = ((real_t)(0.)-(real_t)(1.));
f[23] = (real_t)(0.);
f[24] = (real_t)(0.);
f[25] = (real_t)(0.);
f[26] = (real_t)(0.);
f[27] = (real_t)(0.);
f[28] = (real_t)(0.);
f[29] = (real_t)(0.);
f[30] = (real_t)(0.);
f[31] = (real_t)(1.);
f[32] = (real_t)(0.);
f[33] = (real_t)(0.);
f[34] = (real_t)(0.);
f[35] = (real_t)(0.);
f[36] = (real_t)(0.);
f[37] = (real_t)(0.);
f[38] = (real_t)(0.);
f[39] = (real_t)(0.);
f[40] = (real_t)(0.);
f[41] = ((real_t)(0.)-(real_t)(1.));
f[42] = (real_t)(0.);
f[43] = (real_t)(0.);
f[44] = (real_t)(0.);
f[45] = (real_t)(0.);
f[46] = (real_t)(0.);
f[47] = (real_t)(0.);
f[48] = (real_t)(0.);
f[49] = (real_t)(0.);
f[50] = (real_t)(1.);
f[51] = (real_t)(0.);
f[52] = (real_t)(0.);
f[53] = (real_t)(0.);
f[54] = (real_t)(0.);
f[55] = (real_t)(0.);
f[56] = (real_t)(0.);
f[57] = (real_t)(0.);
f[58] = (real_t)(0.);
f[59] = (real_t)(0.);
f[60] = ((real_t)(0.)-(real_t)(1.));
f[61] = (real_t)(0.);
f[62] = (real_t)(0.);
f[63] = (real_t)(0.);
f[64] = (real_t)(0.);
f[65] = (real_t)(0.);
f[66] = (real_t)(0.);
f[67] = (real_t)(0.);
f[68] = (real_t)(0.);
f[69] = (real_t)(1.);
f[70] = (real_t)(0.);
f[71] = (real_t)(0.);
f[72] = (real_t)(0.);
f[73] = (real_t)(0.);
f[74] = (real_t)(0.);
f[75] = (real_t)(0.);
f[76] = (real_t)(0.);
f[77] = (real_t)(0.);
f[78] = (real_t)(0.);
f[79] = ((real_t)(0.)-(real_t)(1.));
f[80] = (real_t)(0.);
f[81] = (real_t)(0.);
f[82] = (real_t)(0.);
f[83] = (real_t)(0.);
f[84] = (real_t)(0.);
f[85] = (real_t)(0.);
f[86] = (real_t)(0.);
f[87] = (real_t)(0.);
f[88] = (real_t)(1.);
f[89] = (real_t)(0.);
f[90] = (real_t)(0.);
f[91] = (real_t)(0.);
f[92] = (real_t)(0.);
f[93] = (real_t)(0.);
f[94] = (real_t)(0.);
f[95] = (real_t)(0.);
f[96] = (real_t)(0.);
f[97] = (real_t)(0.);
f[98] = ((real_t)(0.)-(real_t)(1.));
f[99] = (real_t)(0.);
f[100] = (real_t)(0.);
f[101] = (real_t)(0.);
f[102] = (real_t)(0.);
f[103] = (real_t)(0.);
f[104] = (real_t)(0.);
f[105] = (real_t)(0.);
f[106] = (real_t)(0.);
f[107] = (real_t)(1.);
f[108] = (real_t)(0.);
f[109] = (real_t)(0.);
f[110] = (real_t)(0.);
f[111] = (real_t)(0.);
f[112] = (real_t)(0.);
f[113] = (real_t)(0.);
f[114] = (real_t)(2.);
f[115] = (real_t)(0.);
f[116] = (real_t)(0.);
f[117] = ((real_t)(0.)-(real_t)(1.));
f[118] = (real_t)(0.);
f[119] = ((real_t)(0.)-(real_t)(1.));
f[120] = (real_t)(0.);
f[121] = (real_t)(0.);
f[122] = (real_t)(0.);
f[123] = (real_t)(0.);
f[124] = (real_t)(0.);
f[125] = (real_t)(0.);
f[126] = (real_t)(0.);
f[127] = (real_t)(0.);
f[128] = (real_t)(0.);
f[129] = (real_t)(0.);
f[130] = (real_t)(0.);
f[131] = (real_t)(0.);
f[132] = (real_t)(0.);
f[133] = (real_t)(2.);
f[134] = (real_t)(0.);
f[135] = (real_t)(0.);
f[136] = ((real_t)(0.)-(real_t)(1.));
f[137] = ((real_t)(0.)-(real_t)(1.));
f[138] = (real_t)(0.);
f[139] = (real_t)(0.);
f[140] = (real_t)(0.);
f[141] = (real_t)(0.);
f[142] = (real_t)(0.);
f[143] = (real_t)(0.);
f[144] = (xa[4]+u[0]);
f[145] = ((real_t)(0.)-(xa[3]+u[0]));
f[146] = (real_t)(0.);
f[147] = (real_t)(0.);
f[148] = (real_t)(0.);
f[149] = (real_t)(0.);
f[150] = (real_t)(0.);
f[151] = (real_t)(0.);
f[152] = (real_t)(0.1);
f[153] = (xd[1]*((real_t)(0.)-(real_t)(1.)));
f[154] = xd[0];
f[155] = ((xd[1]*((real_t)(0.)-(real_t)(1.)))+xd[0]);
f[156] = (real_t)(0.);
f[157] = (real_t)(0.);
f[158] = (real_t)(0.);
f[159] = (real_t)(0.);
f[160] = (real_t)(0.);
f[161] = (real_t)(0.);
f[162] = (real_t)(0.);
f[163] = xa[2];
f[164] = (real_t)(0.);
f[165] = (real_t)(0.);
f[166] = xd[5];
f[167] = xd[4];
f[168] = (real_t)(1.);
f[169] = (real_t)(0.);
f[170] = xd[1];
f[171] = (real_t)(0.);
f[172] = (real_t)(0.);
f[173] = (real_t)(0.);
f[174] = (real_t)(0.);
f[175] = (real_t)(0.);
f[176] = (real_t)(0.);
f[177] = (real_t)(0.);
f[178] = (real_t)(0.);
f[179] = (real_t)(0.);
f[180] = ((real_t)(0.)-xa[2]);
f[181] = (real_t)(0.);
f[182] = (real_t)(0.);
f[183] = ((real_t)(0.)-xd[5]);
f[184] = (real_t)(0.);
f[185] = (xd[3]*((real_t)(0.)-(real_t)(1.)));
f[186] = (real_t)(0.);
f[187] = (real_t)(1.);
f[188] = ((real_t)(0.)-xd[0]);
f[189] = (real_t)(0.);
f[190] = (real_t)(0.);
f[191] = (real_t)(0.);
f[192] = (real_t)(0.);
f[193] = (real_t)(0.);
f[194] = (real_t)(0.);
f[195] = (real_t)(0.);
f[196] = (real_t)(0.);
f[197] = (real_t)(0.);
}



void out( const real_t *x, real_t *f ){
const real_t *xd = x;
const real_t *xa = x + 6;
const real_t *dx = x + 12;

/* COMPUTE INTERMEDIATE STATES: */
/* ---------------------------- */

/* COMPUTE OUTPUT: */
/* --------------- */
f[0] = xa[3];
f[1] = xa[4];
}



void out_jac( const real_t *x, real_t *f ){
const real_t *xd = x;
const real_t *xa = x + 6;
const real_t *dx = x + 12;

/* COMPUTE INTERMEDIATE STATES: */
/* ---------------------------- */

/* COMPUTE OUTPUT: */
/* --------------- */
f[0] = (real_t)(0.);
f[1] = (real_t)(0.);
f[2] = (real_t)(0.);
f[3] = (real_t)(0.);
f[4] = (real_t)(0.);
f[5] = (real_t)(0.);
f[6] = (real_t)(0.);
f[7] = (real_t)(0.);
f[8] = (real_t)(0.);
f[9] = (real_t)(1.);
f[10] = (real_t)(0.);
f[11] = (real_t)(0.);
f[12] = (real_t)(0.);
f[13] = (real_t)(0.);
f[14] = (real_t)(0.);
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
f[25] = (real_t)(0.);
f[26] = (real_t)(0.);
f[27] = (real_t)(0.);
f[28] = (real_t)(1.);
f[29] = (real_t)(0.);
f[30] = (real_t)(0.);
f[31] = (real_t)(0.);
f[32] = (real_t)(0.);
f[33] = (real_t)(0.);
f[34] = (real_t)(0.);
f[35] = (real_t)(0.);
}
