
#include "acado.h"


void rhs( real_t *acado_x, real_t *acado_f ){
real_t *acado_xd = acado_x;
real_t *acado_xa = acado_x + 6;
real_t *acado_u  = acado_x + 11;
real_t *acado_dx = acado_x + 12;

/* COMPUTE INTERMEDIATE STATES: */
/* ---------------------------- */

/* COMPUTE OUTPUT: */
/* --------------- */
acado_f[0] = (acado_dx[0]-acado_xd[3]);
acado_f[1] = (acado_dx[1]-acado_xd[4]);
acado_f[2] = (((real_t)(0.)-acado_xd[5])+acado_dx[2]);
acado_f[3] = (((real_t)(0.)-acado_xa[0])+acado_dx[3]);
acado_f[4] = (((real_t)(0.)-acado_xa[1])+acado_dx[4]);
acado_f[5] = (((real_t)(0.)-acado_xa[2])+acado_dx[5]);
acado_f[6] = ((((real_t)(2.)*acado_xa[0])-acado_xa[3])-acado_u[0]);
acado_f[7] = ((((real_t)(19.62)+((real_t)(2.)*acado_xa[1]))-acado_xa[4])-acado_u[0]);
acado_f[8] = ((((((real_t)(0.)-(acado_xa[3]+acado_u[0]))*acado_xd[1])+((acado_xa[4]+acado_u[0])*acado_xd[0]))+((real_t)(0.1)*acado_xa[2]))-(real_t)(3.5));
acado_f[9] = (((acado_xd[5]*acado_xd[4])+(acado_xa[2]*acado_xd[1]))+acado_xa[0]);
acado_f[10] = (((((real_t)(0.)-acado_xd[5])*acado_xd[3])-(acado_xa[2]*acado_xd[0]))+acado_xa[1]);
}



void rhs_jac( real_t *acado_x, real_t *acado_f ){
real_t *acado_xd = acado_x;
real_t *acado_xa = acado_x + 6;
real_t *acado_u  = acado_x + 11;
real_t *acado_dx = acado_x + 12;

/* COMPUTE INTERMEDIATE STATES: */
/* ---------------------------- */

/* COMPUTE OUTPUT: */
/* --------------- */
acado_f[0] = (real_t)(0.);
acado_f[1] = (real_t)(0.);
acado_f[2] = (real_t)(0.);
acado_f[3] = ((real_t)(0.)-(real_t)(1.));
acado_f[4] = (real_t)(0.);
acado_f[5] = (real_t)(0.);
acado_f[6] = (real_t)(0.);
acado_f[7] = (real_t)(0.);
acado_f[8] = (real_t)(0.);
acado_f[9] = (real_t)(0.);
acado_f[10] = (real_t)(0.);
acado_f[11] = (real_t)(0.);
acado_f[12] = (real_t)(1.);
acado_f[13] = (real_t)(0.);
acado_f[14] = (real_t)(0.);
acado_f[15] = (real_t)(0.);
acado_f[16] = (real_t)(0.);
acado_f[17] = (real_t)(0.);
acado_f[18] = (real_t)(0.);
acado_f[19] = (real_t)(0.);
acado_f[20] = (real_t)(0.);
acado_f[21] = (real_t)(0.);
acado_f[22] = ((real_t)(0.)-(real_t)(1.));
acado_f[23] = (real_t)(0.);
acado_f[24] = (real_t)(0.);
acado_f[25] = (real_t)(0.);
acado_f[26] = (real_t)(0.);
acado_f[27] = (real_t)(0.);
acado_f[28] = (real_t)(0.);
acado_f[29] = (real_t)(0.);
acado_f[30] = (real_t)(0.);
acado_f[31] = (real_t)(1.);
acado_f[32] = (real_t)(0.);
acado_f[33] = (real_t)(0.);
acado_f[34] = (real_t)(0.);
acado_f[35] = (real_t)(0.);
acado_f[36] = (real_t)(0.);
acado_f[37] = (real_t)(0.);
acado_f[38] = (real_t)(0.);
acado_f[39] = (real_t)(0.);
acado_f[40] = (real_t)(0.);
acado_f[41] = ((real_t)(0.)-(real_t)(1.));
acado_f[42] = (real_t)(0.);
acado_f[43] = (real_t)(0.);
acado_f[44] = (real_t)(0.);
acado_f[45] = (real_t)(0.);
acado_f[46] = (real_t)(0.);
acado_f[47] = (real_t)(0.);
acado_f[48] = (real_t)(0.);
acado_f[49] = (real_t)(0.);
acado_f[50] = (real_t)(1.);
acado_f[51] = (real_t)(0.);
acado_f[52] = (real_t)(0.);
acado_f[53] = (real_t)(0.);
acado_f[54] = (real_t)(0.);
acado_f[55] = (real_t)(0.);
acado_f[56] = (real_t)(0.);
acado_f[57] = (real_t)(0.);
acado_f[58] = (real_t)(0.);
acado_f[59] = (real_t)(0.);
acado_f[60] = ((real_t)(0.)-(real_t)(1.));
acado_f[61] = (real_t)(0.);
acado_f[62] = (real_t)(0.);
acado_f[63] = (real_t)(0.);
acado_f[64] = (real_t)(0.);
acado_f[65] = (real_t)(0.);
acado_f[66] = (real_t)(0.);
acado_f[67] = (real_t)(0.);
acado_f[68] = (real_t)(0.);
acado_f[69] = (real_t)(1.);
acado_f[70] = (real_t)(0.);
acado_f[71] = (real_t)(0.);
acado_f[72] = (real_t)(0.);
acado_f[73] = (real_t)(0.);
acado_f[74] = (real_t)(0.);
acado_f[75] = (real_t)(0.);
acado_f[76] = (real_t)(0.);
acado_f[77] = (real_t)(0.);
acado_f[78] = (real_t)(0.);
acado_f[79] = ((real_t)(0.)-(real_t)(1.));
acado_f[80] = (real_t)(0.);
acado_f[81] = (real_t)(0.);
acado_f[82] = (real_t)(0.);
acado_f[83] = (real_t)(0.);
acado_f[84] = (real_t)(0.);
acado_f[85] = (real_t)(0.);
acado_f[86] = (real_t)(0.);
acado_f[87] = (real_t)(0.);
acado_f[88] = (real_t)(1.);
acado_f[89] = (real_t)(0.);
acado_f[90] = (real_t)(0.);
acado_f[91] = (real_t)(0.);
acado_f[92] = (real_t)(0.);
acado_f[93] = (real_t)(0.);
acado_f[94] = (real_t)(0.);
acado_f[95] = (real_t)(0.);
acado_f[96] = (real_t)(0.);
acado_f[97] = (real_t)(0.);
acado_f[98] = ((real_t)(0.)-(real_t)(1.));
acado_f[99] = (real_t)(0.);
acado_f[100] = (real_t)(0.);
acado_f[101] = (real_t)(0.);
acado_f[102] = (real_t)(0.);
acado_f[103] = (real_t)(0.);
acado_f[104] = (real_t)(0.);
acado_f[105] = (real_t)(0.);
acado_f[106] = (real_t)(0.);
acado_f[107] = (real_t)(1.);
acado_f[108] = (real_t)(0.);
acado_f[109] = (real_t)(0.);
acado_f[110] = (real_t)(0.);
acado_f[111] = (real_t)(0.);
acado_f[112] = (real_t)(0.);
acado_f[113] = (real_t)(0.);
acado_f[114] = (real_t)(2.);
acado_f[115] = (real_t)(0.);
acado_f[116] = (real_t)(0.);
acado_f[117] = ((real_t)(0.)-(real_t)(1.));
acado_f[118] = (real_t)(0.);
acado_f[119] = ((real_t)(0.)-(real_t)(1.));
acado_f[120] = (real_t)(0.);
acado_f[121] = (real_t)(0.);
acado_f[122] = (real_t)(0.);
acado_f[123] = (real_t)(0.);
acado_f[124] = (real_t)(0.);
acado_f[125] = (real_t)(0.);
acado_f[126] = (real_t)(0.);
acado_f[127] = (real_t)(0.);
acado_f[128] = (real_t)(0.);
acado_f[129] = (real_t)(0.);
acado_f[130] = (real_t)(0.);
acado_f[131] = (real_t)(0.);
acado_f[132] = (real_t)(0.);
acado_f[133] = (real_t)(2.);
acado_f[134] = (real_t)(0.);
acado_f[135] = (real_t)(0.);
acado_f[136] = ((real_t)(0.)-(real_t)(1.));
acado_f[137] = ((real_t)(0.)-(real_t)(1.));
acado_f[138] = (real_t)(0.);
acado_f[139] = (real_t)(0.);
acado_f[140] = (real_t)(0.);
acado_f[141] = (real_t)(0.);
acado_f[142] = (real_t)(0.);
acado_f[143] = (real_t)(0.);
acado_f[144] = (acado_xa[4]+acado_u[0]);
acado_f[145] = ((real_t)(0.)-(acado_xa[3]+acado_u[0]));
acado_f[146] = (real_t)(0.);
acado_f[147] = (real_t)(0.);
acado_f[148] = (real_t)(0.);
acado_f[149] = (real_t)(0.);
acado_f[150] = (real_t)(0.);
acado_f[151] = (real_t)(0.);
acado_f[152] = (real_t)(0.1);
acado_f[153] = (acado_xd[1]*((real_t)(0.)-(real_t)(1.)));
acado_f[154] = acado_xd[0];
acado_f[155] = ((acado_xd[1]*((real_t)(0.)-(real_t)(1.)))+acado_xd[0]);
acado_f[156] = (real_t)(0.);
acado_f[157] = (real_t)(0.);
acado_f[158] = (real_t)(0.);
acado_f[159] = (real_t)(0.);
acado_f[160] = (real_t)(0.);
acado_f[161] = (real_t)(0.);
acado_f[162] = (real_t)(0.);
acado_f[163] = acado_xa[2];
acado_f[164] = (real_t)(0.);
acado_f[165] = (real_t)(0.);
acado_f[166] = acado_xd[5];
acado_f[167] = acado_xd[4];
acado_f[168] = (real_t)(1.);
acado_f[169] = (real_t)(0.);
acado_f[170] = acado_xd[1];
acado_f[171] = (real_t)(0.);
acado_f[172] = (real_t)(0.);
acado_f[173] = (real_t)(0.);
acado_f[174] = (real_t)(0.);
acado_f[175] = (real_t)(0.);
acado_f[176] = (real_t)(0.);
acado_f[177] = (real_t)(0.);
acado_f[178] = (real_t)(0.);
acado_f[179] = (real_t)(0.);
acado_f[180] = ((real_t)(0.)-acado_xa[2]);
acado_f[181] = (real_t)(0.);
acado_f[182] = (real_t)(0.);
acado_f[183] = ((real_t)(0.)-acado_xd[5]);
acado_f[184] = (real_t)(0.);
acado_f[185] = (acado_xd[3]*((real_t)(0.)-(real_t)(1.)));
acado_f[186] = (real_t)(0.);
acado_f[187] = (real_t)(1.);
acado_f[188] = ((real_t)(0.)-acado_xd[0]);
acado_f[189] = (real_t)(0.);
acado_f[190] = (real_t)(0.);
acado_f[191] = (real_t)(0.);
acado_f[192] = (real_t)(0.);
acado_f[193] = (real_t)(0.);
acado_f[194] = (real_t)(0.);
acado_f[195] = (real_t)(0.);
acado_f[196] = (real_t)(0.);
acado_f[197] = (real_t)(0.);
}



void out( real_t *acado_x, real_t *acado_f ){
real_t *acado_xd = acado_x;
real_t *acado_xa = acado_x + 6;
real_t *acado_dx = acado_x + 12;

/* COMPUTE INTERMEDIATE STATES: */
/* ---------------------------- */

/* COMPUTE OUTPUT: */
/* --------------- */
acado_f[0] = acado_xa[3];
acado_f[1] = acado_xa[4];
}



void out_jac( real_t *acado_x, real_t *acado_f ){
real_t *acado_xd = acado_x;
real_t *acado_xa = acado_x + 6;
real_t *acado_dx = acado_x + 12;

/* COMPUTE INTERMEDIATE STATES: */
/* ---------------------------- */

/* COMPUTE OUTPUT: */
/* --------------- */
acado_f[0] = (real_t)(0.);
acado_f[1] = (real_t)(0.);
acado_f[2] = (real_t)(0.);
acado_f[3] = (real_t)(0.);
acado_f[4] = (real_t)(0.);
acado_f[5] = (real_t)(0.);
acado_f[6] = (real_t)(0.);
acado_f[7] = (real_t)(0.);
acado_f[8] = (real_t)(0.);
acado_f[9] = (real_t)(1.);
acado_f[10] = (real_t)(0.);
acado_f[11] = (real_t)(0.);
acado_f[12] = (real_t)(0.);
acado_f[13] = (real_t)(0.);
acado_f[14] = (real_t)(0.);
acado_f[15] = (real_t)(0.);
acado_f[16] = (real_t)(0.);
acado_f[17] = (real_t)(0.);
acado_f[18] = (real_t)(0.);
acado_f[19] = (real_t)(0.);
acado_f[20] = (real_t)(0.);
acado_f[21] = (real_t)(0.);
acado_f[22] = (real_t)(0.);
acado_f[23] = (real_t)(0.);
acado_f[24] = (real_t)(0.);
acado_f[25] = (real_t)(0.);
acado_f[26] = (real_t)(0.);
acado_f[27] = (real_t)(0.);
acado_f[28] = (real_t)(1.);
acado_f[29] = (real_t)(0.);
acado_f[30] = (real_t)(0.);
acado_f[31] = (real_t)(0.);
acado_f[32] = (real_t)(0.);
acado_f[33] = (real_t)(0.);
acado_f[34] = (real_t)(0.);
acado_f[35] = (real_t)(0.);
}
