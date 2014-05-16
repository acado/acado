#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* Use POSIX clock_gettime() for timing on non-Windows machines. */
#include <time.h>
#include <sys/stat.h>
#include <sys/time.h>

typedef double real_t;

typedef struct timer
{
	struct timeval tic;
	struct timeval toc;
} timer;

/* read current time */
void tic(timer* t)
{
	gettimeofday(&t->tic, 0);
}

/* return time passed since last call to tic on this timer */
real_t toc(timer* t)
{
	struct timeval temp;
	
	gettimeofday(&t->toc, 0);
    
	if ((t->toc.tv_usec - t->tic.tv_usec) < 0)
	{
		temp.tv_sec = t->toc.tv_sec - t->tic.tv_sec - 1;
		temp.tv_usec = 1000000 + t->toc.tv_usec - t->tic.tv_usec;
	}
	else
	{
		temp.tv_sec = t->toc.tv_sec - t->tic.tv_sec;
		temp.tv_usec = t->toc.tv_usec - t->tic.tv_usec;
	}
	
	return (real_t)temp.tv_sec + (real_t)temp.tv_usec / 1e6;
}

/* SOME CONVENIENT DEFINTIONS: */
/* --------------------------------------------------------------- */
   #define ACADO_NX           8      
   #define ACADO_NU           2    
   #define ACADO_NZ			  10
   #define ACADO_NE           2     
   #define TIMING             10000     
/* --------------------------------------------------------------- */
/* --------------------------------------------------------------- */

void initializeX( double* x ) {
	int i, j;
	//~ srand ( time(NULL) );
	srand ( 36 );
	/* INITIALIZATION: */
	/* ---------------------------------------- */
	for( i = 0; i < (ACADO_NX)*(1+ACADO_NX+ACADO_NU)+ACADO_NE+ACADO_NU; i++ ) {
		x[i] = (2*((real_t)rand() / (real_t)RAND_MAX)-1);
	}
	printf( "input: \n[" );
	for( i = 0; i < ACADO_NX; i++ ) {
		if( i < (ACADO_NX-1) ) 	printf( "%.3g, ", x[i] );
		else 					printf( "%.3g]", x[i] );
	}
	printf( "\n[" );
	for( i = 0; i < ACADO_NE; i++ ) {
		if( i < (ACADO_NE-1) ) 	printf( "%.3g, ", x[ACADO_NX+i] );
		else 					printf( "%.3g]", x[ACADO_NX+i] );
	}
	printf( "\n[" );
	for( i = 0; i < ACADO_NX; i++ ) {
		for( j = 0; j < ACADO_NX+ACADO_NU; j++ ) {
			if( j < (ACADO_NX+ACADO_NU-1) ) 	printf( "%.3g, ", x[ACADO_NX+ACADO_NE+i*(ACADO_NX+ACADO_NU)+j] );
			else 								printf( "%.3g;", x[ACADO_NX+ACADO_NE+i*(ACADO_NX+ACADO_NU)+j] );
		}
		if( i < (ACADO_NX-1) ) 	printf( "\n" );
		else 					printf( "]" );
	}
	printf( "\n[" );
	for( i = 0; i < ACADO_NU; i++ ) {
		if( i < (ACADO_NU-1) ) 	printf( "%.3g, ", x[ACADO_NX+ACADO_NE+ACADO_NX*(ACADO_NX+ACADO_NU)+i] );
		else 					printf( "%.3g]\n\n", x[ACADO_NX+ACADO_NE+ACADO_NX*(ACADO_NX+ACADO_NU)+i] );
	}
}

typedef struct ACADOworkspace_
{
/** Column vector of size: 400 */
double acado_aux[ 800 ];


} ACADOworkspace;

ACADOworkspace acadoWorkspace;
/* --------------------------------------------------------------- */
/* --------------------------------------------------------------- */

#include "ADsymbolic_output.c"

int main(){
	printf( "------------------------------------------------------------\nRunning test..\n\n" );

	/* INTRODUCE AUXILIARY VAIRABLES: */
	/* ------------------------------ */
	int i, j, k;
	double x[(ACADO_NX)*(1+ACADO_NX+ACADO_NU)+ACADO_NE+ACADO_NU];
	int numZ = ACADO_NZ*(ACADO_NZ+1)/2.0;
	double f1[numZ];
	double f2[numZ];
	double diff[numZ];
	timer t_timer;
	double t_tmp1, t_tmp2;
	double error;
	
	initializeX( x );
	
	// AD SYMMETRIC:
	tic( &t_timer );
	for( i = 0; i < TIMING; i++ ) {
		symmetricDerivative(x, f1);
	}
	t_tmp1 = toc( &t_timer )/TIMING;
	
	printf( "symmetricDerivative: \n" );
	i = 0;
	for( j = 0; j < ACADO_NZ; j++ ) {
		for( k = 0; k <= j; k++ ) {
			printf( "%.3g, ", f1[i] );
			i = i+1;
		}
		printf( "\n" );
	}
	
	// ALT SYMMETRIC:
	tic( &t_timer );
	for( i = 0; i < TIMING; i++ ) {
		alternativeSymmetric(x, f2);
	}
	t_tmp2 = toc( &t_timer )/TIMING;
	
	printf( "\nalternativeSymmetric: \n" );
	i = 0;
	for( j = 0; j < ACADO_NZ; j++ ) {
		for( k = 0; k <= j; k++ ) {
			printf( "%.3g, ", f2[i] );
			i = i+1;
		}
		printf( "\n" );
	}
	
	// ERROR:
	printf( "\n------------------------------\nDifference: \n" );
	error = -1e10;
	i = 0;
	for( j = 0; j < ACADO_NZ; j++ ) {
		for( k = 0; k <= j; k++ ) {
			diff[i] = fabs( f1[i] - f2[i] );
			if( diff[i] > error ) error = diff[i];
			printf( "%.3g, ", diff[i] );
			i = i+1;
		}
		printf( "\n" );
	}
	printf( "\n" );
	printf( "Error: %.3g\n------------------------------\n", error );
	
	printf( "Timing symmetricDerivative :   %.3g μs\n", 1e6*t_tmp1 );
	printf( "Timing alternativeSymmetric:   %.3g μs\n\n", 1e6*t_tmp2 );

	printf( "done!\n------------------------------------------------------------\n" );
	return 0;
}
