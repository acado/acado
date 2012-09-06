#include "cs.h"
static int cs_nonzero (int i, int j, double aij, void *other)
{
    return ( fabs( aij ) > 2.3e-16 ) ;
}
int cs_dropzeros (cs *A)
{
    return (cs_fkeep (A, &cs_nonzero, NULL)) ;  /* keep all nonzero entries */
} 
