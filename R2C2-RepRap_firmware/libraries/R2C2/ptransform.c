#include "debug.h"
#include "ptransform.h"

void ptransform(tTarget *target, double *x, double *y, double *z) {

  sersendf("ptransform called\r\n");

  *x = target->x;
  *y = target->y;
  *z = target->z;
}