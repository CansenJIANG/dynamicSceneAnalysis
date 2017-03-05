#include <math.h>
#include <matrix.h>
#include <mex.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>

void mexFunction(int nlhs, mxArray *plhs[],
				 int nrhs, const mxArray *prhs[])
{
const mwSize *dims;
int dimCell; 
mxArray *cellEllement; 
const mxArray *cellArray, *cell;
mwIndex jcell;
double *p;
mxArray *cellElement;
cell = prhs[0];
double *output = mxGetPr(cell);
dims = mxGetDimensions(prhs[0]);
for (jcell=0; jcell<dims[0]; jcell++) {
  cellElement = mxGetCell(cell,jcell);
  dimCell = std::max( mxGetM(cellElement), mxGetN(cellElement));
  p = mxGetPr(cellElement);
  mexPrintf("dimCell[0]= %d", dimCell);
  for(int i=0; i<dimCell; i++, p++)
  {
    mexPrintf("The content at %d is %g\n", jcell, *p);
  }
}
return;
}