#include "mex.h"
#include <iostream>
/* 
 * Input: feature positions (x, y);
 *        feature scores scr;
 *        image size (cols, rows);
 *        box size (wdth, lnth);
 *
 * Output: remaining features (x_, y_);
 */
void mexFunction(
        int          nlhs,
        mxArray      *plhs[],
        int          nrhs,
        const mxArray *prhs[]
        )
{
    double *featSz = mxGetM(prhs[0]);
    double *pX; pX = mxGetPr(prhs[0]);
    double *pY; pY = mxGetPr(prhs[0]) + featSz;
    double *matM;
    
}