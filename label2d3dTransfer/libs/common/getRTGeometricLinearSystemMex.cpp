#include "mex.h"
#include "/usr/include/eigen3/Eigen/Dense"
#include "/usr/include/eigen3/Eigen/LU"
#include <iostream>

void solve3PointRegistration(const Eigen::MatrixXf &A, const Eigen::MatrixXf &b, 
                    const int &rows, const int &cols, Eigen::Matrix4f& transMat)
{
    Eigen::MatrixXf solveX(rows, 1); solveX.setZero();
    solveX = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
//     std::cout<<"solveX: "<<solveX<<"\n";
    
    // solveX = [Rx, Ry, Rz, T'x, T'y, T'z];
    Eigen::Matrix3f I_Vx; I_Vx.setOnes();
    I_Vx(0,0) = 1;          I_Vx(0,1) = solveX(2);  I_Vx(0,2) = -solveX(1);
    I_Vx(1,0) = -solveX(2); I_Vx(1,1) = 1;          I_Vx(1,2) = solveX(0);
    I_Vx(2,0) = solveX(1);  I_Vx(2,1) = -solveX(0); I_Vx(2,2) = 1;
    Eigen::Vector3f Tx; Tx.setZero();
    Tx(0) = solveX(3); Tx(1) = solveX(4); Tx(2) = solveX(5);
    Eigen::Vector3f tx; tx.setZero();
    tx = I_Vx.inverse()*Tx;
    transMat(0,3) = tx(0); transMat(1,3) = tx(1); transMat(2,3) = tx(2);
    
    Eigen::Matrix3f IplusVx; IplusVx.setOnes();
    IplusVx(0,0) = 1;          IplusVx(0,1) = -solveX(2);  IplusVx(0,2) = solveX(1);
    IplusVx(1,0) = solveX(2);  IplusVx(1,1) = 1;           IplusVx(1,2) = -solveX(0);
    IplusVx(2,0) = -solveX(1); IplusVx(2,1) = solveX(0);   IplusVx(2,2) = 1;
    Eigen::Matrix3f Ro; Ro.setZero();
    Ro = I_Vx.inverse()*IplusVx;
    transMat(0,0) = Ro(0,0); transMat(0,1) = Ro(0,1); transMat(0,2) = Ro(0,2); transMat(0,3) = tx(0);
    transMat(1,0) = Ro(1,0); transMat(1,1) = Ro(1,1); transMat(1,2) = Ro(1,2); transMat(1,3) = tx(1);
    transMat(2,0) = Ro(2,0); transMat(2,1) = Ro(2,1); transMat(2,2) = Ro(2,2); transMat(2,3) = tx(2);
//     std::cout<<transMat<<std::endl;
}
// Function solve 3point rigid transformation:
// T*X = Y;
// given X and Y, we can get the transformation matrix T to transform
// point cloud X to Y;
void mexFunction(
        int          nlhs,
        mxArray      *plhs[],
        int          nrhs,
        const mxArray *prhs[]
        )
{
    double *pX; pX = mxGetPr(prhs[0]);
    double *pY; pY = mxGetPr(prhs[1]);
    double *matM;
    Eigen::Matrix4f transMat;
    transMat <<  1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;
    // build linear system Ax = b;
    int rows = 9;
    int cols = 6;
    Eigen::MatrixXf A(rows, cols); A.setZero();
    Eigen::MatrixXf b(rows, 1); b.setZero();
    Eigen::MatrixXf X(3,3), Y(3,3);
    for(int i=0; i<rows/3; i++)
    {
        float X1 = pY[3*i];
        float Y1 = pY[3*i+1];
        float Z1 = pY[3*i+2];
        float X0 = pX[3*i];
        float Y0 = pX[3*i+1];
        float Z0 = pX[3*i+2];
        A(3*i,   0) = 0;      A(3*i,   1) = Z0+Z1;  A(3*i,   2) = -Y0-Y1; A(3*i,   3) = 1;
        A(3*i+1, 0) = -Z0-Z1; A(3*i+1, 1) = 0;      A(3*i+1, 2) = X0+X1;  A(3*i+1, 4) = 1;
        A(3*i+2, 0) = Y0+Y1;  A(3*i+1, 1) = -X0-X1; A(3*i+2, 2) = 0;      A(3*i+2, 5) = 1;
        
        b(3*i,   0) = X1-X0;
        b(3*i+1, 0) = Y1-Y0;
        b(3*i+2, 0) = Z1-Z0;
        X(0,i) = X0;
        X(1,i) = Y0;
        X(2,i) = Z0;
        Y(0,i) = X1;
        Y(1,i) = Y1;
        Y(2,i) = Z1;
    }
//     std::cout<<"X = \n"<<X<<std::endl;
//     std::cout<<"Y = \n"<<Y<<std::endl;
//     std::cout<<"A = \n"<<A<<std::endl;
//     std::cout<<"b = \n"<<b<<std::endl;
    
    solve3PointRegistration(A, b, rows, cols, transMat);
    plhs[0] = mxCreateDoubleMatrix(4, 4, mxREAL);
    double *tElement = mxGetPr(plhs[0]);
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<4; j++)
        {
            tElement[i+j*4] = transMat(i,j);
        }
    }
    std::cout<<"T = \n"<<transMat<<std::endl;
}


