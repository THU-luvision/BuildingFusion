/*
* Author: Tian Zheng
* Date: 12/17/2019
* Email: t-zheng@outlook.com
*/
#include "graphmatching.h"
/* feat1 feat2 <- P1 P2 t1 type */

void computeFeatureSimple(const double *pP1, int i, int j, int k, double *pF);

// std::vector<std::vector<double>> computeFeature(
//                     const std::vector<double>& P1,    // nP1 x 3
//                     const std::vector<double>& P2,    // nP2 x 3
//                     const std::vector<double>& T1     // nT1 x 3
//                     )
void computeFeature( double* pP1 , int nP1 , double* pP2 , int nP2 ,
                      int* pT1 , int nT1 , double* pF1 , double* pF2)
{
    /* Input Interface */
    const int nFeature = 3;
    // const double *pP1 = P1.data();
    // int nP1 = P1.size()/3;
    // const double *pP2 = P2.data();
    // int nP2 = P2.size()/3;
    // const double *pT1 = T1.data();
    // int nT1 = T1.size()/3;
    
    // std::cout << "nT1:" << nT1 << std::endl;
    // std::cout << "nP2:" << nP2 << std::endl;

    /* Defining the output */
    // std::vector<std::vector<double>> out(2);
    // out[0].resize(nT1*nFeature);                      // feat1:  nT1 x 3
    // out[1].resize(nP2*nP2*nP2*nFeature);              // feat2:  nP2^3 x 3
    // double *pF1 = out[0].data();
    // double *pF2 = out[1].data();

    /* Do the actual work */
    for (int t = 0; t < nT1; t++)
    {
        computeFeatureSimple(pP1, pT1[t * 3], pT1[t * 3 + 1], pT1[t * 3 + 2], pF1 + t * nFeature);
    }

    for (int i = 0; i < nP2; i++)
        for (int j = 0; j < nP2; j++)
            for (int k = 0; k < nP2; k++)
                computeFeatureSimple(pP2, i, j, k, pF2 + ((i * nP2 + j) * nP2 + k) * nFeature);
    
    // return out;
}

void computeFeatureSimple(const double *pP1, int i, int j, int k, double *pF)
{
    const int nFeature = 3;
    double vecX[nFeature];
    double vecY[nFeature];
    double vecZ[nFeature];
    int ind[nFeature];
    ind[0] = i;
    ind[1] = j;
    ind[2] = k;
    double n;
    if ((ind[0] == ind[1]) || (ind[0] == ind[2]) || (ind[1] == ind[2]))
    {
        pF[0] = pF[1] = pF[2] = -10;
        return;
    }
    for (int f = 0; f < nFeature; f++)
    {
        vecX[f] = pP1[ind[((f + 1) % 3)] * 3] - pP1[ind[f] * 3];
        vecY[f] = pP1[ind[((f + 1) % 3)] * 3 + 1] - pP1[ind[f] * 3 + 1];
        vecZ[f] = pP1[ind[((f + 1) % 3)] * 3 + 2] - pP1[ind[f] * 3 + 2];
        double norm = sqrt(vecX[f] * vecX[f] + vecY[f] * vecY[f] + vecZ[f] * vecZ[f]);
        if (norm != 0)
        {
            vecX[f] /= norm;
            vecY[f] /= norm;
            vecZ[f] /= norm;
        }
        else
        {
            vecX[f] = 0;
            vecY[f] = 0;
            vecZ[f] = 0;

        }
    }
    for (int f = 0; f < nFeature; f++)
    {
        // pF[f] = vecX[((f + 1) % 3)] * vecY[f] - vecY[((f + 1) % 3)] * vecX[f];
        pF[f] = vecX[((f + 1) % 3)] * vecX[f] + vecY[((f + 1) % 3)] * vecY[f] + vecZ[((f + 1) % 3)] * vecZ[f];
    }
}