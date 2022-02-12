#ifndef GRAPHMATCHING_H_
#define GRAPHMATCHING_H_

#include <vector>
#include <math.h> /* sqrt */
#include <assert.h>
#include <iostream>
#include <utility>


void tensorMatching(double *pX, int N1, int N2,
                    int *pIndH1, double *pValH1, int Nt1,
                    int *pIndH2, double *pValH2, int Nt2,
                    int *pIndH3, double *pValH3, int Nt3,
                    int nIter, int sparse, int stoc,
                    double *pXout, double* pScoreOut);

void computeFeature( double* pP1 , int nP1 , double* pP2 , int nP2 ,
                    int* pT1 , int nT1 , double* pF1 , double* pF2);

#endif