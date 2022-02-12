/*
* Author: Tian Zheng
* Date: 12/17/2019
* Email: t-zheng@outlook.com
*/

#include "graphmatching.h"

void tensorMatching(double* pX, int N1, int N2,
                          int* pIndH1, double* pValH1, int Nt1 ,
                          int* pIndH2, double* pValH2, int Nt2 ,
                          int* pIndH3, double* pValH3, int Nt3 ,
                          int nIter, int sparse, int stoc,
                          double* pXout, double* pScoreOut)
{
  int NN=N1*N2;
  double* pXtemp = new double[NN];
  for(int n=0;n<NN;n++)
    pXout[n]=pX[n];
  double score;
  int maxIter=100;
  int maxIter2=1;
  if( stoc == 2)
    maxIter2=10;
  for(int iter=0;iter<maxIter;iter++)
  {
    *pScoreOut=0;
    for(int n=0;n<NN;n++)
      pXtemp[n]=1*pX[n];
    for(int t=0;t<Nt1;t++)
    {
      if(sparse==1)
        score=pXout[pIndH1[t]];
      else
        score=1;
      pXtemp[pIndH1[t]] += score*
        pValH1[t];
      if(iter==(maxIter-1))
      {
        score=pXout[pIndH1[t]];
        *pScoreOut=*pScoreOut+score*score;
      }
    }
    for(int t=0;t<Nt2;t++)
    {
      if(sparse==1)
        score=pXout[pIndH2[t]]*pXout[pIndH2[t+Nt2]];
      else
        score=1;
      pXtemp[pIndH2[t]] += score*
        pValH2[t]*pXout[pIndH2[t+Nt2]];
      pXtemp[pIndH2[t+Nt2]] += score*
        pValH2[t]*pXout[pIndH2[t]];
      if(iter==(maxIter-1))
      {
        score=pXout[pIndH2[t]]*pXout[pIndH2[t+Nt2]];
        *pScoreOut=*pScoreOut+2*score*score;
      }
    }
    for(int t=0;t<Nt3;t++)
    {
      if(sparse==1)
        score=pXout[pIndH3[t]]*pXout[pIndH3[t+Nt3]]*pXout[pIndH3[t+2*Nt3]];
      else
        score=1;
      pXtemp[pIndH3[t]] += score*
        pValH3[t]*pXout[pIndH3[t+Nt3]]*pXout[pIndH3[t+2*Nt3]];
      pXtemp[pIndH3[t+Nt3]] += score*
        pValH3[t]*pXout[pIndH3[t+2*Nt3]]*pXout[pIndH3[t]];
      pXtemp[pIndH3[t+2*Nt3]] += score*
        pValH3[t]*pXout[pIndH3[t]]*pXout[pIndH3[t+Nt3]];
      if(iter==(maxIter-1))
      {
        score= pXout[pIndH3[t]]*pXout[pIndH3[t+Nt3]]*pXout[pIndH3[t+2*Nt3]];
        *pScoreOut=*pScoreOut+3*score*score;
      }
    }
/// normalization    
    if (stoc == 0 )
    {
      double pXnorm=0;
      for(int n2=0;n2<N2;n2++)
        for(int n1=0;n1<N1;n1++)
          pXnorm+=pXtemp[n1+n2*N1]*pXtemp[n1+n2*N1];
      pXnorm=sqrt(pXnorm);
      for(int n2=0;n2<N2;n2++)
        for(int n1=0;n1<N1;n1++)
          pXout[n1+n2*N1]=pXtemp[n1+n2*N1]/pXnorm;
    }
    else
    {
      for(int n=0;n<NN;n++)
        pXout[n]=pXtemp[n];
      for(int iter2=0;iter2<maxIter2;iter2++)
      {
        for(int n2=0;n2<N2;n2++)
        {
          double pXnorm=0;
          for(int n1=0;n1<N1;n1++)
            pXnorm+=pXout[n1+n2*N1]*pXout[n1+n2*N1];
          pXnorm=sqrt(pXnorm);
          if(pXnorm!=0)
            for(int n1=0;n1<N1;n1++)
              pXout[n1+n2*N1]=pXout[n1+n2*N1]/pXnorm;
        }
        
        if( stoc == 2)
          for(int n1=0;n1<N1;n1++)
          {
            double pXnorm=0;
            for(int n2=0;n2<N2;n2++)
              pXnorm+=pXout[n1+n2*N1]*pXout[n1+n2*N1];
            pXnorm=sqrt(pXnorm);
            if(pXnorm!=0)
              for(int n2=0;n2<N2;n2++)
                pXout[n1+n2*N1]=pXout[n1+n2*N1]/pXnorm;
          }
         
      }
      /*
      for(int iter2=0;iter2<maxIter2;iter2++)
      {
        for(int n2=0;n2<N2;n2++)
        {
          double pXnorm=0;
          for(int n1=0;n1<N1;n1++)
            pXnorm+=pXtemp[n1+n2*N1]*pXtemp[n1+n2*N1];
          pXnorm=sqrt(pXnorm);
          for(int n1=0;n1<N1;n1++)
            if(pXnorm!=0)
              pXout[n1+n2*N1]=pXtemp[n1+n2*N1]/pXnorm;
        }
        
        if( stoc == 2)
          for(int n2=0;n2<N2;n2++)
          {
            double pXnorm=0;
            for(int n1=0;n1<N1;n1++)
              pXnorm+=pXtemp[n1+n2*N1]*pXtemp[n1+n2*N1];
            pXnorm=sqrt(pXnorm);
            if(pXnorm!=0)
              for(int n1=0;n1<N1;n1++)
                pXout[n1+n2*N1]=pXtemp[n1+n2*N1]/pXnorm;
          }
         
      }
       */
    }
  }
  delete[] pXtemp;
}