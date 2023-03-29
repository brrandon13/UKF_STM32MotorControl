/*
 * ukf_speed_pos_fdbk.c
 *
 *  Created on: 2023. 3. 27.
 *      Author: KAIST
 */
#include "ukf_speed_pos_fdbk.h"

void UKF_Init(UKF_Handle_t* pHandle);

void UKF_Clear(UKF_Handle_t* pHandle);

/* Iteration for FOC_CurrControlM1() */

int16_t UKF_CalcAvrgMecSpeedUnit(UKF_Handle_t* pHandle, int16_t* pMecSpeedUnit);

int16_t UKF_CalcAvrgElSpeedDpp(UKF_Handle_t* pHandle);

void UKF_Step(UKF_Handle_t* pHandle, Observer_Inputs_t* pInput)
{
	// select sigma points
	UKF_SelectSigmaPoints(pHandle);

	// propagate the sigma points
	// obtain mean and covariance: Xpred, CovarianceP
	UKF_PropagateSigmaPointsX(pHandle, pInput);

	// select sigma points using Xpred, CovarianceP
	UKF_SelectSigmaPoints(pHandle);

	// transfer through measurement model Ypred
	// compute covariance of Py
	// compute covariance of Pxy
	UKF_PropagateSigmaPointsY(pHandle, pInput);

	// update the estimation
	UKF_UpdateParameters(pHandle);
}

void UKF_SelectSigmaPoints(UKF_Handle_t* pHandle)
{

	int16_t sigmaIndex;
	int16_t matSize = pHandle->NumState;
	int16_t stateIndex;

	Matrix_t* pSqrtP = pHandle->CovariancePx;

	UKF_ScalarMul(pSqrtP, matSize); // n*P

	UKF_CholeskyDecomposition(pSqrtP); // sqrt(n*P)

	for (sigmaIndex=0; sigmaIndex<pHandle->SigmaPointSize>>1; sigmaIndex++)
	{
		for (stateIndex=0; stateIndex<pHandle->NumState; stateIndex++) // x_bar added at init() or update()
		{
			sigmaPoint[matSize*stateIndex+sigmaIndex] = pHandle->xPred[stateIndex] + pSqrtP[matSize*sigmaIndex+stateIndex]; // x(i) = x_bar + sqrt(nP)(i)
			sigmaPoint[matSize*stateIndex+(sigmaIndex<<1)] = pHandle->xPred[stateIndex] - pSqrtP[matSize*sigmaIndex+stateIndex]; // x(i) = x_bar - sqrt(nP)(i)
		}

	}
}

void UKF_PropagateSigmaPointsX(UKF_Handle_t* pHandle, Observer_Inputs_t* pInput)
{

}

void UKF_PropagateSigmaPointsY(UKF_Handle_t* pHandle, Observer_Inputs_t* pInput)
{

}

void UKF_UpdateParameters(UKF_Handle_t* pHandle)
{

}

void UKF_SetCovarianceMatrices(UKF_Handle_t* pHandle, int32_t* hQ, int32_t* hR);

void UKF_CholeskyDecomposition(Matrix_t* pHandle)
{
	const int nRow = pHandle->nRows;
	const int nCol = pHandle->nCols;
	float* const mat = pHandle->pVal;

	const int MatSize = nRow;

	int16_t col, row, tmp;
	float sum = 0;

	for(row=0; row<MatSize; row++)
	{
		for(col=0; col<MatSize; col++)
		{
			sum = mat[MatSize*row+col];

			for (tmp=0;tmp<row;tmp++) // (a_kj - sum^(k-1)_(l=1) f_lk * f_lj)
			{
				sum -= mat[MatSize*tmp+row]*mat[MatSize*tmp+col];
			}

			if (row==col)
			{
				mat[Matsize*row+col] = sqrt(sum);
			}
			else
			{
				mat[Matsize*row+col] = sum / mat[Matsize*row+row];
			}

		}
	}
}
