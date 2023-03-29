/*
 * ukf_speed_pos_fdbk.h
 *
 *  Created on: 2023. 3. 27.
 *      Author: KAIST
 *
 *  based on "Design of load torque and mechanical speed estimator of PMSM with unscented Kalman filter — An engineering guide"
 *  by Karol Kyslan
 *  doi : https://doi.org/10.1109/EDPE.2017.8123249
 */

#ifndef MIDDLEWARES_MOTORCONTROL_UKF_SPEED_POS_FDBK_H_
#define MIDDLEWARES_MOTORCONTROL_UKF_SPEED_POS_FDBK_H_

#define SQRT5FACTOR (uint16_t) 0x8F1C // 16384 * 1.732

#include "speed_pos_fdbk.h"
#include "pmsm_motor_parameters.h"
#include "math.h"  // for sqrt



typedef struct
{
	int16_t nRows;
	int16_t nCols;
	int16_t nSize;
	float* pVal;

}Matrix_t;

typedef struct
{
	SpeednPosFdbk_Handle_t _Super;

	int16_t NumState;

	float* xPred; // Ialpha, Ibeta, speed, angle, Tload
	float* yPred;

	float* SigmaPoint;

	int16_t SigmaPointSize;

	Matrix_t* CovariancePx;

	Matrix_t* CovariancePy;

	Matrix_t* CovariancePxy;

	Matrix_t* CovarianceQ;

	Matrix_t* CovarianceR;


}UKF_Handle_t;


void UKF_Init(UKF_Handle_t* pHandle);

void UKF_Clear(UKF_Handle_t* pHandle);

/* Iteration for FOC_CurrControlM1() */
void UKF_Step(UKF_Handle_t* pHandle);

int16_t UKF_CalcAvrgMecSpeedUnit(UKF_Handle_t* pHandle, int16_t* pMecSpeedUnit);

int16_t UKF_CalcAvrgElSpeedDpp(UKF_Handle_t* pHandle);

/* ---------------------------Kalman filter step ----------------------------- */

void UKF_SelectSigmaPoints(UKF_Handle_t* pHandle);

void UKF_PropagateSigmaPointsX(UKF_Handle_t* pHandle, Observer_Inputs_t* pInput);

void UKF_PropagateSigmaPointsY(UKF_Handle_t* pHandle, Observer_Inputs_t* pInput);

void UKF_UpdateParameters(UKF_Handle_t* pHandle);

/* -------------------------------Matrix Operation------------------------------- */

void UKF_CholeskyDecomposition(Matrix_t* pHandle);

void UKF_Transpose(Matrix_t* pHandle);

void UKF_ScalarMul(Matrix_t* pHandle, int16_t pVal);

/* ----------------------------------Settings---------------------------------- */

void UKF_SetCovarianceMatrices(UKF_Handle_t* pHandle, int32_t* hQ, int32_t* hR);




#endif /* MIDDLEWARES_MOTORCONTROL_UKF_SPEED_POS_FDBK_H_ */
