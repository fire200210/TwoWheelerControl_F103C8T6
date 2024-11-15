/*
 * PID_Controller.h
 *
 *  Created on: 2024年9月27日
 *      Author: lin09
 */

#ifndef INC_PID_CONTROLLER_H_
#define INC_PID_CONTROLLER_H_

#include "main.h"
#include <math.h>

typedef struct {
  float     Kp; // 比例增益
  float     Ki; // 積分增益
  float     Kd; // 微分增益
  float     P, I, D;
  float     PWM_Value;
  float     NewError;
  float     PreviousError; // 上一次的誤差
  float     Derivative;
  float     Integral;      // 積分累計值
  float     dt;
  uint32_t  StartTime;
  float     ErrorLast, ErrorNext;
}PID_t;

typedef struct{
  int16_t   Pulse;
  uint32_t  StartTime;
  float     DeltaTime;
  float     CurrentRPM;
  float     TargetRPM;
  float     TargetPWM;
  float     Speed_CM;
  float     DistanceTotal;
  bool      Direction;
}Encoder_t;

typedef enum{
  NONECTRL  = 0x00U,
  STARTCAL  = 0x01U,
  CALEND    = 0x02U,
}CalibrationState_t;

typedef struct{
  uint8_t   CountTemp;
  uint8_t   CalibrationSpeed;
  float     CalibrationRPM;
  uint32_t  PWM_CoefficientTemp;
  uint16_t  TargetPWM_CoefficientTemp[20];

  CalibrationState_t CalState[2];
}PWM_LimitCalibration_t;

void PID_Init(PID_t *PID, float Kp, float Ki, float Kd);
void PID_ParameterClear(PID_t *PID);
float PID_Compute(PID_t *PID, const Encoder_t Encoder);
void CalculateSpeed(TIM_HandleTypeDef *htim, Encoder_t *Encoder);

void CaptureCalibrationValues(Encoder_t              *Encoder,
                              PID_t                  *PID,
                              PWM_LimitCalibration_t *PWM_LimitCal,
                              UART_HandleTypeDef     *huart,
                              const float             FloatingValue,
                              const uint8_t           SelectMOTOR,
                              const uint8_t           CaptureNUM);

double CalculateTargetPWM(const double Coefficient[], const uint8_t speed);
// 高斯消去法求解方程組
void gaussian_elimination(int n, double a[n][n+1], double x[n]);
// 用於計算多項式擬合的係數 (5 次多項式)
void polynomial_fit(uint16_t x[], uint16_t y[], int n, int degree, double coeff[]);
#endif /* INC_PID_CONTROLLER_H_ */
