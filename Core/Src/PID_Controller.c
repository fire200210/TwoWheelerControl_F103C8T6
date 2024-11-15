/*
 *  PID_t_Controller.c
 *
 *  Created on: 2024年9月27日
 *      Author: lin09
 */

#include "PID_Controller.h"

#define PID_Version  4U

void PID_Init(PID_t *PID, float Kp, float Ki, float Kd)
{
  PID->Kp = Kp;
  PID->Ki = Ki;
  PID->Kd = Kd;
  PID->PreviousError = PID->Integral = 0.0;
}

void PID_ParameterClear(PID_t *PID)
{
  PID->PWM_Value = PID->PreviousError = PID->Integral = 0.0;
}

inline void CalculateSpeed(TIM_HandleTypeDef *htim, Encoder_t *Encoder)
{
  Encoder->Pulse = (int16_t)htim->Instance->CNT;
  htim->Instance->CNT = 0;
  Encoder->Direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(htim);

  Encoder->DeltaTime = (float)Gettime(Encoder->StartTime) / 1000.0f;
  Encoder->StartTime = HAL_GetTick();

  if ((Encoder->Pulse > 0 || Encoder->Pulse < 0) && Encoder->DeltaTime > 0.0f)
  {
    Encoder->Pulse = abs(Encoder->Pulse);
    Encoder->CurrentRPM = ((float)Encoder->Pulse / Wheel_PPR) * (60.0f / Encoder->DeltaTime);
    Encoder->Speed_CM = (Encoder->CurrentRPM * Wheel_Circumference) / 60.0f;
    Encoder->DistanceTotal += ((float)Encoder->Pulse / Wheel_PPR) * Wheel_Circumference;
  }else{
    Encoder->CurrentRPM = 0;
    Encoder->Speed_CM = 0;
  }
}

#if(PID_Version == 1)

  float PID_Compute(PID_t *PID, const Encoder_t Encoder)
  {
    PID->dt = (float)Gettime(PID->StartTime) / 1000.0f;
    PID->StartTime = HAL_GetTick();

    PID->NewError = (Encoder.TargetRPM - Encoder.CurrentRPM);
    PID->Integral += PID->NewError * PID->dt;

    PID->Derivative = (PID->NewError - PID->PreviousError) / PID->dt;
    PID->PreviousError = PID->NewError;

    PID->PWM_Value = (PID->Kp * PID->NewError) + (PID->Ki * PID->Integral) + (PID->Kd * PID->Derivative);

    return PID->PWM_Value;
  }

#elif (PID_Version == 2)

  float PID_Compute(PID_t *PID, const Encoder_t Encoder)
  {
    uint8_t index;

    PID->dt = (float)Gettime(PID->StartTime) / 1000.0f;
    PID->StartTime = HAL_GetTick();

    PID->NewError = (Encoder.TargetRPM - Encoder.CurrentRPM);

    (fabsf(PID->NewError) > Encoder.TargetRPM) ? (index = 0) : (index = 1, PID->Integral += PID->NewError * PID->dt);

    PID->Derivative = (PID->NewError - PID->PreviousError) / PID->dt;
    PID->PreviousError = PID->NewError;

    PID->P = (PID->Kp * PID->NewError);
    PID->I = (PID->Ki * PID->Integral * index);
    PID->D = (PID->Kd * PID->Derivative);

    PID->PWM_Value = (PID->P + PID->I + PID->D) * 10;

    if     (PID->PWM_Value <   0.0) PID->PWM_Value = 0;
    else if(PID->PWM_Value > 42000) PID->PWM_Value = 42000;

    return PID->PWM_Value;
  }

#elif (PID_Version == 3)

  float PID_Compute(PID_t *PID, const Encoder_t Encoder)
  {
    // 計算當前誤差
    PID->NewError = (Encoder.TargetRPM - Encoder.CurrentRPM);

    // 微分濾波，減少噪聲對微分項的影響 (alpha 值介於 0.0 ~ 1.0)
    float alpha = 0.9;
    PID->NewError = alpha * PID->NewError + (1 - alpha) * PID->ErrorNext;

    // 抗抖動 (Deadband)，避免輸出振動
    float deadband_threshold = 0.01;  // 設定死區閾值
    if (fabsf(PID->NewError) < deadband_threshold) PID->NewError = 0;

    // 計算 P, I, D 項
    PID->P = (PID->Kp * (PID->NewError - PID->ErrorNext));  // 使用當前誤差直接計算 P 項
    PID->I = (PID->Ki * PID->NewError);  // 保持積分項計算
    PID->D = (PID->Kd * (PID->NewError - 2 * PID->ErrorNext + PID->ErrorLast));  // 微分項計算

    // PWM 值增量計算
    PID->PWM_Value += (PID->P + PID->I + PID->D);

    // 更新誤差
    PID->ErrorLast = PID->ErrorNext;
    PID->ErrorNext = PID->NewError;

    // 抗積分飽和，防止積分項過大
    if (PID->PWM_Value > 42000 && PID->NewError > 0) PID->I = 0;
    if (PID->PWM_Value <     0 && PID->NewError < 0) PID->I = 0;

    // 限制 PWM 值的範圍
    if     (PID->PWM_Value <   0.0) PID->PWM_Value = 0;
    else if(PID->PWM_Value > 42000) PID->PWM_Value = 42000;

    return PID->PWM_Value;
  }

#elif(PID_Version == 4)

  inline float PID_Compute(PID_t *PID, const Encoder_t Encoder)
  {
    PID->NewError = (Encoder.TargetRPM - Encoder.CurrentRPM);

    const float Alpha = 0.9;
    PID->NewError = Alpha * PID->NewError + (1 - Alpha) * PID->ErrorNext;

    if(fabsf(PID->NewError) < 5.0) PID->NewError = 0;

    PID->P = (PID->Kp * (PID->NewError - PID->ErrorNext));
    PID->I = (PID->Ki * PID->NewError);
    PID->D = (PID->Kd * (PID->NewError - 2 * PID->ErrorNext + PID->ErrorLast));

    PID->PWM_Value += (PID->P + PID->I + PID->D);
    PID->ErrorLast = PID->ErrorNext;
    PID->ErrorNext = PID->NewError;

    // 需要增加一個避免馬達暴衝的限制

//    if(Encoder.TargetPWM + 100.0 < PID->PWM_Value) PID->PWM_Value = Encoder.TargetPWM + 100.0;

    if     (PID->PWM_Value <   0.0) PID->PWM_Value = 0.0;
    else if(PID->PWM_Value > 40000) PID->PWM_Value = 40000;

    return PID->PWM_Value;
  }

#endif

void SaveTargetPWM_Coefficient(PWM_LimitCalibration_t *PWM_LimitCal,
                               UART_HandleTypeDef     *huart,
                               uint16_t                TargetPWM_Coefficient[],
                               const uint8_t           SelectMOTOR)
{
  for(uint8_t i = 0; i < 20; i++)
  {
    TargetPWM_Coefficient[i] = PWM_LimitCal->TargetPWM_CoefficientTemp[i];
    PWM_LimitCal->TargetPWM_CoefficientTemp[i] = 0;
  }
  PWM_LimitCal->CalState[SelectMOTOR] = NONECTRL;
  SendINFO(huart, "Copy completed");
}

void CancelCalibrationAction(PWM_LimitCalibration_t *PWM_LimitCal,
                             UART_HandleTypeDef     *huart,
                             uint16_t                TargetPWM_Coefficient[],
                             const uint8_t           SelectMOTOR)
{
  for(uint8_t i = 0; i < 20; i++) PWM_LimitCal->TargetPWM_CoefficientTemp[i] = 0;
  PWM_LimitCal->CalState[SelectMOTOR] = NONECTRL;
  SendINFO(huart, "Cancel Calibration");
}

void CaptureCalibrationValues(Encoder_t              *Encoder,
                              PID_t                  *PID,
                              PWM_LimitCalibration_t *PWM_LimitCal,
                              UART_HandleTypeDef     *huart,
                              const float             FloatingValue,
                              const uint8_t           SelectMOTOR,
                              const uint8_t           CaptureNUM)
{
  if(   PWM_LimitCal->CalibrationRPM - FloatingValue < Encoder->CurrentRPM
     && PWM_LimitCal->CalibrationRPM + FloatingValue > Encoder->CurrentRPM
     && (PWM_LimitCal->CalibrationSpeed <= 100 && PWM_LimitCal->CalibrationSpeed >= 0)
     && PWM_LimitCal->CalState[SelectMOTOR] == STARTCAL)
  {
    PWM_LimitCal->CountTemp++;
    PWM_LimitCal->PWM_CoefficientTemp += (uint32_t)PID->PWM_Value;
    if(PWM_LimitCal->CountTemp == CaptureNUM)
    {
      PWM_LimitCal->CountTemp = 0;
      PWM_LimitCal->PWM_CoefficientTemp /= CaptureNUM;
      if(PWM_LimitCal->CalibrationSpeed) PWM_LimitCal->TargetPWM_CoefficientTemp[(PWM_LimitCal->CalibrationSpeed / 5) - 1] = PWM_LimitCal->PWM_CoefficientTemp;
      PWM_LimitCal->PWM_CoefficientTemp = 0;
      if(PWM_LimitCal->CalibrationSpeed != 0) SendINFO(huart, "TargetPWM_Coefficient[%u]:%u\n", ((PWM_LimitCal->CalibrationSpeed / 5) - 1), PWM_LimitCal->TargetPWM_CoefficientTemp[(PWM_LimitCal->CalibrationSpeed/ 5) - 1]);
      PWM_LimitCal->CalibrationSpeed += 5;
      if(PWM_LimitCal->CalibrationSpeed <= 100) Encoder->TargetRPM = PWM_LimitCal->CalibrationRPM = (float)(8.0f * PWM_LimitCal->CalibrationSpeed);
      if(PWM_LimitCal->CalibrationSpeed >  100)
      {
        Encoder->TargetRPM = PWM_LimitCal->CalibrationRPM = 0;
        PWM_LimitCal->CalState[SelectMOTOR] = CALEND;
      }
    }
  } else if(PWM_LimitCal->CalibrationSpeed > 100 && PWM_LimitCal->CalState[SelectMOTOR] == CALEND) {
    PWM_LimitCal->CalibrationSpeed = 0;
    SendINFO(huart, "Calibration finish!!!\n YES or NO save coefficient\n");
  }
}

void MOTOR_TargetPWM_Calibration(Encoder_t              *Encoder,
                                 PID_t                  *PID,
                                 PWM_LimitCalibration_t *PWM_LimitCal,
                                 UART_HandleTypeDef     *huart,
                                 uint16_t                TargetPWM_Coefficient[],
                                 uint8_t                 Select)
{
  switch(Select)
  {
    case 0x4C: //L

      break;

    case 0x52: //R

      break;
  }
}

static inline double mypow(double base, int exponent)
{
  double result = 1.0;
  while(exponent > 0)
  {
    if(exponent % 2 == 1) result *= base; // 如果指數是奇數
    base *= base;  // 將 base 提升為平方
    exponent /= 2; // 指數減半
  }
  return result;
}

inline double CalculateTargetPWM(const double Coefficient[], const uint8_t speed)
{
  return (Coefficient[5] * mypow(speed, 5) +
          Coefficient[4] * mypow(speed, 4) +
          Coefficient[3] * mypow(speed, 3) +
          Coefficient[2] * mypow(speed, 2) +
          Coefficient[1] * speed +
          Coefficient[0]);
}

// 高斯消去法求解方程組
void gaussian_elimination(int n, double a[n][n+1], double x[n])
{
  for(int i = 0; i < n; i++)
  {
    // 找到最大列的值並交換行
    int max = i;
    for(int k = i + 1; k < n; k++)
    {
      if (fabs(a[k][i]) > fabs(a[max][i]))
      {
        max = k;
      }
    }

    // 交換行
    for(int k = i; k <= n; k++)
    {
      double temp = a[max][k];
      a[max][k] = a[i][k];
      a[i][k] = temp;
    }

    // 高斯消去
    for(int k = i + 1; k < n; k++)
    {
      double factor = a[k][i] / a[i][i];
      for (int j = i; j <= n; j++)
      {
        a[k][j] -= factor * a[i][j];
      }
    }
  }

  // 回代求解
  for (int i = n - 1; i >= 0; i--)
  {
    x[i] = a[i][n] / a[i][i];
    for (int k = i - 1; k >= 0; k--)
    {
      a[k][n] -= a[k][i] * x[i];
    }
  }
}

// 用於計算多項式擬合的係數 (5 次多項式)
void polynomial_fit(uint16_t x[], uint16_t y[], int n, int degree, double coeff[])
{
  int i, j, k;

  // 構建 (degree + 1) * (degree + 2) 的矩陣
  double A[degree+1][degree+2];

  // 初始化矩陣
  for(i = 0; i <= degree; i++)
  {
    for (j = 0; j <= degree; j++)
    {
      A[i][j] = 0;
      for (k = 0; k < n; k++)
      {
        A[i][j] += pow(x[k], i+j);  // 矩陣中的次數和
      }
    }

    A[i][degree+1] = 0;
    for (k = 0; k < n; k++)
    {
      A[i][degree+1] += pow(x[k], i) * y[k];  // 最後一列
    }
  }

  // 使用高斯消去法解方程組
  gaussian_elimination(degree+1, A, coeff);
}
