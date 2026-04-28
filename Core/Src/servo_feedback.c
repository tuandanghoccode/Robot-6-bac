/**
 ******************************************************************************
 * @file    servo_feedback.c
 * @brief   Read MG996R feedback potentiometer via ADC1 (PA0 = ADC1_IN1)
 *
 *          Manually configures ADC1 without CubeMX-generated adc.c,
 *          so you don't need to re-generate code.
 ******************************************************************************
 */

#include "servo_feedback.h"

/* ---------------------------------------------------------------------------*/
/*  Private: ADC handle (managed locally, not by CubeMX)                      */
/* ---------------------------------------------------------------------------*/
static ADC_HandleTypeDef hadc1;

/* ---------------------------------------------------------------------------*/
/*  Init */
/* ---------------------------------------------------------------------------*/

void ServoFB_Init(void) {
  /* ── Enable clocks ── */
  __HAL_RCC_ADC12_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* PA0 already configured as analog in MX_GPIO_Init(), but make sure */
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_0;
  gpio.Mode = GPIO_MODE_ANALOG;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &gpio);

  /* ── ADC1 configuration ── */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4; /* 170/4 = 42.5MHz */
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE; /* Single channel */
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE; /* One-shot */
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.OversamplingMode = DISABLE;

  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  /* Calibrate ADC (important for accuracy on STM32G4) */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  /* ── Channel config: IN1 (PA0) ── */
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5; /* Long sample = stable */
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
}

/* ---------------------------------------------------------------------------*/
/*  Read */
/* ---------------------------------------------------------------------------*/

void ServoFB_Read(ServoFeedback_t *fb) {
  /* Start conversion, wait, read */
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 10); /* 10ms timeout */
  fb->adc_raw = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);

  /* Convert to voltage: V = adc_raw / 4095 * 3.3 */
  fb->voltage = (float)fb->adc_raw / 4095.0f * 3.3f;

  /*
   * Convert to angle (inverted relationship):
   *   ADC_AT_0DEG (high voltage) → 0°
   *   ADC_AT_MAX  (low voltage)  → 270°
   *
   *   angle = (ADC_AT_0DEG - adc_raw) / (ADC_AT_0DEG - ADC_AT_MAX) * 270
   */
  float ratio = (float)(FB_ADC_AT_0DEG - (int32_t)fb->adc_raw) /
                (float)(FB_ADC_AT_0DEG - FB_ADC_AT_MAX);

  fb->actual_angle = ratio * FB_SERVO_RANGE;

  /* Clamp to valid range */
  if (fb->actual_angle < 0.0f)
    fb->actual_angle = 0.0f;
  if (fb->actual_angle > FB_SERVO_RANGE)
    fb->actual_angle = FB_SERVO_RANGE;
}

/* ---------------------------------------------------------------------------*/
/*  Calibrate zero */
/* ---------------------------------------------------------------------------*/

void ServoFB_CalibrateZero(ServoFeedback_t *fb) {
  /* Read current position */
  ServoFB_Read(fb);

  /* Store current ADC and angle as the zero reference */
  fb->adc_at_zero = (float)fb->adc_raw;
  fb->angle_offset = fb->actual_angle;
}

float ServoFB_GetCalibratedAngle(const ServoFeedback_t *fb) {
  /* Angle relative to calibrated zero */
  return fb->actual_angle - fb->angle_offset;
}
