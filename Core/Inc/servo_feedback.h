/**
 ******************************************************************************
 * @file    servo_feedback.h
 * @brief   Read servo feedback potentiometer via ADC
 *
 *          MG996R feedback (VR tap) after removing mechanical stops:
 *            Pot covers full 270° range
 *            ADC values at extremes MUST be measured on your servo!
 *
 *          Currently wired:
 *            J4 feedback → PA0 (ADC1_IN1)
 ******************************************************************************
 */

#ifndef __SERVO_FEEDBACK_H
#define __SERVO_FEEDBACK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ---------------------------------------------------------------------------*/
/*  Calibration constants (adjust after measuring your actual servo)           */
/* ---------------------------------------------------------------------------*/

/** ADC values at known positions — measure these with your servo! */
#define FB_ADC_AT_0DEG   2941   /**< ADC reading when servo is at 0°   (2.37V) */
#define FB_ADC_AT_MAX    709    /**< ADC reading when servo is at 180° (measured) */

/** Full mechanical range of the servo (degrees) */
#define FB_SERVO_RANGE   180.0f

/* ---------------------------------------------------------------------------*/
/*  Debug / Live view variables                                                */
/* ---------------------------------------------------------------------------*/

typedef struct {
  uint32_t adc_raw;       /**< Raw 12-bit ADC value (0-4095)         */
  float    voltage;       /**< Converted voltage (V)                 */
  float    actual_angle;  /**< Computed angle from feedback (deg)    */
  float    adc_at_zero;   /**< ADC value stored as "zero" reference  */
  float    angle_offset;  /**< Offset: actual_angle at calibration   */
} ServoFeedback_t;

/* ---------------------------------------------------------------------------*/
/*  API                                                                        */
/* ---------------------------------------------------------------------------*/

/**
 * @brief  Initialise ADC1 for reading PA0 (single conversion, polling).
 *         Call after HAL_Init() and SystemClock_Config().
 */
void ServoFB_Init(void);

/**
 * @brief  Read ADC, compute voltage and angle. Store in fb struct.
 * @param  fb  Pointer to ServoFeedback_t to fill with results.
 */
void ServoFB_Read(ServoFeedback_t *fb);

/**
 * @brief  Calibrate zero: read current position and store as 0° reference.
 *         After calling this, actual_angle will be relative to this position.
 * @param  fb  Pointer to ServoFeedback_t
 */
void ServoFB_CalibrateZero(ServoFeedback_t *fb);

/**
 * @brief  Get the angle relative to the calibrated zero point.
 * @param  fb  Pointer to ServoFeedback_t (must have called Read first)
 * @return Angle in degrees relative to zero reference
 */
float ServoFB_GetCalibratedAngle(const ServoFeedback_t *fb);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_FEEDBACK_H */
