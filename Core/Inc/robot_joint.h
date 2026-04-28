/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    robot_joint.h
 * @brief   Robot arm 6-DOF joint control module
 *
 *          Hardware mapping (STM32G474 @ 170MHz):
 *            J1 → TIM3_CH1  (PA6)
 *            J2 → TIM3_CH2  (PA4)
 *            J3 → TIM3_CH3  (PB0)
 *            J4 → TIM3_CH4  (PB1)
 *            J5 → TIM4_CH1  (PA11)
 *            J6 → TIM4_CH2  (PA12)
 *
 *          PWM: 50Hz (Prescaler=169, Period=19999)
 *            500us  → 0°     (CCR = 500)
 *            1500us → 90°    (CCR = 1500)
 *            2500us → 180°   (CCR = 2500)
 ******************************************************************************
 */
/* USER CODE END Header */

#ifndef __ROBOT_JOINT_H
#define __ROBOT_JOINT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>

/* ---------------------------------------------------------------------------*/
/*  Constants */
/* ---------------------------------------------------------------------------*/

#define RB_NUM_JOINTS 6

/** PWM pulse-width limits (in timer ticks, 1 tick = 1us at 1MHz) */
#define SERVO_PULSE_MIN 500  /* 0.5 ms   → 0°   */
#define SERVO_PULSE_MID 1500 /* 1.5 ms   → 90°  */
#define SERVO_PULSE_MAX 2500 /* 2.5 ms   → 180° */

/** Full servo range (degrees) */
#define SERVO_RANGE 180.0f

/** Default angle limits (degrees) – override per joint if needed */
#define JOINT_ANGLE_MIN 0.0f
#define JOINT_ANGLE_MAX 180.0f

/* ---------------------------------------------------------------------------*/
/*  Types */
/* ---------------------------------------------------------------------------*/

/** Index enum – use these instead of magic numbers */
typedef enum {
  JOINT_J1 = 0,
  JOINT_J2,
  JOINT_J3,
  JOINT_J4,
  JOINT_J5,
  JOINT_J6
} RbJointIndex_e;

/** Per-joint configuration & state */
typedef struct {
  /* --- Hardware binding (set once at init) --- */
  TIM_HandleTypeDef *htim; /**< Timer handle (htim3 or htim4)         */
  uint32_t channel;        /**< TIM_CHANNEL_x                         */

  /* --- Mechanical parameters (set once at init) --- */
  float angle_min;  /**< Software lower limit  [deg]                      */
  float angle_max;  /**< Software upper limit  [deg]                      */
  float offset;     /**< Mechanical offset     [deg] (added before pulse) */
  int8_t direction; /**< +1 = normal, -1 = reversed servo direction       */
  float home_angle; /**< Home / default angle  [deg]                      */

  /* --- Runtime state --- */
  float current_angle; /**< Last commanded angle [deg]                     */
  float target_angle;  /**< Target angle for interpolation [deg]           */
  float start_angle;   /**< Angle when motion started [deg]                */
} RbJointConfig_t;

/** Top-level struct: the whole robot arm */
typedef struct {
  RbJointConfig_t joint[RB_NUM_JOINTS]; /**< J1..J6                      */

  /* --- Smooth motion --- */
  uint32_t move_start_tick;  /**< HAL_GetTick() when motion started       */
  uint32_t duration_ms;      /**< Total motion duration (ms)              */
  bool is_moving;            /**< True while any joint hasn't reached target */
} RbJoint_t;

/* ---------------------------------------------------------------------------*/
/*  Public API */
/* ---------------------------------------------------------------------------*/

/**
 * @brief  Initialise the RbJoint module.
 *         Binds timers, sets limits/offsets, starts PWM, moves to home.
 * @note   Call AFTER MX_TIM3_Init() and MX_TIM4_Init().
 */
void RbJoint_Init(RbJoint_t *rb);

/**
 * @brief  Set a single joint angle immediately (no interpolation).
 * @param  idx   JOINT_J1 … JOINT_J6
 * @param  angle Desired angle in degrees (clamped to [angle_min, angle_max])
 */
void RbJoint_SetAngle(RbJoint_t *rb, RbJointIndex_e idx, float angle);

/**
 * @brief  Get the current commanded angle of a joint.
 */
float RbJoint_GetAngle(const RbJoint_t *rb, RbJointIndex_e idx);

/**
 * @brief  Set ALL 6 joint angles at once — the main entry for inverse
 * kinematics. Angles are applied immediately (no interpolation).
 * @param  angles  Array of 6 floats [J1, J2, J3, J4, J5, J6] in degrees
 */
void RbJoint_SetAllAngles(RbJoint_t *rb, const float angles[RB_NUM_JOINTS]);

/**
 * @brief  Set ALL 6 target angles for smooth ease-in/ease-out motion.
 *         Call RbJoint_Update() periodically (e.g. every 20ms) to animate.
 * @param  angles  Array of 6 target angles in degrees
 */
void RbJoint_SetTargetAngles(RbJoint_t *rb, const float angles[RB_NUM_JOINTS]);

/**
 * @brief  Animate joints from start to target using ease-in/ease-out.
 *         Call this in main loop or timer ISR (recommended 50Hz).
 * @retval true if all joints have reached their targets
 */
bool RbJoint_Update(RbJoint_t *rb);

/**
 * @brief  Move all joints to their home_angle immediately.
 */
void RbJoint_GoHome(RbJoint_t *rb);

/**
 * @brief  Set the motion duration (how long a move takes).
 * @param  ms  Duration in milliseconds (default 1000ms)
 */
void RbJoint_SetDuration(RbJoint_t *rb, uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* __ROBOT_JOINT_H */
