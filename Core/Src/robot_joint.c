/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    robot_joint.c
  * @brief   Robot arm 6-DOF joint control — implementation
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "robot_joint.h"
#include "tim.h"
#include <math.h>

/* ---------------------------------------------------------------------------*/
/*  Private helpers                                                            */
/* ---------------------------------------------------------------------------*/

/**
 * @brief  Clamp a float value within [lo, hi].
 */
static inline float clampf(float val, float lo, float hi)
{
  if (val < lo) return lo;
  if (val > hi) return hi;
  return val;
}

/**
 * @brief  Convert an angle (degrees) to a PWM CCR value.
 *         Takes into account the joint's offset and direction.
 *
 *         effective_angle = offset + direction * angle
 *         pulse = map(effective_angle, 0°..180°, 500..2500)
 */
static uint32_t angle_to_ccr(const RbJointConfig_t *j, float angle)
{
  float effective = j->offset + (float)j->direction * angle;
  effective = clampf(effective, 0.0f, 180.0f);

  /* Linear map: 0° → SERVO_PULSE_MIN, 180° → SERVO_PULSE_MAX */
  float pulse = (float)SERVO_PULSE_MIN
              + (effective / 180.0f) * (float)(SERVO_PULSE_MAX - SERVO_PULSE_MIN);

  return (uint32_t)(pulse + 0.5f);   /* round to nearest tick */
}

/**
 * @brief  Apply CCR value to the servo's timer channel.
 */
static void apply_ccr(const RbJointConfig_t *j, uint32_t ccr)
{
  __HAL_TIM_SET_COMPARE(j->htim, j->channel, ccr);
}

/* ---------------------------------------------------------------------------*/
/*  Public API                                                                 */
/* ---------------------------------------------------------------------------*/

void RbJoint_Init(RbJoint_t *rb)
{
  /*
   * ── Hardware binding table ──
   *
   *  Index  │  Joint  │  Timer  │  Channel       │  Pin
   * ────────┼─────────┼─────────┼────────────────┼──────
   *   0     │  J1     │  TIM3   │  CH1           │  PA6
   *   1     │  J2     │  TIM3   │  CH2           │  PA4
   *   2     │  J3     │  TIM3   │  CH3           │  PB0
   *   3     │  J4     │  TIM3   │  CH4           │  PB1
   *   4     │  J5     │  TIM4   │  CH1           │  PA11
   *   5     │  J6     │  TIM4   │  CH2           │  PA12
   */

  TIM_HandleTypeDef *timers[RB_NUM_JOINTS]  = {
    &htim3, &htim3, &htim3, &htim3, &htim4, &htim4
  };
  uint32_t channels[RB_NUM_JOINTS] = {
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4,
    TIM_CHANNEL_1, TIM_CHANNEL_2
  };

  for (int i = 0; i < RB_NUM_JOINTS; i++)
  {
    RbJointConfig_t *j = &rb->joint[i];

    /* Hardware */
    j->htim    = timers[i];
    j->channel = channels[i];

    /* Default mechanical parameters — override after Init if needed */
    j->angle_min  = JOINT_ANGLE_MIN;
    j->angle_max  = JOINT_ANGLE_MAX;
    j->offset     = 0.0f;
    j->direction  = 1;       /* normal direction */
    j->home_angle = 0.0f;    /* default position */

    /* Runtime state */
    j->current_angle = j->home_angle;
    j->target_angle  = j->home_angle;
  }

  /* Global defaults */
  rb->interp_step_deg = 2.0f;   /* 2°/tick → 100°/s at 50Hz update rate */
  rb->is_moving       = false;

  /* Start all PWM channels */
  for (int i = 0; i < RB_NUM_JOINTS; i++)
  {
    HAL_TIM_PWM_Start(rb->joint[i].htim, rb->joint[i].channel);
  }

  /* Move to home position */
  RbJoint_GoHome(rb);
}

/* ── Single joint ────────────────────────────────────────────────────────── */

void RbJoint_SetAngle(RbJoint_t *rb, RbJointIndex_e idx, float angle)
{
  if (idx >= RB_NUM_JOINTS) return;

  RbJointConfig_t *j = &rb->joint[idx];
  angle = clampf(angle, j->angle_min, j->angle_max);

  j->current_angle = angle;
  j->target_angle  = angle;
  apply_ccr(j, angle_to_ccr(j, angle));
}

float RbJoint_GetAngle(const RbJoint_t *rb, RbJointIndex_e idx)
{
  if (idx >= RB_NUM_JOINTS) return 0.0f;
  return rb->joint[idx].current_angle;
}

/* ── All joints at once (inverse kinematics entry point) ─────────────── */

void RbJoint_SetAllAngles(RbJoint_t *rb, const float angles[RB_NUM_JOINTS])
{
  for (int i = 0; i < RB_NUM_JOINTS; i++)
  {
    RbJoint_SetAngle(rb, (RbJointIndex_e)i, angles[i]);
  }
}

/* ── Smooth motion (trajectory interpolation) ────────────────────────── */

void RbJoint_SetTargetAngles(RbJoint_t *rb, const float angles[RB_NUM_JOINTS])
{
  for (int i = 0; i < RB_NUM_JOINTS; i++)
  {
    RbJointConfig_t *j = &rb->joint[i];
    j->target_angle = clampf(angles[i], j->angle_min, j->angle_max);
  }
  rb->is_moving = true;
}

bool RbJoint_Update(RbJoint_t *rb)
{
  bool all_done = true;
  float step = rb->interp_step_deg;

  for (int i = 0; i < RB_NUM_JOINTS; i++)
  {
    RbJointConfig_t *j = &rb->joint[i];
    float diff = j->target_angle - j->current_angle;

    if (fabsf(diff) < 0.1f)
    {
      /* Close enough — snap to target */
      j->current_angle = j->target_angle;
    }
    else
    {
      all_done = false;
      /* Move by step in the correct direction */
      if (diff > 0.0f)
        j->current_angle += (diff > step) ? step : diff;
      else
        j->current_angle -= ((-diff) > step) ? step : (-diff);
    }

    apply_ccr(j, angle_to_ccr(j, j->current_angle));
  }

  rb->is_moving = !all_done;
  return all_done;
}

/* ── Utilities ───────────────────────────────────────────────────────── */

void RbJoint_GoHome(RbJoint_t *rb)
{
  for (int i = 0; i < RB_NUM_JOINTS; i++)
  {
    RbJoint_SetAngle(rb, (RbJointIndex_e)i, rb->joint[i].home_angle);
  }
}

void RbJoint_SetSpeed(RbJoint_t *rb, float deg_per_tick)
{
  rb->interp_step_deg = deg_per_tick;
}
