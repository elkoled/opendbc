#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/volkswagen_common.h"

// MEB-specific CAN message addresses
#define MSG_ESC_51        0x0FCU   // RX, ABS wheel speeds, brake pressure
#define MSG_Motor_51      0x10BU   // RX, drivetrain coordinator: TSK_Status, accel pedal
#define MSG_QFK_01        0x13DU   // RX, EPS lateral controller status, measured curvature
#define MSG_ACC_18        0x14DU   // TX, ACC acceleration request to drivetrain coordinator
#define MSG_MEB_ACC_01    0x300U   // TX, ACC HUD to instrument cluster
#define MSG_HCA_03        0x303U   // TX, Heading Control Assist curvature command


#define VW_MEB_COMMON_RX_CHECKS                                                                     \
  {.msg = {{MSG_LH_EPS_03,  0, 8,  100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{MSG_MOTOR_14,   0, 8,  10U,  .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \
  {.msg = {{MSG_GRA_ACC_01, 0, 8,  33U,  .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \
  {.msg = {{MSG_QFK_01,     0, 32, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},

#define VW_MEB_RX_CHECKS                                                                            \
  {.msg = {{MSG_Motor_51,   0, 32, 50U,  .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \
  {.msg = {{MSG_ESC_51,     0, 48, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},


// MEB CRC
static uint32_t volkswagen_meb_compute_crc(const CANPacket_t *msg) {
  int len = GET_LEN(msg);

  uint8_t crc = 0xFFU;
  for (int i = 1; i < len; i++) {
    crc ^= (uint8_t)msg->data[i];
    crc = volkswagen_crc8_lut_8h2f[crc];
  }

  uint8_t counter = volkswagen_mqb_meb_get_counter(msg);
  if (msg->addr == MSG_LH_EPS_03) {
    crc ^= (uint8_t[]){0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5}[counter];
  } else if (msg->addr == MSG_GRA_ACC_01) {
    crc ^= (uint8_t[]){0x6A,0x38,0xB4,0x27,0x22,0xEF,0xE1,0xBB,0xF8,0x80,0x84,0x49,0xC7,0x9E,0x1E,0x2B}[counter];
  } else if (msg->addr == MSG_QFK_01) {
    crc ^= (uint8_t[]){0x20,0xCA,0x68,0xD5,0x1B,0x31,0xE2,0xDA,0x08,0x0A,0xD4,0xDE,0x9C,0xE4,0x35,0x5B}[counter];
  } else if (msg->addr == MSG_ESC_51) {
    crc ^= (uint8_t[]){0x77,0x5C,0xA0,0x89,0x4B,0x7C,0xBB,0xD6,0x1F,0x6C,0x4F,0xF6,0x20,0x2B,0x43,0xDD}[counter];
  } else if (msg->addr == MSG_Motor_51) {
    crc ^= (uint8_t[]){0x77,0x5C,0xA0,0x89,0x4B,0x7C,0xBB,0xD6,0x1F,0x6C,0x4F,0xF6,0x20,0x2B,0x43,0xDD}[counter];
  } else if (msg->addr == MSG_MOTOR_14) {
    crc ^= (uint8_t[]){0x1F,0x28,0xC6,0x85,0xE6,0xF8,0xB0,0x19,0x5B,0x64,0x35,0x21,0xE4,0xF7,0x9C,0x24}[counter];
  } else {
  }
  crc = volkswagen_crc8_lut_8h2f[crc];

  return (uint8_t)(crc ^ 0xFFU);
}

// Curvature steering control
typedef struct {
  const int max_curvature;
  const float curvature_to_can;
  const float send_rate;
  const bool inactive_curvature_is_zero;
  const int max_power;
} CurvatureSteeringLimits;

static struct sample_t curvature_meas; // last 6 measured curvatures
static int desired_curvature_last;
static int desired_steer_power_last;

static bool steer_power_cmd_checks(int desired_steer_power, bool steer_control_enabled, const CurvatureSteeringLimits limits) {
  bool violation = false;

  violation |= safety_max_limit_check(desired_steer_power, limits.max_power, 0);
  violation |= (desired_steer_power > 0) && !steer_control_enabled;
  violation |= !controls_allowed && steer_control_enabled && (desired_steer_power != 0) && (desired_steer_power >= desired_steer_power_last);
  violation |= !controls_allowed && !steer_control_enabled && (desired_steer_power != 0);

  desired_steer_power_last = desired_steer_power;

  return violation;
}

static bool steer_curvature_cmd_checks_average(int desired_curvature, bool steer_control_enabled, const CurvatureSteeringLimits limits) {
  static const float ISO_LATERAL_JERK = 5.0;
  bool violation = false;

  if (controls_allowed && steer_control_enabled) {
    violation |= safety_max_limit_check(desired_curvature, limits.max_curvature, -limits.max_curvature);

    float speed = SAFETY_MAX((vehicle_speed.min / VEHICLE_SPEED_FACTOR) - 1., 1.0);
    float max_curvature_rate_sec = ISO_LATERAL_JERK / (speed * speed);

    float max_curvature_delta     = max_curvature_rate_sec * (float)limits.send_rate;
    float max_curvature_delta_can = (max_curvature_delta * limits.curvature_to_can) + 1.;

    int highest_desired_curvature = desired_curvature_last + (int)max_curvature_delta_can;
    int lowest_desired_curvature  = desired_curvature_last - (int)max_curvature_delta_can;

    // ISO lateral acceleration limit with avg road roll
    static const float MAX_LATERAL_ACCEL_AVG = ISO_LATERAL_ACCEL + (EARTH_G * AVERAGE_ROAD_ROLL);
    float max_curvature = MAX_LATERAL_ACCEL_AVG / (speed * speed);
    max_curvature = (max_curvature * limits.curvature_to_can) + 1.;

    highest_desired_curvature = SAFETY_CLAMP(highest_desired_curvature, -max_curvature, max_curvature) + 1;
    lowest_desired_curvature  = SAFETY_CLAMP(lowest_desired_curvature,  -max_curvature, max_curvature) - 1;

    violation |= safety_max_limit_check(desired_curvature, highest_desired_curvature, lowest_desired_curvature);
  }

  // when not steering, curvature must be 0 or stay near current measured curvature
  if (!steer_control_enabled) {
    const int max_inactive_curvature = SAFETY_CLAMP(curvature_meas.max, -limits.max_curvature, limits.max_curvature) + 1;
    const int min_inactive_curvature = SAFETY_CLAMP(curvature_meas.min, -limits.max_curvature, limits.max_curvature) - 1;
    violation |= (limits.inactive_curvature_is_zero ? (desired_curvature != 0) :
                  safety_max_limit_check(desired_curvature, max_inactive_curvature, min_inactive_curvature));
  }

  desired_curvature_last = desired_curvature;

  return violation;
}

static safety_config volkswagen_meb_init(uint16_t param) {
  // stock-long, lateral-only TX
  static const CanMsg VOLKSWAGEN_MEB_STOCK_TX_MSGS[] = {
    {MSG_HCA_03,      0, 24, .check_relay = true},
    {MSG_GRA_ACC_01,  0, 8,  .check_relay = false},
    {MSG_GRA_ACC_01,  2, 8,  .check_relay = false},
    {MSG_LDW_02,      0, 8,  .check_relay = true},
  };

  // OP-long TX
  static const CanMsg VOLKSWAGEN_MEB_LONG_TX_MSGS[] = {
    {MSG_HCA_03,      0, 24, .check_relay = true},
    {MSG_LDW_02,      0, 8,  .check_relay = true},
    {MSG_ACC_18,      0, 32, .check_relay = true},
    {MSG_MEB_ACC_01,  0, 48, .check_relay = true},
  };

  static RxCheck volkswagen_meb_rx_checks[] = {
    VW_MEB_COMMON_RX_CHECKS
    VW_MEB_RX_CHECKS
  };

  volkswagen_set_button_prev = false;
  volkswagen_resume_button_prev = false;
  desired_curvature_last = 0;
  desired_steer_power_last = 0;
  reset_sample(&curvature_meas);

#ifdef ALLOW_DEBUG
  volkswagen_longitudinal = GET_FLAG(param, FLAG_VOLKSWAGEN_LONG_CONTROL);
#else
  SAFETY_UNUSED(param);
  volkswagen_longitudinal = false;
#endif

  gen_crc_lookup_table_8(0x2F, volkswagen_crc8_lut_8h2f);

  safety_config ret;
  if (volkswagen_longitudinal) {
    SET_TX_MSGS(VOLKSWAGEN_MEB_LONG_TX_MSGS, ret);
  } else {
    SET_TX_MSGS(VOLKSWAGEN_MEB_STOCK_TX_MSGS, ret);
  }
  SET_RX_CHECKS(volkswagen_meb_rx_checks, ret);
  return ret;
}


static void volkswagen_meb_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == 0U) {
    if (msg->addr == MSG_ESC_51) {
      uint32_t fl = msg->data[8]  | (msg->data[9]  << 8);
      uint32_t fr = msg->data[10] | (msg->data[11] << 8);
      uint32_t rl = msg->data[12] | (msg->data[13] << 8);
      uint32_t rr = msg->data[14] | (msg->data[15] << 8);

      vehicle_moving = (fl > 0U) || (fr > 0U) || (rl > 0U) || (rr > 0U);
      UPDATE_VEHICLE_SPEED((fl + fr + rl + rr) * (0.0075 / 4.0 / 3.6));
    }

    // Measured curvature feedback
    if (msg->addr == MSG_QFK_01) {
      uint32_t raw_curvature = ((uint32_t)(msg->data[6] & 0x7FU) << 8) | (uint32_t)msg->data[5];
      int current_curvature = (int)raw_curvature;
      bool current_curvature_sign = GET_BIT(msg, 55U);
      if (!current_curvature_sign) {
        current_curvature *= -1;
      }
      update_sample(&curvature_meas, current_curvature);
    }

    if (msg->addr == MSG_LH_EPS_03) {
      update_sample(&torque_driver, volkswagen_mlb_mqb_driver_input_torque(msg));
    }

    // Motor_51.TSK_Status
    if (msg->addr == MSG_Motor_51) {
      int acc_status = ((msg->data[11] >> 0) & 0x07U);
      bool cruise_engaged = (acc_status == 3) || (acc_status == 4) || (acc_status == 5);
      acc_main_on = cruise_engaged || (acc_status == 2);

      if (!volkswagen_longitudinal) {
        pcm_cruise_check(cruise_engaged);
      }
      if (!acc_main_on) {
        controls_allowed = false;
      }

      int accel_pedal_value = ((msg->data[1] >> 4) & 0x0FU) | ((msg->data[2] & 0x1FU) << 4);
      gas_pressed = accel_pedal_value > 0;
    }

    // cruise buttons
    if (msg->addr == MSG_GRA_ACC_01) {
      if (volkswagen_longitudinal) {
        bool set_button = GET_BIT(msg, 16U);
        bool resume_button = GET_BIT(msg, 19U);
        if ((volkswagen_set_button_prev && !set_button) || (volkswagen_resume_button_prev && !resume_button)) {
          controls_allowed = acc_main_on;
        }
        volkswagen_set_button_prev = set_button;
        volkswagen_resume_button_prev = resume_button;
      }
      if (GET_BIT(msg, 13U)) {
        controls_allowed = false;
      }
    }

    if (msg->addr == MSG_MOTOR_14) {
      brake_pressed = GET_BIT(msg, 28U);
    }
  }
}


static bool volkswagen_meb_tx_hook(const CANPacket_t *msg) {
  // ACC_18 acceleration request limits
  const LongitudinalLimits VOLKSWAGEN_MEB_LONG_LIMITS = {
    .max_accel = 2000,
    .min_accel = -3500,
    .inactive_accel = 3010,  // VW sends one increment above the max range when inactive
  };

  // Curvature limits
  static const CurvatureSteeringLimits VOLKSWAGEN_MEB_STEERING_LIMITS = {
    .max_curvature = 29105,                  // 0.195 rad/m
    .curvature_to_can = 149253.7313,
    .send_rate = 0.02,                       // 50 Hz
    .inactive_curvature_is_zero = true,
    .max_power = 125,                        // 50% duty
  };

  bool tx = true;

  // HCA_03: steering curvature command
  if (msg->addr == MSG_HCA_03) {
    int desired_curvature_raw = GET_BYTES(msg, 3, 2) & 0x7FFFU;
    bool desired_curvature_sign = GET_BIT(msg, 39U);
    if (!desired_curvature_sign) {
      desired_curvature_raw *= -1;
    }

    bool steer_req = (((msg->data[1] >> 4) & 0x0FU) == 4U);
    int steer_power = msg->data[2];

    if (steer_power_cmd_checks(steer_power, steer_req, VOLKSWAGEN_MEB_STEERING_LIMITS)) {
      tx = false;
    }
    if (steer_curvature_cmd_checks_average(desired_curvature_raw, steer_req, VOLKSWAGEN_MEB_STEERING_LIMITS)) {
      tx = false;
    }
  }

  // ACC_18: acceleration request to drivetrain
  if (msg->addr == MSG_ACC_18) {
    int desired_accel = ((((msg->data[4] & 0x7U) << 8) | msg->data[3]) * 5U) - 7220U;
    // MEB override: the TSK expects accel=0
    bool accel_override = controls_allowed && (desired_accel == 0);
    if (!accel_override && longitudinal_accel_checks(desired_accel, VOLKSWAGEN_MEB_LONG_LIMITS)) {
      tx = false;
    }
  }

  // only allow GRA_ACC_01 cancel bit when controls are not allowed
  if ((msg->addr == MSG_GRA_ACC_01) && !controls_allowed) {
    if ((msg->data[2] & 0x9U) != 0U) {
      tx = false;
    }
  }

  return tx;
}


const safety_hooks volkswagen_meb_hooks = {
  .init = volkswagen_meb_init,
  .rx = volkswagen_meb_rx_hook,
  .tx = volkswagen_meb_tx_hook,
  .get_counter = volkswagen_mqb_meb_get_counter,
  .get_checksum = volkswagen_mqb_meb_get_checksum,
  .compute_checksum = volkswagen_meb_compute_crc,
};
