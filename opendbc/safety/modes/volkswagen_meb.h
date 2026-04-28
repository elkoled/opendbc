#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/volkswagen_common.h"

// MEB-specific CAN message addresses (LH_EPS_03, GRA_ACC_01, LDW_02, MOTOR_14 come from volkswagen_common.h)
#define MSG_ESC_51        0x0FCU   // RX, ABS wheel speeds + brake pressure (MEB)
#define MSG_Motor_51      0x10BU   // RX, drivetrain coordinator: TSK_Status + accel pedal (MEB)
#define MSG_QFK_01        0x13DU   // RX, EPS lateral controller status + measured curvature (MEB)
#define MSG_HCA_03        0x303U   // TX, Heading Control Assist curvature command (MEB)


#define VW_MEB_COMMON_RX_CHECKS                                                                     \
  {.msg = {{MSG_LH_EPS_03, 0, 8, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{MSG_MOTOR_14,  0, 8, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \
  {.msg = {{MSG_GRA_ACC_01,0, 8, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \
  {.msg = {{MSG_QFK_01,    0, 32,.max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},

#define VW_MEB_RX_CHECKS                                                                            \
  {.msg = {{MSG_Motor_51,  0, 32,.max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \
  {.msg = {{MSG_ESC_51,    0, 48,.max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},

#define VW_MEB_GEN2_RX_CHECKS                                                                       \
  {.msg = {{MSG_Motor_51,  0, 48,.max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \
  {.msg = {{MSG_ESC_51,    0, 64,.max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},


// MEB CRC computation (counter-XOR LUT variant)
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
    // unknown msg: CRC check expected to fail
  }
  crc = volkswagen_crc8_lut_8h2f[crc];

  return (uint8_t)(crc ^ 0xFFU);
}

// Gen2 (MEB_GEN2) variant: CRC over a fixed signal length subset of the message; falls back to standard CRC if the candidate didn't match.
static uint32_t volkswagen_meb_gen2_compute_crc(const CANPacket_t *msg) {
  if (!volkswagen_alt_crc_variant_1) {
    return volkswagen_meb_compute_crc(msg);
  }

  int len = GET_LEN(msg);
  if (msg->addr == MSG_QFK_01) {
    len = 28;
  } else if (msg->addr == MSG_ESC_51) {
    len = 60;
  } else if (msg->addr == MSG_Motor_51) {
    len = 44;
  } else {
    return volkswagen_meb_compute_crc(msg);
  }

  uint8_t crc = 0xFFU;
  for (int i = 1; i < len; i++) {
    crc ^= (uint8_t)msg->data[i];
    crc = volkswagen_crc8_lut_8h2f[crc];
  }

  uint8_t counter = volkswagen_mqb_meb_get_counter(msg);
  if (msg->addr == MSG_QFK_01) {
    crc ^= (uint8_t[]){0x18,0x71,0x10,0x8D,0xD7,0xAA,0xB0,0x78,0xAC,0x12,0xAE,0x0C,0xDD,0xF1,0x85,0x68}[counter];
  } else if (msg->addr == MSG_ESC_51) {
    crc ^= (uint8_t[]){0x69,0xDC,0xF9,0x64,0x6A,0xCE,0x55,0x2C,0xC4,0x38,0x8F,0xD1,0xC6,0x43,0xB4,0xB1}[counter];
  } else if (msg->addr == MSG_Motor_51) {
    crc ^= (uint8_t[]){0x2C,0xB1,0x1A,0x75,0xBB,0x65,0x79,0x47,0x81,0x2B,0xCC,0x96,0x17,0xDB,0xC0,0x94}[counter];
  } else {
    return volkswagen_meb_compute_crc(msg);
  }

  crc = (uint8_t)(volkswagen_crc8_lut_8h2f[crc] ^ 0xFFU);
  if (crc != msg->data[0]) {
    return volkswagen_meb_compute_crc(msg);
  }
  return (uint8_t)(crc);
}


// Lateral curvature limits — must match opendbc/car/volkswagen carcontroller curvature_to_can scale (1 / 6.7e-6).
// Empirically validated values used by the sunnypilot fork's MEB port.
static const CurvatureSteeringLimits VOLKSWAGEN_MEB_STEERING_LIMITS = {
  .max_curvature = 29105,                  // 0.195 rad/m
  .curvature_to_can = 149253.7313,         // 1 / 6.7e-6 rad/m -> CAN scale
  .send_rate = 0.02,                       // 50 Hz
  .inactive_curvature_is_zero = true,
  .max_power = 125,                        // 50% duty
};


static safety_config volkswagen_meb_init(uint16_t param) {
  // Stock-long, lateral-only TX set: steering curvature, cancel button passthrough on bus 0 and 2, lane-departure HUD passthrough.
  static const CanMsg VOLKSWAGEN_MEB_STOCK_TX_MSGS[] = {
    {MSG_HCA_03,      0, 24, .check_relay = true},
    {MSG_GRA_ACC_01,  0, 8,  .check_relay = false},
    {MSG_GRA_ACC_01,  2, 8,  .check_relay = false},
    {MSG_LDW_02,      0, 8,  .check_relay = true},
  };

  static RxCheck volkswagen_meb_rx_checks[] = {
    VW_MEB_COMMON_RX_CHECKS
    VW_MEB_RX_CHECKS
  };

  static RxCheck volkswagen_meb_gen2_rx_checks[] = {
    VW_MEB_COMMON_RX_CHECKS
    VW_MEB_GEN2_RX_CHECKS
  };

  volkswagen_set_button_prev = false;
  volkswagen_resume_button_prev = false;
  volkswagen_alt_crc_variant_1 = GET_FLAG(param, FLAG_VOLKSWAGEN_ALT_CRC_VARIANT_1);

#ifdef ALLOW_DEBUG
  volkswagen_longitudinal = GET_FLAG(param, FLAG_VOLKSWAGEN_LONG_CONTROL);
#else
  volkswagen_longitudinal = false;
#endif

  gen_crc_lookup_table_8(0x2F, volkswagen_crc8_lut_8h2f);

  safety_config ret;
  SET_TX_MSGS(VOLKSWAGEN_MEB_STOCK_TX_MSGS, ret);
  if (volkswagen_alt_crc_variant_1) {
    SET_RX_CHECKS(volkswagen_meb_gen2_rx_checks, ret);
  } else {
    SET_RX_CHECKS(volkswagen_meb_rx_checks, ret);
  }
  return ret;
}


static void volkswagen_meb_rx_hook(const CANPacket_t *msg) {
  if (msg->bus != 0U) {
    return;
  }

  // Wheel speeds + vehicle motion (ESC_51 layout)
  if (msg->addr == MSG_ESC_51) {
    uint32_t fl = msg->data[8]  | msg->data[9]  << 8;
    uint32_t fr = msg->data[10] | msg->data[11] << 8;
    uint32_t rl = msg->data[12] | msg->data[13] << 8;
    uint32_t rr = msg->data[14] | msg->data[15] << 8;

    vehicle_moving = (fl > 0U) || (fr > 0U) || (rl > 0U) || (rr > 0U);
    UPDATE_VEHICLE_SPEED(((fl + fr + rl + rr) / 4U) * 0.0075 / 3.6);
  }

  // Measured curvature feedback for inactive-curvature checks (QFK_01.Curvature, sign in bit 55)
  if (msg->addr == MSG_QFK_01) {
    int current_curvature = ((msg->data[6] & 0x7F) << 8) | msg->data[5];
    bool current_curvature_sign = GET_BIT(msg, 55U);
    if (!current_curvature_sign) {
      current_curvature *= -1;
    }
    update_sample(&curvature_meas, current_curvature);
  }

  // Driver input torque sample (used by torque-driver-limited safety modes; harmless to track here)
  if (msg->addr == MSG_LH_EPS_03) {
    update_sample(&torque_driver, volkswagen_mlb_mqb_driver_input_torque(msg));
  }

  // Cruise state from drivetrain coordinator (Motor_51.TSK_Status at bits 88..90)
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

    // Accel pedal state (Motor_51.Accel_Pedal_Pressure)
    int accel_pedal_value = ((msg->data[1] >> 4) & 0x0FU) | ((msg->data[2] & 0x1FU) << 4);
    gas_pressed = accel_pedal_value > 0;
  }

  // Cancel button always disengages controls (also on stock-long, even if not enabling)
  if (msg->addr == MSG_GRA_ACC_01) {
    if (GET_BIT(msg, 13U)) {
      controls_allowed = false;
    }
  }

  // Brake pedal switch (MOTOR_14.MO_Fahrer_bremst at bit 28)
  if (msg->addr == MSG_MOTOR_14) {
    brake_pressed = GET_BIT(msg, 28U);
  }
}


static bool volkswagen_meb_tx_hook(const CANPacket_t *msg) {
  bool tx = true;

  // HCA_03 — steering curvature command
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

  // FORCE CANCEL: only allow GRA_ACC_01 cancel bit when controls are not allowed (avoids accidental engage from set/resume).
  if ((msg->addr == MSG_GRA_ACC_01) && !controls_allowed) {
    // Block bits 16 (set) and 19 (resume) — bit 13 (cancel) is always allowed.
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
  .compute_checksum = volkswagen_meb_gen2_compute_crc,
};
