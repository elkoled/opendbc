#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/volkswagen_common.h"

#define MSG_ESC_51           0xFCU    // RX, for wheel speeds
#define MSG_HCA_03           0x303U   // TX by OP, Heading Control Assist curvature
#define MSG_QFK_01           0x13DU   // RX, for measured curvature
#define MSG_MOTOR_51         0x10BU   // RX for TSK state and accel pedal

static safety_config volkswagen_meb_init(uint16_t param) {
  // Transmit of GRA_ACC_01 is allowed on bus 0 and 2 to keep compatibility with gateway and camera integration
  static const CanMsg VOLKSWAGEN_MEB_TX_MSGS[] = {
    {MSG_HCA_03, 0, 24, .check_relay = true},
    {MSG_LDW_02, 0, 8, .check_relay = true},
    {MSG_GRA_ACC_01, 0, 8, .check_relay = false},
    {MSG_GRA_ACC_01, 2, 8, .check_relay = false},
    {MSG_LH_EPS_03, 2, 8, .check_relay = true},
  };

  static RxCheck volkswagen_meb_rx_checks[] = {
    {.msg = {{MSG_ESC_51, 0, 48, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_LH_EPS_03, 0, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_QFK_01, 0, 32, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_MOTOR_51, 0, 32, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_MOTOR_14, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_GRA_ACC_01, 0, 8, 33U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  volkswagen_common_init();

  SAFETY_UNUSED(param);

  return BUILD_SAFETY_CFG(volkswagen_meb_rx_checks, VOLKSWAGEN_MEB_TX_MSGS);
}

static void volkswagen_meb_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == 0U) {
    // Update in-motion state by sampling wheel speeds
    if (msg->addr == MSG_ESC_51) {
      uint32_t fr = (uint32_t)msg->data[10] | ((uint32_t)msg->data[11] << 8);
      uint32_t rr = (uint32_t)msg->data[14] | ((uint32_t)msg->data[15] << 8);
      uint32_t rl = (uint32_t)msg->data[12] | ((uint32_t)msg->data[13] << 8);
      uint32_t fl = (uint32_t)msg->data[8] | ((uint32_t)msg->data[9] << 8);

      vehicle_moving = (fr > 0U) || (rr > 0U) || (rl > 0U) || (fl > 0U);

      UPDATE_VEHICLE_SPEED(((fr + rr + rl + fl) * 0.0075F / 4.0F) / 3.6F);
    }

    if (msg->addr == MSG_QFK_01) {
      int current_curvature = (((msg->data[6] & 0x7FU) << 8) | msg->data[5]);
      bool current_curvature_sign = GET_BIT(msg, 55U);
      if (!current_curvature_sign) {
        current_curvature *= -1;
      }
      update_sample(&angle_meas, current_curvature);
    }

    // Update driver input torque samples
    // Signal: LH_EPS_03.EPS_Lenkmoment (absolute torque)
    // Signal: LH_EPS_03.EPS_VZ_Lenkmoment (direction)
    if (msg->addr == MSG_LH_EPS_03) {
      update_sample(&torque_driver, volkswagen_mlb_mqb_driver_input_torque(msg));
    }

    if (msg->addr == MSG_MOTOR_51) {
      // When using stock ACC, enter controls on rising edge of stock ACC engage, exit on disengage
      // Always exit controls on main switch off
      // Signal: TSK_06.TSK_Status
      int acc_status = ((msg->data[11] >> 0) & 0x07U);
      bool cruise_engaged = (acc_status == 3) || (acc_status == 4) || (acc_status == 5);
      acc_main_on = cruise_engaged || (acc_status == 2);

      pcm_cruise_check(cruise_engaged);

      if (!acc_main_on) {
        controls_allowed = false;
      }

      // update accel pedal
      int accel_pedal_value = ((msg->data[1] >> 4) & 0x0FU) | ((msg->data[2] & 0x1FU) << 4);
      gas_pressed = accel_pedal_value > 0;
    }

    if (msg->addr == MSG_GRA_ACC_01) {
      // Always exit controls on rising edge of Cancel
      // Signal: GRA_ACC_01.GRA_Abbrechen
      if (GET_BIT(msg, 13U)) {
        controls_allowed = false;
      }
    }

    // Signal: Motor_14.MO_Fahrer_bremst (ECU detected brake pedal switch F63)
    if (msg->addr == MSG_MOTOR_14) {
      brake_pressed = GET_BIT(msg, 28U);
    }
  }
}

static bool volkswagen_meb_tx_hook(const CANPacket_t *msg) {
  // lateral limits for curvature (rad/m)
  const AngleSteeringLimits VOLKSWAGEN_MEB_STEERING_LIMITS = {
    .max_angle = 30000,                    // ~0.20 rad/m
    .angle_deg_to_can = 149253.7313,       // 1 / 6.7e-6 rad/m to CAN
    .angle_rate_up_lookup = {
      {5., 25., 25.},
      {0.02, 0.008, 0.008}
    },
    .angle_rate_down_lookup = {
      {5., 25., 25.},
      {0.02, 0.008, 0.008}
    },
    .max_angle_error = 300,                // ~0.002 rad/m * angle_deg_to_can
    .angle_error_min_speed = 10.0,
    .frequency = 50U,
    .angle_is_curvature = true,
    .enforce_angle_error = true,
    .inactive_angle_is_zero = true,
  };

  bool tx = true;

  // Safety check for HCA_03 Heading Control Assist curvature
  if (msg->addr == MSG_HCA_03) {
    int desired_curvature_raw = GET_BYTES(msg, 3, 2) & 0x7FFFU;
    bool desired_curvature_sign = GET_BIT(msg, 39U);
    if (!desired_curvature_sign) {
      desired_curvature_raw *= -1;
    }

    bool steer_req = (((msg->data[1] >> 4) & 0x0FU) == 4U);
    int steer_power = msg->data[2];

    if (steer_angle_cmd_checks(desired_curvature_raw, steer_req, VOLKSWAGEN_MEB_STEERING_LIMITS)) {
      tx = false;
    }

    if (steer_power > 125) {
      tx = false;
    }
  }

  // FORCE CANCEL: ensuring that only the cancel button press is sent when controls are off.
  // This avoids unintended engagements while still allowing resume spam
  if ((msg->addr == MSG_GRA_ACC_01) && !controls_allowed) {
    // disallow resume and set: bits 16 and 19
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
  .compute_checksum = volkswagen_mqb_meb_compute_crc,
};
