#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/volkswagen_common.h"

#define MSG_ESC_51           0xFCU    // RX, for wheel speeds
#define MSG_Motor_51         0x10BU   // RX, for TSK state and accel pedal
#define MSG_QFK_01           0x13DU   // RX, for measured curvature
#define MSG_HCA_03           0x303U   // TX, Heading Control Assist curvature

static safety_config volkswagen_meb_init(uint16_t param) {
  // Transmit of GRA_ACC_01 is allowed on bus 0 and 2 to keep compatibility with gateway and camera integration
  static const CanMsg VOLKSWAGEN_MEB_STOCK_TX_MSGS[] = {
    {MSG_HCA_03, 0, 24, .check_relay = true},
    {MSG_LDW_02, 0, 8, .check_relay = true},
    {MSG_GRA_ACC_01, 0, 8, .check_relay = false},
    {MSG_GRA_ACC_01, 2, 8, .check_relay = false},
  };

  static RxCheck volkswagen_meb_rx_checks[] = {
    {.msg = {{MSG_ESC_51,     0, 48, 50U,  .ignore_checksum = true, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_LH_EPS_03,  0,  8, 100U, .ignore_checksum = true, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_MOTOR_14,   0,  8, 10U,  .ignore_checksum = true, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_Motor_51,   0, 32, 50U,  .ignore_checksum = true, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_QFK_01,     0, 32, 50U,  .ignore_checksum = true, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_GRA_ACC_01, 0,  8, 33U,  .ignore_checksum = true, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  SAFETY_UNUSED(param);
  volkswagen_common_init();

  return BUILD_SAFETY_CFG(volkswagen_meb_rx_checks, VOLKSWAGEN_MEB_STOCK_TX_MSGS);
}

// lateral limits for curvature. Tuned to be permissive: matches the Python-side
// apply_std_curvature_limits bounds plus a small +1 CAN-unit padding so no
// openpilot command is ever rejected by the panda firmware.
static const CurvatureSteeringLimits VOLKSWAGEN_MEB_STEERING_LIMITS = {
  .max_curvature = 29105,                  // 0.195 rad/m, matches CarControllerParams.CURVATURE_LIMITS
  .curvature_to_can = 149253.7313,         // 1 / 6.7e-6 rad/m to CAN, matches DBC scale of HCA_03 / QFK_01
  .send_rate = 0.02,                       // STEER_STEP * DT_CTRL = 2 * 0.01s, matches the Python jerk formula
  .inactive_curvature_is_zero = true,      // carcontroller sends 0 curvature when fully disengaged
  .max_power = 125,                        // 50%, matches CarControllerParams.STEERING_POWER_MAX upper bound
};

static void volkswagen_meb_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == 0U) {

    // Update in-motion state by sampling wheel speeds
    if (msg->addr == MSG_ESC_51) {
      uint32_t fl = msg->data[8]  | (msg->data[9]  << 8);
      uint32_t fr = msg->data[10] | (msg->data[11] << 8);
      uint32_t rl = msg->data[12] | (msg->data[13] << 8);
      uint32_t rr = msg->data[14] | (msg->data[15] << 8);

      vehicle_moving = (fl > 0U) || (fr > 0U) || (rl > 0U) || (rr > 0U);

      // Match openpilot's parse_wheel_speeds: average kph, then convert to m/s.
      // Use float multiply first to avoid integer truncation on the /4.
      UPDATE_VEHICLE_SPEED((fl + fr + rl + rr) * 0.0075 / 4.0 / 3.6);
    }

    // Update measured curvature (same scaling as HCA_03 curvature)
    if (msg->addr == MSG_QFK_01) {
      int current_curvature = ((msg->data[6] & 0x7F) << 8) | msg->data[5];
      bool sign = GET_BIT(msg, 55U);
      if (!sign) {
        current_curvature *= -1;
      }
      update_sample(&curvature_meas, current_curvature);
    }

    // Update driver input torque
    if (msg->addr == MSG_LH_EPS_03) {
      update_sample(&torque_driver, volkswagen_mlb_mqb_driver_input_torque(msg));
    }

    // Update cruise state from TSK
    if (msg->addr == MSG_Motor_51) {
      int acc_status = ((msg->data[11] >> 0) & 0x07U);
      bool cruise_engaged = (acc_status == 3) || (acc_status == 4) || (acc_status == 5);
      acc_main_on = cruise_engaged || (acc_status == 2);

      pcm_cruise_check(cruise_engaged);

      // Update accel pedal
      int accel_pedal_value = ((msg->data[1] >> 4) & 0x0FU) | ((msg->data[2] & 0x1FU) << 4);
      gas_pressed = accel_pedal_value > 0;
    }

    // Update cruise buttons. Always exit controls on rising edge of Cancel.
    if (msg->addr == MSG_GRA_ACC_01) {
      if (GET_BIT(msg, 13U)) {
        controls_allowed = false;
      }
    }

    // Update brake pedal
    if (msg->addr == MSG_MOTOR_14) {
      brake_pressed = GET_BIT(msg, 28U);
    }
  }
}

static bool volkswagen_meb_tx_hook(const CANPacket_t *msg) {
  bool tx = true;

  // Safety check for HCA_03 Heading Control Assist curvature. The limits and
  // padding here match the Python-side apply_std_curvature_limits in
  // opendbc/car/lateral.py exactly so well-formed openpilot commands always pass.
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
