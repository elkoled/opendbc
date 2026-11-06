#pragma once

#include "opendbc/safety/declarations.h"

#define PSA_STEERING              757U  // RX from XXX, driver torque
#define PSA_STEERING_ALT          773U  // RX from EPS, steering angle
#define PSA_DRIVER                1390U  // RX from XXX, gas pedal
#define PSA_HS2_DYN_ABR_38D       909U  // RX from UC_FREIN, speed
#define PSA_HS2_DAT_MDD_CMD_452   1106U  // RX from BSI, cruise state
#define PSA_DAT_BSI               1042U  // RX from BSI, brake
#define PSA_LANE_KEEP_ASSIST      1010U  // TX from OP,  EPS
#define PSA_REQ_DIAG_ARTIV        1718U  // TX from OP,  radar diagnostics
#define PSA_HS2_DYN1_MDD_ETAT_2B6 694U  // TX from OP,  radar emulation
#define PSA_HS2_DYN_MDD_ETAT_2F6  758U  // TX from OP,  radar emulation

// CAN bus
#define PSA_MAIN_BUS 0U
#define PSA_ADAS_BUS 1U
#define PSA_CAM_BUS  2U

#define PSA_PARAM_LONGITUDINAL 1U
#define PSA_PARAM_EPS_CAM_BUS 2U

// the 508 has the EPS on the camera bus
#define PSA_RX_CHECKS(eps_bus)                                                                                                                                         \
  {.msg = {{PSA_HS2_DAT_MDD_CMD_452, PSA_ADAS_BUS, 6, 20U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  /* cruise state */                       \
  {.msg = {{PSA_HS2_DYN_ABR_38D, PSA_MAIN_BUS, 8, 25U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  /* speed */                                  \
  {.msg = {{PSA_STEERING_ALT, (eps_bus), 7, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  /* steering angle */ \
  {.msg = {{PSA_STEERING, PSA_MAIN_BUS, 7, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  /* driver torque */   \
  {.msg = {{PSA_DRIVER, PSA_CAM_BUS, 6, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},  /* gas pedal, 6 bytes on the 208 */       \
           {PSA_DRIVER, PSA_CAM_BUS, 7, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }}},  /* 7 bytes on the 508 */         \
  {.msg = {{PSA_DAT_BSI, PSA_CAM_BUS, 8, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  /* brake */              \


static uint8_t psa_get_counter(const CANPacket_t *msg) {
  uint8_t cnt = 0;
  if (msg->addr == PSA_HS2_DAT_MDD_CMD_452) {
    cnt = (msg->data[3] >> 4) & 0xFU;
  }
  if (msg->addr == PSA_HS2_DYN_ABR_38D) {
    cnt = (msg->data[5] >> 4) & 0xFU;
  }
  return cnt;
}

static uint32_t psa_get_checksum(const CANPacket_t *msg) {
  return msg->data[5] & 0xFU;
}

static uint8_t _psa_compute_checksum(const CANPacket_t *msg, uint8_t chk_ini, int chk_pos) {
  int len = GET_LEN(msg);

  uint8_t sum = 0;
  for (int i = 0; i < len; i++) {
    uint8_t b = msg->data[i];

    if (i == chk_pos) {
      // set checksum in low nibble to 0
      b &= 0xF0U;
    }
    sum += (b >> 4) + (b & 0xFU);
  }
  return (chk_ini - sum) & 0xFU;
}

static uint32_t psa_compute_checksum(const CANPacket_t *msg) {
  uint8_t chk = 0;
  if (msg->addr == PSA_HS2_DAT_MDD_CMD_452) {
    chk = _psa_compute_checksum(msg, 0x4, 5);
  }
  if (msg->addr == PSA_HS2_DYN_ABR_38D) {
    chk = _psa_compute_checksum(msg, 0x7, 5);
  }
  return chk;
}

static void psa_rx_hook(const CANPacket_t *msg) {
  if (msg_matches(msg, PSA_STEERING_ALT, safety.psa_eps_bus)) {
    int angle_meas_new = to_signed((msg->data[0] << 8) | msg->data[1], 16);  // ANGLE
    update_sample(&safety.angle_meas, angle_meas_new);
  }
  if (msg_matches(msg, PSA_HS2_DYN_ABR_38D, PSA_MAIN_BUS)) {
    int speed = (msg->data[0] << 8) | msg->data[1];
    safety.vehicle_moving = speed > 0;
    UPDATE_VEHICLE_SPEED(speed * 0.01 * KPH_TO_MS);  // VITESSE_VEHICULE_ROUES
  }

  if (msg_matches(msg, PSA_HS2_DAT_MDD_CMD_452, PSA_ADAS_BUS)) {
    pcm_cruise_check((msg->data[2U] >> 7U) & 1U);  // RVV_ACC_ACTIVATION_REQ
  }

  if (msg_matches(msg, PSA_DRIVER, PSA_CAM_BUS)) {
    safety.gas_pressed = msg->data[3] > 0U;  // GAS_PEDAL
  }
  if (msg_matches(msg, PSA_DAT_BSI, PSA_CAM_BUS)) {
    safety.brake_pressed = (msg->data[0U] >> 5U) & 1U;  // P013_MainBrake
  }
}

static bool psa_tx_hook(const CANPacket_t *msg) {
  static const AngleSteeringLimits PSA_STEERING_LIMITS = {
    .max_angle = 3900,
    .angle_deg_to_can = 10,
    .frequency = 100U,
  };

  // NOTE: based off PSA_PEUGEOT_208 to match openpilot
  static const AngleSteeringParams PSA_STEERING_PARAMS = {
    .slip_factor = -0.0006603473577153204,  // calc_slip_factor(VM)
    .steer_ratio = 17.6,
    .wheelbase = 2.54,
  };

  // torques are offset by 4000 Nm
  static const LongitudinalLimits PSA_LONG_LIMITS = {
    .max_accel = 253,       // 2.0 m/s^2
    .min_accel = 143,       // -3.5 m/s^2
    .inactive_accel = 254,  // 2.05 m/s^2
    .max_gas = 5500,        // 1500 Nm
    .min_gas = 3600,        // -400 Nm
    .inactive_gas = 0,      // -4000 Nm
  };

  bool tx = true;

  if (msg->addr == PSA_LANE_KEEP_ASSIST) {
    // SET_ANGLE
    int desired_angle = to_signed((msg->data[6] << 6) | ((msg->data[7] & 0xFCU) >> 2), 14);
    // TORQUE_FACTOR
    bool lka_active = ((msg->data[5] & 0xFEU) >> 1) == 100U;
    tx = !steer_angle_cmd_checks_vm(desired_angle, lka_active, PSA_STEERING_LIMITS, PSA_STEERING_PARAMS);
  }

  // only the programming session and tester present, which keep the radar silent
  if (msg->addr == PSA_REQ_DIAG_ARTIV) {
    uint64_t data = GET_BYTES_64_LE(msg, 0, 8);
    tx = (data == 0x80021002ULL) || (data == 0x3E02ULL);
  }

  if (msg->addr == PSA_HS2_DYN1_MDD_ETAT_2B6) {
    int decel = msg->data[0];  // MDD_DESIRED_DECELERATION
    int potential_torque = ((msg->data[2] << 4) | (msg->data[3] >> 4)) << 2;  // GMP_POTENTIAL_WHEEL_TORQUE
    int wheel_torque = (msg->data[4] << 6) | (msg->data[5] >> 2);  // GMP_WHEEL_TORQUE
    bool decel_req = GET_BIT(msg, 53U);  // MDD_DECEL_CONTROL_REQ

    // the lowest decel holds the car, the stock radar sends it from 0.3 m/s, margin for the speed window
    bool hold = decel_req && (decel == 0) && ((vehicle_speed.max / VEHICLE_SPEED_FACTOR) < 1.0);

    bool violation = hold ? !get_longitudinal_allowed() : longitudinal_accel_checks(decel, PSA_LONG_LIMITS);
    violation |= longitudinal_gas_checks(potential_torque, PSA_LONG_LIMITS);
    violation |= longitudinal_gas_checks(wheel_torque, PSA_LONG_LIMITS);
    violation |= decel_req && !get_longitudinal_allowed();
    tx = !violation;
  }

  return tx;
}

static safety_config psa_init(uint16_t param) {
  static const CanMsg PSA_TX_MSGS[] = {
    {PSA_LANE_KEEP_ASSIST, PSA_MAIN_BUS, 8, .check_relay = true},  // EPS steering
  };

  static const CanMsg PSA_LONG_TX_MSGS[] = {
    {PSA_LANE_KEEP_ASSIST, PSA_MAIN_BUS, 8, .check_relay = true},          // EPS steering
    {PSA_REQ_DIAG_ARTIV, PSA_ADAS_BUS, 8, .check_relay = false},         // radar diagnostics
    {PSA_HS2_DYN1_MDD_ETAT_2B6, PSA_ADAS_BUS, 8, .check_relay = false},  // radar emulation
    {PSA_HS2_DYN_MDD_ETAT_2F6, PSA_ADAS_BUS, 8, .check_relay = false},   // radar emulation
  };

  static const RxCheck psa_rx_checks[] = {
    PSA_RX_CHECKS(PSA_MAIN_BUS)
  };

  static const RxCheck psa_eps_cam_rx_checks[] = {
    PSA_RX_CHECKS(PSA_CAM_BUS)
  };

  safety.psa_eps_bus = GET_FLAG(param, PSA_PARAM_EPS_CAM_BUS) ? PSA_CAM_BUS : PSA_MAIN_BUS;

  safety_config ret = BUILD_SAFETY_CFG(psa_rx_checks, PSA_TX_MSGS);
  if (safety.psa_eps_bus == PSA_CAM_BUS) {
    SET_RX_CHECKS(psa_eps_cam_rx_checks, ret);
  }
  if (GET_FLAG(param, PSA_PARAM_LONGITUDINAL)) {
    SET_TX_MSGS(PSA_LONG_TX_MSGS, ret);
  }
  return ret;
}

const safety_hooks psa_hooks = {
  .init = psa_init,
  .rx = psa_rx_hook,
  .tx = psa_tx_hook,
  .get_counter = psa_get_counter,
  .get_checksum = psa_get_checksum,
  .compute_checksum = psa_compute_checksum,
};
