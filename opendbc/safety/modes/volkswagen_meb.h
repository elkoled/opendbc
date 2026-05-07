#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/volkswagen_common.h"

#define VOLKSWAGEN_MEB_MAX_POWER 125U      // 50% duty cycle, EPS hard rate limits beyond this
#define VOLKSWAGEN_MEB_CURVATURE_SCALE 6.7e-6f                          // rad/m per HCA_03/QFK_01 CAN unit
#define VOLKSWAGEN_MEB_RAD_TO_DEG 57.295779513f

// ID.4 bicycle vehicle model parameters (from VolkswagenCarSpecs / calc_slip_factor)
static const AngleSteeringParams VOLKSWAGEN_MEB_STEERING_PARAMS = {
  .slip_factor = -0.0006055171512345705f,
  .steer_ratio = 15.6f,
  .wheelbase = 2.77f,
};

// Convert curvature in CAN units to steering wheel angle in 0.1 deg via the bicycle model
static int volkswagen_meb_curvature_to_angle(int curvature_can) {
  float speed = SAFETY_MAX(vehicle_speed.values[0] / VEHICLE_SPEED_FACTOR, 1.0f);
  float cf = 1.0f / (1.0f - (VOLKSWAGEN_MEB_STEERING_PARAMS.slip_factor * speed * speed)) / VOLKSWAGEN_MEB_STEERING_PARAMS.wheelbase;
  float curvature = (float)curvature_can * VOLKSWAGEN_MEB_CURVATURE_SCALE;
  return ROUND(curvature * VOLKSWAGEN_MEB_STEERING_PARAMS.steer_ratio / cf * VOLKSWAGEN_MEB_RAD_TO_DEG * 10.0f);
}

static safety_config volkswagen_meb_init(uint16_t param) {
  // Transmit of GRA_ACC_01 is allowed on bus 0 and 2 to keep compatibility with gateway and camera integration
  static const CanMsg VOLKSWAGEN_MEB_STOCK_TX_MSGS[] = {{MSG_HCA_03, 0, 24, .check_relay = true}, {MSG_GRA_ACC_01, 0, 8, .check_relay = false}, {MSG_GRA_ACC_01, 2, 8, .check_relay = false},
                                                        {MSG_LDW_02, 0, 8, .check_relay = true}};

  static const CanMsg VOLKSWAGEN_MEB_LONG_TX_MSGS[] = {{MSG_HCA_03, 0, 24, .check_relay = true}, {MSG_LDW_02, 0, 8, .check_relay = true},
                                                       {MSG_ACC_18, 0, 32, .check_relay = true}, {MSG_MEB_ACC_01, 0, 48, .check_relay = true}};

  static RxCheck volkswagen_meb_rx_checks[] = {
    {.msg = {{MSG_LH_EPS_03,  0, 8,  100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_QFK_01,     0, 32, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_MOTOR_51,   0, 32, 50U,  .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_ESC_51,     0, 48, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_MOTOR_14,   0, 8,  10U,  .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_GRA_ACC_01, 0, 8,  33U,  .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  volkswagen_common_init();

#ifdef ALLOW_DEBUG
  volkswagen_longitudinal = GET_FLAG(param, FLAG_VOLKSWAGEN_LONG_CONTROL);
#else
  SAFETY_UNUSED(param);
#endif

  return volkswagen_longitudinal ? BUILD_SAFETY_CFG(volkswagen_meb_rx_checks, VOLKSWAGEN_MEB_LONG_TX_MSGS) : \
                                   BUILD_SAFETY_CFG(volkswagen_meb_rx_checks, VOLKSWAGEN_MEB_STOCK_TX_MSGS);
}

static void volkswagen_meb_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == 0U) {
    // Update in-motion state and vehicle speed by sampling all four wheel speeds
    // Signals: ESC_51.[VL|VR|HL|HR]_Radgeschw
    if (msg->addr == MSG_ESC_51) {
      uint32_t fl = msg->data[8]  | (msg->data[9]  << 8);
      uint32_t fr = msg->data[10] | (msg->data[11] << 8);
      uint32_t rl = msg->data[12] | (msg->data[13] << 8);
      uint32_t rr = msg->data[14] | (msg->data[15] << 8);
      vehicle_moving = (fl + fr + rl + rr) > 0U;
      UPDATE_VEHICLE_SPEED((fl + fr + rl + rr) * (0.0075 / 4.0 / 3.6));
    }

    // Update driver input torque samples
    if (msg->addr == MSG_LH_EPS_03) {
      update_sample(&torque_driver, volkswagen_mlb_mqb_driver_input_torque(msg));
    }

    // Update measured steering wheel angle samples (converted from curvature via bicycle model)
    // Signal: QFK_01.Curvature, QFK_01.Curvature_VZ
    if (msg->addr == MSG_QFK_01) {
      int curvature_meas_new = msg->data[5] | ((msg->data[6] & 0x7FU) << 8);
      if (!GET_BIT(msg, 55U)) {
        curvature_meas_new *= -1;
      }
      update_sample(&angle_meas, volkswagen_meb_curvature_to_angle(curvature_meas_new));
    }

    // When using stock ACC, enter controls on rising edge of stock ACC engage, exit on disengage
    // Always exit controls on main switch off
    // Signal: Motor_51.TSK_Status
    if (msg->addr == MSG_MOTOR_51) {
      int acc_status = msg->data[11] & 0x7U;
      bool cruise_engaged = (acc_status == 3) || (acc_status == 4) || (acc_status == 5);
      acc_main_on = cruise_engaged || (acc_status == 2);

      if (!volkswagen_longitudinal) {
        pcm_cruise_check(cruise_engaged);
      }

      if (!acc_main_on) {
        controls_allowed = false;
      }

      // Signal: Motor_51.Accel_Pedal_Pressure
      int accel_pedal_value = ((msg->data[1] >> 4) & 0x0FU) | ((msg->data[2] & 0x1FU) << 4);
      gas_pressed = accel_pedal_value > 0;
    }

    if (msg->addr == MSG_GRA_ACC_01) {
      // If using openpilot longitudinal, enter controls on falling edge of Set or Resume with main switch on
      // Signal: GRA_ACC_01.GRA_Tip_Setzen
      // Signal: GRA_ACC_01.GRA_Tip_Wiederaufnahme
      if (volkswagen_longitudinal) {
        bool set_button = GET_BIT(msg, 16U);
        bool resume_button = GET_BIT(msg, 19U);
        if ((volkswagen_set_button_prev && !set_button) || (volkswagen_resume_button_prev && !resume_button)) {
          controls_allowed = acc_main_on;
        }
        volkswagen_set_button_prev = set_button;
        volkswagen_resume_button_prev = resume_button;
      }
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
  // lateral limits
  // ID.4 EPS rack lock-to-lock is ~480 deg, max_angle of 600 deg leaves headroom while
  // bounds checking the converted-from-curvature command
  static const AngleSteeringLimits VOLKSWAGEN_MEB_STEERING_LIMITS = {
    .max_angle = 6000,        // 600 deg, in 0.1 deg
    .angle_deg_to_can = 10,
    .frequency = 50U,
  };

  // longitudinal limits
  // acceleration in m/s2 * 1000 to avoid floating point math
  const LongitudinalLimits VOLKSWAGEN_MEB_LONG_LIMITS = {
    .max_accel = 2000,
    .min_accel = -3500,
    .inactive_accel = 3010,  // VW sends one increment above the max range when inactive
  };

  bool tx = true;

  // Safety check for HCA_03 Heading Control Assist curvature
  if (msg->addr == MSG_HCA_03) {
    int desired_curvature_can = msg->data[3] | ((msg->data[4] & 0x7FU) << 8);
    if (!GET_BIT(msg, 39U)) {
      desired_curvature_can *= -1;
    }
    bool steer_req = ((msg->data[1] >> 4) & 0xFU) == 4U;
    int desired_power = msg->data[2];

    int desired_angle = volkswagen_meb_curvature_to_angle(desired_curvature_can);
    if (steer_angle_cmd_checks_vm(desired_angle, steer_req, VOLKSWAGEN_MEB_STEERING_LIMITS, VOLKSWAGEN_MEB_STEERING_PARAMS)) {
      tx = false;
    }

    // EPS power must be within cap and zero when not actuating
    if (desired_power > (int)VOLKSWAGEN_MEB_MAX_POWER) {
      tx = false;
    }
    if (!steer_req && (desired_power != 0)) {
      tx = false;
    }
  }

  // Safety check for ACC_18 acceleration request
  if (msg->addr == MSG_ACC_18) {
    // Signal: ACC_18.ACC_Sollbeschleunigung_02 (acceleration in m/s2, scale 0.005, offset -7.22)
    int desired_accel = ((((msg->data[4] & 0x7U) << 8) | msg->data[3]) * 5U) - 7220U;

    // Allow accel == 0 while controls are allowed: TSK requires this during driver gas override
    bool accel_override = controls_allowed && (desired_accel == 0);
    if (!accel_override && longitudinal_accel_checks(desired_accel, VOLKSWAGEN_MEB_LONG_LIMITS)) {
      tx = false;
    }
  }

  // FORCE CANCEL: ensuring that only the cancel button press is sent when controls are off.
  // This avoids unintended engagements while still allowing resume spam
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
  .compute_checksum = volkswagen_mqb_meb_compute_crc,
};
