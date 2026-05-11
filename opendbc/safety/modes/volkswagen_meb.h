#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/volkswagen_common.h"

#define MSG_ESC_51           0xFCU    // RX, for wheel speeds
#define MSG_HCA_03           0x303U   // TX by OP, Heading Control Assist steering torque
#define MSG_QFK_01           0x13DU   // RX, for steering angle
#define MSG_GRA_ACC_01       0x12BU   // TX by OP, ACC control buttons for cancel/resume
#define MSG_MOTOR_14         0x3BEU   // RX from ECU, for brake switch status
#define MSG_Motor_51         0x10BU   // RX for TSK state and accel pedal


static uint8_t volkswagen_crc8_lut_8h2f[256]; // Static lookup table for CRC8 poly 0x2F, aka 8H2F/AUTOSAR

static uint32_t volkswagen_meb_get_checksum(const CANPacket_t *msg) {
  return (uint8_t)msg->data[0];
}

static uint8_t volkswagen_meb_get_counter(const CANPacket_t *msg) {
  // MQB message counters are consistently found at LSB 8.
  return (uint8_t)msg->data[1] & 0xFU;
}

static uint32_t volkswagen_meb_compute_crc(const CANPacket_t *msg) {
  int len = GET_LEN(msg);

  // This is CRC-8H2F/AUTOSAR with a twist. See the OpenDBC implementation
  // of this algorithm for a version with explanatory comments.

  uint8_t crc = 0xFFU;
  for (int i = 1; i < len; i++) {
    crc ^= (uint8_t)msg->data[i];
    crc = volkswagen_crc8_lut_8h2f[crc];
  }

  uint8_t counter = volkswagen_meb_get_counter(msg);
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
  }
  else {
    // Undefined CAN message, CRC check expected to fail
  }
  crc = volkswagen_crc8_lut_8h2f[crc];

  return (uint8_t)(crc ^ 0xFFU);
}

static safety_config volkswagen_meb_init(uint16_t param) {
  UNUSED(param);
  // Transmit of GRA_ACC_01 is allowed on bus 0 and 2 to keep compatibility with gateway and camera integration
  static const CanMsg VOLKSWAGEN_MEB_STOCK_TX_MSGS[] = {{MSG_HCA_03, 0, 24, .check_relay = true},
                                                        {MSG_GRA_ACC_01, 0, 8, .check_relay = false},
                                                        {MSG_GRA_ACC_01, 2, 8, .check_relay = false}};

  static RxCheck volkswagen_meb_rx_checks[] = {
    {.msg = {{MSG_LH_EPS_03, 0, 8, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_MOTOR_14, 0, 8, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_GRA_ACC_01, 0, 8, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_QFK_01, 0, 32, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_Motor_51, 0, 32, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_ESC_51, 0, 48, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  volkswagen_set_button_prev = false;
  volkswagen_resume_button_prev = false;

  gen_crc_lookup_table_8(0x2F, volkswagen_crc8_lut_8h2f);

  safety_config ret;
  SET_TX_MSGS(VOLKSWAGEN_MEB_STOCK_TX_MSGS, ret);
  SET_RX_CHECKS(volkswagen_meb_rx_checks, ret);

  return ret;
}

static void volkswagen_meb_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == 0U) {

    // Update in-motion state by sampling wheel speeds
    if (msg->addr == MSG_ESC_51) {
      uint32_t fr = msg->data[10] | msg->data[11] << 8;
      uint32_t rr = msg->data[14] | msg->data[15] << 8;
      uint32_t rl = msg->data[12] | msg->data[13] << 8;
      uint32_t fl = msg->data[8] | msg->data[9] << 8;

      vehicle_moving = (fr > 0U) || (rr > 0U) || (rl > 0U) || (fl > 0U);

      UPDATE_VEHICLE_SPEED(((fr + rr + rl + fl) / 4 ) * 0.0075 / 3.6);
    }

    // Update driver input torque samples
    // Signal: LH_EPS_03.EPS_Lenkmoment (absolute torque)
    // Signal: LH_EPS_03.EPS_VZ_Lenkmoment (direction)
    if (msg->addr == MSG_LH_EPS_03) {
      update_sample(&torque_driver, volkswagen_mlb_mqb_driver_input_torque(msg));
    }

    // Update cruise state
    if (msg->addr == MSG_Motor_51) {
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
    }

    // update cruise buttons
    if (msg->addr == MSG_GRA_ACC_01) {
      // Always exit controls on rising edge of Cancel
      // Signal: GRA_ACC_01.GRA_Abbrechen
      if (GET_BIT(msg, 13U)) {
        controls_allowed = false;
      }
    }

    // update brake pedal
    if (msg->addr == MSG_MOTOR_14) {
      brake_pressed = GET_BIT(msg, 28U);
    }

    // update accel pedal
    if (msg->addr == MSG_Motor_51) {
      int accel_pedal_value = ((msg->data[1] >> 4) & 0x0FU) | ((msg->data[2] & 0x1FU) << 4);
      gas_pressed = accel_pedal_value > 0;
    }

  }
}

static bool volkswagen_meb_tx_hook(const CANPacket_t *msg) {
  bool tx = true;

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
  .get_counter = volkswagen_meb_get_counter,
  .get_checksum = volkswagen_meb_get_checksum,
  .compute_checksum = volkswagen_meb_compute_crc,
};
