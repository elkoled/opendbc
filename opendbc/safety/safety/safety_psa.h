#define PSA_STEERING              757  // RX from XXX, driver torque
#define PSA_STEERING_ALT          773  // RX from EPS, steering angle
#define PSA_DRIVER                1390 // RX from BSI, gas pedal
#define PSA_DAT_BSI               1042 // RX from BSI, doors
#define PSA_HS2_DYN_ABR_38D       909  // RX from UC_FREIN, speed
#define PSA_HS2_DAT_MDD_CMD_452   1106 // RX from BSI, cruise state
#define PSA_LANE_KEEP_ASSIST      1010 // TX from OP,  EPS

// CAN bus
#define PSA_CAM_BUS  0U
#define PSA_ADAS_BUS 1U
#define PSA_MAIN_BUS 2U

const CanMsg PSA_TX_MSGS[] = {
  {PSA_LANE_KEEP_ASSIST, PSA_CAM_BUS, 8, .check_relay = true}, // EPS steering
};

RxCheck psa_rx_checks[] = {
  // TODO: counters and checksums
  {.msg = {{PSA_STEERING, PSA_CAM_BUS, 7, .ignore_checksum = true, .ignore_counter = true, .frequency = 100U}, { 0 }, { 0 }}},            // driver torque
  {.msg = {{PSA_STEERING_ALT, PSA_CAM_BUS, 7, .ignore_checksum = true, .ignore_counter = true, .frequency = 100U}, { 0 }, { 0 }}},        // steering angle
  // {.msg = {{PSA_DRIVER, PSA_MAIN_BUS, 6, .ignore_checksum = true, .ignore_counter = true, .frequency = 10U}, { 0 }, { 0 }}},              // gas pedal
  // {.msg = {{PSA_DAT_BSI, PSA_MAIN_BUS, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 20U}, { 0 }, { 0 }}},             // doors
  // {.msg = {{PSA_HS2_DYN_ABR_38D, PSA_ADAS_BUS, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 25U}, { 0 }, { 0 }}},     // speed
  {.msg = {{PSA_HS2_DAT_MDD_CMD_452, PSA_ADAS_BUS, 6, .ignore_checksum = true, .ignore_counter = true, .frequency = 20U}, { 0 }, { 0 }}}, // cruise state
};

static bool psa_lkas_msg_check(int addr) {
  return addr == PSA_LANE_KEEP_ASSIST;
}

static void psa_rx_hook(const CANPacket_t *to_push) {
  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);

  if (bus == PSA_ADAS_BUS) {
    if (addr == PSA_HS2_DAT_MDD_CMD_452) {
      // pcm_cruise_check(GET_BIT(to_push, 23)); // DDE_ACTIVATION_RVV_ACC
    }
  }
}

static bool psa_tx_hook(const CANPacket_t *to_send) {
  UNUSED(to_send);
  return true;
}

static bool psa_fwd_hook(int bus_num, int addr) {
  bool block_msg = false;

  if (bus_num == PSA_MAIN_BUS) {
    block_msg = psa_lkas_msg_check(addr);
  }

  return block_msg;
}

static safety_config psa_init(uint16_t param) {
  UNUSED(param);
  print("psa_init\n");
  return BUILD_SAFETY_CFG(psa_rx_checks, PSA_TX_MSGS);
}

const safety_hooks psa_hooks = {
  .init = psa_init,
  .rx = psa_rx_hook,
  .tx = psa_tx_hook,
  .fwd = psa_fwd_hook,
  // .get_counter = psa_get_counter,
  // .get_checksums = psa_get_checksum,
};
