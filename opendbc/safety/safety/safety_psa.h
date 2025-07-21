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
  {.msg = {{PSA_HS2_DAT_MDD_CMD_452, PSA_ADAS_BUS, 6, .ignore_checksum = true, .ignore_counter = true, .frequency = 20U}, { 0 }, { 0 }}}, // cruise state
};

static bool psa_lkas_msg_check(int addr) {
  return addr == PSA_LANE_KEEP_ASSIST;
}

static void psa_rx_hook(const CANPacket_t *to_push) {
  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);

  print("PSA_RX: bus=");
  puth(bus);
  print(" addr=");
  puth(addr);
  print(" controls_allowed=");
  puth(controls_allowed);
  print("\n");

  if (bus == PSA_ADAS_BUS) {
    if (addr == PSA_HS2_DAT_MDD_CMD_452) {
      print("PSA: Got cruise msg, data: ");
      for (int i = 0; i < 6; i++) {
        puth(to_push->data[i]);
        print(" ");
      }
      print("\n");

      bool cruise_bit = GET_BIT(to_push, 23);
      print("PSA: Cruise bit 23 = ");
      puth(cruise_bit);
      print(" cruise_engaged_prev = ");
      puth(cruise_engaged_prev);
      print("\n");

      print("PSA: Before pcm_cruise_check - controls_allowed = ");
      puth(controls_allowed);
      print("\n");

      pcm_cruise_check(cruise_bit);

      print("PSA: After pcm_cruise_check - controls_allowed = ");
      puth(controls_allowed);
      print(" cruise_engaged_prev = ");
      puth(cruise_engaged_prev);
      print("\n");
    }
  }

  if (bus == PSA_CAM_BUS) {
    if (addr == PSA_STEERING) {
      print("PSA: Got steering torque msg\n");
    }
    if (addr == PSA_STEERING_ALT) {
      print("PSA: Got steering angle msg\n");
    }
  }

  print("PSA: Safety state - gas_pressed=");
  puth(gas_pressed);
  print(" brake_pressed=");
  puth(brake_pressed);
  print(" regen_braking=");
  puth(regen_braking);
  print(" steering_disengage=");
  puth(steering_disengage);
  print(" vehicle_moving=");
  puth(vehicle_moving);
  print(" relay_malfunction=");
  puth(relay_malfunction);
  print("\n");
}

static bool psa_tx_hook(const CANPacket_t *to_send) {
  int addr = GET_ADDR(to_send);
  int bus = GET_BUS(to_send);

  print("PSA_TX: addr=");
  puth(addr);
  print(" bus=");
  puth(bus);
  print(" controls_allowed=");
  puth(controls_allowed);
  print(" relay_malfunction=");
  puth(relay_malfunction);

  if (addr == PSA_LANE_KEEP_ASSIST) {
    print(" LKAS_MSG data: ");
    for (int i = 0; i < 8; i++) {
      puth(to_send->data[i]);
      print(" ");
    }
  }
  print("\n");

  return true;
}

static bool psa_fwd_hook(int bus_num, int addr) {
  bool block_msg = false;

  if (bus_num == PSA_MAIN_BUS) {
    block_msg = psa_lkas_msg_check(addr);
    if (block_msg) {
      print("PSA_FWD: Blocking LKAS msg addr=");
      puth(addr);
      print(" on bus=");
      puth(bus_num);
      print("\n");
    }
  }

  return block_msg;
}

static safety_config psa_init(uint16_t param) {
  print("PSA: psa_init called with param=");
  puth(param);
  print("\n");

  print("PSA: Initial state - controls_allowed=");
  puth(controls_allowed);
  print(" gas_pressed=");
  puth(gas_pressed);
  print(" brake_pressed=");
  puth(brake_pressed);
  print(" cruise_engaged_prev=");
  puth(cruise_engaged_prev);
  print("\n");

  return BUILD_SAFETY_CFG(psa_rx_checks, PSA_TX_MSGS);
}

const safety_hooks psa_hooks = {
  .init = psa_init,
  .rx = psa_rx_hook,
  .tx = psa_tx_hook,
  .fwd = psa_fwd_hook,
};