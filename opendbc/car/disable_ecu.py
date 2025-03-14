from opendbc.car.carlog import carlog
from opendbc.car.isotp_parallel_query import IsoTpParallelQuery
from panda import Panda
from opendbc.car import uds
import time

EXT_DIAG_REQUEST = b'\x10\x03'
EXT_DIAG_RESPONSE = b'\x50\x03'

COM_CONT_RESPONSE = b''

def disable_ecu(can_recv, can_send, bus=0, addr=0x7d0, sub_addr=None, timeout=0.1, retry=10):
  """
  Silence an ECU by disabling sending and receiving messages using UDS services.

  Options disable communication or put ECU in a non-functional state.

  WARNING: THIS MAY DISABLE SAFETY SYSTEMS LIKE AEB!
  """

  carlog.warning(f"ecu disable {hex(addr)}, sub_addr={sub_addr} ...")

  for i in range(retry):
    try:
      panda = Panda()
      radar_uds = uds.UdsClient(panda, tx_addr=addr, rx_addr=696, bus=bus, sub_addr=sub_addr)

      # === Option 1 === Disable RX and TX Completely
      radar_uds.communication_control(
        control_type=uds.CONTROL_TYPE.DISABLE_RX_DISABLE_TX,
        message_type=uds.MESSAGE_TYPE.NORMAL_AND_NETWORK_MANAGEMENT
      )
      carlog.warning("Option 1: Sent Communication Control (Disable RX & TX)")

      # === Option 2 === ECU Reset (Soft)
      # radar_uds.ecu_reset(reset_type=uds.RESET_TYPE.SOFT)
      # carlog.warning("Option 2: Sent ECU Soft Reset")

      # === Option 3 === ECU Reset (Hard)
      # radar_uds.ecu_reset(reset_type=uds.RESET_TYPE.HARD)
      # carlog.warning("Option 3: Sent ECU Hard Reset")

      # === Option 4 === Switch to Programming Session (Disrupt normal operations)
      # radar_uds.diagnostic_session_control(uds.SESSION_TYPE.PROGRAMMING)
      # carlog.warning("Option 4: Entered Programming Session")

      # === Option 5 === Control DTC Setting: Turn OFF DTC Reporting
      # radar_uds.control_dtc_setting(uds.DTC_SETTING_TYPE.OFF)
      # carlog.warning("Option 5: Turned off DTC Reporting (optional)")

      # === Option 6 === Response On Event: Stop All Responses
      # radar_uds.response_on_event(
      #   response_event_type=uds.RESPONSE_EVENT_TYPE.STOP_RESPONSE_ON_EVENT,
      #   store_event=False, window_time=0, event_type_record=0, service_response_record=0
      # )
      # carlog.warning("Option 6: Stopped Response On Event")

      carlog.error("ecu disabled successfully")
      return True

    except Exception:
      carlog.exception("ecu disable exception")

    carlog.error(f"ecu disable retry ({i + 1}) ...")
  carlog.error("ecu disable failed")
  return False
