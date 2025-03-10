from opendbc.car.disable_ecu import disable_ecu
from opendbc.car.carlog import carlog
import time

def test_disable_radar_ecu(can_recv, can_send):
    """
    Test various combinations of parameters to disable the ARTIV Radar ECU.
    Attempts different buses, addresses, sub-addresses, and communication control requests.
    """
    # Known addresses for ARTIV from the documentation
    primary_addrs = [
        0x6B6,  # ARTIV diag request (1718 decimal)
        0x696,  # ARTIV diag response (1686 decimal)
        0x7d0,  # Common UDS diagnostic address
        0x750,  # Alternative UDS address (from Toyota example)
    ]

    # Additional potential addresses from the DBC
    secondary_addrs = [
        0x116,  # HS2_VERS_ARTIV_116
        0x2B2,  # HS2_DYN2_CMM_2B2
        0x2B6,  # HS2_DYN1_MDD_ETAT_2B6
        0x2F6,  # HS2_DYN_MDD_STATUS_2F6
        0x32D,  # HS2_DYN_UCF_MDD_32D
        0x38D,  # HS2_DYN_ABR_38D
        0x452,  # HS2_DAT_MDD_CMD_452
        0x4F6,  # HS2_DAT_ARTIV_V2_4F6
        0x796,  # HS2_SUPV_ARTIV_796
    ]

    # Possible sub-addresses
    sub_addrs = [None, 0x01, 0xf, 0x10]

    # Common communication control requests from examples
    com_cont_reqs = [
        b'\x28\x83\x01',  # Standard communication control
        b'\x28\x03\x01',  # Subaru example
        b'\x28\x83\x03',  # Honda example
        b'\x28\x03\x03',  # Alternative
        b'\x28\x00\x01',  # Try with different control types
        b'\x28\x01\x01',
        b'\x28\x02\x01',
    ]

    # Buses to try
    buses = [1]  # Bus 1 is mentioned as primary, but let's try others too
    # buses = [0, 1, 2]

    # Extra parameters to try
    timeouts = [0.1, 0.5, 1.0]
    retries = [10, 15]

    # Keep track of successful combinations
    successful_combos = []

    # Log start of testing
    carlog.warning("Starting comprehensive ARTIV ECU disable testing...")

    # Try primary addresses first with more variations
    for addr in primary_addrs:
        for bus in buses:
            for sub_addr in sub_addrs:
                for com_cont_req in com_cont_reqs:
                    for timeout in timeouts:
                        for retry in retries:
                            carlog.warning(f"Testing: Bus={bus}, Addr=0x{addr:x}, Sub_addr={sub_addr}, " +
                                          f"Com_req={com_cont_req.hex()}, Timeout={timeout}, Retry={retry}")

                            result = disable_ecu(
                                can_recv=can_recv,
                                can_send=can_send,
                                bus=bus,
                                addr=addr,
                                sub_addr=sub_addr,
                                com_cont_req=com_cont_req,
                                timeout=timeout,
                                retry=retry
                            )

                            if result:
                                successful_combo = {
                                    'bus': bus,
                                    'addr': f"0x{addr:x}",
                                    'sub_addr': sub_addr,
                                    'com_cont_req': com_cont_req.hex(),
                                    'timeout': timeout,
                                    'retry': retry
                                }
                                successful_combos.append(successful_combo)
                                carlog.warning(f"SUCCESS! Found working combination: {successful_combo}")

                            # Brief pause between attempts to avoid flooding the bus
                            time.sleep(0.5)

    # If no success with primary addresses, try secondary addresses with fewer variations
    if not successful_combos:
        carlog.warning("No success with primary addresses, trying secondary addresses...")
        for addr in secondary_addrs:
            for bus in buses:
                for com_cont_req in com_cont_reqs[:3]:  # Try only the first 3 communication requests
                    carlog.warning(f"Testing: Bus={bus}, Addr=0x{addr:x}, " +
                                  f"Com_req={com_cont_req.hex()}")

                    result = disable_ecu(
                        can_recv=can_recv,
                        can_send=can_send,
                        bus=bus,
                        addr=addr,
                        sub_addr=None,
                        com_cont_req=com_cont_req,
                        timeout=0.5,  # Medium timeout
                        retry=10
                    )

                    if result:
                        successful_combo = {
                            'bus': bus,
                            'addr': f"0x{addr:x}",
                            'sub_addr': None,
                            'com_cont_req': com_cont_req.hex(),
                            'timeout': 0.5,
                            'retry': 10
                        }
                        successful_combos.append(successful_combo)
                        carlog.warning(f"SUCCESS! Found working combination: {successful_combo}")

                    # Brief pause between attempts
                    time.sleep(0.5)

    # Final report
    if successful_combos:
        carlog.warning(f"Testing complete. Found {len(successful_combos)} working combinations:")
        for idx, combo in enumerate(successful_combos, 1):
            carlog.warning(f"Working combination {idx}: {combo}")

        # Return the first successful combination
        return successful_combos[0]
    else:
        carlog.error("Testing complete. No working combinations found.")
        return None


def implement_best_disable_strategy(can_recv, can_send):
    """
    Implements the best strategy to disable the ARTIV Radar ECU based on testing.
    This function should be called from your main initialization routine.
    """
    # First attempt: Direct disable with most likely parameters
    # ARTIV is on BUS 1 according to documentation
    direct_result = disable_ecu(
        can_recv=can_recv,
        can_send=can_send,
        bus=1,
        addr=0x6B6,  # ARTIV diag request address
        sub_addr=None,
        com_cont_req=b'\x28\x83\x01',
        timeout=0.5,
        retry=10
    )

    if direct_result:
        carlog.warning("Successfully disabled ARTIV ECU using direct parameters.")
        return True

    # Second attempt: Try address from Honda example
    second_result = disable_ecu(
        can_recv=can_recv,
        can_send=can_send,
        bus=1,
        addr=0x18DAB0F1,
        sub_addr=None,
        com_cont_req=b'\x28\x83\x03',
        timeout=0.5,
        retry=10
    )

    if second_result:
        carlog.warning("Successfully disabled ARTIV ECU using Honda-like parameters.")
        return True

    # If direct attempts fail, run the comprehensive test
    carlog.warning("Direct disable attempts failed. Running comprehensive testing...")
    best_combo = test_disable_radar_ecu(can_recv, can_send)

    if best_combo:
        carlog.warning(f"Found working combination after comprehensive testing: {best_combo}")
        return True

    return False