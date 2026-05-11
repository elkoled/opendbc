def create_steering_control(packer, bus, apply_curvature, lkas_enabled, power):
  values = {
    "Curvature": abs(apply_curvature),  # in rad/m
    "Curvature_VZ": 1 if apply_curvature > 0 and lkas_enabled else 0,
    "Power": power if lkas_enabled else 0,
    "RequestStatus": 4 if lkas_enabled else 2,
    "HighSendRate": lkas_enabled,
  }
  return packer.make_can_msg("HCA_03", bus, values)
