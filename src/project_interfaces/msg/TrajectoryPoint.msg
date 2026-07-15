# One sample of a planned trajectory in the world frame.
# yaw and curvature are provided for downstream consumers and debugging;
# the shipped controller derives its own heading from x/y and uses speed.
float64 x          # position [m]
float64 y          # position [m]
float64 yaw        # heading [rad]
float64 curvature  # unsigned path curvature magnitude [1/m]
float64 speed      # target speed at this point [m/s]
