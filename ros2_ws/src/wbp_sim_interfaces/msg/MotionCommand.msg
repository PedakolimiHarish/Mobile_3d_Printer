# ROS-SIM INTERNAL — NOT A FIRMWARE CONTRACT
# Represents intent-only motion request

uint32 sequence_id        # Monotonic, assigned by execution layer
uint8 intent              # Motion intent type

# Intent enum
uint8 TRAVEL    = 0
uint8 DEPOSIT   = 1
uint8 RETRACT   = 2
uint8 UNRETRACT = 3
uint8 WAIT      = 4
