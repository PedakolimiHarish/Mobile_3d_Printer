# ROS-SIM INTERNAL — NOT A FIRMWARE CONTRACT
# Represents outcome of simulated motion

uint32 sequence_id

uint8 status

# Result enum
uint8 ACCEPTED    = 0
uint8 IN_PROGRESS = 1
uint8 COMPLETED   = 2
uint8 REJECTED    = 3
uint8 FAULTED     = 4
