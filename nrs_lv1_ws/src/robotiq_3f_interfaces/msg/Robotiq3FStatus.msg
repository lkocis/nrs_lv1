# Complete Robotiq 3F input process image plus transport health.

uint8 MODE_BASIC=0
uint8 MODE_PINCH=1
uint8 MODE_WIDE=2
uint8 MODE_SCISSOR=3

uint8 INITIALIZATION_RESET=0
uint8 INITIALIZATION_ACTIVATING=1
uint8 INITIALIZATION_MODE_CHANGE=2
uint8 INITIALIZATION_READY=3

uint8 MOTION_MOVING=0
uint8 MOTION_PARTIAL_CONTACT=1
uint8 MOTION_ALL_CONTACT=2
uint8 MOTION_REACHED=3

uint8 OBJECT_MOVING=0
uint8 OBJECT_CONTACT_OPENING=1
uint8 OBJECT_CONTACT_CLOSING=2
uint8 OBJECT_REACHED=3

builtin_interfaces/Time stamp
bool connected
uint64 connection_id
# ID of the most recent command successfully written on this connection.
uint64 last_command_id
string diagnostic

bool activated
uint8 mode
bool go_to
uint8 initialization_status
uint8 motion_status

uint8 object_status_a
uint8 object_status_b
uint8 object_status_c
uint8 object_status_scissor
uint8 fault_status

uint8 requested_position_a
uint8 actual_position_a
uint8 current_a

uint8 requested_position_b
uint8 actual_position_b
uint8 current_b

uint8 requested_position_c
uint8 actual_position_c
uint8 current_c

uint8 requested_position_scissor
uint8 actual_position_scissor
uint8 current_scissor
