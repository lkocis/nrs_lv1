# Complete 15-byte Robotiq 3F output process image, represented semantically.
# Zero is reserved; the controller assigns a monotonically increasing ID.
uint64 command_id

uint8 MODE_BASIC=0
uint8 MODE_PINCH=1
uint8 MODE_WIDE=2
uint8 MODE_SCISSOR=3

bool activate
uint8 mode
bool go_to
bool automatic_release

bool glove_mode
bool individual_finger_control
bool individual_scissor_control

uint8 position_a
uint8 speed_a
uint8 force_a

uint8 position_b
uint8 speed_b
uint8 force_b

uint8 position_c
uint8 speed_c
uint8 force_c

uint8 position_scissor
uint8 speed_scissor
uint8 force_scissor
