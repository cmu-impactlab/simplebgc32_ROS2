# Identity of the connected SimpleBGC32 controller, from CMD_BOARD_INFO.
#
# Published with transient_local durability: it changes only when the board
# does, so a node that starts late should still receive it rather than wait for
# a reconnect.

# stamp is when the identifying reply was received from the board, not when
# this message was published — so a consumer can tell how old the identity is.
# frame_id is left empty: board identity is not spatial data.
std_msgs/Header header

# Board hardware version. The board reports these packed, e.g. 31 meaning 3.1.
uint8 board_version_major
uint8 board_version_minor

# Firmware version. The board reports a single number, e.g. 2630 meaning
# 2.63 b0, which is kept alongside the split fields because it is what the
# vendor documentation and support forums quote.
uint16 firmware_version_raw
uint8 firmware_version_major
uint8 firmware_version_minor
uint8 firmware_version_beta

# Raw board state and capability words. Left undecoded on purpose: naming bits
# whose meaning has not been verified against hardware would be a guess, and a
# wrong capability claim is worse than an opaque number.
uint8 state_flags
uint16 board_features
uint8 connection_flag
