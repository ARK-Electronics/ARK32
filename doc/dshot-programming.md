# Legacy DShot settings writes

While armed and stopped, send command 36 six times, then one address frame,
one byte-value frame, and command 37 to commit the byte to RAM. Command 12
saves settings to flash. Frames with or without the telemetry bit are accepted.

The EEPROM buffer contains 192 bytes. At commit, an address outside 0..191
is rejected and the transaction ends without writing. The decoder still
consumes the address and value as programming data, so rejecting an address
does not turn the following value into throttle. Valid writes and the legacy
byte-value conversion are unchanged.

This guard prevents out-of-bounds writes only. Timeout, interrupted-transaction
recovery, and the ambiguity between zero data and MOTOR_STOP remain separate
protocol concerns. Clients must pause their normal throttle stream while
programming.
