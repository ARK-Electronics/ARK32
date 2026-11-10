# Legacy DShot settings writes

While armed and stopped, send command 36 six times in a row, then one address frame, one value frame, and command 37. Command 37 writes the byte to RAM; command 12 saves settings to flash. Frames with or without the telemetry bit are accepted.

Addresses run 0..191, the size of the EEPROM buffer. An out-of-range address still consumes its value frame and command 37, and the transaction ends without writing. Only the low 8 bits of the value frame are stored.

After command 36, every CRC-valid frame is programming data until command 37 follows the value frame, or until a frame fails its CRC. Throttle frames, stop (0) included, are read as the address or value, so pause the throttle stream for the whole transaction.
