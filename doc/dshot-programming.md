# Legacy DShot settings writes

While armed and stopped, send command 36 six times, then one address frame,
one byte-value frame, and command 37 to commit the byte to RAM. Command 12
saves settings to flash. Complete the transaction within 100 ms of entry.
Addresses must be 0..191 and values 0..255; invalid stages abort without a
write. A late frame is discarded, and a fresh command-36 sequence can retry.
Bad CRC aborts the transaction. Nonzero programming frames require the
telemetry bit, matching PX4's command sender; zero payloads may omit it.

The legacy format has no transaction identifier or independent payload type.
A zero data byte is identical to MOTOR_STOP, and a marked in-range throttle
value is identical to programming data. Consequently the decoder cannot
ignore every stop/throttle-looking frame while preserving existing clients.
Bounds, a fixed deadline and a final commit limit malformed transactions;
they do not authenticate a write or distinguish all interleaved payloads.
Clients must stop their normal throttle stream while programming. A fully
unambiguous protocol would require a coordinated client/firmware extension.
