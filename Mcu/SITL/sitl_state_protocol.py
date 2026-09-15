"""ARK extensions to the upstream SITL state-port command protocol.

Upstream owns the low command numbers. Keep local test/control hooks in
0x80..0xff so additions such as WATCH_VARS (8) and RESET (9) stay compatible.
"""

STATE_MAGIC_CMD = 0x5353
STATE_CMD_ZC_FAULT = 0x80
STATE_CMD_ZC_STATS = 0x81
STATE_CMD_GOV_FORCE = 0x82
