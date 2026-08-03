'''decode the AM32 debug1 structure carried in DroneCAN FlexDebug'''

import struct

FLEXDEBUG_AM32_DEBUG1 = 100

V1_FMT = '<BIHHHHiB'
V1_FIELDS = ('ver', 'ci', 'ncmd', 'ninp', 'rx_errors', 'rxframe_error',
             'rx_ecode', 'adv')
# Upstream PR#394 / ark-release v2: duty/input/ADC/flags.
V2_FMT = V1_FMT + 'HHHHHB'
V2_FIELDS = V1_FIELDS + ('duty', 'duty_max', 'adj_input',
                         'adc_current', 'adc_volts', 'flags')
# ARK append-only after upstream v2 (version stays 2): start-resisted count.
V2_ARK_FMT = V2_FMT + 'B'
V2_ARK_FIELDS = V2_FIELDS + ('acq_resist_events',)


def decode_debug1(data):
    '''return a dict of fields, or None if not a known debug1 layout'''
    data = bytes(data)
    n = len(data)
    v2 = struct.calcsize(V2_FMT)
    v2_ark = struct.calcsize(V2_ARK_FMT)
    if n == struct.calcsize(V1_FMT) and data[0] == 1:
        fmt, fields = V1_FMT, V1_FIELDS
    elif data[0] == 2 and n >= v2:
        # Accept exact v2 or ARK-appended tail; ignore any further unknown
        # bytes so future appends stay forward-compatible for this decoder.
        if n >= v2_ark:
            fmt, fields = V2_ARK_FMT, V2_ARK_FIELDS
            data = data[:v2_ark]
        else:
            fmt, fields = V2_FMT, V2_FIELDS
            data = data[:v2]
    else:
        return None
    row = dict(zip(fields, struct.unpack(fmt, data)))
    del row['ver']
    if 'flags' in row:
        row['armed'] = row['flags'] & 1
        row['running'] = (row['flags'] >> 1) & 1
        row['sine'] = (row['flags'] >> 2) & 1
        del row['flags']
    return row
