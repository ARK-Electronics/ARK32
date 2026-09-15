"""A corrupt signal must not keep a previously running motor alive."""

import time

import sitl_dshot as sd
from sitl_harness import Sender, rpm_from_state, wait_for_state


def test_bad_crc_after_running_triggers_signal_timeout(sitl_factory, state_stream):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim), sitl.log_tail()
    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    try:
        time.sleep(2.2)
        tx.value = 700
        time.sleep(3.5)
        assert rpm_from_state(sim) > 3000, sitl.log_tail()
    finally:
        tx.stop()

    port = sd.InputPort('127.0.0.1', sitl.input_port)
    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            port.send_dshot(700, ptype=sd.TYPE_DSHOT600, corrupt=True)
            time.sleep(0.002)
        assert sitl.proc.poll() is None, sitl.log_tail()
        assert sim.samples, 'state stream missing'
        rpm = rpm_from_state(sim, 0.5)
        assert 0 <= rpm < 800, 'bad CRC retained drive: rpm=%.0f\n%s' % (rpm, sitl.log_tail())
    finally:
        port.close()
