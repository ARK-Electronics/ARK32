"""Construct OpenOCD debuggers from :class:`~hwci.config.RigConfig`."""
from __future__ import annotations

from typing import TYPE_CHECKING

from .openocd import OpenOcdDebugger

if TYPE_CHECKING:
    from ..config import RigConfig


def openocd_from_rig(rig: "RigConfig", **overrides) -> OpenOcdDebugger:
    """Build an :class:`OpenOcdDebugger` with target-correct SWD settings.

    Pulls ``openocd_configs``, ``app_load_addr``, and ``adapter_speed_khz``
    from the rig (already filled by :func:`hwci.config.apply_target_preset`
    for known targets like ``ARK_G431_CAN``).
    """
    kwargs = dict(
        configs=list(rig.openocd_configs),
        openocd_bin=rig.openocd_bin,
        search_dirs=list(rig.openocd_search_dirs),
        app_load_addr=int(rig.app_load_addr),
        adapter_speed_khz=rig.adapter_speed_khz,
    )
    kwargs.update(overrides)
    return OpenOcdDebugger(**kwargs)
