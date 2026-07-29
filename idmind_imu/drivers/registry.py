"""
Name-based lookup for concrete IMU driver classes.

Drivers are imported lazily inside ``get_driver`` so that a driver whose hardware bindings
are not installed (e.g. ``tinkerforge`` for ``brick_v2``) does not prevent importing this
registry or listing the other available drivers.
"""

from typing import List, Type

from idmind_imu.drivers.base import ImuDriver

#: Maps a driver name to a "module path, class name" pair, imported on demand.
_DRIVERS = {
    "brick_v2": ("idmind_imu.drivers.brick_v2", "ImuBrickV2Driver"),
}


def available_drivers() -> List[str]:
    """Return the list of registered driver names, without importing any driver module."""
    return list(_DRIVERS.keys())


def get_driver(name: str) -> Type[ImuDriver]:
    """
    Return the driver class registered under ``name``.

    Raises ``KeyError`` with a message listing the available names if ``name`` is not
    registered, or if the driver's module fails to import (e.g. a missing dependency).
    """
    if name not in _DRIVERS:
        raise KeyError(
            "Unknown IMU driver '{}'; available drivers: {}".format(
                name, ", ".join(available_drivers())
            )
        )
    module_name, class_name = _DRIVERS[name]
    import importlib

    try:
        module = importlib.import_module(module_name)
    except ImportError as exc:
        raise KeyError(
            "IMU driver '{}' is registered but its module failed to import "
            "(missing dependency?): {}".format(name, exc)
        )
    return getattr(module, class_name)
