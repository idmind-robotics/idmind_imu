"""
Driver contract shared by all IMU hardware backends.

This module defines the data the node receives (``ImuSample``), the status a driver reports
about itself (``DriverState``), and the abstract interface (``ImuDriver``) every concrete
driver must implement. It has no dependency on ROS: samples are plain dataclasses and the
logger passed to a driver is duck-typed rather than a ``rclpy`` logger, so this layer can be
imported and unit tested without a ROS environment.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Callable, Optional, Tuple


@dataclass(frozen=True)
class ImuSample:
    """
    One reading from an IMU, already converted to SI units in ROS conventions.

    Every field defaults to ``None`` (``orientation_valid`` to ``False``) so a driver that
    cannot supply a given quantity simply omits it rather than fabricating a value.
    """

    orientation: Optional[Tuple[float, float, float, float]] = None
    orientation_valid: bool = False
    angular_velocity: Optional[Tuple[float, float, float]] = None
    linear_acceleration: Optional[Tuple[float, float, float]] = None
    magnetic_field: Optional[Tuple[float, float, float]] = None
    gravity: Optional[Tuple[float, float, float]] = None
    euler: Optional[Tuple[float, float, float]] = None
    temperature: Optional[float] = None
    calibration: Optional[Tuple[int, int, int, int]] = None
    fusion_mode: Optional[int] = None


@dataclass
class DriverState:
    """A snapshot of a driver's connection and device status, for diagnostics."""

    connected: bool = False
    device_present: bool = False
    hardware_id: str = "unknown"
    detail: str = ""


class ImuDriver(ABC):
    """
    Abstract base for a hardware-specific IMU driver.

    A driver owns the transport connection to one IMU and reports readings by calling
    ``on_sample`` with an ``ImuSample``, from whatever thread the underlying hardware library
    delivers them on. The node is responsible for thread-safety on its own side; the driver is
    responsible for never blocking the caller of ``start()``.
    """

    def __init__(self, config: dict, on_sample: Callable[[ImuSample], None], logger):
        """
        Store the initial config, sample callback, and a duck-typed logger.

        ``logger`` need only provide ``.info``, ``.warning``, ``.error``, and ``.debug``,
        each taking a single string, so callers can pass a ``rclpy`` logger, a stdlib
        ``logging.Logger``, or anything else with a matching shape.
        """
        self._config = dict(config)
        self._on_sample = on_sample
        self._logger = logger

    @abstractmethod
    def start(self) -> None:
        """Begin connecting and streaming. Must not block; spawn threads if needed."""
        raise NotImplementedError

    @abstractmethod
    def stop(self) -> None:
        """Tear down the driver. Idempotent: safe to call twice or before ``start()``."""
        raise NotImplementedError

    @abstractmethod
    def apply_config(self, config: dict) -> None:
        """Apply a changed configuration, e.g. in response to updated ROS parameters."""
        raise NotImplementedError

    @abstractmethod
    def state(self) -> DriverState:
        """Return a snapshot of the driver's current connection and device status."""
        raise NotImplementedError
