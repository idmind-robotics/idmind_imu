"""
Hardware-agnostic IMU driver layer.

This package defines the driver contract (:mod:`idmind_imu.drivers.base`), a name-based
lookup for concrete drivers (:mod:`idmind_imu.drivers.registry`), and the drivers themselves,
e.g. :mod:`idmind_imu.drivers.brick_v2`. Nothing here imports ``rclpy`` or any ROS message
type; drivers are plain Python and communicate with the node exclusively through
``ImuSample`` instances handed to an ``on_sample`` callback.
"""
