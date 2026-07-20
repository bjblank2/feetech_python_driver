# feetech_python_driver

Shared Python Feetech servo bus driver (`FeetechMotorsBus`, `Motor`, `MotorCalibration`) and interactive calibration routine (`So101CalibrationNodeBase`), used by [`so101_ros2`](https://github.com/bjblank2/so101_ros2), [`lekiwi_ros2`](https://github.com/bjblank2/lekiwi_ros2), and [`lerre_ros2`](https://github.com/bjblank2/lerre_ros2) for direct-serial access outside the ros2_control realtime loop.

Robot-specific packages depend on this for calibration; realtime motor control goes through [`feetech_ros2_driver`](https://github.com/bjblank2/feetech_ros2_driver)'s ros2_control hardware interface instead.

ament_python package, ROS 2.
