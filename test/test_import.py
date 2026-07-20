import feetech_python_driver


def test_public_api_importable():
    assert hasattr(feetech_python_driver, 'FeetechMotorsBus')
    assert hasattr(feetech_python_driver, 'OperatingMode')
    assert hasattr(feetech_python_driver, 'Motor')
    assert hasattr(feetech_python_driver, 'MotorCalibration')
    assert hasattr(feetech_python_driver, 'MotorNormMode')
    assert hasattr(feetech_python_driver, 'So101CalibrationNodeBase')
    assert hasattr(feetech_python_driver, 'SO101_ARM_MOTORS')
    assert len(feetech_python_driver.SO101_ARM_MOTORS) == 6
