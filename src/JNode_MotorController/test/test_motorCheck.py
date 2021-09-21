# from ..JNode_MotorController.motorController import motorConnectTest
from JNode_MotorController.motorController import motorConnectTest
import pytest
# Check the motorcontrollers


@pytest.mark.core
def test_motorConnected():
    rc = motorConnectTest()
    assert rc == 0, 'Could not connect to Motor'
