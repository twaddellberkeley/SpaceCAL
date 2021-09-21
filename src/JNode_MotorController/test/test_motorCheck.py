# from ..JNode_MotorController.motorController import motorConnectTest
from JNode_MotorController.motorController import motorConnectTest
import pytest
# Check the motorcontrollers


def motorConnected(adr):
    rc = motorConnectTest(adr)
    assert rc == 0, 'Could not connect to Motor %d' % adr


@pytest.mark.core
@pytest.mark.motor
def test_allMotors():
    for x in range(12, 17, 1):
        assert motorConnected(x)
