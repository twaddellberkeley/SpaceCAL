from ..JNode_MotorController.motorController import motorConnectTest
# Check the motorcontrollers


def test_motorConnected():
    rc = motorConnectTest()
    assert rc == 0, 'Could not connect to Motor'
