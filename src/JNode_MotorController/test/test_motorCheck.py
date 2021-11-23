# from ..JNode_MotorController.motorController import motorConnectTest
from JNode_MotorController.motorController import *
import pytest

rotMotors = [12,13,14,15,16]
liftMotors = [14,15,16,17]


# Check the motorcontrollers
def motorConnected(adr):
    rc = motorConnectTest(adr)
    assert rc == 0, 'Could not connect to Motor %d' % adr

#Check to make sure motor controllers on
@pytest.mark.core
@pytest.mark.motor
@pytest.mark.testCon
def test_allMotors():
    #for x in rotMotors:
    #    assert motorConnected(x)
    for x in liftMotors:
        assert motorConnected(x)

#Check to make sure we can get position
@pytest.mark.core
@pytest.mark.motor
def test_allMValue():
    for x in rotMotors:
        assert motorPositionTest(x)
    for x in liftMotors:
        assert motorPositionTest(x)

# Check that limit switches are homed correctly
@pytest.mark.core
@pytest.mark.motor
def test_liftHome():
    for x in liftMotors:
        motorHomeTest(x)
    print("Motors started homing") 
    for x in liftMotors:
        assert doneHoming(x)

# Check to make sure we can move up

# Check to make sure we can move back down

# Check to make sure we rotate/set velocity
@pytest.mark.core
@pytest.mark.motor
def test_rotation():
    for x in rotMotors:
        assert velocityTest(x)
    for x in rotMotors:
        assert velcotyStopTest(x)
