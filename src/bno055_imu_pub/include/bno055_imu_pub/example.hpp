#pragma once

//#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <fcntl.h>
#include <cerrno>
#include <chrono>
#include <thread>

/** BNO055 Address Alternative **/
#define BNO055_ADDRESS_A (0x28) // This requires the ADR pin on the bno055 to be low
/** BNO055 Address Default **/
#define BNO055_ADDRESS_DEFAULT (0x29) // This requires the ADR pin to the bno055 to be high
/** BNO055 Adress being used **/
#define BNO055_ADDRESS BNO055_ADDRESS_DEFAULT
/** BNO055 ID **/
#define BNO055_ID (0xA0)

/** Offsets registers **/
#define NUM_BNO055_OFFSET_REGISTERS (22)
#define NUM_BNO055_QUATERNION_REGISTERS (8)
#define NUM_BNO055_EULER_REGISTERS (6)