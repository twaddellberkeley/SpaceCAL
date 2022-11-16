// #include <cstdio>

// int main(int argc, char ** argv)
// {
//   (void) argc;
//   (void) argv;
//   run()

//   printf("hello world test_pakage package\n");
//   return 0;
// }


/*------------------------------------------------------------------------------
 * Copyright (c) 2019 Texas Instruments Incorporated - http://www.ti.com/
 *------------------------------------------------------------------------------
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

 /**
  * \file   dlpc343x_samples
  * \brief  Sample code for communicating with a DLPC34xx Display controller.
  */

#include "dlpc_common.h"
#include "dlpc34xx.h"
#include "cypress_i2c.h"
#include "devasys_i2c.h"
#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "time.h"
// #include <iostream>
#include <unistd.h> 

#include "proj_exec/proj_utils.h"

#define CYPRESS_I2C_COMMUNICATION         1		/* set to 0 for DeVaSys I2C Communication */

#define MAX_WIDTH                         DLP2010_WIDTH
#define MAX_HEIGHT                        DLP2010_HEIGHT

#define FLASH_WRITE_BLOCK_SIZE            1024
#define FLASH_READ_BLOCK_SIZE             256

#define MAX_WRITE_CMD_PAYLOAD             (FLASH_WRITE_BLOCK_SIZE + 8)
#define MAX_READ_CMD_PAYLOAD              (FLASH_READ_BLOCK_SIZE  + 8)

static uint8_t                            s_WriteBuffer[MAX_WRITE_CMD_PAYLOAD];
static uint8_t                            s_ReadBuffer[MAX_READ_CMD_PAYLOAD];

bool I2CCommSucceed = false;

/**
 * Implement the I2C write transaction here. The sample code here sends
 * data to the controller via the Cypress USB-Serial adapter.
 */
uint32_t WriteI2C(uint16_t             WriteDataLength,
	uint8_t*                           WriteData,
	DLPC_COMMON_CommandProtocolData_s* ProtocolData)
{
	bool Status = true;
	//printf("Write I2C Starts, length %d!!! \n", WriteDataLength);

	if (CYPRESS_I2C_COMMUNICATION)
	{
		Status = CYPRESS_I2C_WriteI2C(WriteDataLength, WriteData);
	}
	else
	{
		Status = DEVASYS_I2C_WriteI2C(WriteDataLength, WriteData);
	}

	if (Status != true)
	{
		printf("Write I2C Error!!! \n");
		return FAIL;
	}

	return SUCCESS;
}

/**
 * Implement the I2C write/read transaction here. The sample code here
 * receives data from the controller via the Cypress USB-Serial adapter.
 */
uint32_t ReadI2C(uint16_t              WriteDataLength,
	uint8_t*                           WriteData,
	uint16_t                           ReadDataLength,
	uint8_t*                           ReadData,
	DLPC_COMMON_CommandProtocolData_s* ProtocolData)
{
	bool Status = 0;
	//printf("Read/Write I2C Starts%d!!! \n", WriteDataLength);

	if (CYPRESS_I2C_COMMUNICATION)
	{
		Status = CYPRESS_I2C_WriteI2C(WriteDataLength, WriteData);
	}
	else
	{
		Status = DEVASYS_I2C_WriteI2C(WriteDataLength, WriteData);
	}

	if (Status != true)
	{
		printf("Write I2C Error!!! \n");
		return FAIL;
	}
	if (CYPRESS_I2C_COMMUNICATION)
	{
		Status = CYPRESS_I2C_ReadI2C(ReadDataLength, ReadData);
	}
	else
	{
		Status = DEVASYS_I2C_ReadI2C(ReadDataLength, ReadData);
	}

	if (Status != true)
	{
		printf("Read I2C Error!!! \n");
		return FAIL;
	}

	return SUCCESS;
}

/**
 * Initialize the command layer by setting up the read/write buffers and
 * callbacks.
 */
void InitConnectionAndCommandLayer(void)
{
	DLPC_COMMON_InitCommandLibrary(s_WriteBuffer,
		sizeof(s_WriteBuffer),
		s_ReadBuffer,
		sizeof(s_ReadBuffer),
		WriteI2C,
		ReadI2C);

	if (CYPRESS_I2C_COMMUNICATION)
	{
		I2CCommSucceed = CYPRESS_I2C_ConnectToCyI2C();
	}
	else
	{
		I2CCommSucceed = DEVASYS_I2C_ConnectToDevI2C();
	}
}


bool dlp_init()
{

	printf("Initializing DLP Controller\n");
	system("vcgencmd display_power 1");
	sleep(1);
	InitConnectionAndCommandLayer();


	if (!I2CCommSucceed)
	{
		printf("Error Init Connection & Command Layer!!!\n");
		return false;
	}

	/* TI DLP Pico EVMs use a GPIO handshake scheme for Cypress I2C bus arbitration.
	 * Call to request/relinquish I2C Bus Access if using a TI EVM; remove otherwise.
	 * For DeVaSys I2C; need to manually request/relinquish before running the sample.
	 */
	if (CYPRESS_I2C_COMMUNICATION)
	{
		
		bool Status = CYPRESS_I2C_RequestI2CBusAccess();
		if (Status != true)
		{
			printf("Error Request I2C Bus ACCESS!!!\n");
			return false;
		}
	}

	printf("Succesfully Aquire I2C Buss Acces!!\n");
	
	
  return true;
}

bool requestI2CBuss(void)
{
	/* TI DLP Pico EVMs use a GPIO handshake scheme for Cypress I2C bus arbitration.
	 * Call to request/relinquish I2C Bus Access if using a TI EVM; remove otherwise.
	 * For DeVaSys I2C; need to manually request/relinquish before running the sample.
	 */
	if (CYPRESS_I2C_COMMUNICATION)
	{
		bool Status = CYPRESS_I2C_RequestI2CBusAccess();
		if (Status != true)
		{
			printf("Error Request I2C Bus ACCESS!!!\n");
			return false;
		}
		return Status;
	}
}

bool relinquishI2CBuss(void)
{
	printf("Relesing BussAccess!!\n");
	if (CYPRESS_I2C_COMMUNICATION)
	{
		return CYPRESS_I2C_RelinquishI2CBusAccess();
	}
	return false;
}


bool led_on(void)
{
	bool r,g,b;
	int count = 0;
	while (b != 1 && count < 100) {
		DLPC34XX_WriteRgbLedEnable(false,false,1);
	
		DLPC34XX_ReadRgbLedEnable(&r,&g,&b);
		printf("RGB Enabled: r %d, g %d, b %d \n",r,g,b);
		count++;
	}
	if (b != 1) {
		printf("Could not enable led\n");
		
	}

	DLPC34XX_WriteRgbLedCurrent(0x000, 0x000, 0x3FF);
	uint16_t rc,gc,bc;
	DLPC34XX_ReadRgbLedCurrent(&rc,&gc,&bc);
	printf("RGB Led Current: r %d, g %d, b %d \n",rc,gc,bc);
	return true;
}

bool led_off(void)
{
	DLPC34XX_WriteRgbLedEnable(false,false,true);
	DLPC34XX_WriteRgbLedCurrent(0x000, 0x000, 0x000);
	return true;
}

void led_time_on(uint16_t time) 
{
	led_on();
	sleep(time);
	led_off();
}

// DLPC34XX_ControllerDeviceId_e DeviceId = DLPC34XX_CDI_DLPC3479;
// 	DLPC34XX_ReadControllerDeviceId(&DeviceId);
// 	printf("Controller Devicde Id = %d \n", DeviceId);
// 	// printf("post sleep");

// 	int cmd;
// 	std::cout << "Enter command: ";
// 	std::cin >> cmd;
// 	while (cmd != 10) {
		
// 		switch(cmd) {
// 			case 0:
// 				// code block
// 				break;
// 			case 1:
// 				// code block
// 				DLPC34XX_WriteRgbLedEnable(false,false,true);
// 				DLPC34XX_WriteRgbLedCurrent(0x000, 0x000, 0x3FF);
// 				printf("post sleep");
// 				break;
// 			case 2:
// 				// code block
// 				DLPC34XX_WriteRgbLedEnable(false,false,true);
// 				DLPC34XX_WriteRgbLedCurrent(0x000, 0x000, 0x000);
// 				break;
// 			case 3:
// 				// code block
// 				DLPC34XX_WriteRgbLedEnable(false,false,true);
// 				DLPC34XX_WriteRgbLedCurrent(0x000, 0x000, 0x3FF);
// 				sleep(8);
// 				DLPC34XX_WriteRgbLedEnable(false,false,true);
// 				DLPC34XX_WriteRgbLedCurrent(0x000, 0x000, 0x000);
// 				break;
// 			case 4:
// 				// code block
// 				break;
// 			// default:
// 			// 	// code block
// 		}
// 		std::cout << "Enter new command: ";
// 		std::cin >> cmd;
// 	}




	// DLPC34XX_WriteRgbLedEnable(false,false,true);
	// DLPC34XX_WriteRgbLedCurrent(0x000, 0x000, 0x3FF);
	/**
	uint16_t rc,gc,bc;
	DLPC34XX_ReadRgbLedCurrent(&rc,&gc,&bc);
	printf("r %d, g %d, b %d \n",rc,gc,bc);
	bool r,g,b;
	uint32_t status2 = DLPC34XX_ReadRgbLedEnable(&r,&g,&b);
	printf("r %d, g %d, b %d \n",r,g,b);
	**/

	// if (CYPRESS_I2C_COMMUNICATION)
	// {
	// 	CYPRESS_I2C_RelinquishI2CBusAccess();
	// }