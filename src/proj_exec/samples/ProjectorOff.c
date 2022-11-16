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

static FILE*								s_FilePointer;

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
void InitConnectionAndCommandLayer()
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


void main()
{
	InitConnectionAndCommandLayer();

	if (!I2CCommSucceed)
	{
		printf("Error Init Connection & Command Layer!!!");
		return;
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
			printf("Error Request I2C Bus ACCESS!!!");
			return;
		}
	}
	bool uselessReturn = 0;
	DLPC34XX_ReadColorCoordinateAdjustmentControl(&uselessReturn);

	if (CYPRESS_I2C_COMMUNICATION)
	{
		CYPRESS_I2C_RelinquishI2CBusAccess();
	}
}
