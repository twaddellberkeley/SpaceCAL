
#include "dlpc_common.h"
#include "dlpc34xx.h"


void InitConnectionAndCommandLayer(void);


uint32_t WriteI2C(uint16_t             WriteDataLength,
	uint8_t*                           WriteData,
	DLPC_COMMON_CommandProtocolData_s* ProtocolData);


uint32_t ReadI2C(uint16_t              WriteDataLength,
	uint8_t*                           WriteData,
	uint16_t                           ReadDataLength,
	uint8_t*                           ReadData,
	DLPC_COMMON_CommandProtocolData_s* ProtocolData);


bool dlp_init();

bool requestI2CBuss(void);

bool relinquishI2CBuss(void);

bool led_on(void);

bool led_off(void);

void led_time_on(uint16_t time);