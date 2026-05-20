#ifndef __I2C_LIB_H__
#define __I2C_LIB_H__
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

	int xt_i2c_open(void);
	int xt_i2c_write_reg(uint8_t addr, uint8_t data);
	int xt_i2c_read_reg(uint8_t addr, uint8_t* data);
	int xt_i2c_read_reg_bulk(uint8_t addr, uint8_t* data, uint16_t len);
	int xt_i2c_close(void);

	uint64_t xt_getTimeStap_us();
#ifdef __cplusplus
}
#endif

#endif