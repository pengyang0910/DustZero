#ifndef __TLV_H__
#define __TLV_H__
#include "tcp_server.h"
#include "stdint.h"

int read_tl_f(int& client, char data[6]);
int read_tl(int& client, char data[6], int timeout_us);
int read_value_f(int& client, char *  data, int len);
int read_value(int& client, char *  data, int len, int timeout_us);
int write_tlv(int& client, uint16_t type, uint32_t len, char* data, uint32_t len2 = 0, char* data2 = 0);
int write_tl(int& client, uint16_t type, uint32_t len);
int write_value(int& client, char* data, uint32_t len);


#endif
