#include "tlv.h" 
#include "stdlib.h"
#include <iostream>
#include "../xutils.h"

int read_tl_f(int& client, char data[6])
{
	return tcp_server_read(client, data, 6);		
}

int read_tl(int& client, char data[6], int timeout_us)
{
	int index = 0;
	int tmp;

	if (index < 6)
	{
		tmp = tcp_server_read_timeout(client, data + index, 6 - index, timeout_us / 1000000, timeout_us % 1000000);
		if (tmp < 0)
		{
			index = 0;
			return -1;
		}
		index += tmp;
	}

	if (index < 6)
	{
		return 0;
	}

	return 6;
}

int read_value_f(int& client, char *  data, int len)
{
	int tmp = 0;
	int index = 0;
	uint32_t start_time = getTimeStap_ms();

	while ((getTimeStap_ms() - start_time) < 20)
	{
		tmp = tcp_server_read(client, data + index, len - index);
		if (tmp < 0)
			return -1;

		index += tmp;
		if (index == len)
		{
			break;
		}
	}

	return index;

}

int read_value(int& client, char *  data, int len, int timeout_us)
{
	int tmp = 0;
	int index = 0;
	uint32_t start_time = getTimeStap_ms();

	while ((getTimeStap_ms() - start_time) < static_cast<uint32_t>(timeout_us / 1000))
	{
		tmp = tcp_server_read_timeout(client, data + index, len - index, timeout_us / 1000000, timeout_us % 1000000);
		if (tmp < 0)
			return -1;

		index += tmp;
		if (index == len)
		{
			break;
		}
	}

	return index;

}

int write_tlv(int& client, uint16_t type, uint32_t len, char* data, uint32_t len2, char* data2)
{
	uint8_t header[6];
	uint32_t totalLength = len + len2;

	header[0] = type >> 8;
	header[1] = type;
	header[2] = totalLength >> 24;
	header[3] = totalLength >> 16;
	header[4] = totalLength >> 8;
	header[5] = totalLength;

	int cnt = tcp_server_write(client, (char*)header, 6);
	if (cnt < 0)
	{
		client = -1;
		return -1;
	}

	if (len != 0)
	{
		cnt = tcp_server_write(client, data, len);
		if (cnt < 0)
		{
			client = -1;
			return -1;
		}

		if (len2 != 0)
		{
			cnt = tcp_server_write(client, data2, len2);
			if (cnt < 0)
			{
				client = -1;
				return -1;
			}
		}
	}

	return 0;
}

int write_tl(int& client, uint16_t type, uint32_t len)
{
	uint8_t header[6];

	header[0] = type >> 8;
	header[1] = type;
	header[2] = len >> 24;
	header[3] = len >> 16;
	header[4] = len >> 8;
	header[5] = len;

	int cnt = tcp_server_write(client, (char*)header, 6);
	if (cnt < 0)
	{
		client = -1;
		return -1;
	}

	return 0;
}

int write_value(int& client, char* data, uint32_t len)
{
	if (len != 0)
	{
		int cnt = tcp_server_write(client, data, len);
		if (cnt < 0)
		{
			client = -1;
			return -1;
		}
	}

	return 0;
}