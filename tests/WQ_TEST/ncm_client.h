#ifndef __NCM_CLIENT_H__
#define __NCM_CLIENT_H__

#include "Socket/tlv.h"
#include "xutils.h"
#include "NavService/MapServer/navCompressedMapConfig.h"
#include "NavService/MapServer/Internal/navCompressedMapMsg/gui_req_t.hpp"
#include "NavService/MapServer/Internal/navCompressedMapMsg/nav_ack_t.hpp"
#include <vector>

typedef struct
{
	int map_id;
	int width;
	int height;
	uint8_t* data;
} map_results_t;


int nav_com_map_client_init(const char* ip);
int nav_com_map_client_update(std::vector<map_results_t>& orResult);
int nav_com_map_client_deinit();
#endif
