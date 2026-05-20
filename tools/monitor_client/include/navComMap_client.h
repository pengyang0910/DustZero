/*
 * @Description:
 * @version:
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2022-01-09 19:02:27
 * @LastEditors: jephy jephy.zhang@any-eye.com
 * @LastEditTime: 2022-12-13 18:22:30
 * @Version: 2.0
 * @Autor: Jephy Zhang
 */
#ifndef __NAV_COM_MAP_CLIENT_H__
#define __NAV_COM_MAP_CLIENT_H__

#include "Socket/tlv.h"
#include "xutils.h"
#include "MapServer/navCompressedMapConfig.h"
#include "MapServer/Internal/navCompressedMapMsg/gui_req_t.hpp"
#include "MapServer/Internal/navCompressedMapMsg/nav_ack_t.hpp"
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