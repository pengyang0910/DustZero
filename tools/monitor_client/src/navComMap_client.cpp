#include "navComMap_client.h"


#include <thread>
#include <vector>
#include <map>
#include <string>

class nav_map_uncompressed
{
public:
	nav_map_uncompressed(int8_t _map_type, int _unit_mm, int _width, int _height)
	{
		map_type = _map_type;
		unit_mm = _unit_mm;
		width = _width;
		height = _height;
		data = new uint8_t[width*height]();
		if (map_type == 1)
			memset(data, 3, width*height);
		else
			memset(data, 0, width*height);

		seq = new uint16_t[(width / NAV_COM_MAP_BLOCK_SIZE) * (height / NAV_COM_MAP_BLOCK_SIZE)]();
		memset(seq, 0, (width / NAV_COM_MAP_BLOCK_SIZE) * (height / NAV_COM_MAP_BLOCK_SIZE));

		block_NUM_X = width / NAV_COM_MAP_BLOCK_SIZE;
		block_NUM_Y = height / NAV_COM_MAP_BLOCK_SIZE;


		server_block_u_start = server_block_u_end = block_NUM_X / 2;
		server_block_v_start = server_block_v_end = block_NUM_Y / 2;
	}

	int check_seq(navCompressedMapMsg::map_ack_t ack_map, std::vector<navCompressedMapMsg::block_request_t>& orReq_blocks)
	{
		uint32_t current_block_u_start = ack_map.roi_block_x_start + block_NUM_X / 2;
		uint32_t current_block_u_end = ack_map.roi_block_x_end + block_NUM_X / 2;
		uint32_t current_block_v_start = ack_map.roi_block_y_start + block_NUM_Y / 2;
		uint32_t current_block_v_end = ack_map.roi_block_y_end + block_NUM_Y / 2;

		if (current_block_u_start > current_block_u_end || current_block_u_end > block_NUM_X ||
			current_block_v_start > current_block_v_end || current_block_v_end > block_NUM_Y)
		{
			return -1;
		}
		else if ((current_block_u_start > server_block_u_start) || (current_block_v_start > server_block_v_start) ||
			(current_block_u_end < server_block_u_end) || (current_block_v_end < server_block_v_end))
		{
			// reset map
			if (map_type == NAV_COM_MAP_TYPE_SLAM)
				memset(data, 3, width*height);
			else
				memset(data, 0, width*height);

			memset(seq, 0, (width / NAV_COM_MAP_BLOCK_SIZE) * (height / NAV_COM_MAP_BLOCK_SIZE));
		}
		server_block_u_start = current_block_u_start;
		server_block_v_start = current_block_v_start;
		server_block_u_end = current_block_u_end;
		server_block_v_end = current_block_v_end;


		if (ack_map.map_type != map_type)
			return -1;

		int tmp_idx = 0;
		for (int block_v = server_block_v_start; block_v < server_block_v_end; block_v++)
		{
			for (int block_u = server_block_u_start; block_u < server_block_u_end; block_u++, tmp_idx++)
			{
				if (ack_map.block_seqs[tmp_idx] != seq[block_v * block_NUM_X + block_u])
				{
					navCompressedMapMsg::block_request_t block;
					block.map_type = map_type;
					block.block_x = block_u - block_NUM_X / 2;
					block.block_y = block_v - block_NUM_Y / 2;
					orReq_blocks.push_back(block);
				}
			}
		}

		return orReq_blocks.size();
	}

	int updateBlockAndSeq(navCompressedMapMsg::map_ack_t ack_map)
	{
		if (ack_map.map_type != map_type)
			return -1;

		for (auto iter = ack_map.ack_blocks.begin(); iter != ack_map.ack_blocks.end(); iter++)
		{
			// 1. update seq
			uint32_t tmp_block_u = iter->block_x - ack_map.roi_block_x_start;
			uint32_t tmp_block_v = iter->block_y - ack_map.roi_block_y_start;

			if (tmp_block_u >= ack_map.roi_block_x_end - ack_map.roi_block_x_start ||
				tmp_block_v >= ack_map.roi_block_y_end - ack_map.roi_block_y_start)
			{
				continue;
			}
			uint16_t new_seq = ack_map.block_seqs[tmp_block_v * (ack_map.roi_block_x_end - ack_map.roi_block_x_start) + tmp_block_u];

			uint32_t block_u = iter->block_x + block_NUM_X / 2;
			uint32_t block_v = iter->block_y + block_NUM_Y / 2;

			if (block_u >= block_NUM_X || block_v >= block_NUM_Y)
			{
				continue;
			}
			seq[block_v * block_NUM_X + block_u] = new_seq;

			// 2. update data
			int dataIdx = 0;
			int bpp = ack_map.bpp;
			uint8_t bit_mask = uint8_t((1 << bpp) - 1);
			for (int v = block_v * NAV_COM_MAP_BLOCK_SIZE; v < (block_v + 1) * NAV_COM_MAP_BLOCK_SIZE; v++)
			{
				for (int u = block_u * NAV_COM_MAP_BLOCK_SIZE; u < (block_u + 1) * NAV_COM_MAP_BLOCK_SIZE; dataIdx++)
				{
					uint8_t tmpdata = iter->data[dataIdx];
					for (int i = 0; i < 8; i += bpp, u++)
					{
						data[v*width + u] = (tmpdata & (bit_mask << i)) >> i;
					}
				}
			}
		}

		return 0;
	}

	int8_t map_type;
	int unit_mm;
	int width;
	int height;
	uint8_t* data;
	uint16_t* seq;

	int block_NUM_X;
	int block_NUM_Y;

	int server_block_u_start;
	int server_block_u_end;
	int server_block_v_start;
	int server_block_v_end;
};

#define SEND_BUF_LEN      (10240)
#define RECEIVE_BUF_LEN   (102400)
static int fd_sock = -1;
static uint8_t sendbuf[SEND_BUF_LEN]; // 10K
static uint8_t receivebuf[RECEIVE_BUF_LEN]; // 100K
static uint8_t seq = 0;
static navCompressedMapMsg::gui_req_t gui_req;
static navCompressedMapMsg::nav_ack_t nav_ack;

static std::string ip_address;
int nav_com_map_client_init(const char* ip)
{
	seq = 0;
	gui_req.combined_map_types = 0; //all
	gui_req.seq = 0;
	gui_req.req_block_num = 0;
	gui_req.req_blocks.clear();

	nav_ack.combined_map_types = 0;
	nav_ack.seq = 0;
	nav_ack.ack_map_cnt = 0;
	nav_ack.ack_maps.clear();

	ip_address = std::string(ip);

	return 0;
}

int nav_com_map_client_update(std::vector<map_results_t>& orResult)
{
	orResult.clear();
	if (fd_sock < 0)
	{
		fd_sock = tcp_client_connect_to(ip_address.c_str(), NAV_COM_MAP_DEBUG_PORT);
		printf("map no data, fd: %d\r\n", fd_sock);
		sleep_ms(500);
	}

	else
	{
		int build_req_packet();
		build_req_packet();
		int lSendSize = gui_req.encode(sendbuf, 0, SEND_BUF_LEN);
		if (lSendSize <= 0 || lSendSize > SEND_BUF_LEN)
		{
			printf("send size < 0 or send size > send_buff_len\r\n");
			return 0;
		}

		write_tlv(fd_sock, 0xFEFE, lSendSize, (char*)sendbuf);
		sleep_ms(30);
		uint8_t header[6];
		int ret = read_tl(fd_sock, (char*)header, 500000);
		if (ret < 0)
		{
			tcp_client_close(fd_sock);
			fd_sock = -1;
			printf("1 Disconnected!!!!!!!!!!!!\n");
			sleep_ms(500);
		}
		else if (ret != 6)
		{
			tcp_client_close(fd_sock);
			fd_sock = -1;
			printf("2 Disconnected!!!!!!!!!!!! %d\n", ret);
			sleep_ms(500);
		}
		else
		{
			uint16_t type = (header[0] << 8) + header[1];
			if (type != 0xFDFD)
				return -1;

			uint32_t len = ((uint32_t)header[2] << 24) + ((uint32_t)header[3] << 16) + ((uint32_t)header[4] << 8) + header[5];
			int read_len = read_value(fd_sock, (char*)receivebuf, len, 500000);
			if (read_len == len)
			{
				if (len = nav_ack.decode(receivebuf, 0, RECEIVE_BUF_LEN))
				{
					int process_ack_packet(std::vector<map_results_t>& orResult);
					return process_ack_packet(orResult);
				}
			}
			else
			{
				tcp_client_close(fd_sock);
				fd_sock = -1;
				printf("3 Disconnected!!!!!!!!!!!! %d %d\n", read_len, len);
				sleep_ms(500);
			}
		}

	}
	return -1;
}

int nav_com_map_client_deinit()
{
	if (fd_sock)
	{
		tcp_client_close(fd_sock);
		fd_sock = -1;
	}

	return 0;
}




nav_map_uncompressed slamMap((int8_t)NAV_COM_MAP_TYPE_SLAM, NAV_COM_SLAM_MAP_UNIT_MM, NAV_COM_MAP_MAX_WIDTH_MM / NAV_COM_SLAM_MAP_UNIT_MM, NAV_COM_MAP_MAX_HEIGHT_MM / NAV_COM_SLAM_MAP_UNIT_MM);
nav_map_uncompressed obsMap((int8_t)NAV_COM_MAP_TYPE_OBS, NAV_COM_OBS_MAP_UNIT_MM, NAV_COM_MAP_MAX_WIDTH_MM / NAV_COM_OBS_MAP_UNIT_MM, NAV_COM_MAP_MAX_HEIGHT_MM / NAV_COM_OBS_MAP_UNIT_MM);
nav_map_uncompressed stuckMap((int8_t)NAV_COM_MAP_TYPE_STUCK, NAV_COM_STUCK_MAP_UNIT_MM, NAV_COM_MAP_MAX_WIDTH_MM / NAV_COM_STUCK_MAP_UNIT_MM, NAV_COM_MAP_MAX_HEIGHT_MM / NAV_COM_STUCK_MAP_UNIT_MM);
nav_map_uncompressed cleanMap((int8_t)NAV_COM_MAP_TYPE_CLEANED, NAV_COM_CLEANED_MAP_UNIT_MM, NAV_COM_MAP_MAX_WIDTH_MM / NAV_COM_CLEANED_MAP_UNIT_MM, NAV_COM_MAP_MAX_HEIGHT_MM / NAV_COM_CLEANED_MAP_UNIT_MM);
nav_map_uncompressed obscleanMap((int8_t)NAV_COM_MAP_TYPE_OBS_CLEANED, NAV_COM_OBS_CLEANED_MAP_UNIT_MM, NAV_COM_MAP_MAX_WIDTH_MM / NAV_COM_OBS_CLEANED_MAP_UNIT_MM, NAV_COM_MAP_MAX_HEIGHT_MM / NAV_COM_OBS_CLEANED_MAP_UNIT_MM);
nav_map_uncompressed dynamicMap((int8_t)NAV_COM_MAP_TYPE_DYNAMIC_MAP, NAV_COM_DYNAMIC_MAP_UNIT_MM, NAV_COM_MAP_MAX_WIDTH_MM / NAV_COM_DYNAMIC_MAP_UNIT_MM, NAV_COM_MAP_MAX_HEIGHT_MM / NAV_COM_DYNAMIC_MAP_UNIT_MM);
nav_map_uncompressed dstarMap((int8_t)NAV_COM_MAP_TYPE_DSTAR_MAP, NAV_COM_DSTAR_MAP_UNIT_MM, NAV_COM_MAP_MAX_WIDTH_MM / NAV_COM_DSTAR_MAP_UNIT_MM, NAV_COM_MAP_MAX_HEIGHT_MM / NAV_COM_DSTAR_MAP_UNIT_MM);

int build_req_packet()
{
	gui_req.combined_map_types = 0;
	gui_req.seq = seq++;
	gui_req.req_blocks.clear();
	// req_.clear();

	std::map<int8_t, navCompressedMapMsg::map_ack_t> map_ack_map;
	for (auto iter = nav_ack.ack_maps.begin(); iter != nav_ack.ack_maps.end(); iter++)
	{
		map_ack_map[iter->map_type] = *iter;
	}

	// req_.emplace(NAV_COM_MAP_TYPE_SLAM,        slamMap);
	// req_.emplace(NAV_COM_MAP_TYPE_OBS,         obsMap);
	// req_.emplace(NAV_COM_MAP_TYPE_STUCK,       stuckMap );
	// req_.emplace(NAV_COM_MAP_TYPE_CLEANED,     cleanMap);
	// req_.emplace(NAV_COM_MAP_TYPE_OBS_CLEANED, obscleanMap);

	// for(std::map<nav_map_uncompressed, nav_com_map_type_t>::iterator it = req_.begin(); 
	//     it != req_.end(); it++){
	//     gui_req.combined_map_types |= it->first;
	//     if (map_ack_map.find(it->first) != map_ack_map.end())
	//     {
	//         it->second.check_seq(map_ack_map[it->first], gui_req.req_blocks);
	//     }
	// }


	// 1. check slam map
	gui_req.combined_map_types |= NAV_COM_MAP_TYPE_SLAM;
	if (map_ack_map.find(NAV_COM_MAP_TYPE_SLAM) != map_ack_map.end())
	{
		slamMap.check_seq(map_ack_map[NAV_COM_MAP_TYPE_SLAM], gui_req.req_blocks);
	}

	gui_req.combined_map_types |= NAV_COM_MAP_TYPE_OBS;
	if (map_ack_map.find(NAV_COM_MAP_TYPE_OBS) != map_ack_map.end())
	{
		obsMap.check_seq(map_ack_map[NAV_COM_MAP_TYPE_OBS], gui_req.req_blocks);
	}

	gui_req.combined_map_types |= NAV_COM_MAP_TYPE_STUCK;
	if (map_ack_map.find(NAV_COM_MAP_TYPE_STUCK) != map_ack_map.end())
	{
		stuckMap.check_seq(map_ack_map[NAV_COM_MAP_TYPE_STUCK], gui_req.req_blocks);
	}

	gui_req.combined_map_types |= NAV_COM_MAP_TYPE_CLEANED;
	if (map_ack_map.find(NAV_COM_MAP_TYPE_CLEANED) != map_ack_map.end())
	{
		cleanMap.check_seq(map_ack_map[NAV_COM_MAP_TYPE_CLEANED], gui_req.req_blocks);
	}

	gui_req.combined_map_types |= NAV_COM_MAP_TYPE_OBS_CLEANED;
	if (map_ack_map.find(NAV_COM_MAP_TYPE_OBS_CLEANED) != map_ack_map.end())
	{
		obscleanMap.check_seq(map_ack_map[NAV_COM_MAP_TYPE_OBS_CLEANED], gui_req.req_blocks);
	}

	gui_req.combined_map_types |= NAV_COM_MAP_TYPE_DYNAMIC_MAP;
	if (map_ack_map.find(NAV_COM_MAP_TYPE_DYNAMIC_MAP) != map_ack_map.end())
	{
		dynamicMap.check_seq(map_ack_map[NAV_COM_MAP_TYPE_DYNAMIC_MAP], gui_req.req_blocks);
	}

	gui_req.combined_map_types |= NAV_COM_MAP_TYPE_DSTAR_MAP;
	if (map_ack_map.find(NAV_COM_MAP_TYPE_DSTAR_MAP) != map_ack_map.end())
	{
		dstarMap.check_seq(map_ack_map[NAV_COM_MAP_TYPE_DSTAR_MAP], gui_req.req_blocks);
	}


	if (gui_req.req_blocks.size() > NAV_COM_MAP_COMM_MAX_BLOCK_CNT)
		gui_req.req_blocks.resize(NAV_COM_MAP_COMM_MAX_BLOCK_CNT);

	gui_req.req_block_num = gui_req.req_blocks.size();
	return 0;
}

int process_ack_packet(std::vector<map_results_t>& orResult)
{
	orResult.clear();
	printf("[%d] %d %d %d %d\n", getTimeStap_ms(), nav_ack.seq, seq, (int)gui_req.req_blocks.size(), (int)nav_ack.ack_maps.size());
	std::map<int8_t, navCompressedMapMsg::map_ack_t> map_ack_map;
	for (auto iter = nav_ack.ack_maps.begin(); iter != nav_ack.ack_maps.end(); iter++)
	{
		map_ack_map[iter->map_type] = *iter;
	}


	// for(std::map<nav_map_uncompressed, nav_com_map_type_t>::iterator it = req_.begin(); 
	//     it != req_.end(); it++){
	//     if (map_ack_map.find(it->first) != map_ack_map.end()){
	//         map_results_t res;
	//         res.map_id = (int)it->first;
	//         res.width = it->second.width;
	//         res.height = it->second.height;
	//         res.data = it->second.data;
	//         orResult.push_back(res);
	//     }
	// }


	// 1. process slam map
	if (map_ack_map.find(NAV_COM_MAP_TYPE_SLAM) != map_ack_map.end())
	{
		slamMap.updateBlockAndSeq(map_ack_map[NAV_COM_MAP_TYPE_SLAM]);

		map_results_t res;
		res.map_id = NAV_COM_MAP_TYPE_SLAM;
		res.width = slamMap.width;
		res.height = slamMap.height;
		res.data = slamMap.data;
		orResult.push_back(res);
	}

	if (map_ack_map.find(NAV_COM_MAP_TYPE_OBS) != map_ack_map.end())
	{
		obsMap.updateBlockAndSeq(map_ack_map[NAV_COM_MAP_TYPE_OBS]);

		map_results_t res;
		res.map_id = NAV_COM_MAP_TYPE_OBS;
		res.width = obsMap.width;
		res.height = obsMap.height;
		res.data = obsMap.data;
		orResult.push_back(res);
	}

	if (map_ack_map.find(NAV_COM_MAP_TYPE_STUCK) != map_ack_map.end())
	{
		stuckMap.updateBlockAndSeq(map_ack_map[NAV_COM_MAP_TYPE_STUCK]);

		map_results_t res;
		res.map_id = NAV_COM_MAP_TYPE_STUCK;
		res.width = stuckMap.width;
		res.height = stuckMap.height;
		res.data = stuckMap.data;
		orResult.push_back(res);
	}

	if (map_ack_map.find(NAV_COM_MAP_TYPE_CLEANED) != map_ack_map.end())
	{
		cleanMap.updateBlockAndSeq(map_ack_map[NAV_COM_MAP_TYPE_CLEANED]);

		map_results_t res;
		res.map_id = NAV_COM_MAP_TYPE_CLEANED;
		res.width = cleanMap.width;
		res.height = cleanMap.height;
		res.data = cleanMap.data;
		orResult.push_back(res);
	}

	if (map_ack_map.find(NAV_COM_MAP_TYPE_OBS_CLEANED) != map_ack_map.end())
	{
		obscleanMap.updateBlockAndSeq(map_ack_map[NAV_COM_MAP_TYPE_OBS_CLEANED]);

		map_results_t res;
		res.map_id = NAV_COM_MAP_TYPE_OBS_CLEANED;
		res.width = obscleanMap.width;
		res.height = obscleanMap.height;
		res.data = obscleanMap.data;
		orResult.push_back(res);
	}

	if (map_ack_map.find(NAV_COM_MAP_TYPE_DYNAMIC_MAP) != map_ack_map.end())
	{
		dynamicMap.updateBlockAndSeq(map_ack_map[NAV_COM_MAP_TYPE_DYNAMIC_MAP]);

		map_results_t res;
		res.map_id = NAV_COM_MAP_TYPE_DYNAMIC_MAP;
		res.width = dynamicMap.width;
		res.height = dynamicMap.height;
		res.data = dynamicMap.data;
		orResult.push_back(res);
	}

	if (map_ack_map.find(NAV_COM_MAP_TYPE_DSTAR_MAP) != map_ack_map.end())
	{
		dstarMap.updateBlockAndSeq(map_ack_map[NAV_COM_MAP_TYPE_DSTAR_MAP]);

		map_results_t res;
		res.map_id = NAV_COM_MAP_TYPE_DSTAR_MAP;
		res.width = dstarMap.width;
		res.height = dstarMap.height;
		res.data = dstarMap.data;
		orResult.push_back(res);
	}


	return 0;
}