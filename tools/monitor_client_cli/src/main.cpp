/*
 * monitor_client_cli — ROS-free CLI tool to verify monitor_server TCP link
 *
 * Connects to monitor_server:8989, requests data types in a loop,
 * prints received TLV info and optionally saves map frames to files.
 *
 * Usage: ./monitor_client_cli [host] [port]
 *   Default: host=127.0.0.1, port=8989
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <chrono>
#include <vector>
#include <fstream>

#include "Socket/tcp_server.h"
#include "Socket/tlv.h"
#include "global.h"

static const char* requestName(int req)
{
    switch (req) {
        case 99:  return "MAP";
        case 2:   return "LASER";
        case 3:   return "AMCL_ODOM";
        case 4:   return "AMCL_PARTICLES";
        case 5:   return "NEXTLINE";
        case 7:   return "ODOM";
        case 8:   return "GM_ODOM";
        case 9:   return "DSTAR_MAP";
        case 10:  return "GLOBAL_COSTMAP";
        case 11:  return "LOCAL_COSTMAP";
        case 12:  return "GLOBAL_PATH";
        case 13:  return "LOCAL_PATH";
        case 14:  return "PRED_PATH";
        case 15:  return "DISTRICT_SHAPE";
        case 16:  return "LASER_X1Q";
        case 17:  return "FAST_SLAM";
        case 18:  return "MAPEXP_3D_OBS";
        case 24:  return "LASER_X1C";
        case 25:  return "ODOM_NEXTLINE";
        default:  return "UNKNOWN";
    }
}

static bool readTlvFully(int fd, uint16_t& outType, std::vector<uint8_t>& outData)
{
    char header[6];
    int n = read_tl(fd, header, 2000000); // 2s timeout
    if (n != 6) return false;

    outType = (uint8_t)header[0] << 8 | (uint8_t)header[1];
    uint32_t len = ((uint8_t)header[2] << 24) | ((uint8_t)header[3] << 16)
                 | ((uint8_t)header[4] << 8)  | (uint8_t)header[5];

    if (len > 10 * 1024 * 1024) { // sanity cap 10MB
        printf("[cli] FATAL: declared len=%u too large\n", len);
        return false;
    }

    outData.resize(len);
    if (len > 0) {
        int got = read_value(fd, (char*)outData.data(), len, 3000000); // 3s timeout
        if (got != (int)len) {
            printf("[cli] WARN: short read %d/%u\n", got, len);
            return false;
        }
    }
    return true;
}

static void saveMapPgm(const std::string& prefix, int idx,
                       const uint8_t* data, size_t len)
{
    char fname[256];
    snprintf(fname, sizeof(fname), "%s_%04d.bin", prefix.c_str(), idx);
    std::ofstream ofs(fname, std::ios::binary);
    if (ofs) {
        ofs.write((const char*)data, len);
        printf("[cli] saved %s (%zu bytes)\n", fname, len);
    }
}

int main(int argc, char** argv)
{
    const char* host = (argc > 1) ? argv[1] : "127.0.0.1";
    int port = (argc > 2) ? std::atoi(argv[2]) : MONITOR_PORT;

    setbuf(stdout, NULL);
    printf("[monitor_client_cli] connecting to %s:%d ...\n", host, port);

    int fd = tcp_client_connect_to(host, port);
    if (fd < 0) {
        printf("[monitor_client_cli] connect failed\n");
        return 1;
    }
    printf("[monitor_client_cli] connected (fd=%d)\n", fd);

    // request types we care about
    const std::vector<int> requests = {
        99, 2, 7, 3, 12, 13, 5, 10, 11
    };

    int mapFrameIdx = 0;
    int loopCount = 0;

    while (true) {
        for (int req : requests) {
            // send 1-byte request
            char cmd = (char)req;
            int w = tcp_server_write(fd, &cmd, 1);
            if (w < 0) {
                printf("[cli] write error, disconnect\n");
                tcp_client_close(fd);
                return 1;
            }

            // receive TLV response
            uint16_t respType;
            std::vector<uint8_t> respData;
            bool ok = readTlvFully(fd, respType, respData);
            if (!ok) {
                printf("[cli] read timeout/error for %s (req=%d)\n", requestName(req), req);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            if (respData.size() == 1 && respData[0] == 0) {
                // "old data" indicator from server
                // printf("[cli] %s: no new data\n", requestName(req));
            } else {
                printf("[cli] %s: type=%u size=%zu\n",
                       requestName(req), respType, respData.size());

                if (req == 99 && respData.size() > 100) {
                    saveMapPgm("/tmp/map_cli", mapFrameIdx++,
                               respData.data(), respData.size());
                }
            }
        }

        loopCount++;
        if (loopCount % 10 == 0) {
            printf("[cli] --- loop %d completed ---\n", loopCount);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    tcp_client_close(fd);
    return 0;
}
