/*
 * @Author: jephy jephy.zhang@any-eye.com
 * @Date: 2023-07-28 13:32:57
 * @LastEditors: jephy jephy.zhang@any-eye.com
 * @LastEditTime: 2023-08-06 16:21:09
 * @FilePath: /xt212Alpha/Hmi/hmiServer.cpp
 * @Description: HMI server — rest_rpc frontend for AppServer, TCP text client to navclean:9001
 */
#include "xutils/Rpc/rest_rpc.hpp"
#include <fstream>
#include "hmi.h"
#include "xutils/Socket/tcp_server.h"

using namespace rest_rpc;
using namespace rpc_service;

MSGPACK_ADD_ENUM(RobotStateMachine_t);

// ── TCP text helper to navclean (port 9001) ──────────────────────────
static std::string sendNavcleanCmd(const std::string& cmd)
{
    int fd = tcp_client_connect_to("127.0.0.1", 9001);
    if (fd < 0) {
        return "ERR connect_failed";
    }
    std::string req = cmd;
    if (req.empty() || req.back() != '\n') {
        req += '\n';
    }
    (void)::send(fd, req.c_str(), req.size(), MSG_NOSIGNAL);

    char buf[1024] = {};
    ssize_t n = ::recv(fd, buf, sizeof(buf) - 1, 0);
    tcp_client_close(fd);

    if (n > 0) {
        return std::string(buf, static_cast<size_t>(n));
    }
    return "ERR no_response";
}

// Simple key=value parser for status response
static bool parseStatusField(const std::string& resp, const std::string& key, int& outVal)
{
    size_t pos = resp.find(key + "=");
    if (pos == std::string::npos) return false;
    outVal = std::atoi(resp.c_str() + pos + key.size() + 1);
    return true;
}

static void updateHmiFromStatus(Hmi_t& hmi, const std::string& resp)
{
    if (resp.find("OK") != 0) return;

    int charging = 0, error = 0;
    if (parseStatusField(resp, "charging", charging)) {
        hmi.SetRobotCharging(charging != 0);
    }
    if (parseStatusField(resp, "error", error)) {
        hmi.SetRobotStopped(error != 0);
    }
}

static std::string hmiStateChangeToCmd(const RobotData_t& prev, const RobotData_t& cur)
{
    if (prev.stateMachine == RobotStateMachine_t::StandBy &&
        cur.stateMachine == RobotStateMachine_t::Working) {
        if (cur.workMode == RobotWork_t::BackToStation) return "charge";
        return "clean";
    }
    if (prev.stateMachine == RobotStateMachine_t::Working &&
        cur.stateMachine == RobotStateMachine_t::Pause) {
        return "pause";
    }
    if (prev.stateMachine == RobotStateMachine_t::Pause &&
        cur.stateMachine == RobotStateMachine_t::Working) {
        return "resume";
    }
    if ((prev.stateMachine == RobotStateMachine_t::Working ||
         prev.stateMachine == RobotStateMachine_t::Pause) &&
        cur.stateMachine == RobotStateMachine_t::StandBy) {
        return "stop";
    }
    return "";
}

// ── Main ─────────────────────────────────────────────────────────────
int main()
{
    bool isRunning = false;
    Hmi_t hmi;
    hmi.Start();

    // HMI rest_rpc server for AppServer (port 9002, was 9001)
    rpc_server server(RpcPort_Hmi, 1);

    server.register_handler("SetAppCmd", &Hmi_t::appCmdSet, &hmi);
    server.register_handler("SetStaCmd", &Hmi_t::staCmdSet, &hmi);
    server.register_handler("SetKeyCmd", &Hmi_t::keyCmdSet, &hmi);
    server.register_handler("SetKeyCode", &Hmi_t::KeyCodeSet, &hmi);
    server.register_handler("RpcApi_APPCMD", &Hmi_t::getAppCmd, &hmi);
    server.register_handler("RpcApi_APPRAWCMD", &Hmi_t::getAppRawCmd, &hmi);

    server.register_handler("Quit", [&](rpc_conn conn)
                            { isRunning = false; });
    server.set_network_err_callback(
        [](std::shared_ptr<connection> conn, std::string reason)
        {
          std::cout << "remote client address: " << conn->remote_address()
                    << " networking error, reason: " << reason << "\n";
        });

    isRunning = true;
    server.async_run();

#ifndef WIN32
	prctl(PR_SET_NAME, "HmiStartUp");
#endif
	bindCpuCore(BIND_CPU_ID_MISC);

    RobotData_t prevRobot = hmi.GetRobot();

    while(isRunning)
    {
        // 1. Query navclean status via TCP text
        std::string statusResp = sendNavcleanCmd("status");
        updateHmiFromStatus(hmi, statusResp);

        // 2. Forward App commands from AppServer to navclean (basic mapping)
        std::vector<APP_CMD_DP_S> hmi_app_cmds;
        if (hmi.getAppCmds(hmi_app_cmds) > 0)
        {
            for (size_t i = 0; i < hmi_app_cmds.size(); i++)
            {
                int val = std::atoi(hmi_app_cmds[i].value.c_str());
                if (hmi_app_cmds[i].dpid == 1) {
                    sendNavcleanCmd(val ? "clean" : "stop");
                } else if (hmi_app_cmds[i].dpid == 2) {
                    sendNavcleanCmd(val ? "pause" : "resume");
                } else if (hmi_app_cmds[i].dpid == 3) {
                    sendNavcleanCmd(val ? "charge" : "stop");
                } else if (hmi_app_cmds[i].dpid == 128) {
                    sendNavcleanCmd(val ? "clean" : "stop");
                } else if (hmi_app_cmds[i].dpid == 132) {
                    sendNavcleanCmd(val ? "charge" : "stop");
                } else {
                    std::cout << "[HMI] Unhandled app cmd dpid=" << (int)hmi_app_cmds[i].dpid << std::endl;
                }
                hmi.resetAppCmd(hmi_app_cmds[i]);
            }
        }

        // 3. Raw app commands (logged but not forwarded — navclean text RPC does not support them yet)
        std::vector<APP_RAW_DP_S> hmi_app_rawcmds;
        if (hmi.getAppRawCmds(hmi_app_rawcmds) > 0)
        {
            for (size_t i = 0; i < hmi_app_rawcmds.size(); i++)
            {
                std::cout << "[HMI] Unhandled raw cmd dpid=" << (int)hmi_app_rawcmds[i].dpid << std::endl;
                hmi.resetAppRawCmd(hmi_app_rawcmds[i]);
            }
        }

        // 4. Nav raw commands (logged but not forwarded)
        std::vector<Nav_RAW_DP_S> nav_rawcmds;
        if (hmi.getNavCmds(nav_rawcmds) > 0)
        {
            for (size_t i = 0; i < nav_rawcmds.size(); i++)
            {
                std::cout << "[HMI] Unhandled nav raw cmd=" << (int)nav_rawcmds[i].cmd << std::endl;
                hmi.resetNavRawCmd(nav_rawcmds[i]);
            }
        }

        // 5. Detect HMI internal state changes (keys / station) and forward to navclean
        RobotData_t curRobot = hmi.GetRobot();
        std::string navCmd = hmiStateChangeToCmd(prevRobot, curRobot);
        if (!navCmd.empty()) {
            std::cout << "[HMI] State change -> navclean cmd=" << navCmd << std::endl;
            sendNavcleanCmd(navCmd);
        }
        prevRobot = curRobot;

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    hmi.Stop();
    printf("Prepare to Stop Server!\r\n");
    server.Stop();

    printf("Server Stopped!!!\r\n");
    std::this_thread::sleep_for(std::chrono::seconds(3));
    return 0;
}
