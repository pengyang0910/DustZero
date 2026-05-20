/*
 * tm_rpc.cpp
 * TmRpc_t minimal TCP implementation.
 */
#include "navtask/tm_rpc.h"
#include <iostream>
#include <chrono>
#include <cerrno>
#include <cstring>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

TmRpc_t::TmRpc_t()
    : bridgePt(nullptr)
{
}

TmRpc_t::TmRpc_t(NavBridge_t* bridge)
    : bridgePt(bridge)
{
}

TmRpc_t::~TmRpc_t()
{
    Stop();
}

void TmRpc_t::Start()
{
    if (isRunning.load())
        return;
    isRunning.store(true);
    rpcThread = std::thread([this]() { this->main(); });
    std::cout << "[TmRpc] RPC server started (tcp text mode, port 9001)" << std::endl;
}

void TmRpc_t::Stop()
{
    if (!isRunning.load())
        return;
    isRunning.store(false);
    if (rpcThread.joinable())
        rpcThread.join();
    std::cout << "[TmRpc] RPC server stopped." << std::endl;
}

void TmRpc_t::main()
{
    int serverFd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (serverFd < 0) {
        std::cerr << "[TmRpc] socket() failed: " << std::strerror(errno) << std::endl;
        return;
    }

    int opt = 1;
    ::setsockopt(serverFd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    int rpcPort = 9001;
    const char* envPort = std::getenv("FJ212_RPC_PORT");
    if (envPort) rpcPort = std::atoi(envPort);
    addr.sin_port = htons(rpcPort);

    if (::bind(serverFd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "[TmRpc] bind(9001) failed: " << std::strerror(errno) << std::endl;
        ::close(serverFd);
        return;
    }

    if (::listen(serverFd, 8) < 0) {
        std::cerr << "[TmRpc] listen() failed: " << std::strerror(errno) << std::endl;
        ::close(serverFd);
        return;
    }

    while (isRunning.load()) {
        fd_set readSet;
        FD_ZERO(&readSet);
        FD_SET(serverFd, &readSet);
        timeval timeout {0, 100000};

        int ready = ::select(serverFd + 1, &readSet, nullptr, nullptr, &timeout);
        if (ready <= 0)
            continue;

        int clientFd = ::accept(serverFd, nullptr, nullptr);
        if (clientFd < 0)
            continue;

        char buf[1024] {};
        ssize_t n = ::recv(clientFd, buf, sizeof(buf) - 1, 0);
        std::string response = "ERR empty\n";
        if (n > 0) {
            response = handleCommand(std::string(buf, static_cast<size_t>(n)));
            if (response.empty() || response.back() != '\n')
                response.push_back('\n');
        }
        ::send(clientFd, response.c_str(), response.size(), 0);
        ::close(clientFd);
    }

    ::close(serverFd);
}

RobotData_t TmRpc_t::GetRobot()
{
    std::lock_guard<std::mutex> lk(mtx);
    return robotData;
}

StaCmd_t TmRpc_t::GetStaCmd()
{
    std::lock_guard<std::mutex> lk(mtx);
    StaCmd_t ret = staCmd;
    staCmd = StaCmd_t::None;
    return ret;
}

RobotStateMachine_t TmRpc_t::GetRobotStateMachine()
{
    std::lock_guard<std::mutex> lk(mtx);
    return machine;
}

void TmRpc_t::UpdateRobot(const RobotData_t& data, RobotStateMachine_t machine_)
{
    std::lock_guard<std::mutex> lk(mtx);
    robotData = data;
    machine = machine_;
}

void TmRpc_t::setStaCmd(StaCmd_t cmd_)
{
    std::lock_guard<std::mutex> lk(mtx);
    staCmd = cmd_;
}

void TmRpc_t::setStaStateCmd(bool lcd, bool lock)
{
    std::lock_guard<std::mutex> lk(mtx);
    lcdState  = lcd;
    lockState = lock;
}

bool TmRpc_t::getGoOutStationStatus()
{
    std::lock_guard<std::mutex> lk(mtx);
    return RpcGoOutStation;
}

void TmRpc_t::setGoOutStaitonStatus(bool en)
{
    std::lock_guard<std::mutex> lk(mtx);
    RpcGoOutStation = en;
}

std::string TmRpc_t::handleCommand(const std::string& line)
{
    std::istringstream iss(line);
    std::string cmd;
    iss >> cmd;

    if (cmd == "status" || cmd == "get_robot") {
        std::lock_guard<std::mutex> lk(mtx);
        std::ostringstream os;
        os << "OK battery=" << robotData.batteryPercent
           << " charging=" << (robotData.isCharging ? 1 : 0)
           << " x=" << robotData.x
           << " y=" << robotData.y
           << " angle=" << robotData.angle
           << " error=" << robotData.errorCode
           << " state=" << static_cast<int>(machine)
           << " task=" << robotData.curTask;
        return os.str();
    }

    if (!bridgePt)
        return "ERR bridge_not_ready";

    if (cmd == "clean" || cmd == "start_clean") {
        bridgePt->rpcPause = false;
        bridgePt->rpcCleanUp = true;
        bridgePt->robotWorkRuning = true;
        bridgePt->rState = RobotState_t::Cleaning;
        return "OK clean";
    }

    if (cmd == "pause") {
        bridgePt->rpcPause = true;
        bridgePt->rState = RobotState_t::Paused;
        return "OK pause";
    }

    if (cmd == "resume") {
        bridgePt->rpcPause = false;
        bridgePt->rpcCleanUp = true;
        bridgePt->rState = RobotState_t::Cleaning;
        return "OK resume";
    }

    if (cmd == "stop") {
        bridgePt->rpcPause = false;
        bridgePt->rpcCleanUp = false;
        bridgePt->onlyBackToDock = false;
        bridgePt->remoteCtrlModeRunning = false;
        bridgePt->remoteVelocity = {};
        bridgePt->FinishThisJob = true;
        bridgePt->rState = RobotState_t::StandBy;
        return "OK stop";
    }

    if (cmd == "charge" || cmd == "back_to_dock") {
        bridgePt->rpcPause = false;
        bridgePt->onlyBackToDock = true;
        bridgePt->robotActiveTrigger = RobOuterTask_t::BackToDock;
        bridgePt->rState = RobotState_t::GoCharge;
        return "OK charge";
    }

    if (cmd == "remote") {
        RemoteVelocity_t vel;
        iss >> vel.linearX >> vel.linearY >> vel.angularZ;
        vel.active = true;
        bridgePt->PublishRemoteVelocity(vel);
        bridgePt->remoteCtrlModeRunning = true;
        bridgePt->rState = RobotState_t::Cleaning;
        return "OK remote";
    }

    if (cmd == "station") {
        int value = 0;
        iss >> value;
        setStaCmd(static_cast<StaCmd_t>(value));
        return "OK station";
    }

    return "ERR unknown_command";
}
