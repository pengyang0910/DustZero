/*
 * @Description: 
 * @version: 
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2020-08-24 10:00:20
 * @LastEditors: Jephy Zhang
 * @LastEditTime: 2021-10-19 20:36:30
 */
#ifndef     __TELEOP_H__
#define     __TELEOP_H__


#include <termios.h>
#include <assert.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h> /* for strcpy() */
#include <stdlib.h>  /* for atoi(3),rand(3) */
#include <time.h>
#include <math.h>
#include <thread>
#include <lcm/lcm-cpp.hpp>
#include "Msg/NavMsg/VelCmd_t.hpp"
#include "Msg/SensorMsg/Bumper_t.hpp"
#include "Msg/NavMsg/GridMap_t.hpp"
#include "XLog/xlog.h"
#include "linux/input.h"
#include "Msg/RobotMsg/HackCmd_t.hpp"


#define KEYCODE_I 0x69
#define KEYCODE_J 0x6a
#define KEYCODE_K 0x6b
#define KEYCODE_L 0x6c
#define KEYCODE_Q 0x71
#define KEYCODE_Z 0x7a
#define KEYCODE_W 0x77
#define KEYCODE_X 0x78
#define KEYCODE_E 0x65
#define KEYCODE_C 0x63
#define KEYCODE_U 0x75
#define KEYCODE_O 0x6F
#define KEYCODE_M 0x6d
#define KEYCODE_R 0x72
#define KEYCODE_V 0x76
#define KEYCODE_T 0x74
#define KEYCODE_B 0x62

#define KEYCODE_WAVE 126

#define KEYCODE_COMMA 0x2c
#define KEYCODE_PERIOD 0x2e

#define COMMAND_TIMEOUT_SEC 0.2

class TeleOp_t
{
private:
    /* data */
    lcm::LCM lcm;

    /* motion control data */
    double vSpd;
    double wSpd;
    int m_speed;
    int m_turn;
    double max_speed;
    double max_turn;
    

    /* Main task  */
    std::thread mThread;
    std::thread msgThread;
    RobotMsg::HackCmd_t hackCmd;
    bool isRunning;
    bool keyUpdate;
    void Main();
    void readKey();
    NavMsg::VelCmd_t velCmd;
    int kfd;
    int keys_fd;
    struct input_event t;

    void spinProcess();
    void bumperUpdate(const lcm::ReceiveBuffer* rbuf, 
                    const std::string &channel, 
                    const SensorMsg::Bumper_t *msg);
    void gmMapUpdate(const lcm::ReceiveBuffer* rbuf, 
                    const std::string &channel, 
                    const NavMsg::GridMap_t *msg);
public:
    TeleOp_t(/* args */);
    ~TeleOp_t();

    void Start();
    void Stop();

};




#endif