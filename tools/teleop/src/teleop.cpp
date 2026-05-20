/*
 * @Description: 
 * @version: 
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2020-08-24 10:00:16
 * @LastEditors: Jephy Zhang
 * @LastEditTime: 2021-10-20 10:20:27
 */
#include "teleop.h"
#include "xutils.h"
#include "global.h"

#ifdef FJ212_TELEOP_USE_TCP
#include "Socket/tcp_server.h"
#include <sys/socket.h>
#include <cstring>

static void sendNavcleanRemote(float vx, float vy, float wz)
{
    int fd = tcp_client_connect_to("127.0.0.1", 9001);
    if (fd < 0) {
        printf("[teleop] connect navclean:9001 failed\n");
        return;
    }
    char buf[128];
    snprintf(buf, sizeof(buf), "remote %.3f %.3f %.3f\n", vx, vy, wz);
    (void)::send(fd, buf, strlen(buf), MSG_NOSIGNAL);
    // small delay to let navclean process before closing
    usleep(1000);
    tcp_client_close(fd);
}
#endif

TeleOp_t::TeleOp_t(/* args */)
{
    isRunning = false;
    hackCmd.data = 0;
}

TeleOp_t::~TeleOp_t()
{
}

void TeleOp_t::Start()
{
    m_speed = 0;
    m_turn = 0;
    vSpd = 0;
    wSpd = 0;
    max_speed = 0.25;
    max_turn = M_PI/2;

    isRunning = true;
    keyUpdate = false;
    //lcm.subscribe(LCM_CHANNEL_Bumper, &TeleOp_t::bumperUpdate, this);
    //lcm.subscribe(LCM_CHANNEL_MAP, &TeleOp_t::gmMapUpdate, this);
    mThread = createBindThread(ProTeleOp+"Main", std::bind(&TeleOp_t::Main, this), BIND_CPU_ID_Teleop);
    msgThread = createBindThread(ProTeleOp+"Msg", std::bind(&TeleOp_t::spinProcess, this), BIND_CPU_ID_Teleop);
    //mThread = std::thread(&TeleOp_t::Main, this);
    //msgThread = std::thread(&TeleOp_t::spinProcess, this);

   

}

void TeleOp_t::gmMapUpdate(const lcm::ReceiveBuffer* rbuf, 
                    const std::string &channel, 
                    const NavMsg::GridMap_t *msg)
{
    printf("teleop gmamapupdate msg.width:%d, msg.data.size:%d\n", msg->width, msg->data.size());
}
            

void TeleOp_t::spinProcess()
{
    while(isRunning && (0==lcm.handle()))
    {
       usleep(50000);
    }
}


void TeleOp_t::bumperUpdate(const lcm::ReceiveBuffer* rbuf, 
                    const std::string &channel, 
                    const SensorMsg::Bumper_t *msg)
{
    printf("bumper -----\n");
}

void TeleOp_t::Stop()
{
    isRunning = false;
    if(mThread.joinable())
        mThread.join();
    printf("TeleOp Stopped!");
}

void TeleOp_t::Main()
{
    //bindCpuCore(BIND_CPU_ID_Teleop);
    printf("TeleOp Main loop is started!\r\n");
    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Moving around:");
    puts("   u    i    o");
    puts("   j    k    l");
    puts("   m    ,    .");
    puts("");
    puts("q/z : increase/decrease max speeds by 10%");
    puts("w/x : increase/decrease only linear speed by 10%");
    puts("e/c : increase/decrease only angular speed by 10%");

    puts("");
    puts("anything else : stop");
    puts("---------------------------");

    kfd = 0;

    struct termios cooked, raw;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    uint64_t lastSendTs_us = 0;
    uint64_t curTs_us = 0;

    while (isRunning)
    {
        readKey();
        printf("after readkey\r\n");
        if(keyUpdate)
        {
            curTs_us = getTimeStap_us();
            if(curTs_us - lastSendTs_us > 500000)
            {
                velCmd.vSpd = 0;
                velCmd.wSpd = 0;
                velCmd.timestamp_us = getTimeStap_us();
                velCmd.name = "/teleOp/velCmd";
                velCmd.resetOdom = false;
                velCmd.spdLoopEnable = true;
#ifdef FJ212_TELEOP_USE_TCP
                sendNavcleanRemote(0.0f, 0.0f, 0.0f);
#else
                lcm.publish(LCM_CHANNEL_VelCmd, &velCmd);
#endif
            }
            else 
            {
                velCmd.vSpd = m_speed * max_speed;
                velCmd.wSpd = m_turn * max_turn;
                velCmd.timestamp_us = getTimeStap_us();
                velCmd.name = "/teleOp/velCmd";
                velCmd.resetOdom = false;
                velCmd.spdLoopEnable = true;
                keyUpdate = false;
                //printf("TeleOp: publish v=%.2f w=%.2f\r\n", velCmd.vSpd, velCmd.wSpd);
#ifdef FJ212_TELEOP_USE_TCP
                sendNavcleanRemote(velCmd.vSpd, 0.0f, velCmd.wSpd);
#else
                lcm.publish(LCM_CHANNEL_VelCmd, &velCmd);
#endif
            }
            lastSendTs_us = curTs_us;
            if(hackCmd.data != 0)
            {
                printf("Send Esc Command\r\n");
                lcm.publish(LCM_CHANNEL_HACK, &hackCmd);
                hackCmd.data = 0;
            }
            //lcm.publish(LCM_CHANNEL_XBASE_VelCmd, &velCmd);
        }

        
    }
    
}

void TeleOp_t::readKey()
{
    static bool init = false;
    char c = 0;
    double &max_tv = max_speed;
    double &max_rv = max_turn;
    int speed = 0;
    int turn = 0;
    static int tick = 0;
    //for(isRunning)
    {
        // get the next event from the keyboard
        if(read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }
        printf("tick=%d keyCode = %d\r\n",tick++, (int)c);
        switch(c)
        {
        case KEYCODE_I:
            speed = 1;
            turn = 0;
            keyUpdate = true;
            break;
        case KEYCODE_K:
            speed = 0;
            turn = 0;
            keyUpdate = true;
            break;
        case KEYCODE_O:
            speed = 1;
            turn = -1;
            keyUpdate = true;
            break;
        case KEYCODE_J:
            speed = 0;
            turn = 1;
            keyUpdate = true;
            break;
        case KEYCODE_L:
            speed = 0;
            turn = -1;
            keyUpdate = true;
            break;
        case KEYCODE_U:
            turn = 1;
            speed = 1;
            keyUpdate = true;
            break;
        case KEYCODE_COMMA:
            turn = 0;
            speed = -1;
            keyUpdate = true;
            break;
        case KEYCODE_PERIOD:
            turn = 1;
            speed = -1;
            keyUpdate = true;
            break;
        case KEYCODE_M:
            turn = -1;
            speed = -1;
            keyUpdate = true;
            break;
        case KEYCODE_Q:
            max_tv += max_tv / 10.0;
            max_rv += max_rv / 10.0;
            init = false;
            //keyUpdate = true;
            break;
        case KEYCODE_Z:
            max_tv -= max_tv / 10.0;
            max_rv -= max_rv / 10.0;
            init = false;
            //keyUpdate = true;
            break;
        case KEYCODE_W:
            max_tv += max_tv / 10.0;
            //keyUpdate = true;
            break;
        case KEYCODE_X:
            max_tv -= max_tv / 10.0;
            //keyUpdate = true;
            break;
        case KEYCODE_E:
            max_rv += max_rv / 10.0;
            //keyUpdate = true;
            break;
        case KEYCODE_C:
            max_rv -= max_rv / 10.0;
            //keyUpdate = true;
            break;
        case 98:
            printf("TAB key %u\r\n", int(c));
            hackCmd.data = KEYCODE_WAVE;
            keyUpdate = true;
            break;
        default:
            speed = 0;
            turn = 0;
            //usleep(10000);
            break;
        }
        if(keyUpdate)
        {
            if(!init)
            {
                m_speed = 0;
                m_turn = 0;
                init = true;
                return;
            }
            else
            {
                /* code */
                m_speed = speed;
                m_turn = turn;
                //printf(" iputKeyTs=%.4f \r\n", getTs());
            }
        }
        else
        {
            init = false;
        }
        

        
        
    }

}