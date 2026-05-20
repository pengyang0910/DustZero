/*
 * @Description: 
 * @version: 
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2020-08-24 13:07:32
 * @LastEditors: Jephy Zhang
 * @LastEditTime: 2021-11-27 18:08:20
 */
#include "teleop.h"
#include "unistd.h"
#include "global.h"
#include "Msg/NavMsg/GridMap_t.hpp"
#include "Msg/NavMsg/PoseWithCovariance.hpp"
#include "Msg/RobotMsg/HackCmd_t.hpp"
#include "Msg/NavMsg/SlamCmd_t.hpp"
#include <iostream>
#include "Msg/TuyaMsg/TaskFinishStatus_t.hpp"

class Handler 
{
    public:
        ~Handler() {}
        void testMapMsg(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const NavMsg::GridMap_t* msg)
        {
            int i;
            printf("ts=%.2f name=%s size=%d\r\n", msg->timestamp_us/1000000.0, msg->name.c_str(), msg->data.size());
            for (int i = 0; i < 256; i++)
            {
                /* code */
                printf("%d ", msg->data[i]);
            }
            printf("\r\n");
            
        }

        void testAmclMsg(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const NavMsg::PoseWithCovariance* msg)
        {
            int i;
            printf("ts=%.2f name=%s (%.2f, %.2f, %.2f)\r\n", msg->timestamp_us/1000000.0, msg->name.c_str(), msg->px, msg->py, msg->pa);
            
        }


};

int hackCmd()
{
    lcm::LCM lcm;
    RobotMsg::HackCmd_t hackCmd;
    NavMsg::SlamCmd_t slamCmd;
    
    while (true)
    {
        /* code */
        printf("-1: reset to idle task\r\n");
        printf("1: Stop CurTask\r\n");
        printf("2: Stop CurTask, Start ObsClen\r\n");
        printf("3: fake new obs bumped, Start ObsClen\r\n");
        printf("4: dwaPlanner test\r\n");
        printf("5: switch amcl to reLocalization mode\r\n");
        printf("6: switch amcl to normal mode\r\n");
        printf("10: RoomClean start\r\n");
        printf("11: TestTask start\r\n");
        printf("12: RoomCleanV2 start\r\n");
        printf("13: ObsCleanTask start\r\n");
        printf("14: Reset Slam\r\n");
        printf("15: Stop Slam but not reset\n");
        printf("16: Stop clean\n");
        printf("17: Start clean\n");
        printf("18: Save Map\n");
        printf("19: Go to dock\n");
        printf("100-118: exception handler task\n");
        printf(">>");
        hackCmd.data = 0;
        while(!(std::cin>>hackCmd.data))
        {
            std::cin.clear();
            std::cin.sync();
        }
        if (hackCmd.data == 14)
        {
            slamCmd.startSlam = true;
            slamCmd.resetSlam = true;
            printf("Send Reset Slam Cmd\r\n");
            lcm.publish(LCM_CHANNEL_SlamCmd, &slamCmd);
        }else if(hackCmd.data == 18){
            TuyaMsg::TaskFinishStatus_t msg;
            msg.key = (int)TuyaMsg::TaskFinishStatusKey_t::SaveMap;
            //msg.updateVirInfo = false;
            printf("Send save map Cmd\r\n");
            lcm.publish(LCM_CHANNEL_SaveMap, &msg);
        }else
        {
            lcm.publish(LCM_CHANNEL_HACK, &hackCmd);
            printf("    publish data: %d\r\n", hackCmd.data);
        }
        

    }
    

}

void teleOp()
{
    lcm::LCM lcm;
    TeleOp_t teleop;
    Handler hand;

    //lcm.subscribe(LCM_CHANNEL_MAP, &Handler::testMapMsg, &hand);
    //lcm.subscribe(LCM_CHANNEL_AmclPose, &Handler::testAmclMsg, &hand);
    teleop.Start();
    while (0==lcm.handle())
    {
        //sleep(1);
        usleep(50000);
    }
    teleop.Stop();
}

int main(int argc, char** argv)
{
    int cmd;
    std::cout<<"1: teleOp"<<std::endl;
    std::cout<<"2: hackCmd"<<std::endl;
    std::cin>>cmd;
    switch (cmd)
    {
    case 1:
        /* code */
        teleOp();
        break;
    case 2:
        hackCmd();
        break;
    default:
        break;
    }
}

