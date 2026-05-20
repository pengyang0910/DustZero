/*
 * @Author: jephy jephy.zhang@any-eye.com
 * @Date: 2023-07-26 15:24:40
 * @LastEditors: jephy jephy.zhang@any-eye.com
 * @LastEditTime: 2023-08-03 13:30:06
 * @FilePath: /xt212_navclean/Hmi/hmiClient.cpp
 * @Description: 
 */
#include <chrono>
#include <fstream>
#include <iostream>
#include <rest_rpc.hpp>
#include "global.h"
#include "hmi.h"

using namespace rest_rpc;
using namespace rest_rpc::rpc_service;

int main()
{
    rpc_client client("127.0.0.1", RpcPort_Hmi);

    bool r = client.connect();
    if (!r) {
      std::cout << "connect timeout" << std::endl;
      return -1;
    }
    client.enable_auto_reconnect(true);
    client.enable_auto_heartbeat(true);
    int cmd = 0;
 //   client.run();
    while (true)
    {
        /* code */
        printf(">>>>>>>>>>>>>>>> Cmd List <<<<<<<<<<<<<<<\r\n");
        printf(" 1. KeyCmd_WakeUp\r\n");
        printf(" 2. KeyCmd_StartClean\r\n");
        printf(" 3. KeyCmd_Pause\r\n");
        printf(" 4. KeyCmd_Continue\r\n");
        printf(" 5: KeyCmd_WiFiReset\r\n");
        printf(" 6: KeyCmd_BackToCharge\r\n");
        printf(" 7: KeyCmd_PowerOff\r\n");
        printf("10: KeyCode_PowerKeyShortPressed\r\n");
        printf("11: KeyCode_PowerKeyLongPressed\r\n");
        printf("12: KeyCode_HomeKeyShortPressed\r\n");
        printf("13: KeyCode_HomeKeyLongPressed\r\n");
        printf("14: KeyCode_HomeAndPowerLongPressed\r\n");

        printf("20. AppCmd_StartRoomClean\r\n");
        printf("21. AppCmd_StartBlockClean\r\n");
        printf("22. AppCmd_StartSectionClean\r\n");
        printf("23. AppCmd_StartSpotClean\r\n");
        printf("24. AppCmd_StartRemoteCtrlClean\r\n");
        printf("25. AppCmd_StartFastMapping\r\n");
        printf("26. AppCmd_StartBackToWashMop\r\n");
        printf("27. AppCmd_Pause\r\n");
        printf("28. AppCmd_Continue\r\n");
        printf("29. AppCmd_Stop\r\n");
        printf("30. AppCmd_BackToStation\r\n");
        printf("31. AppCmd_KidLockOn\r\n");
        printf("32. AppCmd_KidLockOff\r\n");
        printf("33. AppCmd_StaLcdOn\r\n");
        printf("34. AppCmd_StaLcdOff\r\n");
        printf("35. AppCmd_ModeClean\r\n");
        printf("36. AppCmd_ModeMop\r\n");
        printf("37. AppCmd_ModeCleanAndMop\r\n");
        printf("38. AppCmd_StartCustomClean\r\n");

        printf("51. StaCmd_StartRoomClean\r\n");
        printf("52. StaCmd_Pause\r\n");
        printf("53. StaCmd_Continue\r\n");
        printf("54. StaCmd_MoveOutStation\r\n");
        printf("55. StaCmd_BackToStation\r\n");
        printf("56. StaCmd_KidLockOn\r\n");
        printf("57. StaCmd_KidLockOff\r\n");
        printf("58. StaCmd_LcdOn\r\n");
        printf("59. StaCmd_LcdOff\r\n");
        printf("60. StaCmd_ModeClean\r\n");
        printf("61. StaCmd_ModeMop\r\n");
        printf("62. StaCmd_MopCleanAndMop\r\n");
        printf("63. StaCmd_CustomClean\r\n");

        printf("100: Quit Server\r\n");

        printf(">>");
        cmd = 0;
        while(!(std::cin>>cmd))
        {
            std::cin.clear();
            std::cin.sync();
        }

        switch (cmd)
        {
        /* 1~19  KeyCmd */
        case  1:
            client.call<RobKeyCmd_t>("SetKeyCmd", RobKeyCmd_t::WakeUp);
            break;
        case  2:
            client.call<RobKeyCmd_t>("SetKeyCmd", RobKeyCmd_t::StartClean);
            break;
        case  3:
            client.call<RobKeyCmd_t>("SetKeyCmd", RobKeyCmd_t::Pause);
            break;
        case  4:
            client.call<RobKeyCmd_t>("SetKeyCmd", RobKeyCmd_t::Continue);
            break;
        case  5:
            client.call<RobKeyCmd_t>("SetKeyCmd", RobKeyCmd_t::WiFiReset);
            break;
        case  6:
            client.call<RobKeyCmd_t>("SetKeyCmd", RobKeyCmd_t::BackToCharge);
            break;
        case  7:
            client.call<RobKeyCmd_t>("SetKeyCmd", RobKeyCmd_t::PowerOff);
            break;
        case  10:
            client.call<RobKeyCode_t>("SetKeyCode", RobKeyCode_t::PowerKeyShortPressed);
            break;
        case  11:
            client.call<RobKeyCode_t>("SetKeyCode", RobKeyCode_t::PowerKeyLongPressed);
            break;
        case  12:
            client.call<RobKeyCode_t>("SetKeyCode", RobKeyCode_t::FnKeyShortPressed);
            break;
        case  13:
            client.call<RobKeyCode_t>("SetKeyCode", RobKeyCode_t::FnKeyLongPressed);
            break;
        case  14:
            client.call<RobKeyCode_t>("SetKeyCode", RobKeyCode_t::FnPowerCombinedLongPressed);
            break;

        /* 20~29  AppCmd */
        case 20:
            client.call<AppCmd_t>("SetAppCmd", AppCmd_t::StartRoomClean);
            break;
        case 21:
            client.call<AppCmd_t>("SetAppCmd", AppCmd_t::StartBlockClean);
            break;
        case 22:
            client.call<AppCmd_t>("SetAppCmd", AppCmd_t::StartSectionClean);
            break; 
        case 23:
            client.call<AppCmd_t>("SetAppCmd", AppCmd_t::StartSpotClean);
            break; 
        case 24:
            client.call<AppCmd_t>("SetAppCmd", AppCmd_t::StartRemoteCtrlClean);
            break; 
        case 25:
            client.call<AppCmd_t>("SetAppCmd", AppCmd_t::StartFastMapping);
            break; 
        case 26:
            client.call<AppCmd_t>("SetAppCmd", AppCmd_t::StartBackToWashMop);
            break; 
        case 27:
            client.call<AppCmd_t>("SetAppCmd", AppCmd_t::Pause);
            break; 
        case 28:
            client.call<AppCmd_t>("SetAppCmd", AppCmd_t::Continue);
            break; 
        case 29:
            client.call<AppCmd_t>("SetAppCmd", AppCmd_t::Stop);
            break; 
        case 30:
            client.call<AppCmd_t>("SetAppCmd", AppCmd_t::BackToStation);
            break; 
        case 31:
            client.call<AppCmd_t>("SetAppCmd", AppCmd_t::KidLockOn);
            break; 
        case 32:
            client.call<AppCmd_t>("SetAppCmd", AppCmd_t::KidLockOff);
            break; 
        case 33:
            client.call<AppCmd_t>("SetAppCmd", AppCmd_t::StaLcdOn);
            break; 
        case 34:
            client.call<AppCmd_t>("SetAppCmd", AppCmd_t::StaLcdOff);
            break; 
        case 35:
            client.call<AppCmd_t>("SetAppCmd", AppCmd_t::ModeClean);
            break; 
        case 36:
            client.call<AppCmd_t>("SetAppCmd", AppCmd_t::ModeMop);
            break; 
        case 37:
            client.call<AppCmd_t>("SetAppCmd", AppCmd_t::ModeCleanAndMop);
            break; 
        case 38:
            client.call<AppCmd_t>("SetAppCmd", AppCmd_t::StartCustomClean);
            break; 


        case 51:
            client.call<StaCmd_t>("SetStaCmd", StaCmd_t::StartRoomClean);
            break;
        case 52:
            client.call<StaCmd_t>("SetStaCmd", StaCmd_t::Pause);
            break;
        case 53:
            client.call<StaCmd_t>("SetStaCmd", StaCmd_t::Continue);
            break;
        case 54:
            client.call<StaCmd_t>("SetStaCmd", StaCmd_t::MoveOutStation);
            break;
        case 55:
            client.call<StaCmd_t>("SetStaCmd", StaCmd_t::BackToStation);
            break;
        case 56:
            client.call<StaCmd_t>("SetStaCmd", StaCmd_t::KidLockOn);
            break;
        case 57:
            client.call<StaCmd_t>("SetStaCmd", StaCmd_t::KidLockOff);
            break;
        case 58:
            client.call<StaCmd_t>("SetStaCmd", StaCmd_t::LcdOn);
            break;
        case 59:
            client.call<StaCmd_t>("SetStaCmd", StaCmd_t::LcdOff);
            break;
        case 60:
            client.call<StaCmd_t>("SetStaCmd", StaCmd_t::ModeClean);
            break;
        case 61:
            client.call<StaCmd_t>("SetStaCmd", StaCmd_t::ModeMop);
            break;
        case 62:
            client.call<StaCmd_t>("SetStaCmd", StaCmd_t::ModeCleanAndMop);
            break;
        case 63:
            client.call<StaCmd_t>("SetStaCmd", StaCmd_t::StartCustomClean);
            break;
        case 100:
            client.call("Quit");

        default:
            break;
        }

        //client.run();

    }


    return 0;
}