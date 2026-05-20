/*
 * @Author: jephy jephy.zhang@any-eye.com
 * @Date: 2023-07-19 20:36:13
 * @LastEditors: jephy jephy.zhang@any-eye.com
 * @LastEditTime: 2023-08-06 16:47:40
 * @FilePath: /xt212_navclean/Hmi/hmi.cpp
*/
#include "hmi.h"
#include "tuyaXt.h"
#include "Msg/NavMsg/VelCmd_t.hpp"

using namespace TuyaXt;

int fdPwr = -1;
int fdFn = -1;
int fdB = -1;
int fdR = -1;
int fdW = -1;

Hmi_t::Hmi_t(/* args */)
{
    xlogPtr = std::make_shared<XLog>(true);
    xlogPtr->Initialise("hmi.log");
    xlogPtr->SetThreshold(XLOG_LEVEL::XLOG_LEVEL_DEBUG);
    xlogPtr->Debug("init");

    fd = open("/dev/xtgpio0", O_RDWR);
    writeKeyData = OUTGPIO_MASK_POW_ON | OUTGPIO_MASK_3V3_ON | OUTGPIO_MASK_5V_ON;

    robot_info_app.robot_error = 0;
    robot_info_app.station_error =0;
    robot_info_app.station_status = 5;
    robot_info_app.robot_status = 0;
    robot_info_app.is_InStation = 0;
    robot_info_app.outerTask = 0;

    hasMap = false;

}

Hmi_t::~Hmi_t()
{
    
}

void Hmi_t::Init()
{

}

int32_t last_workMode=0;
std::vector< int > cmdBoolApp{1,2,3,4,128,132};
int ischargeCnt=0;
int8_t is_robot_stoped = 0;
RpcMsg::RobotParaData_t robot_para; 
int8_t app_is_connect = 0;
float remoteVSpeed=0;
float remoteWSpeed=0;
int remoteCtrl=0;
int8_t is_robot_chargeToWork=0;
int8_t robot_led_on =0;

RobKeyCode_t Hmi_t::robKeyDetect()
{
    // int fnKey = -1;
    // int powerKey = -1;
    static int fnKeyTick = 0;
    static int powerKeyTick = 0;
    static int bothPressTick = 0;
    bool fnKeyLongPressed = false;
    bool powerKeyLongPressed = false;
    bool fnKeyShortPressed = false;
    bool powerKeyShortPressed = false;
    bool bothKeyLongPressed = false;

    if(-1 == fd)
        return RobKeyCode_t::None;
    
    read(fd, &readKeyData, 1);

    if((readKeyData & INGPIO_MASK_FN_KEY) && (readKeyData & INGPIO_MASK_PWR_KEY))
    {
        bothPressTick++;
        fnKeyTick = 0;
        powerKeyTick = 0;
        if(bothPressTick > KEY_LONG_PRESS)
        {
            bothKeyLongPressed = true;
            bothPressTick = KEY_LONG_PRESS;
        }
    }
    else
    {
        bothPressTick = 0;
        
        if(readKeyData & INGPIO_MASK_FN_KEY)
        {
            fnKeyTick++;
            if(fnKeyTick > KEY_LONG_PRESS)
            {
                fnKeyTick = KEY_LONG_PRESS;
                fnKeyLongPressed = true;
            }                
        }
        else // release detection
        {
            if(fnKeyTick >= KEY_LONG_PRESS)
                fnKeyLongPressed = true;
            else if(fnKeyTick > KEY_SHORT_PRESS)
                fnKeyShortPressed = true;

            fnKeyTick = 0;
        }

        if(readKeyData & INGPIO_MASK_PWR_KEY)
        {
            powerKeyTick++;
            if (powerKeyTick > KEY_LONG_PRESS)
            {
                powerKeyTick = KEY_LONG_PRESS;
                powerKeyLongPressed = true;
            }
                
        }
        else // release detection
        {
            if(powerKeyTick >= KEY_LONG_PRESS)
                powerKeyLongPressed = true;
            else if(powerKeyTick > KEY_SHORT_PRESS)
                powerKeyShortPressed = true;
            powerKeyTick = 0;
        }
    }

    
    if(bothKeyLongPressed)
    {
        printf("Both Long!\r\n");
        return RobKeyCode_t::FnPowerCombinedLongPressed;
    }
    else if(powerKeyLongPressed)
    {
        printf("Power Long!\r\n");
        return RobKeyCode_t::PowerKeyLongPressed;
    }
    else if(fnKeyLongPressed)
    {
        printf("Fn Long!\r\n");
        return RobKeyCode_t::FnKeyLongPressed;
    }
    else if(powerKeyShortPressed)
    {
        printf("Power Short!\r\n");
        return RobKeyCode_t::PowerKeyShortPressed;
    }
    else if(fnKeyShortPressed)
    {
        printf("Fn Short!\r\n");
        return RobKeyCode_t::FnKeyShortPressed;
    }
    else 
        return RobKeyCode_t::None;
}

void Hmi_t::robLedUpdate()
{
    switch (robotData.stateMachine)
    {
    case RobotStateMachine_t::Sleeping:
        robotLedState = RobotStateLed_t::Sleeping;
        break;
    case RobotStateMachine_t::StandBy:
        robotLedState = RobotStateLed_t::StandBy;
        break;
    case RobotStateMachine_t::Working:
        if(robotData.workMode == RobotWork_t::BackToStation&&robotData.isCharging==0)
        {
            robotLedState = RobotStateLed_t::BackToCharge;
        }
        else if(robotData.workMode == RobotWork_t::WorkingWithStation)
        {
            robotLedState = RobotStateLed_t::WorkingWithStation;
        }
        else if(robotData.isCharging)
        {
            robotLedState = RobotStateLed_t::Charging;
        } 
        else 
            robotLedState = RobotStateLed_t::Working;
        break;
    case RobotStateMachine_t::Pause:
        robotLedState = RobotStateLed_t::Pause;
        break;
    case RobotStateMachine_t::PowerOff:
        
        break;
    default:
        break;
    }

    if (is_robot_stoped)
    {
        robotLedState = RobotStateLed_t::Error;
    }
    


    switch (robotLedState)
    {
    case RobotStateLed_t::Working:
        /* code */
        ledWhiteOn();
        break;
    case RobotStateLed_t::Pause:
        ledWhiteOn();
        break;
    case RobotStateLed_t::StandBy:
        ledWhiteOn();
        break;
    case RobotStateLed_t::Sleeping:
        if (robot_led_on==0)
        {
            ledOff();
        }  
        break;
    case RobotStateLed_t::Error:
        ledRedFlashQuick();
        break;
    case RobotStateLed_t::BackToCharge:
        ledWhiteFlashSlow();
        break;
    case RobotStateLed_t::WorkingWithStation:
        ledGreenFlashSlow();
        break;
    case RobotStateLed_t::Charging:
        ledWhiteFlashSlow();
        break;
    case RobotStateLed_t::ChargeComleted:
        ledWhiteOn();
        break;
    default:
        break;
    }
}


/* LED Display Mode */
void Hmi_t::ledWhiteOn()
{
    writeKeyData |= (OUTGPIO_MASK_LED_Y);
    writeKeyData &= (~OUTGPIO_MASK_LED_R);
    writeKeyData &= (~OUTGPIO_MASK_LED_B);
}

void Hmi_t::ledOff()
{
    //printf("ledOff\r\n");
    writeKeyData &= (~OUTGPIO_MASK_LED_Y);
    writeKeyData &= (~OUTGPIO_MASK_LED_R);
    writeKeyData &= (~OUTGPIO_MASK_LED_B);

}

void Hmi_t::ledRedFlashQuick()
{
    writeKeyData &= (~OUTGPIO_MASK_LED_Y);
    writeKeyData &= (~OUTGPIO_MASK_LED_B);

    static uint32_t tick = 0;

    if( tick < 16 ) 
    {
        writeKeyData |= OUTGPIO_MASK_LED_R;
    }
    else if(tick < 33)
    {
        writeKeyData &= (~OUTGPIO_MASK_LED_R);
    }
    else 
        tick = 0;

    tick++;    

}

void Hmi_t::ledWhiteFlashSlow()
{
    
    writeKeyData &= (~OUTGPIO_MASK_LED_R);
    writeKeyData &= (~OUTGPIO_MASK_LED_B);
    static uint32_t tick = 0;
    
    if( tick < 50 ) 
    {
        writeKeyData &= (~OUTGPIO_MASK_LED_Y);
    }
    else if(tick < 100)
    {
        writeKeyData |= OUTGPIO_MASK_LED_Y;
    }
    else 
        tick = 0;

    tick++;    
    
}

void Hmi_t::ledGreenFlashSlow()
{

    writeKeyData &= (~OUTGPIO_MASK_LED_R);
    writeKeyData &= (~OUTGPIO_MASK_LED_Y);
    static uint32_t tick = 0;
    
    if( tick < 50 ) 
    {
        writeKeyData &= (~OUTGPIO_MASK_LED_B);
    }
    else if(tick < 100)
    {
        writeKeyData |= OUTGPIO_MASK_LED_B;
    }
    else 
        tick = 0;

    tick++;  
}

int robot_cleanMode = 0;
RobKeyCmd_t Hmi_t::robKeyDecode(RobKeyCode_t keyCode)
{
    RobKeyCmd_t ret = RobKeyCmd_t::None;

    if(robotData.stateMachine == RobotStateMachine_t::Sleeping)
    {
        ret = RobKeyCmd_t::WakeUp;
    }
    else
    {
        RobOuterTask_t newTask = (RobOuterTask_t)(robot_info_app.outerTask);
        switch (keyCode)
        {
        case RobKeyCode_t::PowerKeyLongPressed:
            {
                ret = RobKeyCmd_t::PowerOff;
            }
            break;
        case RobKeyCode_t::PowerKeyShortPressed: 
           //
            if(robotData.stateMachine == RobotStateMachine_t::StandBy)
            {
                ret = RobKeyCmd_t::StartClean;
            }
            else if(robotData.stateMachine == RobotStateMachine_t::Working)
            {
                ret = RobKeyCmd_t::Pause;
            }
           /*  else if(robotData.stateMachine == RobotStateMachine_t::Pause && robotData.workMode == RobotWork_t::RoomClean&&robotData.isCharging==false)
            {
                ret = RobKeyCmd_t::Continue;
            } */
            else if(robotData.stateMachine == RobotStateMachine_t::Pause && robotData.workMode == RobotWork_t::RoomClean&&robotData.isCharging==true&&newTask == RobOuterTask_t::BackToStation)
            {
                robotData.stateMachine = RobotStateMachine_t::StandBy;
                ret = RobKeyCmd_t::StartClean;
                xlogPtr->Debug("BackToStation  stoped  ");
            } 
            else if(robotData.stateMachine == RobotStateMachine_t::Pause && robotData.workMode == RobotWork_t::RoomClean&&robotData.isCharging==true&&newTask == RobOuterTask_t::BreakClean)
            {
                robotData.stateMachine = RobotStateMachine_t::StandBy;
                ret = RobKeyCmd_t::StartClean;
                xlogPtr->Debug("BreakClean  stoped  ");
            } 
            else if(robotData.stateMachine == RobotStateMachine_t::Pause && robotData.workMode == RobotWork_t::RoomClean&&robotData.isCharging==true&&is_robot_chargeToWork==1)
            {
                robotData.stateMachine = RobotStateMachine_t::StandBy;
                ret = RobKeyCmd_t::StartClean;
                xlogPtr->Debug("working   stoped  ");
            } 
            else if(robotData.stateMachine == RobotStateMachine_t::Pause && robotData.workMode != RobotWork_t::RoomClean)
            {
                robotData.stateMachine = RobotStateMachine_t::StandBy;
                ret = RobKeyCmd_t::StartClean;
            }
            else if(robotData.stateMachine == RobotStateMachine_t::Pause)
            {
                ret = RobKeyCmd_t::Continue;
            }
            else 
            {
                xlogPtr->Debug("Invalid PowerKeyShortPressed");
            }
            break;
        case RobKeyCode_t::FnKeyShortPressed:
            if(robotData.stateMachine == RobotStateMachine_t::Working)
            {
                ret = RobKeyCmd_t::Pause;
            }
            else if(robotData.stateMachine == RobotStateMachine_t::Pause ||
                    robotData.stateMachine == RobotStateMachine_t::StandBy )
            {
                ret = RobKeyCmd_t::BackToCharge;
            }

            break;
        case RobKeyCode_t::FnPowerCombinedLongPressed:
            if(robotData.stateMachine == RobotStateMachine_t::StandBy || robotData.stateMachine == RobotStateMachine_t::Pause)
            {
                ret = RobKeyCmd_t::WiFiReset;
            }
            break;

        default:
            break;
        }
    }

    return ret;
}

void Hmi_t::robCmdUpdate()
{
    /* Robot Key Command */
#if 1
    robKeyCode = robKeyDetect();
    if(robKeyCode != RobKeyCode_t::None)
        keyCmd = robKeyDecode(robKeyCode);
#else 
    //robKeyCode = robKeyDetect();
    if(robKeyCode != RobKeyCode_t::None)
        keyCmd = robKeyDecode(robKeyCode);
#endif
    robKeyCode = RobKeyCode_t::None;   

    if (is_robot_stoped)
    {
        //return;
    }
      

    switch(keyCmd)
    {
        case RobKeyCmd_t::None:
            break;
        case RobKeyCmd_t::WakeUp:
        {
            if(robotData.stateMachine == RobotStateMachine_t::Sleeping)
            {
                robotData.stateMachine = RobotStateMachine_t::StandBy;
                xlogPtr->Debug("KeyCmd WakeUp: Sleeping --> StandBy");
            }
            else
            {
                xlogPtr->Debug("Invalid key cmd: %d", int(keyCmd));
            }
        }
        break;
        case RobKeyCmd_t::StartClean:
        {
            if(robotData.stateMachine == RobotStateMachine_t::StandBy)
            {
                if (robotData.workMode != RobotWork_t::RoomClean)
                {
                    xlogPtr->Debug("KeyCmd StartClean: StandBy ---> Working");
                    RobotWork_t last_workMode = robotData.workMode;
                    RpcMsg::RobotData_t robotDataMsg;
                    robotData.workMode = RobotWork_t::None;
                    robotDataMsg.timestamp_us = getTimeStap_us();
                    robotDataMsg.stateMachine = (int)robotData.stateMachine;
                    robotDataMsg.workMode = (int)robotData.workMode;
                    robotDataMsg.robotStopped = is_robot_stoped;
                    lcm.publish(Hmi_Broadcast_RobotState, &robotDataMsg);  

                    //robotData.workMode = RobotWork_t::RoomClean;
                    if (last_workMode == RobotWork_t::None)
                    {     
                        xlogPtr->Debug("KeyCmd last: StandBy");
                        robotData.workMode = RobotWork_t::RoomClean;
                    }
                    else if (last_workMode == RobotWork_t::BackToStation)
                    {
                        xlogPtr->Debug("KeyCmd last: backDock");
                        robotData.workMode = RobotWork_t::RoomClean;
                    }
                    else 
                    {
                        xlogPtr->Debug("KeyCmd last: working");
                        robotData.workMode = last_workMode;
                    }
                    
                    robotData.stateMachine = RobotStateMachine_t::Working;

                    ischargeCnt = 0;
                }
                else
                {
                    RpcMsg::RobotData_t robotDataMsg;
                    robotData.workMode = RobotWork_t::None;
                    robotDataMsg.timestamp_us = getTimeStap_us();
                    robotDataMsg.stateMachine = (int)robotData.stateMachine;
                    robotDataMsg.workMode = (int)robotData.workMode;
                    robotDataMsg.robotStopped = is_robot_stoped;
                    lcm.publish(Hmi_Broadcast_RobotState, &robotDataMsg);  

                    robotData.stateMachine = RobotStateMachine_t::Working;
                    robotData.workMode = RobotWork_t::RoomClean;
                    xlogPtr->Debug("KeyCmd StartClean: StandBy ---> Working  continue");
                }
            }
            else
            {
                xlogPtr->Debug("Invalid key cmd: %d", int(keyCmd));
            }
        }
        break;
        case RobKeyCmd_t::Pause:
        {
            if(robotData.stateMachine == RobotStateMachine_t::Working)
            {
                robotData.stateMachine = RobotStateMachine_t::Pause;
                xlogPtr->Debug("KeyCmd Pause: Working ---> Pause");
            }
            else
            {
                xlogPtr->Debug("Invalid key cmd: %d", int(keyCmd));
            }
        }
        break;
        case RobKeyCmd_t::Continue:
        {
            if(robotData.stateMachine == RobotStateMachine_t::Pause)
            {
                robotData.stateMachine = RobotStateMachine_t::Working;
                robotData.workMode = RobotWork_t::RoomClean;
                xlogPtr->Debug("KeyCmd StartClean: Pause ---> Working");
            }
            else
            {
                xlogPtr->Debug("Invalid KeyCmd: %d", int(keyCmd));
            }
        }
        break;
        case RobKeyCmd_t::WiFiReset:
        {
            if(robotData.stateMachine == RobotStateMachine_t::StandBy||robotData.stateMachine == RobotStateMachine_t::Pause)
            {
                xlogPtr->Debug("KeyCmd WiFi Reset");
#ifdef RK3566_BUILD
                system("rm /userdata/ipc_sweeper_robot/tuya*");
                system("rm /userdata/ipc_sweeper_robot/totalArea.bin");
                system("aplay /app/fj212/resource/robot_voice/93.wav"); 
                xlogPtr->Debug("wifi reset");
                //system("/app/fj212/restartApp.sh");
                system("killall appTask");
                system("/app/fj212/bin/appTask  >/dev/null &" );
#else 
                system("rm /mnt/UDISK/fj212/resource/ipc_sweeper_robot/tuya*");
                system("aplay /mnt/UDISK/fj212/resource/robot_voice/1002.wav"); 
                xlogPtr->Debug("wifi reset");
                system("killall appTask");
                system("/mnt/UDISK/fj212/bin/appTask");
#endif
            }
            else
            {
                xlogPtr->Debug("Invalid KeyCmd: %d", int(keyCmd));
            }
        }
        break;
        case RobKeyCmd_t::BackToCharge:
        {
            if((!robotData.isCharging)&&(robotData.stateMachine == RobotStateMachine_t::StandBy || 
              robotData.stateMachine == RobotStateMachine_t::Pause ))
            {
                if(robotData.workMode != RobotWork_t::BackToStation)
                {
                    xlogPtr->Debug("KeyCmd start BackToCharge");
                    robotData.stateMachine = RobotStateMachine_t::Working;
                    robotData.workMode = RobotWork_t::None;

                    RpcMsg::RobotData_t robotDataMsg;
                    robotDataMsg.timestamp_us = getTimeStap_us();
                    robotDataMsg.stateMachine = (int)robotData.stateMachine;
                    robotDataMsg.workMode = (int)robotData.workMode;
                    robotDataMsg.robotStopped = is_robot_stoped;
                    lcm.publish(Hmi_Broadcast_RobotState, &robotDataMsg);  

                    robotData.workMode = RobotWork_t::BackToStation;

                }
                else 
                {
                    xlogPtr->Debug("KeyCmd continue BackToCharge");
                    robotData.stateMachine = RobotStateMachine_t::Working;
                    robotData.workMode = RobotWork_t::BackToStation;
                }
            }
            else
            {
                xlogPtr->Debug("Invalid KeyCmd: %d", int(keyCmd));
            }
        }
        break;
        case RobKeyCmd_t::PowerOff:
        {
            powerOffTrigger = true;
            writeKeyData = OUTGPIO_MASK_MCU_RST;  // PowerOff

            xlogPtr->Debug("PowerOff");
            robotData.stateMachine = RobotStateMachine_t::PowerOff;
        }
        break;
    }
        
    keyCmd = RobKeyCmd_t::None;
}


void Hmi_t::staCmdUpdate()
{
    APP_CMD_DP_S  cmdS;
    char strValue[100]={0};



    if(!powerOffTrigger)
    {
        // valid check
        switch (staCmd)
        {
        case StaCmd_t::StartRoomClean:
        {
            if(robotData.stateMachine == RobotStateMachine_t::StandBy || 
                robotData.stateMachine == RobotStateMachine_t::Sleeping)
            {
                robotData.stateMachine = RobotStateMachine_t::Working;
                robotData.workMode = RobotWork_t::RoomClean;

                cmdS.dpid = 41;
                cmdS.type = TYPE_ENUM;
                int reportBool = 1;
                sprintf(strValue,"%d",reportBool);
                cmdS.value = strValue;
                app_cmds.push_back(cmdS);

                ischargeCnt = 0;

                xlogPtr->Debug("Station Cmd: start room clean!");
            }
            else
            {
                xlogPtr->Error("Invalid  StaCmd=%d!!! Reset to None!", staCmd);
                staCmd = StaCmd_t::None;
            }
        }
        break;
        case StaCmd_t::Pause:
        {
            if(robotData.stateMachine == RobotStateMachine_t::Working)  
            {
                if (robotData.isCharging ==true)
                {
                    robotData.stateMachine = RobotStateMachine_t::StandBy;
                    xlogPtr->Debug("Station Cmd: Working --> standy!");
                }
                else
                {
                    robotData.stateMachine = RobotStateMachine_t::Pause;
                    xlogPtr->Debug("Station Cmd: Working --> Pause!");
                } 
            }
            else 
            {
                xlogPtr->Error("Invalid  StaCmd=%d!!! Reset to None!", staCmd);
                staCmd = StaCmd_t::None;
            }
        }
        break;
        case StaCmd_t::Continue:
        {
            if(robotData.stateMachine == RobotStateMachine_t::Pause)  
            {
                robotData.stateMachine = RobotStateMachine_t::Working;
                xlogPtr->Debug("Station Cmd: Pause --> Working!");
            }
            else 
            {
                xlogPtr->Error("Invalid  StaCmd=%d!!! Reset to None!", staCmd);
                staCmd = StaCmd_t::None;
            }
        }
        break;
        case StaCmd_t::StartMop:
        {
            if(robotData.stateMachine == RobotStateMachine_t::StandBy || 
                robotData.stateMachine == RobotStateMachine_t::Sleeping)
            {
                robotData.stateMachine = RobotStateMachine_t::Working;
                robotData.workMode = RobotWork_t::RoomClean;
                
                cmdS.dpid = 41;
                cmdS.type = TYPE_ENUM;
                int reportBool = 2;
                sprintf(strValue,"%d",reportBool);
                cmdS.value = strValue;
                app_cmds.push_back(cmdS);

                //xlogPtr->Debug("Station Cmd: start room clean! cleanMode is %d",(int)robotData.cleanMode);
            }
            else
            {
                xlogPtr->Error("Invalid  StaCmd=%d!!! Reset to None!", staCmd);
                staCmd = StaCmd_t::None;
            }
            //robotData.cleanMode = CleanMode_t::CleanOnly;
            //xlogPtr->Debug("Station Cmd: CleanMode: CleanOnly");
        }
        break;
        case StaCmd_t::StartMopAndClean:
        {
            if(robotData.stateMachine == RobotStateMachine_t::StandBy || 
                robotData.stateMachine == RobotStateMachine_t::Sleeping)
            {
                robotData.stateMachine = RobotStateMachine_t::Working;
                //robotData.isCharging = false; // fake 
                robotData.workMode = RobotWork_t::RoomClean;
                //robotData.cleanMode = CleanMode_t::CleanAndMop;

                cmdS.dpid = 41;
                cmdS.type = TYPE_ENUM;
                int reportBool = 0;
                sprintf(strValue,"%d",reportBool);
                cmdS.value = strValue;
                app_cmds.push_back(cmdS);

                ischargeCnt = 0;

                xlogPtr->Debug("Station Cmd: start Mop clean! ");
            }
            else
            {
                xlogPtr->Error("Invalid  StaCmd=%d!!! Reset to None!", staCmd);
                staCmd = StaCmd_t::None;
            }
        }
        break;
        case StaCmd_t::MoveOutStation:
        {
            if( (robotData.stateMachine == RobotStateMachine_t::StandBy || 
                robotData.stateMachine == RobotStateMachine_t::Sleeping) && 
                robotData.isCharging )
            {
                //robotData.stateMachine = RobotStateMachine_t::StandBy;
                //robotData.workMode = RobotWork_t::MoveOutStation;

                cmdS.dpid = 133;
                cmdS.type = TYPE_BOOL;
                int reportBool = 1;
                sprintf(strValue,"%d",reportBool);
                cmdS.value = strValue;
                app_cmds.push_back(cmdS);

                ischargeCnt = 0;

                xlogPtr->Debug("Station Cmd: Move out Station!");
            }
            else  
            {
                xlogPtr->Error("Invalid  StaCmd=%d!!! Reset to None!", staCmd);
                staCmd = StaCmd_t::None;
            }
        }
        break;
        case StaCmd_t::BackToStation:
        {
            if(!robotData.isCharging)
            {
                if(robotData.workMode != RobotWork_t::BackToStation)
                {
                    xlogPtr->Debug("KeyCmd start BackToCharge");
                    robotData.stateMachine = RobotStateMachine_t::Working;
                    robotData.workMode = RobotWork_t::None;

                    RpcMsg::RobotData_t robotDataMsg;
                    robotDataMsg.timestamp_us = getTimeStap_us();
                    robotDataMsg.stateMachine = (int)robotData.stateMachine;
                    robotDataMsg.workMode = (int)robotData.workMode;
                    robotDataMsg.robotStopped = is_robot_stoped;
                    lcm.publish(Hmi_Broadcast_RobotState, &robotDataMsg);  

                    robotData.workMode = RobotWork_t::BackToStation;
                    xlogPtr->Debug("Station Cmd: Back to Station1!");
                }
                else 
                {    
                    robotData.stateMachine = RobotStateMachine_t::Working;
                    robotData.workMode = RobotWork_t::BackToStation;
                    xlogPtr->Debug("Station Cmd: Back to Station!");
                }
            }            
            else 
            {
                xlogPtr->Error("Invalid  StaCmd=%d!!! Reset to None!", staCmd);
                staCmd = StaCmd_t::None;
            }

        }
        break;
        
        case StaCmd_t::StartCustomClean:
        {
            if(robotData.stateMachine == RobotStateMachine_t::StandBy ||
                robotData.stateMachine == RobotStateMachine_t::Sleeping)
            {
                robotData.stateMachine = RobotStateMachine_t::Working;
                robotData.workMode = RobotWork_t::CustomClean;
                xlogPtr->Debug("StaCmd StartCustomClean: Working");
                ischargeCnt = 0;
            }
            else
            {
                xlogPtr->Error("Invalid StaCmd = %d", int(staCmd));
            }
        }
        break;

        default:
            break;
        }
    }


    staCmd = StaCmd_t::None;
}



void Hmi_t::staStateUpdate()
{
    if(!powerOffTrigger)
    {
        // valid check
        switch (staState)
        {
            case StaState_t::KidLockOff:
            {
            //    robotData.StaKidLock = false;
                xlogPtr->Debug("Station Cmd: kidLock OFF!");
            }
            break;
            case StaState_t::KidLockOn:
            {
            //    robotData.StaKidLock = true;
                xlogPtr->Debug("Station Cmd: kidLock ON!");
            }
            break;
            case StaState_t::LcdOff:
            {
            //    robotData.StaLcdOn = false;
                xlogPtr->Debug("Station Cmd: Lcd OFF!");
            }
            break;
            case StaState_t::LcdOn:
            {
            //    robotData.StaLcdOn = true;
                xlogPtr->Debug("Station Cmd: Lcd ON!");
            }
            break;
            default:
                break;
        }
    }

    staState = StaState_t::None;
}

void Hmi_t::appCmdUpdate()
{
    if(!powerOffTrigger)
    {
        switch (appCmd)
        {
        case AppCmd_t::StartRoomClean:
        {
            if(robotData.stateMachine == RobotStateMachine_t::StandBy || 
                robotData.stateMachine == RobotStateMachine_t::Sleeping)
            {    
                robotData.stateMachine = RobotStateMachine_t::Working;
                robotData.workMode = RobotWork_t::RoomClean;

                xlogPtr->Debug("App Cmd: start RoomClean task!");
            }
            else if (robotData.stateMachine == RobotStateMachine_t::Pause&&robotData.workMode == RobotWork_t::None)
            {
                robotData.stateMachine = RobotStateMachine_t::Working;
                robotData.workMode = RobotWork_t::RoomClean;
                //robotData.isCharging = false; // fake r->Debug("App Cmd 1: start RoomClean task!!!  pause");
            } 
            else if (robotData.stateMachine == RobotStateMachine_t::Working&&robotData.workMode == RobotWork_t::None)
            {
                robotData.stateMachine = RobotStateMachine_t::Working;
                robotData.workMode = RobotWork_t::RoomClean;
                //robotData.isCharging = false; // fake 
                xlogPtr->Debug("App Cmd 2: start RoomClean task!!!  pause");
            } 
            /* else if (robotData.stateMachine == RobotStateMachine_t::Working&&robotData.workMode == RobotWork_t::MoveOutStation)
            {
                robotData.stateMachine = RobotStateMachine_t::Working;
                robotData.workMode = RobotWork_t::RoomClean;
                robotData.isCharging = false; // fake 
                xlogPtr->Debug("App Cmd 3: start RoomClean task!!!  pause");
            }  */
            else
            {
                xlogPtr->Error("Invalid  AppCmd=%d!!! Reset to None!", appCmd);
                appCmd = AppCmd_t::None;
            }
        }
        break;
        case AppCmd_t::StartBlockClean:
        {
            if(robotData.stateMachine == RobotStateMachine_t::StandBy || 
                robotData.stateMachine == RobotStateMachine_t::Sleeping)
            {
                robotData.stateMachine = RobotStateMachine_t::Working;
                robotData.workMode = RobotWork_t::BlockClean;
                //robotData.isCharging = false; // fake 
                xlogPtr->Debug("App Cmd: start BlockClean task!");
            }
            else
            {
                xlogPtr->Error("Invalid  AppCmd=%d!!! Reset to None!", appCmd);
                appCmd = AppCmd_t::None;
            }
        }
        break;
        case AppCmd_t::StartSectionClean:
        {
            if(robotData.stateMachine == RobotStateMachine_t::StandBy || 
                robotData.stateMachine == RobotStateMachine_t::Sleeping)
            {
                robotData.stateMachine = RobotStateMachine_t::Working;
                robotData.workMode = RobotWork_t::SectionClean;
                //robotData.isCharging = false; // fake 
                xlogPtr->Debug("App Cmd: start SectionClean task!");
            }
            else
            {
                xlogPtr->Error("Invalid  AppCmd=%d!!! Reset to None!", appCmd);
                appCmd = AppCmd_t::None;
            }
        }
        break;
        case AppCmd_t::StartSpotClean:
        {
            if(robotData.stateMachine == RobotStateMachine_t::StandBy || 
                robotData.stateMachine == RobotStateMachine_t::Sleeping)
            {
                robotData.stateMachine = RobotStateMachine_t::Working;
                robotData.workMode = RobotWork_t::SpotClean;
                //robotData.isCharging = false; // fake 
                xlogPtr->Debug("App Cmd: start SpotClean task!");
            }
            else
            {
                xlogPtr->Error("Invalid  AppCmd=%d!!! Reset to None!", appCmd);
                appCmd = AppCmd_t::None;
            }
        }
        break;
        case AppCmd_t::StartFastMapping:
        {
            if(robotData.stateMachine == RobotStateMachine_t::StandBy || 
                robotData.stateMachine == RobotStateMachine_t::Sleeping)
            {
                robotData.stateMachine = RobotStateMachine_t::Working;
                robotData.workMode = RobotWork_t::FastMapping;
                //robotData.isCharging = false; // fake 
                xlogPtr->Debug("App Cmd: start FastMapping task!");
            }
            else
            {
                xlogPtr->Error("Invalid  AppCmd=%d!!! Reset to None!", appCmd);
                appCmd = AppCmd_t::None;
            }
        }
        break;
        case AppCmd_t::StartRemoteCtrlClean:
        {
            if(robotData.stateMachine == RobotStateMachine_t::StandBy || 
                robotData.stateMachine == RobotStateMachine_t::Sleeping)
            {
                robotData.stateMachine = RobotStateMachine_t::Working;
                robotData.workMode = RobotWork_t::RemoteCtrl;
                //robotData.isCharging = false; // fake 
                xlogPtr->Debug("App Cmd: start RemoteCtrl task!");
            }
            else
            {
                xlogPtr->Error("Invalid  AppCmd=%d!!! Reset to None!", appCmd);
                appCmd = AppCmd_t::None;
            }
        }
        break;
        case AppCmd_t::StartBackToWashMopAndCharge:
        {
            if(!robotData.isCharging)
            {
                robotData.stateMachine = RobotStateMachine_t::Working;
                robotData.workMode = RobotWork_t::BackToWashMopAndCharge;
                xlogPtr->Debug("App Cmd: start BackToWash charge task!");
            }
            else
            {
                xlogPtr->Error("Invalid  AppCmd=%d!!! Reset to None!", appCmd);
                appCmd = AppCmd_t::None;
            }
        }
        break;
        case AppCmd_t::StartCustomClean:
        {
            if(robotData.stateMachine == RobotStateMachine_t::StandBy || 
                robotData.stateMachine == RobotStateMachine_t::Sleeping)
            {
                robotData.stateMachine = RobotStateMachine_t::Working;
                robotData.workMode = RobotWork_t::CustomClean;
                //robotData.isCharging = false; // fake 
                xlogPtr->Debug("App Cmd: start CustomClean task!");
            }
            else
            {
                xlogPtr->Error("Invalid  AppCmd=%d!!! Reset to None!", appCmd);
                appCmd = AppCmd_t::None;
            }
        }
        break;
        case AppCmd_t::Pause:
        {
            if(robotData.stateMachine == RobotStateMachine_t::Working)  
            {
                robotData.stateMachine = RobotStateMachine_t::Pause;
                xlogPtr->Debug("App Cmd: Working --> Pause!");

                /* if (robotData.workMode ==RobotWork_t::StartBackToWashMop)
                {
                    robotData.workMode = RobotWork_t::None;
                } */
                
            }
            else 
            {
                xlogPtr->Error("Invalid  AppCmd=%d!!! Reset to None!", appCmd);
                robotData.stateMachine = RobotStateMachine_t::Pause;
                appCmd = AppCmd_t::None;
            }
        }
        break;
        case AppCmd_t::Continue:
        {
            if(robotData.stateMachine == RobotStateMachine_t::Pause)  
            {
                robotData.stateMachine = RobotStateMachine_t::Working;
                if (robotData.workMode == RobotWork_t::None)
                {
                    robotData.workMode = RobotWork_t::RoomClean;
                }
                
                xlogPtr->Debug("App Cmd: Pause --> Working!");
            }
            else 
            {
                xlogPtr->Error("Invalid  AppCmd=%d!!! Reset to None!", appCmd);
                robotData.stateMachine = RobotStateMachine_t::Working;
                appCmd = AppCmd_t::None;
            }
        }
        break;
        case AppCmd_t::Stop:
        {
            if(robotData.stateMachine == RobotStateMachine_t::Pause ||
               robotData.stateMachine == RobotStateMachine_t::Working )  
            {
                robotData.stateMachine = RobotStateMachine_t::StandBy;
                robotData.workMode = RobotWork_t::None;
                xlogPtr->Debug("App Cmd: Pause or Working --> StandBy!");



                /* RpcMsg::RobotData_t robotDataMsg;
                robotDataMsg.timestamp_us = getTimeStap_us();
                robotDataMsg.stateMachine = (int)robotData.stateMachine;
                robotDataMsg.cleanMode = (int)robotData.cleanMode;
                robotDataMsg.workMode = (int)robotData.workMode;
                robotDataMsg.robotErr = robotData.robotErr;
                robotDataMsg.staErr = robotData.staErr;
                robotDataMsg.isCharging = robotData.isCharging;
                robotDataMsg.stakidLock = robotData.StaKidLock;
                robotDataMsg.staLcdOn = robotData.StaLcdOn;

                lcm.publish(Hmi_Broadcast_RobotState, &robotDataMsg); */
            }
            else 
            {
                xlogPtr->Error("Invalid  AppCmd=%d!!! Reset to None!", appCmd);
                appCmd = AppCmd_t::None;
            }
        }
        break;
        case AppCmd_t::ModeClean:
        {
            //robotData.cleanMode = CleanMode_t::CleanOnly;
        }
        break;
        case AppCmd_t::ModeMop:
        {
            //robotData.cleanMode = CleanMode_t::MopOnly;
        }
        break;
        case AppCmd_t::ModeCleanAndMop:
        {
            //robotData.cleanMode = CleanMode_t::CleanAndMop;
        }
        break;
        case AppCmd_t::BackToStation:
        {
            if(!robotData.isCharging)
            {
                robotData.stateMachine = RobotStateMachine_t::Working;
                robotData.workMode = RobotWork_t::BackToStation;
            }
        }
        break;
        case AppCmd_t::KidLockOff:
        {
            //robotData.StaKidLock = false;
        }
        break;
        case AppCmd_t::KidLockOn:
        {
            //robotData.StaKidLock = true;
        }
        break;
        case AppCmd_t::StaLcdOff:
        {
            //robotData.StaLcdOn = false;
        }
        break;
        case AppCmd_t::StaLcdOn:
        {
            //robotData.StaLcdOn = true;
        }
        break;
        default:
            break;
        }
    }

    appCmd = AppCmd_t::None;
}

void Hmi_t::staCmdSetDemo(StaCmd_t cmd,StaState_t state)
{
    staCmd = cmd;
    staState = state;
    xlogPtr->Debug("staCmdSetDemo called: cmd = %d sta is %d", int(cmd),int(state));
}

void Hmi_t::staCmdSet(rpc_conn conn, StaCmd_t cmd)
{
    staCmd = cmd;
    xlogPtr->Debug("StaCmdSet called: cmd = %d", int(cmd));
}

void Hmi_t::appCmdSet(rpc_conn conn, AppCmd_t cmd)
{
    appCmd = cmd;
    xlogPtr->Debug("AppCmdSet called: cmd = %d", int(cmd));
}

void Hmi_t::keyCmdSet(rpc_conn conn, RobKeyCmd_t cmd)
{
    keyCmd = cmd;
    xlogPtr->Debug("KeyCmdSet called: cmd=%d", int(cmd));
}

void Hmi_t::KeyCodeSet(rpc_conn conn, RobKeyCode_t cd)
{
    robKeyCode = cd;
    xlogPtr->Debug("KeyCodeSet called: cd=%d", int(cd));
}

bool Hmi_t::getAppCmd(rpc_conn conn, APP_CMD_DP_S cmd)
{
    int dpid = cmd.dpid;
    
    xlogPtr->Debug("hmi update cmd id is %d ",dpid);

    if (std::count(cmdBoolApp.begin(), cmdBoolApp.end(),dpid))
    {
        cmd_data = cmd;
        cmd_dpid = cmd.dpid;
        appCmdDataAnalysis();
    }
    else 
    {
        if (dpid==140)
        {
            app_is_connect = 1;
            return true;
        }
        else if (dpid==118)
        {
            int value = atoi(cmd.value.c_str());
            robot_led_on = value;
            xlogPtr->Debug("led on !");
        }
        
        else if (dpid==13) //reset map
        {
            APP_RAW_DP_S  raw;
            raw.dpid= 64;
            raw.len=1;
            raw.data.push_back(1);
            app_rawcmds.push_back(raw);
        }
        else if (dpid==12 && robotData.workMode != RobotWork_t::BackToStation)  // remote ctrl
        {
            int value = atoi(cmd.value.c_str());
            xlogPtr->Debug("value is %d \n",value);
           
            if (remoteCtrl==0&&value!=5)
            {
                // stop all task
                RpcMsg::RobotData_t robotDataMsg;
                robotDataMsg.timestamp_us = getTimeStap_us();
                robotData.stateMachine = RobotStateMachine_t::StandBy;
                robotDataMsg.stateMachine = (int)robotData.stateMachine;
                robotData.workMode = RobotWork_t::None;
                robotDataMsg.workMode = (int)(RobotWork_t::None);
                robotDataMsg.robotStopped = is_robot_stoped;
                lcm.publish(Hmi_Broadcast_RobotState, &robotDataMsg);  
                xlogPtr->Debug(" StateMachine: remote stop ");
            }
            
            if (value==0)
            {
                remoteVSpeed=0.2;
                remoteWSpeed=0;
                remoteCtrl=1;
            }
            else if (value==1)
            {
                remoteVSpeed=-0.2;
                remoteWSpeed=0;
                remoteCtrl=1;
            }
            else if (value==2)
            {
                remoteVSpeed=0;
                remoteWSpeed=1;
                remoteCtrl=1;
            }
            else if (value==3)
            {
                remoteVSpeed=0;
                remoteWSpeed=-1;
                remoteCtrl=1;
            }
            else if (value==4)
            {
                remoteVSpeed=0;
                remoteWSpeed=0;
                remoteCtrl=1;
            }
            else if (value==5&&robotData.workMode == RobotWork_t::RemoteCtrl)  //Exit
            {
                xlogPtr->Debug("exit RemoteCtrl Mode ");
                remoteVSpeed=0;
                remoteWSpeed=0;
                remoteCtrl=0;
                 //idel task
                robotData.workMode = RobotWork_t::None;
                robotData.stateMachine = RobotStateMachine_t::StandBy; 
            }
            else if (value==5)  //Exit
            {
                xlogPtr->Debug("exit RemoteCtrl  ");
                remoteVSpeed=0;
                remoteWSpeed=0;
                remoteCtrl=0;
            }

            if (remoteCtrl==1)
            {
                robotData.workMode = RobotWork_t::RemoteCtrl;
                robotData.stateMachine = RobotStateMachine_t::Working; 
            }
            
            
        }
        
        /* else if (dpid==11)
        {
            Nav_RAW_DP_S _rawCmds;
            _rawCmds.cmd = 4;
            _rawCmds.u8data.clear();
            _rawCmds.fdata.clear();
            nav_rawCmds.push_back(_rawCmds);  
        } */
        else 
        {
            app_cmds.push_back(cmd);
        }
               
        if (dpid == 117)  // backWashAndCharge 
        {
            if (robotData.workMode != RobotWork_t::RoomClean)
            {
                xlogPtr->Debug("KeyCmd StartClean: StandBy ---> Working washMop");
                RpcMsg::RobotData_t robotDataMsg;
                robotData.workMode = RobotWork_t::None;
                robotDataMsg.timestamp_us = getTimeStap_us();
                robotDataMsg.stateMachine = (int)robotData.stateMachine;
                robotDataMsg.workMode = (int)robotData.workMode;
                robotDataMsg.robotStopped = is_robot_stoped;
                lcm.publish(Hmi_Broadcast_RobotState, &robotDataMsg);   
            }

            remoteVSpeed=0;
            remoteWSpeed=0;
            remoteCtrl=0;
            
            robotData.stateMachine = RobotStateMachine_t::Working;
            robotData.workMode = RobotWork_t::RoomClean;
            xlogPtr->Debug("117  update cmd ");
        }  
    }
    
    return true; 
}

bool Hmi_t::getAppRawCmd(rpc_conn conn, APP_RAW_DP_S  raw)
{
    int dpid = raw.dpid;
    raw_id = raw.dpid;
    rawcmd_data = raw;
    xlogPtr->Debug("hmi update raw cmd id is %d ",dpid);

    if (dpid==0x14)  //select room 
    {
        Nav_RAW_DP_S _rawCmds;
        _rawCmds.cmd=1;
        _rawCmds.u8data.clear();
        std::vector<uint8_t> tmData;
        for (int i = 4; i < raw.len; i++)
        {
           tmData.push_back(raw.data[i]);
            //xlogPtr->Debug("selection clean %d ",raw.data[i]);
        }
        for (size_t i = 2; i < 4; i++)
        {
            _rawCmds.u8data.push_back(raw.data[i]);
            xlogPtr->Debug("selection clean %d ",raw.data[i]);
        }
        // for (size_t i = tmData.size(); i >0; i--)
        // {
        //     _rawCmds.u8data.push_back(tmData[i-1]);
        //     xlogPtr->Debug("selection clean order %d ",tmData[i-1]);
        // }
        for (size_t i = 0; i <tmData.size(); i++)
        {
            _rawCmds.u8data.push_back(tmData[i]);
            xlogPtr->Debug("selection clean order %d ",tmData[i]);
        } 
        nav_rawCmds.push_back(_rawCmds);   
        
    }
    else if (dpid==0x16)  //pose_clean 
    {
        Nav_RAW_DP_S _rawCmds;
        _rawCmds.cmd=2;
        _rawCmds.u8data.clear();
        _rawCmds.fdata.clear();
        bool isX = false;
        for (int j = 0; j < 2; j++) // pose 2 points, fixed
        {
            if( j % 2 == 0 )
            {
                isX = true;
            }else{
                isX = false;
            } 
            uint8_t loc0 = raw.data[j*2 + 2];
            uint8_t loc1 = raw.data[j*2 + 3];

            float pos = ex_vitrual_location(loc0, loc1, isX); 
            _rawCmds.fdata.push_back(pos);
            xlogPtr->Debug("poseclean pose is %f ",pos);  
            //xlog->Debug("roomSplitLineData loc trans to slam %d: %f", j, lcm_data.roomSplitLineData[j]);                    
        } 

        nav_rawCmds.push_back(_rawCmds);  
    }
    else if (dpid==0x3A)   //block zone clean
    {
        Nav_RAW_DP_S _rawCmds;
        _rawCmds.cmd=3;
        _rawCmds.u8data.clear();
        _rawCmds.fdata.clear();
        int Num = raw.data[3];
        bool isX = false;
        _rawCmds.u8data.push_back(Num);
        for (int i = 0; i < Num; i++)
        {
            for (size_t j = 0; j < 8; j++)
            {
                if( j % 2 == 0 )
                {
                    isX = true;
                }
                else
                {
                    isX = false;
                } 
                uint8_t loc0 = raw.data[j*2 + 6 + i*36];
                uint8_t loc1 = raw.data[j*2 + 7 + i*36];

                float pos = ex_vitrual_location(loc0, loc1, isX); 
                _rawCmds.fdata.push_back(pos);
                xlogPtr->Debug(" huaqu pose is %f ",pos);  
            }
        }
        nav_rawCmds.push_back(_rawCmds);     
    }
    else if (dpid==0x32)
    {
        Nav_RAW_DP_S _rawCmds;
        _rawCmds.cmd=4;
        _rawCmds.u8data.clear();
        for (int i = 2; i < raw.len; i++)
        {
            _rawCmds.u8data.push_back(raw.data[i]);
            xlogPtr->Debug("disturb time  %d ",raw.data[i]);
        }
        nav_rawCmds.push_back(_rawCmds);   
    }
    else if (dpid==0x22)   //special  clean   room
    {
        APP_CMD_DP_S  cmdS;
        cmdS.dpid = 41;
        cmdS.type = TYPE_ENUM;
        int reportBool = 3;
        char strValue[100]={0};
        sprintf(strValue,"%d",reportBool);
        cmdS.value = strValue;
        app_cmds.push_back(cmdS); // send to navClean

        app_rawcmds.push_back(raw); // send to slam
    }
    else
    {
        app_rawcmds.push_back(raw);
    }
  
    return true;
}

bool Hmi_t::appCmdDataAnalysis()
{
    if (cmd_data.dpid!=0)
    {
        if (cmd_data.type == TYPE_BOOL)
        {
            int value = atoi(cmd_data.value.c_str());
            xlogPtr->Debug("    StateMachine: app cmd is  %d %d",value,cmd_data.dpid);
            if (cmd_data.dpid==1)
            {
                if (value) 
                {
                    if (robotData.stateMachine == RobotStateMachine_t::Working&&robotData.workMode == RobotWork_t::RoomClean)
                    {
                        xlogPtr->Error("    invalid app cmd  start clean ");
                    }
                    else
                    {
                        RpcMsg::RobotData_t robotDataMsg;
                        robotDataMsg.timestamp_us = getTimeStap_us();
                        robotDataMsg.stateMachine = (int)robotData.stateMachine;
                        robotData.workMode = RobotWork_t::None;
                        robotDataMsg.workMode = (int)(RobotWork_t::None);
                        robotDataMsg.robotStopped = is_robot_stoped;
                        lcm.publish(Hmi_Broadcast_RobotState, &robotDataMsg);  
                        xlogPtr->Debug("    StateMachine: app stop  start clean ");

                        appCmd= AppCmd_t::StartRoomClean;
                    }  
                }
                else       
                {
                    appCmd= AppCmd_t::Stop; 
                }         
            }
            else if (cmd_data.dpid==2)
            {
                if (value) appCmd= AppCmd_t::Pause;
                else       appCmd= AppCmd_t::Continue;            
            }
            else if (cmd_data.dpid==3)
            {
                if (value) 
                {  
                    if (robotData.stateMachine == RobotStateMachine_t::Working&&robotData.workMode == RobotWork_t::BackToStation)
                    {
                        xlogPtr->Error("    invalid app cmd  start charge ");
                    }
                    else
                    {
                        RpcMsg::RobotData_t robotDataMsg;
                        robotDataMsg.timestamp_us = getTimeStap_us();
                        //robotDataMsg.stateMachine = (int)robotData.stateMachine;
                        robotDataMsg.stateMachine = (int)RobotStateMachine_t::StandBy;
                        robotData.workMode = RobotWork_t::None;
                        robotDataMsg.workMode = (int)(RobotWork_t::None);
                        robotDataMsg.robotStopped = is_robot_stoped;
                        lcm.publish(Hmi_Broadcast_RobotState, &robotDataMsg);  
                        xlogPtr->Debug("    StateMachine: app stop  start charge ");

                        appCmd= AppCmd_t::BackToStation;
                    }
                }
                else      appCmd= AppCmd_t::Stop;            
            }
            else if (cmd_data.dpid==128)
            {
                if (value) 
                {
                    if (hasMap==false)
                    {
                        appCmd= AppCmd_t::StartFastMapping;
                    }
                    else
                    {
                        if (robotData.stateMachine == RobotStateMachine_t::Working&&robotData.workMode == RobotWork_t::RoomClean)
                        {
                            xlogPtr->Error("    invalid app cmd  start mapping ");
                        }
                        else
                        {
                            xlogPtr->Debug("    has map : change to  start roomClean ");
                            RpcMsg::RobotData_t robotDataMsg;
                            robotDataMsg.timestamp_us = getTimeStap_us();
                            robotDataMsg.stateMachine = (int)robotData.stateMachine;
                            robotData.workMode = RobotWork_t::None;
                            robotDataMsg.workMode = (int)(RobotWork_t::None);
                            robotDataMsg.robotStopped = is_robot_stoped;
                            lcm.publish(Hmi_Broadcast_RobotState, &robotDataMsg);  
                            xlogPtr->Debug("    StateMachine: app stop  start clean ");

                            appCmd= AppCmd_t::StartRoomClean;
                        }
                    }    
                }
                
            }   
            else if (cmd_data.dpid==132)
            {
                if (value) 
                {
                    if (robotData.stateMachine == RobotStateMachine_t::Working&&robotData.workMode == RobotWork_t::BackToWashMopAndCharge)
                    {
                        xlogPtr->Error("    invalid app cmd  start BackToWashMopAndCharge ");
                    }
                    else
                    {
                        RpcMsg::RobotData_t robotDataMsg;
                        robotDataMsg.timestamp_us = getTimeStap_us();
                        robotDataMsg.stateMachine = (int)robotData.stateMachine;
                        robotData.workMode = RobotWork_t::None;
                        robotDataMsg.workMode = (int)(RobotWork_t::None);
                        robotDataMsg.robotStopped = is_robot_stoped;
                        lcm.publish(Hmi_Broadcast_RobotState, &robotDataMsg);  
                        xlogPtr->Debug("    StateMachine: mapping stop  start washCharge ");
                        appCmd= AppCmd_t::StartBackToWashMopAndCharge;
                    }
                }      
            }   
        }
        else if (cmd_data.type == TYPE_ENUM)
        {
            // int value = atoi(cmd_data.value.c_str());
            if (cmd_data.dpid==4)
            {
                //if (value==0) appCmd= AppCmd_t::ModeCleanAndMop;
            }
            if (cmd_data.dpid==41)
            {
                //if (value==1) appCmd= AppCmd_t::ModeCleanAndMop;
                //else  if (value==2) appCmd= AppCmd_t::ModeClean;   
                //else  if (value==3) appCmd= AppCmd_t::ModeMop;          
            }
        }
        
    }
    cmd_data.dpid = 0;
    
    return true;
}

void Hmi_t::syncData()
{
    static uint32_t tick = 0;
    static uint32_t jumpTick = 0;
    if(robotData.stateMachine == RobotStateMachine_t::Pause || 
      robotData.stateMachine == RobotStateMachine_t::StandBy )
    {
        tick++;
    }
    else 
        tick = 0;

    if(tick > 30000)  // 300s 
    //if(tick > 6000) //60s
    {
        robotData.stateMachine = RobotStateMachine_t::Sleeping;
        robotData.workMode = RobotWork_t::None;
        tick = 0;
    }

    jumpTick++;
    if(jumpTick % 10 == 0) // 10Hz   
    {
        // Publish data To NavTaskPro, SlamPro, AppPro, LocalizationPro
        RpcMsg::RobotData_t robotDataMsg;
        robotDataMsg.timestamp_us = getTimeStap_us();
        robotDataMsg.stateMachine = (int)robotData.stateMachine;
        //robotDataMsg.cleanMode = (int)robotData.cleanMode;
        robotDataMsg.workMode = (int)robotData.workMode;
        robotDataMsg.robotStopped = is_robot_stoped;
        /* robotDataMsg.robotErr = robotData.robotErr;
        robotDataMsg.staErr = robotData.staErr;
        robotDataMsg.isCharging = robotData.isCharging;
        robotDataMsg.stakidLock = robotData.StaKidLock;
        robotDataMsg.staLcdOn = robotData.StaLcdOn; */

        lcm.publish(Hmi_Broadcast_RobotState, &robotDataMsg);   
    }
    else if (jumpTick % 12 == 0)
    {
        lcm.publish("lcm_robotState_hmiToApp", &robot_info_app);
    }
}

RobotData_t Hmi_t::GetRobot()
{
    return robotData; // lock in future
}

void Hmi_t::SetRobotCharging(bool charging)
{
    robotData.isCharging = charging ? 1 : 0;
}

void Hmi_t::SetRobotStopped(bool stopped)
{
    is_robot_stoped = stopped ? 1 : 0;
}

void Hmi_t::ShowRobot()
{
    static uint32_t tick = 0;

    if (tick%10==0)
    {
        if (remoteCtrl==1)
        {
             NavMsg::VelCmd_t vel;
             vel.vSpd = remoteVSpeed;
             vel.wSpd = remoteWSpeed;
             //vel.vSpd = 0;
             //vel.wSpd = 0;
             lcm.publish("hmi_publish_vel_nav", &vel);  
             xlogPtr->Info("send speed is %f %f \n",remoteVSpeed,remoteWSpeed);
        }
    }
    

    if(tick < 150)
    {
        tick++;
        return;
    }
    else 
        tick = 0;
    
    if(robotData.stateMachine == RobotStateMachine_t::Sleeping)
    {
        return;
    }

    xlogPtr->Debug(">>>>>>>>>>>>>> Robot Status <<<<<<<<<<<<");
    xlogPtr->Debug("    StateMachine: %d", int(robotData.stateMachine));
    xlogPtr->Debug("    WorkMode:     %d", int(robotData.workMode));
    xlogPtr->Debug("    IsCharging:   %d", int(robotData.isCharging));
    xlogPtr->Debug("    robot_status:   %d", int(robot_info_app.robot_status));
    xlogPtr->Debug("    station_status:   %d", int(robot_info_app.station_status));
    xlogPtr->Debug("    RobotErr:     %x", int(robot_info_app.robot_error));
    xlogPtr->Debug("    StaErr:       %x", int(robot_info_app.station_error)); 
    xlogPtr->Debug("    outTask:       %x", int(robot_info_nav.outerTask));  
    xlogPtr->Debug("    clean_finished:       %d", int(robot_info_app.is_clean_finished));  
    
  /*   xlogPtr->Debug("    status: %d", int(robotInfo.statusValue));
    xlogPtr->Debug("    stationStatus:   %d", int(robotInfo.stationStatusValue));
    xlogPtr->Debug("    battery:   %d", int(robotInfo.battery_percentage));
    //xlogPtr->Debug("    WorkMode:     %d", int(robotInfo.statusValue));
    xlogPtr->Debug("    RobotErr:     %x", int(robotData.robotErr));
    xlogPtr->Debug("    StaErr:       %x", int(robotData.staErr)); */
    xlogPtr->Debug("----------------------------------------");
}

void Hmi_t::MainLoop()
{
    while(isRunning)
    {
        // 1. update robot input key
        robCmdUpdate();
        // 2. update XStation input  
        staCmdUpdate();
        // 3. update App input
        //appCmdDataAnalysis();
        appCmdUpdate();
        // 4. CmdArbitration
        ShowRobot();
        // 4. sync data to XStation&App
        syncData();
        // 5. Update Robot Led
        robLedUpdate();

        lcm.handleTimeout(1);

        if (app_is_connect==1)
        {
            app_is_connect = 2;
            lcm.publish("hmi_publish_para_app", &robot_para);  
            xlogPtr->Debug(">>>>>>>>>>>>>> hmi_publish_para_app <<<<<<<<<<<<");
        }

        //printf("write0 is %d \n",writeKeyData);

        if(-1 != fd)
            write(fd, &writeKeyData, sizeof(writeKeyData));

        //printf("write is %d \n",writeKeyData);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void Hmi_t::Start()
{
    isRunning = true;
    robot_status = StatusValue_t::StandyBy;
    lastTask = RobOuterTask_t::None;

#ifndef WIN32
	prctl(PR_SET_NAME, "ILcmHMI");
	bindCpuCore(BIND_CPU_ID_INNER_LCM);
#endif
    lcm.subscribe("lcm_robotState_navProToHmi", &Hmi_t::robotInfoMsgUpdate, this);

    mapInfoSub = lcm.subscribe(LCM_CHANNEL_MapInfo, &Hmi_t::gridMapInfoUpdate, this);
    mapInfoSub->setQueueCapacity(4);

    lcm.subscribe(LCM_CHANNEL_HACK, &Hmi_t::getTeleCmd, this);
    
#ifndef WIN32
	prctl(PR_SET_NAME, "main");
	bindCpuCore(BIND_CPU_ID_MISC);
#endif

    ini::IniFile myIni;
    myIni.setMultiLineValues(true);
    std::ifstream cfgCheck(BusinessCfg);
    bool hasCfg = cfgCheck.is_open();
    if (hasCfg) {
        cfgCheck.close();
        myIni.load(BusinessCfg);
    } else {
        xlogPtr->Debug("BusinessCfg not found, using defaults");
    }

    std::vector<int8_t> bData;
    bool boolData = false;
    if (hasCfg) {
        boolData = myIni["App"][myIni.GetKey("carpetMode_b")].as<bool>();
    }
    bData.push_back(boolData);
    boolData = false;
    if (hasCfg) {
        boolData = myIni["App"][myIni.GetKey("doNotDisturb_b")].as<bool>();
    }
    bData.push_back(boolData);
    boolData = false;
    if (hasCfg) {
        boolData = myIni["App"][myIni.GetKey("dryAuto_b")].as<bool>();
    }
    bData.push_back(boolData);
    boolData = false;
    if (hasCfg) {
        boolData = myIni["App"][myIni.GetKey("dustCollectoin_b")].as<bool>();
    }
    bData.push_back(boolData);
    boolData = false;
    if (hasCfg) {
        boolData = myIni["App"][myIni.GetKey("fluidAuto_b")].as<bool>();
    }
    bData.push_back(boolData);
    boolData = false;
    if (hasCfg) {
        boolData = myIni["App"][myIni.GetKey("kidsLock_b")].as<bool>();
    }
    bData.push_back(boolData);
    boolData = false;
    if (hasCfg) {
        boolData = myIni["App"][myIni.GetKey("stationWinLight_b")].as<bool>();
    }
    bData.push_back(boolData);
    boolData = false;
    if (hasCfg) {
        boolData = myIni["App"][myIni.GetKey("breakPointClean_b")].as<bool>();
    }
    bData.push_back(boolData);
    boolData = false;
    if (hasCfg) {
        boolData = myIni["App"][myIni.GetKey("enXd1QMute_b")].as<bool>();
    }
    bData.push_back(boolData);

    std::vector<int16_t> intData;
    int iData = 0;
    if (hasCfg) {
        iData = myIni["App"][myIni.GetKey("carpetCleanMode_i")].as<int>();
    }
    intData.push_back(iData);
    iData = 0;
    if (hasCfg) {
        iData = myIni["App"][myIni.GetKey("cleanMode_i")].as<int>();
    }
    intData.push_back(iData);
    iData = 0;
    if (hasCfg) {
        iData = myIni["App"][myIni.GetKey("cleanTimeMin_i")].as<int>();
    }
    intData.push_back(iData);
    iData = 0;
    if (hasCfg) {
        iData = myIni["App"][myIni.GetKey("clean_life_i")].as<int>();
    }
    intData.push_back(iData);
    iData = 0;
    if (hasCfg) {
        iData = myIni["App"][myIni.GetKey("cristern_i")].as<int>();
    }
    intData.push_back(iData);
    iData = 0;
    if (hasCfg) {
        iData = myIni["App"][myIni.GetKey("dust_internal_i")].as<int>();
    }
    intData.push_back(iData);
    iData = 0;
    if (hasCfg) {
        iData = myIni["App"][myIni.GetKey("fanMotorRank_i")].as<int>();
    }
    intData.push_back(iData);
    iData = 0;
    if (hasCfg) {
        iData = myIni["App"][myIni.GetKey("speakerVolume_i")].as<int>();
    }
    intData.push_back(iData);
    iData = 0;
    if (hasCfg) {
        iData = myIni["App"][myIni.GetKey("suction_i")].as<int>();
    }
    intData.push_back(iData);
    iData = 0;
    if (hasCfg) {
        iData = myIni["App"][myIni.GetKey("waterYield_i")].as<int>();
    }
    intData.push_back(iData);
    iData = 0;
    if (hasCfg) {
        iData = myIni["App"][myIni.GetKey("cleanAreaMM_i")].as<int>();
    }
    intData.push_back(iData);
     

    robot_para.intData = intData;
    robot_para.iDataLen = robot_para.intData.size();

    robot_para.boolData = bData;
    robot_para.bDataLen = robot_para.boolData.size();

    robot_para.floatData.clear();
    robot_para.lData.clear();
    robot_para.sData.clear();

    robot_para.fDataLen = 0;
    robot_para.lDataLen = 0;
    robot_para.sDataLen = 0;


    for (size_t i = 0; i < robot_para.intData.size(); i++)
    {
        xlogPtr->Debug(">>>>>>> %d  %d<<<<<<",i,robot_para.intData[i]);
    }


/*     carpetCleanMode_i = 1
    carpetMode_b = true
    cleanAreaMM_i = 10
    cleanMode_i = 3
    cleanTimeMin_i = 9
    clean_life_i = 105
    cristern_i = 0
    doNotDisturb_b = true
    dryAuto_b = true
    dustCollectoin_b = true
    dust_internal_i = 1
    edgeBrushLifeMin_i = 258
    fanMotorRank_i = 2
    filterLifeMin_i = 8706
    fluidAuto_b = true
    kidsLock_b = false
    ragLifeMin_i = 8706
    rollBrushLifeMin_i = 17706
    speakerVolume_i = 63
    stationWinLight_b = false
    suction_i = 0
    waterYield_i = 0
 */
    t1 = createBindThread(ProHmiName+"Main", std::bind(&Hmi_t::MainLoop, this), BIND_CPU_ID_RPC);
    //t1 = std::thread(&Hmi_t::MainLoop, this);
    //bindCpuCore(BIND_CPU_ID_RPC);
}

void Hmi_t::Stop()
{
    isRunning = false;
    xlogPtr->Debug("isRunning stop");
    if(t1.joinable())
        t1.join();
}

int  Hmi_t::getNavCmds(std::vector<Nav_RAW_DP_S>& all_nav_rawcmds)
{
    all_nav_rawcmds = nav_rawCmds;

    return all_nav_rawcmds.size();
}

int  Hmi_t::getAppCmds(std::vector<APP_CMD_DP_S>& all_app_cmds)
{
    all_app_cmds = app_cmds;

    return all_app_cmds.size();
}

bool Hmi_t::resetAppCmd(APP_CMD_DP_S cmd)
{
    for (std::vector<APP_CMD_DP_S>::const_iterator it = app_cmds.begin(); it != app_cmds.end(); ++it)
    {
        if (it->dpid == cmd.dpid&& it->value == cmd.value)
        {
            app_cmds.erase(it);
            return true;
        }    
    }

    return false;
}

bool Hmi_t::resetAppRawCmd(APP_RAW_DP_S rawCmd)
{
    for (std::vector<APP_RAW_DP_S>::const_iterator it = app_rawcmds.begin(); it != app_rawcmds.end(); ++it)
    {
        if (it->dpid == rawCmd.dpid)
        {
            app_rawcmds.erase(it);
            return true;
        }    
    }

    return false;
}

bool Hmi_t::resetNavRawCmd(Nav_RAW_DP_S rawCmd)
{
    if (rawCmd.cmd==1)   // select room 
    {
        robotData.stateMachine = RobotStateMachine_t::Working;
        robotData.workMode = RobotWork_t::SectionClean;
        
    }
    else if (rawCmd.cmd==2) // pose clean  定点 
    {
        robotData.stateMachine = RobotStateMachine_t::Working;
        robotData.workMode = RobotWork_t::SpotClean;
    }
    else if (rawCmd.cmd==3)  // hua qu   zone clean 
    {
        robotData.stateMachine = RobotStateMachine_t::Working;
        robotData.workMode = RobotWork_t::BlockClean;
        
    }
    
    for (std::vector<Nav_RAW_DP_S>::const_iterator it = nav_rawCmds.begin(); it != nav_rawCmds.end(); ++it)
    {
        if (it->cmd == rawCmd.cmd)
        {
            nav_rawCmds.erase(it);
            return true;
        }    
    }

    return false;
}

int  Hmi_t::getAppRawCmds(std::vector<APP_RAW_DP_S>& all_app_rawcmds)
{
    all_app_rawcmds = app_rawcmds;
    return all_app_rawcmds.size();
}

int last_is_clean_finished =0;
void Hmi_t::robotInfoMsgUpdate(const lcm::ReceiveBuffer* rbuf, 
                        const std::string &channel, 
                        const RpcMsg::RobotInfoData_t *msg)
{
   robot_info_nav = *msg;
   robotData.isCharging = robot_info_nav.isCharging;

   robot_info_app.robot_error = robot_info_nav.robot_error;
   robot_info_app.station_error = robot_info_nav.station_error;
   robot_info_app.robot_work_error = robot_info_nav.robot_work_error;
  
    int new_station_status = 5;
    if (robot_info_nav.station_status==1)
    {
        new_station_status = 4;
    }
    else if (robot_info_nav.station_status==2)
    {
        new_station_status = 3;
    }
    else if (robot_info_nav.station_status==3)
    {
        new_station_status = 7;
    }
    else 
    {
        new_station_status = 5;
    } 


    if (robot_info_nav.isCharging)
    {
        ischargeCnt++;
    }
    else ischargeCnt=0;


    robot_info_app.outerTask = 0;
    if (ischargeCnt>10)
    {
        robot_status = StatusValue_t::Charging;

        // int8_t tempData = (robot_info_nav.outerTask>>6)&1;
        //robot_info_nav.outerTask &= ~(1<<6);
        int8_t tempTaskData = (robot_info_nav.outerTask & ~(1<<6));
        robot_info_app.outerTask = tempTaskData;
        if (tempTaskData !=0)
        {   
            RobOuterTask_t newTask = (RobOuterTask_t)(tempTaskData);
            if (newTask == RobOuterTask_t::CleanFished&&lastTask != RobOuterTask_t::CleanFished)
            {
                xlogPtr->Debug(">>>>>>>>>>>>> outTask1 is %d  <<<<<<<<<<<<",robot_info_app.outerTask);
                robotData.stateMachine = RobotStateMachine_t::StandBy;
                robotData.workMode = RobotWork_t::None;
                
                robot_status = StatusValue_t::StandyBy;

                xlogPtr->Debug(">>>>>>>>>>>>>> clean  finished <<<<<<<<<<<<");

            //    APP_RAW_DP_S  raw;
            //    raw.dpid= 100;
            //    app_rawcmds.push_back(raw);
            APP_RAW_DP_S  raw;
            raw.dpid= 100;
            raw.len=1;
            raw.data.push_back(1);
            app_rawcmds.push_back(raw);
            }    
        }
        else if(last_is_clean_finished!=1&&msg->is_clean_finished==1)
        {
            robotData.stateMachine = RobotStateMachine_t::StandBy;
            robotData.workMode = RobotWork_t::None;
            
            robot_status = StatusValue_t::StandyBy;

            xlogPtr->Debug(">>>>>>>>>>>>>> clean  finished 000 <<<<<<<<<<<<");
        }
        else if (robotData.stateMachine == RobotStateMachine_t::Pause)
        {
            robotData.stateMachine = RobotStateMachine_t::StandBy;
            robotData.workMode = RobotWork_t::None;
            
            robot_status = StatusValue_t::StandyBy;

            xlogPtr->Debug(">>>>>>>>>>>>>> clean  finished 222 <<<<<<<<<<<<");
        }
        
        lastTask = (RobOuterTask_t)(tempTaskData); 

        int is_lowPower = robot_info_app.robot_error& 0x01;
        if (is_lowPower&&robotData.stateMachine == RobotStateMachine_t::Working)
        {
            xlogPtr->Debug(">>>>>>>>>>>>>> low power charging   <<<<<<<<<<<<");
            is_robot_chargeToWork = 1;
        }


        
    }
    else if (robotData.stateMachine == RobotStateMachine_t::Sleeping)
    {
        robot_status = StatusValue_t::Sleep;
    }
    else
    {
        is_robot_chargeToWork =0;

        if(robotData.stateMachine == RobotStateMachine_t::Pause)
        {
            robot_status = StatusValue_t::Paused;
        }
        else if (robotData.stateMachine == RobotStateMachine_t::Sleeping)
        {
            robot_status = StatusValue_t::Sleep;
        }
        else if (robotData.stateMachine == RobotStateMachine_t::StandBy)
        {
            robot_status = StatusValue_t::StandyBy;
        }
        else if (robotData.stateMachine == RobotStateMachine_t::Working)
        {
            if (robotData.workMode == RobotWork_t::RoomClean)
            {
                robot_status = StatusValue_t::Smart;
            }
            else if (robotData.workMode == RobotWork_t::FastMapping)
            {
                robot_status = StatusValue_t::FastBuilding;
            }
            else if (robotData.workMode == RobotWork_t::BackToStation||robotData.workMode == RobotWork_t::BackToWashMopAndCharge)
            {
                robot_status = StatusValue_t::GoToCharge;
            }
            else if (robotData.workMode == RobotWork_t::BlockClean)
            {
                robot_status = StatusValue_t::ZoneClean;
            }
            else if (robotData.workMode == RobotWork_t::SpotClean)
            {
                robot_status = StatusValue_t::PartClean;
            }
            else if (robotData.workMode == RobotWork_t::SectionClean)
            {
                robot_status = StatusValue_t::SelectRoom;
            }
            else if(last_is_clean_finished!=1&&msg->is_clean_finished==1)
            {
                robotData.stateMachine = RobotStateMachine_t::StandBy;
                robotData.workMode = RobotWork_t::None;
                
                robot_status = StatusValue_t::StandyBy;

                xlogPtr->Debug(">>>>>>>>>>>>>> clean  finished 111 <<<<<<<<<<<<");

                APP_RAW_DP_S  raw;
                raw.dpid= 100;
                //app_rawcmds.push_back(raw);

            }
        }  



        //int8_t tempData = (~0x20 &  robot_info_nav.outerTask);
        int8_t tempData = (robot_info_nav.outerTask>>6)&1;
        //robot_info_nav.outerTask &= ~(1<<6);
        int8_t tempTaskData = (robot_info_nav.outerTask & ~(1<<6));
        robot_info_app.outerTask = tempTaskData;
        if (tempTaskData !=0)
        {
        //    xlogPtr->Debug(">>>>>>>>>>>>> outTask is %d  <<<<<<<<<<<<",robot_info_app.outerTask);
            RobOuterTask_t newTask = (RobOuterTask_t)(tempTaskData);
            if (robotData.stateMachine == RobotStateMachine_t::Working && (newTask == RobOuterTask_t::BackToStation||newTask == RobOuterTask_t::BackToWashMop))
            {
                robot_status = StatusValue_t::GoToCharge;
                if (newTask == RobOuterTask_t::BackToStation)
                {
                    /* code */
                }

                xlogPtr->Debug(">>>>>>>>>>>>>> GoToCharge 222 <<<<<<<<<<<<");
                
            }
            /* else if (newTask == RobOuterTask_t::ErrorStoped)
            {
                robotData.stateMachine = RobotStateMachine_t::Pause;
                robot_status = StatusValue_t::Paused;
                printf("------------error stop ----------\n");
            } */
            else if (newTask == RobOuterTask_t::None&&lastTask == RobOuterTask_t::OutStationFinish)
            {
                robotData.stateMachine = RobotStateMachine_t::StandBy;
                robot_status = StatusValue_t::StandyBy;
                xlogPtr->Debug(">>>>>>>>>>>>>> outstation  finished <<<<<<<<<<<<");
            }
            else if (newTask == RobOuterTask_t::CleanFished&&lastTask != RobOuterTask_t::CleanFished)
            {
                robotData.stateMachine = RobotStateMachine_t::StandBy;
                robotData.workMode = RobotWork_t::None;
                
                robot_status = StatusValue_t::StandyBy;

                xlogPtr->Debug(">>>>>>>>>>>>>> clean  finished  ischarging <<<<<<<<<<<<");

                APP_RAW_DP_S  raw;
                raw.dpid= 100;
                raw.len=1;
                raw.data.push_back(1);
                app_rawcmds.push_back(raw);
                //app_rawcmds.push_back(raw);
            }    
        }
    //    printf(" data is %d %d %d\n",tempData,robot_info_nav.outerTask,tempTaskData);
        if (tempData!=0)
        {
            if (robotData.workMode==RobotWork_t::None)
            {
                robotData.stateMachine = RobotStateMachine_t::StandBy;
                robot_status = StatusValue_t::StandyBy;
                is_robot_stoped =1;
            }
            else 
            {
                robotData.stateMachine = RobotStateMachine_t::Pause;
                robot_status = StatusValue_t::Paused;
                is_robot_stoped =1;
            }
              
            xlogPtr->Debug("------------error stop 1----------\n");
        }
        else
        {
            is_robot_stoped =0;
        }
        
        lastTask = (RobOuterTask_t)(tempTaskData); 
		
		int baterrError = robot_info_app.robot_error& 0x10000;
		int openlaserError = robot_info_app.robot_error& 0x400000;
		int updatelaserError = robot_info_app.robot_error& 0x02000000;
		if(baterrError!=0||openlaserError !=0||updatelaserError !=0) 
		{
			 robotData.stateMachine = RobotStateMachine_t::Pause;
             robot_status = StatusValue_t::Paused;
             is_robot_stoped =1;
			 xlogPtr->Debug("------------laser error stop ----------%d %d %d \n",baterrError,openlaserError,updatelaserError);
		}
    }

    last_is_clean_finished = msg->is_clean_finished;

    robot_info_app.station_status = new_station_status;
    robot_info_app.robot_status = (int8_t)(robot_status);

    robot_info_app.battery_percent = robot_info_nav.battery_percent;
    robot_info_app.clean_area = robot_info_nav.clean_area;
    robot_info_app.clean_time = robot_info_nav.clean_time;
    robot_info_app.clean_mode = robot_info_nav.clean_mode;
    robot_info_app.suction_value = robot_info_nav.suction_value;
    robot_info_app.sta_KidLock = robot_info_nav.sta_KidLock;
    robot_info_app.sta_LcdOn = robot_info_nav.sta_LcdOn;
    robot_info_app.is_InStation = robot_info_nav.is_InStation;
    if(robot_info_app.is_InStation)
    {
        robot_info_app.station_px = robot_info_nav.station_px;
        robot_info_app.station_py = robot_info_nav.station_py;
        robot_info_app.station_pa = robot_info_nav.station_pa;
    }
    robot_info_app.is_clean_finished = robot_info_nav.is_clean_finished;
}


void Hmi_t::gridMapInfoUpdate(const lcm::ReceiveBuffer* rbuf,
		                const std::string &channel,
		                const NavMsg::GridMapInfo_t *msg)
{
    _mapInfo.originPx = msg->originPx*-200;
    _mapInfo.originPy = msg->originPy*-200;
    _mapInfo.width = msg->width*10;
    if (msg->width<=0&&msg->height<=0)
    {
        hasMap = false;
    }
    else if (msg->width>0&&msg->height>0)
    {
        hasMap = true;
    } 
}

float Hmi_t::ex_vitrual_location(uint8_t loc0, uint8_t loc1, bool isX)
{
    float res = 0.00;

    int16_t ex;
    // BYTE_T *pt = (BYTE_T *)&ex; 
    // pt[0] = loc1;
    // pt[1] = loc0;
    ex = ( loc0 * 256 ) + loc1;

  //  xlogPtr->Debug("map_ox : %f, map_oy: %f, map width: %f", _mapInfo.originPx, _mapInfo.originPy, _mapInfo.width);

    if( isX )
    {
        res = _mapInfo.width - ex - _mapInfo.originPx; 
    }
    else
    {
        res = -ex - _mapInfo.originPy ;
    }

    //printf(" map %d %d %d",loc0,loc1,map_ox);
    return res / 200 * 1.00;
}

void Hmi_t::getTeleCmd(const lcm::ReceiveBuffer* rbuf,
		                const std::string &channel,
		                const RobotMsg::HackCmd_t *msg)
{
    if (msg->data == 212)
    {
        //staCmd = StaCmd_t::StartRoomClean;
        appCmd= AppCmd_t::StartRoomClean;
        xlogPtr->Debug("tele called  startclean ");
    }
    else  if (msg->data == 213)
    {
        appCmd= AppCmd_t::Pause;
        xlogPtr->Debug("tele called  pause clean");
    }
    else  if (msg->data == 214)
    {
       appCmd= AppCmd_t::Continue;
        xlogPtr->Debug("tele called  pause clean");
    }
    
}