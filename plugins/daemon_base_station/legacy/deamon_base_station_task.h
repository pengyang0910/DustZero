/*
 * @Description:
 * @version:
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2022-04-14 11:29:17
 * @LastEditors: jephy jephy.zhang@any-eye.com
 * @LastEditTime: 2023-07-31 16:53:08
 */

#ifndef __TEST_TASK_H__
#define __TEST_TASK_H__

#include "baseTask.h"
#include "xprotocol.h"
#include "xbase.h"
#include "Msg/RobotMsg/RobotStatus_t.hpp"
#include "global.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "XStation.h"
using namespace std;
using namespace XStation;

class DeamonBaseStationTask : public BaseTask_t
{
private:
    void PreStart();
    void PreStop();
    void PreResume();
    void MainLoop();
    void PreSuspend();

    RFData rx_data;
    RFData tx_data;

    XLog *xlog;   
    uint8_t  rf_robot_add_L;
    uint16_t rf_robot_add_H ;
    uint8_t  rf_base_add_L;
    uint16_t rf_base_add_H;
    
    uint32_t ConnectBaseStationTicks;

    XStation::RobotRunningStatus_t LastRobotRunningStatus;

    /****************************************************/
    uint8_t CleanWaterPumpPower[10] = {0};
    uint8_t SewagePumpPower[10] = {0};
    uint8_t CleanLiquidPumpPower[10] = {0};
    uint8_t DrainagePumpPower[10] = {0};
    uint8_t FanPower[10] = {0};
    int16_t MopDuty[10] = {0};
    uint8_t DustCollection[10] = {0};
    uint8_t PtcModule[10] = {0};
    uint8_t StepMax = 0;
    uint8_t CurStep = 0;
    uint32_t StepDelayTicks[10] = {0};

public:

    DeamonBaseStationTask(NavBridge_t *brigdePt, std::string name);
    ~DeamonBaseStationTask();

    void BaseStationRunningNone(void);
    void BaseStationRunningDust(void);
    void BaseStationRunningMopCloth(void);
    void BaseStationRunningDrying(void);
    void BaseStationRunningWetMop(void);
    void BaseStationRunningBackEnablePower(void);
    void BaseStationRunningCompleteFlow(void);
    void BaseStationRunningReset(void);
    void run_step_actions(uint8_t CleanWaterPump,uint8_t SewagePump,uint8_t CleanLiquidPump,uint8_t DrainagePump,uint8_t Fan,uint8_t DustCollection,uint8_t PtcModule,int16_t MopDuty);
};
#endif