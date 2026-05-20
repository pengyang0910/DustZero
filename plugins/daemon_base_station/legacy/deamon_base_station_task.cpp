/*
 * @Description:
 * @version:
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2022-04-14 11:29:23
 * @LastEditors: jephy jephy.zhang@any-eye.com
 * @LastEditTime: 2023-07-31 16:44:55
 */

#include "deamon_base_station_task.h"
#include "Msg/UtilsMsg/RpcCmd_t.hpp"
#include "version.h"
#include "DemonStation_version.h"

DeamonBaseStationTask::DeamonBaseStationTask(NavBridge_t *nabrigdePt, std::string name) : BaseTask_t(nabrigdePt, name)
{
    bool enLog   = bridgePt->GetRobotCfg().GetProperty("BaseStationTask", "enLog_b").as<bool>();
    int logLevel = bridgePt->GetRobotCfg().GetProperty("BaseStationTask", "logLevel_i").as<int>();

    /** get robot and basestation addr */
    bridgePt->xStation.RFDataToMCU.robot_addr_L = bridgePt->GetPlatformCfg().GetProperty("DeamonBaseStation", "robotAddrL_i").as<int>();
    bridgePt->xStation.RFDataToMCU.robot_addr_H = bridgePt->GetPlatformCfg().GetProperty("DeamonBaseStation", "robotAddrH_i").as<int>();
    bridgePt->xStation.RFDataToMCU.base_addr_L  = bridgePt->GetPlatformCfg().GetProperty("DeamonBaseStation", "baseAddrL_i").as<int>();
    bridgePt->xStation.RFDataToMCU.base_addr_H  = bridgePt->GetPlatformCfg().GetProperty("DeamonBaseStation", "baseAddrH_i").as<int>();

    mainVersionInfo = XT212_DEPENDENCE_VERSION;
    subVersionInfo  = DSTATION_VERSION;

    if (enLog)
        xlog = new XLog(true);
    else
        xlog = new XLog(false);
    
    xlog->Initialise("DeamonBaseStationTask.log");
    xlog->SetThreshold(Type(logLevel));

    xlog->Debug("DeamonBaseStation  mainVersion is %s!", mainVersionInfo.c_str());
    xlog->Debug("DeamonBaseStation subVersion is %s!", subVersionInfo.c_str());

}

DeamonBaseStationTask::~DeamonBaseStationTask()
{
    if (NULL != xlog)
        delete xlog;
}

void DeamonBaseStationTask::PreStart()
{
    bridgePt->enableResponse = false;
}

void DeamonBaseStationTask::PreStop()
{
    xlog->Info("TestTask PreStop called!\r\n");
}

void DeamonBaseStationTask::PreResume()
{
    xlog->Info("TestTask Presume called!\r\n");
}
void DeamonBaseStationTask::MainLoop()
{
    static uint8_t CompleteFlag = 0;
    static RFData RFDataFromMCUTemp = {0};
    static uint8_t dry_flag = 0;
    static uint32_t connect_timeout_ticks = 0;
    static uint32_t LastRFTicks = 0;
    static XStation::PanelCtrlMsg_t mcu_panel = XStation::PanelCtrlMsg_t::none;
    
#if 0
    static uint32_t enbale_speed = 0;
    if(enbale_speed == 50)
    {
        bridgePt->GetXBasePt()->setSpeed(0.1,0);
    }
    else
    {
        enbale_speed++;
        if(enbale_speed > 100)
        {
            enbale_speed--;
        }
    }
    
    if(bridgePt->GetXBasePt()->IsVirBumperBumped())
    {
        bridgePt->GetXBasePt()->setSpeed(0.001,0);
    }

#endif

    bridgePt->GetXBasePt()->getRFPacket(&RFDataFromMCUTemp);

    if((RFDataFromMCUTemp.base_addr_L == bridgePt->xStation.RFDataToMCU.base_addr_L) && (RFDataFromMCUTemp.base_addr_H == bridgePt->xStation.RFDataToMCU.base_addr_H) /
      (RFDataFromMCUTemp.robot_addr_L == bridgePt->xStation.RFDataToMCU.robot_addr_L) && (RFDataFromMCUTemp.robot_addr_H == bridgePt->xStation.RFDataToMCU.robot_addr_H))
      {
        memcpy((uint8_t *)&bridgePt->xStation.RFDataFromMCU,(uint8_t *)&RFDataFromMCUTemp,16);
      }
/**
 * check recive_mcu_data timeout
*/
    connect_timeout_ticks++;
    if(LastRFTicks != bridgePt->xStation.RFDataFromMCU.ticks)
    {
        LastRFTicks = bridgePt->xStation.RFDataFromMCU.ticks;
        connect_timeout_ticks = 0;
        bridgePt->xStation.ConnectStationStatus = true;
    }
    if(connect_timeout_ticks > 50*10)
    {
        connect_timeout_ticks = 0;
        bridgePt->xStation.ConnectStationStatus = false;
        bridgePt->xStation.SetRobotWorkingProcess(XStation::RobotRunningStatus_t::none);
    }
#if 1
    if((XStation::PanelCtrlMsg_t)bridgePt->xStation.RFDataFromMCU.data.station2robot.WorkStatus.PanelCtrlMsg != mcu_panel)
    {
        mcu_panel = (XStation::PanelCtrlMsg_t)bridgePt->xStation.RFDataFromMCU.data.station2robot.WorkStatus.PanelCtrlMsg;
        xlog->Error("(XStation::PanelCtrlMsg_t)bridgePt->xStation.RFDataFromMCU.data.station2robot.WorkStatus.PanelCtrlMsg = %d",(XStation::PanelCtrlMsg_t)bridgePt->xStation.RFDataFromMCU.data.station2robot.WorkStatus.PanelCtrlMsg);
    }
#endif

#if 0
    bridgePt->GetXBasePt()->setMBrushUpDown(false);
    bridgePt->GetXBasePt()->setMBrushUpDownDuty(50);

    bridgePt->GetXBasePt()->setMainBrushDuty(50);

    bridgePt->GetXBasePt()->setRighBrushDuty(80);

    bridgePt->GetXBasePt()->setSpeed(0.5,0);

    bridgePt->GetXBasePt()->mopLiftDown();
    bridgePt->GetXBasePt()->setMopDuty(80);

    bridgePt->GetXBasePt()->GetErrorCode();
#endif
    static int32_t last_isstation = 0;
    if(bridgePt->GetXBasePt()->isRobotInStation() == false)
    {
        last_isstation++;
        if(last_isstation >= 100)
        {
            last_isstation = 0;
            CurStep = StepMax;
            bridgePt->xStation.SetRobotIsBaseStation(false);
        }
    }
    else
    {
        last_isstation--;
        if(last_isstation <= -100)
        {
            last_isstation = 0;
            bridgePt->xStation.SetRobotIsBaseStation(true);
        }
    }
    /**检测在执行流程的时候主机是否被拖出基站*/
    static uint8_t LastRobotIsStation = 0; 
    if(LastRobotIsStation != (uint8_t)bridgePt->xStation.RFDataToMCU.data.robot2station.IO.SetRobotIsBaseStaton)
    {
        LastRobotIsStation = (uint8_t)bridgePt->xStation.RFDataToMCU.data.robot2station.IO.SetRobotIsBaseStaton;
        if(LastRobotIsStation == 0)//出基站
        {
            bridgePt->xStation.SetRobotWorkingProcess(XStation::RobotRunningStatus_t::none);
            bridgePt->xStation.RFDataToMCU.data.robot2station.RobotStatus.res = 0;
            bridgePt->GetXBasePt()->setMopDuty(0);
        }
    }
    /**添加自动烘干工程状态切换测试*/
    if(bridgePt->getStationCleanStatus().bf.dry_auto != dry_flag)
    {
        dry_flag = bridgePt->getStationCleanStatus().bf.dry_auto;
        xlog->Error("bridgePt->getStationCleanStatus().bf.dry_auto = %d\r\n",bridgePt->getStationCleanStatus().bf.dry_auto);
    }
#if 1
    /***********************************************************************************/
    if(LastRobotRunningStatus != (XStation::RobotRunningStatus_t)bridgePt->xStation.RFDataToMCU.data.robot2station.RobotStatus.WorkingCondition)
    {
        LastRobotRunningStatus = (XStation::RobotRunningStatus_t)bridgePt->xStation.RFDataToMCU.data.robot2station.RobotStatus.WorkingCondition;

        BaseStationRunningReset();
        xlog->Error("LastRobotRunningStatus = %d\r\n",LastRobotRunningStatus);
        switch (LastRobotRunningStatus)
        {
            case XStation::RobotRunningStatus_t::none:
                bridgePt->xStation.SetRobotWorkingProcess(XStation::RobotRunningStatus_t::none);
                break;
            case XStation::RobotRunningStatus_t::IsDustRunning:
            {
                xlog->Error("start running IsDustRunning\r\n");
                StepMax = 1;
                CurStep = 0;

                StepDelayTicks[0]       = 30*50;//集尘
                CleanWaterPumpPower[0]  = 0;
                SewagePumpPower[0]      = 0;
                CleanLiquidPumpPower[0] = 0;
                DrainagePumpPower[0]    = 0;
                FanPower[0]             = 0;
                MopDuty[0]              = 0;
                DustCollection[0]       = 1;
                PtcModule[0]            = 0;
                bridgePt->xStation.SetRobotWorkingProcess(XStation::RobotRunningStatus_t::IsDustRunning);
                break;
            }
            case XStation::RobotRunningStatus_t::IsMopClothRunning:
                {
                    xlog->Error("start running IsMopClothRunning\r\n");
                    StepMax = 8;
                    CurStep = 0;

                    StepDelayTicks[0]       = 30*50;//抽清水和清洁液并洗拖布
                    CleanWaterPumpPower[0]  = 60;
                    SewagePumpPower[0]      = 0;
                    CleanLiquidPumpPower[0] = 100;
                    DrainagePumpPower[0]    = 0;
                    FanPower[0]             = 0;
                    MopDuty[0]              = 80;
                    DustCollection[0]       = 0;
                    PtcModule[0]            = 0;

                    StepDelayTicks[1]       = 20*50;//洗拖布
                    CleanWaterPumpPower[1]  = 0;
                    SewagePumpPower[1]      = 0;
                    CleanLiquidPumpPower[1] = 0;
                    DrainagePumpPower[1]    = 0;
                    FanPower[1]             = 0;
                    MopDuty[1]              = 80;
                    DustCollection[1]       = 0;
                    PtcModule[1]            = 0;

                    StepDelayTicks[2]       = 20*50;//抽清水排污水洗拖布
                    CleanWaterPumpPower[2]  = 60;
                    SewagePumpPower[2]      = 80;
                    CleanLiquidPumpPower[2] = 0;
                    DrainagePumpPower[2]    = 0;
                    FanPower[2]             = 0;
                    MopDuty[2]              = 80;
                    DustCollection[2]       = 0;
                    PtcModule[2]            = 0;

                    StepDelayTicks[3]       = 35*50;//洗拖布
                    CleanWaterPumpPower[3]  = 0;
                    SewagePumpPower[3]      = 0;
                    CleanLiquidPumpPower[3] = 0;
                    DrainagePumpPower[3]    = 0;
                    FanPower[3]             = 0;
                    MopDuty[3]              = 100;
                    DustCollection[3]       = 0;
                    PtcModule[3]            = 0;

                    StepDelayTicks[4]       = 120*50;//排污水洗拖布
                    CleanWaterPumpPower[4]  = 0;
                    SewagePumpPower[4]      = 100;
                    CleanLiquidPumpPower[4] = 0;
                    DrainagePumpPower[4]    = 0;
                    FanPower[4]             = 0;
                    MopDuty[4]              = 100;
                    DustCollection[4]       = 0;
                    PtcModule[4]            = 0;

                    StepDelayTicks[5]       = 5*50;//排污水风干拖布
                    CleanWaterPumpPower[5]  = 0;
                    SewagePumpPower[5]      = 100;
                    CleanLiquidPumpPower[5] = 0;
                    DrainagePumpPower[5]    = 0;
                    FanPower[5]             = 0;
                    MopDuty[5]              = -40;
                    DustCollection[5]       = 0;
                    PtcModule[5]            = 0;

                    StepDelayTicks[6]       = 20*50;//开始排水
                    CleanWaterPumpPower[6]  = 0;
                    SewagePumpPower[6]      = 0;
                    CleanLiquidPumpPower[6] = 0;
                    DrainagePumpPower[6]    = 100;
                    FanPower[6]             = 0;
                    MopDuty[6]              = 0;
                    DustCollection[6]       = 0;
                    PtcModule[6]            = 0;

                    /**********************************************************************************/
                    StepDelayTicks[7] = 5*50;//
                    CleanWaterPumpPower[7]  = 0;
                    SewagePumpPower[7]      = 0;
                    CleanLiquidPumpPower[7] = 0;
                    DrainagePumpPower[7]    = 0;
                    FanPower[7]             = 0;
                    MopDuty[7]              = 0;
                    DustCollection[7]       = 0;
                    PtcModule[7]            = 0;

                    /**********************************************************************************/
                    bridgePt->xStation.SetRobotWorkingProcess(XStation::RobotRunningStatus_t::IsMopClothRunning);
                    bridgePt->xStation.RFDataToMCU.data.robot2station.RobotStatus.res = 2;
                }
                break;
            case XStation::RobotRunningStatus_t::IsDryingRunning:
                {
                    xlog->Error("start running IsDryingRunning\r\n");
                    StepMax = 1;
                    CurStep = 0;

                    StepDelayTicks[0]       = 2*60*60*50;//
                    CleanWaterPumpPower[0]  = 0;
                    SewagePumpPower[0]      = 0;
                    CleanLiquidPumpPower[0] = 0;
                    DrainagePumpPower[0]    = 0;
                    FanPower[0]             = 100;
                    MopDuty[0]              = 0;
                    DustCollection[0]       = 0;
                    PtcModule[0]            = 1;
                    bridgePt->xStation.SetRobotWorkingProcess(XStation::RobotRunningStatus_t::IsDryingRunning);
                }
                break;
            case XStation::RobotRunningStatus_t::IsWetMopRunning:
                {
                    xlog->Error("start running IsWetMopRunning\r\n");
                    StepMax = 5;
                    CurStep = 0;

                    StepDelayTicks[0]       = 30*50;//
                    CleanWaterPumpPower[0]  = 30;
                    SewagePumpPower[0]      = 0;
                    CleanLiquidPumpPower[0] = 0;
                    DrainagePumpPower[0]    = 0;
                    FanPower[0]             = 0;
                    MopDuty[0]              = 80;
                    DustCollection[0]       = 0;
                    PtcModule[0]            = 0;

                    StepDelayTicks[1]       = 7*50;//
                    CleanWaterPumpPower[1]  = 0;
                    SewagePumpPower[1]      = 50;
                    CleanLiquidPumpPower[1] = 0;
                    DrainagePumpPower[1]    = 0;
                    FanPower[1]             = 0;
                    MopDuty[1]              = 80;
                    DustCollection[1]       = 0;
                    PtcModule[1]            = 0;

                    StepDelayTicks[2]       = 5*50;//
                    CleanWaterPumpPower[2]  = 0;
                    SewagePumpPower[2]      = 50;
                    CleanLiquidPumpPower[2] = 0;
                    DrainagePumpPower[2]    = 0;
                    FanPower[2]             = 0;
                    MopDuty[2]              = -40;
                    DustCollection[2]       = 0;
                    PtcModule[2]            = 0;

                    StepDelayTicks[3]       = 10*50;//排水
                    CleanWaterPumpPower[3]  = 0;
                    SewagePumpPower[3]      = 0;
                    CleanLiquidPumpPower[3] = 0;
                    DrainagePumpPower[3]    = 100;//开始排水
                    FanPower[3]             = 0;
                    MopDuty[3]              = 0;
                    DustCollection[3]       = 0;
                    PtcModule[3]            = 0;
                    
                    StepDelayTicks[4]       = 2*50;//
                    CleanWaterPumpPower[4]  = 0;
                    SewagePumpPower[4]      = 0;
                    CleanLiquidPumpPower[4] = 0;
                    DrainagePumpPower[4]    = 0;
                    FanPower[4]             = 0;
                    MopDuty[4]              = 0;
                    DustCollection[4]       = 0;
                    PtcModule[4]            = 0;
                }
                bridgePt->xStation.SetRobotWorkingProcess(XStation::RobotRunningStatus_t::IsDryingRunning);
                break;
            case XStation::RobotRunningStatus_t::IsBackEnablePowerRunning:
                xlog->Error("start running IsBackEnablePowerRunning\r\n");
                bridgePt->GetXBasePt()->mopLiftUp();
                bridgePt->xStation.SetRobotWorkingProcess(XStation::RobotRunningStatus_t::IsBackEnablePowerRunning);
                break;
            case XStation::RobotRunningStatus_t::IsCompleteFlowRunning:
            {
                xlog->Error("start running IsCompleteFlowRunning\r\n");
                StepMax = 10;
                CurStep = 0;
                StepDelayTicks[0]       = 30*50;//抽清水和清洁液并洗拖布
                CleanWaterPumpPower[0]  = 60;
                SewagePumpPower[0]      = 0;
                CleanLiquidPumpPower[0] = 100;
                DrainagePumpPower[0]    = 0;
                FanPower[0]             = 0;
                MopDuty[0]              = 80;
                DustCollection[0]       = 0;
                PtcModule[0]            = 0;

                StepDelayTicks[1]       = 20*50;//洗拖布
                CleanWaterPumpPower[1]  = 0;
                SewagePumpPower[1]      = 0;
                CleanLiquidPumpPower[1] = 0;
                DrainagePumpPower[1]    = 0;
                FanPower[1]             = 0;
                MopDuty[1]              = 80;
                DustCollection[1]       = 0;
                PtcModule[1]            = 0;

                StepDelayTicks[2]       = 20*50;//抽清水排污水洗拖布
                CleanWaterPumpPower[2]  = 60;
                SewagePumpPower[2]      = 80;
                CleanLiquidPumpPower[2] = 0; 
                DrainagePumpPower[2]    = 0;
                FanPower[2]             = 0;
                MopDuty[2]              = 80;
                DustCollection[2]       = 0;
                PtcModule[2]            = 0;

                StepDelayTicks[3]       = 35*50;//洗拖布
                CleanWaterPumpPower[3]  = 0;
                SewagePumpPower[3]      = 0;
                CleanLiquidPumpPower[3] = 0;
                DrainagePumpPower[3]    = 0;
                FanPower[3]             = 0;
                MopDuty[3]              = 100;
                DustCollection[3]       = 0;
                PtcModule[3]            = 0;

                StepDelayTicks[4]       = 120*50;//排污水洗拖布//改成125s
                CleanWaterPumpPower[4]  = 0;
                SewagePumpPower[4]      = 100;
                CleanLiquidPumpPower[4] = 0;
                DrainagePumpPower[4]    = 0;
                FanPower[4]             = 0;
                MopDuty[4]              = 100;
                DustCollection[4]       = 0;
                PtcModule[4]            = 0;

                StepDelayTicks[5]       = 20*50;//排污水风干拖布
                CleanWaterPumpPower[5]  = 0;
                SewagePumpPower[5]      = 0;
                CleanLiquidPumpPower[5] = 0;
                DrainagePumpPower[5]    = 100;//开始排水20s
                FanPower[5]             = 0;
                MopDuty[5]              = -40;
                DustCollection[5]       = 0;
                PtcModule[5]            = 0;


                StepDelayTicks[6]       = 4*50;//等待播报语音
                CleanWaterPumpPower[6]  = 0;
                SewagePumpPower[6]      = 0;
                CleanLiquidPumpPower[6] = 0;
                DrainagePumpPower[6]    = 0;
                FanPower[6]             = 0;
                MopDuty[6]              = 0;
                DustCollection[6]       = 0;
                PtcModule[6]            = 0;

                StepDelayTicks[7]       = 30*50;//集尘
                CleanWaterPumpPower[7]  = 0;
                SewagePumpPower[7]      = 0;
                CleanLiquidPumpPower[7] = 0;
                DrainagePumpPower[7]    = 0;
                FanPower[7]             = 0;
                MopDuty[7]              = -40;//集尘时候也让拖布上升
                DustCollection[7]       = 1;
                PtcModule[7]            = 0;

                StepDelayTicks[8]       = 5*50;//烘干
                CleanWaterPumpPower[8]  = 0;
                SewagePumpPower[8]      = 0;
                CleanLiquidPumpPower[8] = 0;
                DrainagePumpPower[8]    = 0;
                FanPower[8]             = 0;
                MopDuty[8]              = 0;
                DustCollection[8]       = 0;
                PtcModule[8]            = 0;

                StepDelayTicks[9]       = 2*60*60*50;//烘干
                StepDelayTicks[9]       = 10*50;//烘干
                CleanWaterPumpPower[9]  = 0;
                SewagePumpPower[9]      = 0;
                CleanLiquidPumpPower[9] = 0;
                DrainagePumpPower[9]    = 0;
                FanPower[9]             = 100;
                MopDuty[9]              = 0;
                DustCollection[9]       = 0;
                PtcModule[9]            = 1;
                bridgePt->xStation.SetRobotWorkingProcess(XStation::RobotRunningStatus_t::IsCompleteFlowRunning);
                break;
            }
        default:
                break;
        }
    }
    switch (LastRobotRunningStatus)
    {
        case XStation::RobotRunningStatus_t::none:
            BaseStationRunningNone();
            break;
        case XStation::RobotRunningStatus_t::IsDustRunning:
            BaseStationRunningDust();
            break;
        case XStation::RobotRunningStatus_t::IsMopClothRunning:
            BaseStationRunningMopCloth();
            break;
        case XStation::RobotRunningStatus_t::IsDryingRunning:
            BaseStationRunningDrying();
            break;
        case XStation::RobotRunningStatus_t::IsWetMopRunning:
            BaseStationRunningWetMop();
            break;
        case XStation::RobotRunningStatus_t::IsBackEnablePowerRunning:
            BaseStationRunningBackEnablePower();
            break;
        case XStation::RobotRunningStatus_t::IsCompleteFlowRunning:
            BaseStationRunningCompleteFlow();
            break;
      default:
            break;
    }
    
#endif
    bridgePt->GetXBasePt()->setRFPacket(bridgePt->xStation.RFDataToMCU, true);
}
void DeamonBaseStationTask::BaseStationRunningNone(void)
{
    static uint8_t idleticks = 0;
    if(idleticks++ > 50)
    {
        idleticks = 0;
    }
    bridgePt->xStation.RFDataToMCU.data.robot2station.RobotStatus.res = 0;
    BaseStationRunningReset();
}
void DeamonBaseStationTask::BaseStationRunningDust(void)
{
    if(CurStep < 0 || CurStep > 9){
        xlog->Error("CurStep = %d\n",CurStep);
    }
    if(StepDelayTicks[CurStep] >0)
    {
        StepDelayTicks[CurStep]--;

        switch(CurStep)
        {
            case 0:
            {
                bridgePt->xStation.RFDataToMCU.data.robot2station.RobotStatus.res = 1;
                if(bridgePt->xStation.GetDustBoxMissing())//集尘袋不在位
                {
                    DustCollection[0] = 0;
                }
                break;
            }
            default :
                break;
        }
        /**********************************************************************************************************************************/
        
        run_step_actions(CleanWaterPumpPower[CurStep],SewagePumpPower[CurStep],CleanLiquidPumpPower[CurStep],DrainagePumpPower[CurStep],FanPower[CurStep],DustCollection[CurStep],PtcModule[CurStep],MopDuty[CurStep]);
    }
    else
    {
        CurStep++;
        if(CurStep >= StepMax)
        {
            bridgePt->xStation.RFDataToMCU.data.robot2station.RobotStatus.res = 0;
            if(StepMax < 1){
                xlog->Error("StepMax = %d\n",StepMax);
            }
            CurStep = StepMax-1;
            bridgePt->xStation.SetRobotWorkingProcess(XStation::RobotRunningStatus_t::none);
        }
    }
}
void DeamonBaseStationTask::BaseStationRunningMopCloth(void)
{

    if(bridgePt->xStation.GetWaterInjectionModuleMissing())//不在位
    {
        StepDelayTicks[6] = 0;//跳过排污水的步骤
    }

    if(CurStep < 0 || CurStep > 9){
        xlog->Error("CurStep = %d\n",CurStep);
    }
    if(StepDelayTicks[CurStep] >0)
    {
        StepDelayTicks[CurStep]--;
        switch(CurStep)
        {
            case 0:
            {
                bridgePt->GetXBasePt()->mopLiftDown();
                if(bridgePt->xStation.GetCleanWaterBoxMissing() || bridgePt->xStation.GetSewageBoxMissing())//清水箱不在位或污水箱子不在位
                {
                    if(bridgePt->xStation.GetCleanWaterBoxMissing())
                    {
                        bridgePt->xStation.RFDataToMCU.data.robot2station.RobotStatus.res = 6;//清水箱不在位
                    }
                    else
                    {
                        bridgePt->xStation.RFDataToMCU.data.robot2station.RobotStatus.res = 7;//污水箱异常
                    }
                    CurStep = 6;
                }
                break;
            }
            case 1:
            {
                bridgePt->GetXBasePt()->mopLiftDown();
                break;
            }
            case 2:
            {
                bridgePt->GetXBasePt()->mopLiftDown();
                if(bridgePt->xStation.GetCleanWaterBoxMissing() || bridgePt->xStation.GetSewageBoxMissing())//清水箱不在位或污水箱不在位
                {
                    CurStep = 3;
                }
                break;
            }
            case 3:
            {
                 bridgePt->GetXBasePt()->mopLiftDown();
                break;
            }
            case 4:
            {
                 bridgePt->GetXBasePt()->mopLiftDown();
                if(bridgePt->xStation.GetSewageBoxMissing())
                {
                    CurStep = 6;
                }
                break;
            }
            case 5:
            {
                bridgePt->GetXBasePt()->mopLiftUp();
                break;
            }
            case 6:
            {
                bridgePt->GetXBasePt()->mopLiftUp();
                if(bridgePt->xStation.GetDustBoxMissing())
                {
                    CurStep = 7;
                }
                break;
            }
            case 7:
            {
                bridgePt->GetXBasePt()->mopLiftUp();
                break;
            }
            default :
                break;
        }
        /**********************************************************************************************************************************/
        run_step_actions(CleanWaterPumpPower[CurStep],SewagePumpPower[CurStep],CleanLiquidPumpPower[CurStep],DrainagePumpPower[CurStep],FanPower[CurStep],DustCollection[CurStep],PtcModule[CurStep],MopDuty[CurStep]);
    }
    else
    {
        CurStep++;
        if(CurStep >= StepMax)
        {
            bridgePt->xStation.RFDataToMCU.data.robot2station.RobotStatus.res = 0;
            if(StepMax < 1){
                xlog->Error("StepMax = %d\n",StepMax);
            }
            CurStep = StepMax-1;
            bridgePt->xStation.SetRobotWorkingProcess(XStation::RobotRunningStatus_t::none);
        }
    }
}
void DeamonBaseStationTask::BaseStationRunningDrying(void)
{
    if(CurStep < 0 || CurStep > 9){
        xlog->Error("CurStep = %d\n",CurStep);
    }
    if(StepDelayTicks[CurStep] >0)
    {
        StepDelayTicks[CurStep]--;
        switch(CurStep)
        {
            case 0:
            {
                bridgePt->xStation.RFDataToMCU.data.robot2station.RobotStatus.res = 3;
                break;
            }
            default :
                break;
        }
        /**********************************************************************************************************************************/
        run_step_actions(CleanWaterPumpPower[CurStep],SewagePumpPower[CurStep],CleanLiquidPumpPower[CurStep],DrainagePumpPower[CurStep],FanPower[CurStep],DustCollection[CurStep],PtcModule[CurStep],MopDuty[CurStep]);
    }
    else
    {
        CurStep++;
        if(CurStep >= StepMax)
        {
            bridgePt->xStation.RFDataToMCU.data.robot2station.RobotStatus.res = 0;
            if(StepMax < 1){
                xlog->Error("StepMax = %d\n",StepMax);
            }
            CurStep = StepMax-1;
            bridgePt->xStation.SetRobotWorkingProcess(XStation::RobotRunningStatus_t::none);
        }
    }
}
void DeamonBaseStationTask::BaseStationRunningWetMop(void)
{
    if(CurStep < 0 || CurStep > 9){
        xlog->Error("CurStep = %d\n",CurStep);
    }
    if(StepDelayTicks[CurStep] >0)
    {
        StepDelayTicks[CurStep]--;
        switch(CurStep)
        {
            case 0:
            {
                if(bridgePt->xStation.GetCleanWaterBoxMissing() || bridgePt->xStation.GetSewageBoxMissing())//清水箱不在位或污水箱子不在位
                {
                    if(bridgePt->xStation.GetCleanWaterBoxMissing())
                    {
                        bridgePt->xStation.RFDataToMCU.data.robot2station.RobotStatus.res = 6;//清水箱不在位
                    }
                    else
                    {
                        bridgePt->xStation.RFDataToMCU.data.robot2station.RobotStatus.res = 7;//污水箱异常
                    }
                    CurStep = 3;
                }
                break;
            }
            case 1:
            {
                if(bridgePt->xStation.GetSewageBoxMissing())//污水箱不在位
                {
                    CurStep = 3;
                }
                break;
            }
            case 2:
            {
                if(bridgePt->xStation.GetSewageBoxMissing())//污水箱不在位
                {
                    CurStep = 3;
                }
                break;
            }
            case 3:
            {
                if(bridgePt->xStation.GetSewageBoxMissing())//污水箱不在位
                {
                    CurStep = 4;
                }
                break;
            }
            case 4:
            {
                break;
            }
            default :
                break;
        }
        run_step_actions(CleanWaterPumpPower[CurStep],SewagePumpPower[CurStep],CleanLiquidPumpPower[CurStep],DrainagePumpPower[CurStep],FanPower[CurStep],DustCollection[CurStep],PtcModule[CurStep],MopDuty[CurStep]);
    }
    else
    {
        CurStep++;
        if(CurStep >= StepMax)
        {
            if(StepMax < 1){
                xlog->Error("StepMax = %d\n",StepMax);
            }
            CurStep = StepMax-1;
            bridgePt->xStation.RFDataToMCU.data.robot2station.RobotStatus.res = 0;
            bridgePt->xStation.SetRobotWorkingProcess(XStation::RobotRunningStatus_t::none);
        }
    }
}

void DeamonBaseStationTask::BaseStationRunningBackEnablePower(void)
{
    bridgePt->xStation.SetOpenHomeLed(true);
    bridgePt->xStation.RFDataToMCU.data.robot2station.RobotStatus.res = 4;
}
void DeamonBaseStationTask::BaseStationRunningCompleteFlow(void)
{
    if(bridgePt->getStationCleanStatus().bf.dry_auto == 0)
    {
        // xlog->Error("bridgePt->getStationCleanStatus().bf.dry_auto == 0");
        // StepDelayTicks[7] = 0;
        StepDelayTicks[8] = 0;
        StepDelayTicks[9] = 0;
    }
    if(bridgePt->getStationCleanStatus().bf.dust_collectoin_auto == 0)
    {
        StepDelayTicks[7] = 0;
    }
    if(bridgePt->xStation.GetWaterInjectionModuleMissing())//不在位
    {
        StepDelayTicks[5] = 0;//跳过排污水的步骤
    }
    if(CurStep < 0 || CurStep > 9){
        xlog->Error("CurStep = %d\n",CurStep);
    }
    if(StepDelayTicks[CurStep] >0)
    {
        StepDelayTicks[CurStep]--;

        switch(CurStep)
        {
            case 0:
            {
                bridgePt->GetXBasePt()->mopLiftDown();
                if(bridgePt->xStation.GetCleanWaterBoxMissing() || bridgePt->xStation.GetSewageBoxMissing())//清水箱不在位或污水箱子不在位
                {
                    if(bridgePt->xStation.GetCleanWaterBoxMissing())
                    {
                        bridgePt->xStation.RFDataToMCU.data.robot2station.RobotStatus.res = 6;//清水箱不在位
                    }
                    else
                    {
                        bridgePt->xStation.RFDataToMCU.data.robot2station.RobotStatus.res = 7;//污水箱异常
                    }
                    CurStep = 6;
                }
                bridgePt->xStation.RFDataToMCU.data.robot2station.RobotStatus.res = 2;
                break;
            }
            case 1:
            {
                bridgePt->GetXBasePt()->mopLiftDown();
                break;
            }
            case 2:
            {
                bridgePt->GetXBasePt()->mopLiftDown();
                if(bridgePt->xStation.GetCleanWaterBoxMissing() || bridgePt->xStation.GetSewageBoxMissing())//清水箱不在位或污水箱不在位
                {
                    CurStep = 3;
                }
                break;
            }
            case 3:
            {
                bridgePt->GetXBasePt()->mopLiftDown();
                break;
            }
            case 4:
            {
                bridgePt->GetXBasePt()->mopLiftDown();
                if(bridgePt->xStation.GetSewageBoxMissing())
                {
                    CurStep = 6;
                }
                break;
            }
            case 5:
            {
                bridgePt->GetXBasePt()->mopLiftUp();
                break;
            }
            case 6:
            {
                bridgePt->xStation.RFDataToMCU.data.robot2station.RobotStatus.res = 1;
                break;
            }
            case 7:
            {
                bridgePt->GetXBasePt()->mopLiftUp();
                if(bridgePt->xStation.GetDustBoxMissing())
                {
                    CurStep = 8;
                }
                break;
            }
            case 8:
            {
                bridgePt->GetXBasePt()->mopLiftUp();
                bridgePt->xStation.RFDataToMCU.data.robot2station.RobotStatus.res = 3;
                bridgePt->xStation.SetRobotWorkingProcess(XStation::RobotRunningStatus_t::IsDryingRunning);
                break;
            }
            case 9:
            {
                bridgePt->GetXBasePt()->mopLiftUp();
                bridgePt->xStation.RFDataToMCU.data.robot2station.RobotStatus.res = 0;
                break;
            }
            default :
                break;
        }
        /**********************************************************************************************************************************/
        run_step_actions(CleanWaterPumpPower[CurStep],SewagePumpPower[CurStep],CleanLiquidPumpPower[CurStep],DrainagePumpPower[CurStep],FanPower[CurStep],DustCollection[CurStep],PtcModule[CurStep],MopDuty[CurStep]);
    }
    else
    {
        CurStep++;
        if(CurStep >= StepMax)
        {
            bridgePt->xStation.RFDataToMCU.data.robot2station.RobotStatus.res = 0;
            if(StepMax < 1){
                xlog->Error("StepMax = %d\n",StepMax);
            }
            CurStep = StepMax-1;
            bridgePt->xStation.SetRobotWorkingProcess(XStation::RobotRunningStatus_t::none);
        }
    }
}
void DeamonBaseStationTask::run_step_actions(uint8_t CleanWaterPump,uint8_t SewagePump,uint8_t CleanLiquidPump,uint8_t DrainagePump,uint8_t Fan,uint8_t DustCollection,uint8_t PtcModule,int16_t MopDuty)
{
    bridgePt->xStation.SetCleanWaterPumpPower(CleanWaterPump);
    bridgePt->xStation.SetSewagePumpPower(SewagePump);

    if(bridgePt->getStationCleanStatus().bf.fluid_auto == 0)
    {
        bridgePt->xStation.SetCleanLiquidPumpPower(0);
    }
    else
    {
        bridgePt->xStation.SetCleanLiquidPumpPower(CleanLiquidPump);
    }
    bridgePt->xStation.SetDrainagePumpPower(DrainagePump);
    bridgePt->xStation.SetFanPower(Fan);
    bridgePt->xStation.SetOpenDustCollection(DustCollection);
    bridgePt->xStation.SetEnablePtcModule(PtcModule);
    bridgePt->GetXBasePt()->setMopDuty(MopDuty);
}
void DeamonBaseStationTask::BaseStationRunningReset(void)
{
    bridgePt->xStation.SetCleanLiquidPumpPower(0);
    bridgePt->xStation.SetCleanWaterPumpPower(0);

    bridgePt->xStation.SetDrainagePumpPower(0);
    bridgePt->xStation.SetFanPower(0);
    
    bridgePt->xStation.SetSewagePumpPower(0);

    bridgePt->xStation.SetEnablePtcModule(false);
    bridgePt->xStation.SetOpenDustCollection(false);
    bridgePt->xStation.SetOpenHomeLed(false);
    bridgePt->xStation.SetOpenWaterInjectioinValve(false);

    // bridgePt->xStation.RFDataToMCU.data.robot2station.RobotStatus.res = 0;
}
void DeamonBaseStationTask::PreSuspend()
{
    printf("TestTask Presuspend called!\r\n");
}
// must be implemented by any new task
BaseTask_t *CreatClass(NavBridge_t *navbridgePt, std::string name)
{
    return new DeamonBaseStationTask(navbridgePt, name);
}
