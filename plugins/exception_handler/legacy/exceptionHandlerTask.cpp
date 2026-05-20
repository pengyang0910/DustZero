/*
 * @Description:
 * @version:
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2022-04-14 11:29:23
 * @LastEditors: jephy jephy.zhang@any-eye.com
 * @LastEditTime: 2023-07-31 16:45:09
 */

#include "exceptionHandlerTask.h"

ExceptionHandlerTask_t::ExceptionHandlerTask_t(NavBridge_t *navBridge, std::string name) : BaseTask_t(navBridge, name)
{
    bool enLog = bridgePt->GetRobotCfg().GetProperty("ExceptionTask", "enLog_b").as<bool>();
    int logLevel = bridgePt->GetRobotCfg().GetProperty("ExceptionTask", "logLevel_i").as<int>();
    if (enLog)
        xlog = new XLog(true);
    else
        xlog = new XLog(false);

    xlog->Initialise("ExceptionHandlerTask.log");
    xlog->SetThreshold(Type(logLevel));

    mainVersionInfo = XT212_DEPENDENCE_VERSION;
    subVersionInfo = EXCEPTIONHANDLERTASK_VERSION;
    xlog->Debug("ExceptionTask  mainVersion is %s ", mainVersionInfo.c_str());
    xlog->Debug("ExceptionTask subVersion is %s!", subVersionInfo.c_str());

    REGISTER_EXCEPTION_HANDLER(handlerPool, ExceptionType_t::BothWheelLiftUp, bothWheelsLiftUpHandle); 
    REGISTER_EXCEPTION_HANDLER(handlerPool, ExceptionType_t::BumperStuck, bumperStuckHandle);       
    REGISTER_EXCEPTION_HANDLER(handlerPool, ExceptionType_t::WheelStuck, wheelStuckHandle);         
    REGISTER_EXCEPTION_HANDLER(handlerPool, ExceptionType_t::MainBrushStuck, mainBrushStuckHandle); 
    REGISTER_EXCEPTION_HANDLER(handlerPool, ExceptionType_t::SideBrushStuck, sideBrushStuckHandle); 
    REGISTER_EXCEPTION_HANDLER(handlerPool, ExceptionType_t::None, noneExceptionHandle);         
}

ExceptionHandlerTask_t::~ExceptionHandlerTask_t()
{
    if(xlog != NULL)
        delete xlog;
    xlog = NULL;
}

void ExceptionHandlerTask_t::PreStart()
{
    xlog->Info("ExceptionHandlerTask_t::PreStart");
    bridgePt->GetXBasePt()->MotorEnable(true);
    bridgePt->GetXBasePt()->bumperAutoHandle(false);//set bumpoer
    isRelocalizaitonEnable = false;    
    lastExceptionType = ExceptionType_t::None;
    isExceptionhandleDone=false;

    setFixedExceptionType=false;
    fixedExceptionType=ExceptionType_t::None;
    handlerFinishResult=ExceptionType_t::None;
    curExceptionType = ExceptionType_t::None;
}

void ExceptionHandlerTask_t::PreStop()
{

}

void ExceptionHandlerTask_t::PreResume()
{

}

void ExceptionHandlerTask_t::PreSuspend()
{
    printf("TestTask Presuspend called!\r\n");
}

void ExceptionHandlerTask_t::MainLoop()
{
    if(curExceptionType == ExceptionType_t::Error)
    {
        xlog->Debug("current exception type:%d,handler result:%d",int(curExceptionType),int(handlerFinishResult));
        Stop();
        return;
    }    
    updateExceptionType();
    updateHandler();
}

void ExceptionHandlerTask_t::updateExceptionType()
{
    //sort exception type priority
    RobotHardwareErrorCode_t RobotHardwareErrorCode=bridgePt->GetXBasePt()->GetErrorCode();
    if(RobotHardwareErrorCode.bf.BothWheelLiftUp ==1) 
    {
        curExceptionType=ExceptionType_t::BothWheelLiftUp;
    }
    else if(RobotHardwareErrorCode.bf.BumperStuck ==1) 
    {
        curExceptionType=ExceptionType_t::BumperStuck;
    }
    else if(RobotHardwareErrorCode.bf.WheelStuck ==1) 
    {
        curExceptionType=ExceptionType_t::WheelStuck;
    }
    else if(RobotHardwareErrorCode.bf.MainBrushStuck ==1) 
    {
        curExceptionType=ExceptionType_t::MainBrushStuck;
    }
    else if(RobotHardwareErrorCode.bf.SideBrushStuck ==1) 
    {
        curExceptionType=ExceptionType_t::SideBrushStuck;
    }
    else
    {
        curExceptionType=ExceptionType_t::None;
        xlog->Debug("exception recover,current:%d,last:%d",int(curExceptionType),int(lastExceptionType));
    }
    // if (setFixedExceptionType)
    // {
    //     curExceptionType=fixedExceptionType;
    // }
    if(curExceptionType == ExceptionType_t::None && lastExceptionType != ExceptionType_t::None)
    {
        xlog->Debug("exception handle success,exception type:%d",int(lastExceptionType));
        curExceptionType=ExceptionType_t::Error;
        handlerFinishResult=ExceptionType_t::None;
        isExceptionhandleDone=true;
        return;
    }
    if(curExceptionType != lastExceptionType)
    {
        xlog->Debug("exception type changed,current:%d,last:%d",int(curExceptionType),int(lastExceptionType));
        resetHandler();
    }
    lastExceptionType=curExceptionType;
}

void ExceptionHandlerTask_t::updateHandler()
{
    if(isExceptionhandleDone)
    {
        return;
    }
    int isHandlerDone=false;
    std::map<ExceptionType_t, ExceptionHandlerCb_t>::iterator handlerPoolIter=handlerPool.find(curExceptionType);
    if(handlerPoolIter == handlerPool.end())
    {
        xlog->Error("can not find exception type handler,exception type:%d",int(curExceptionType));
        isHandlerDone=handlerPool[ExceptionType_t::None]();
    }
    else
    {
        isHandlerDone=handlerPool[curExceptionType]();
    }
    if(!isHandlerDone)
    {
        //xlog->Debug("handling");
    }
    else
    {
        resetHandler();
        isExceptionhandleDone=true;
        if(curExceptionType != ExceptionType_t::None)
        {
            // handle fail
            xlog->Error("handler process done,but exception still exists,exception type:%d",int(curExceptionType));
            handlerFinishResult=curExceptionType;
        }
        else
        {
            xlog->Error("exception type error!");
        }
        curExceptionType=ExceptionType_t::Error;
    }
}

void ExceptionHandlerTask_t::resetHandler()
{
    // bumper
    bumpHandleLoopCount=0;
    bumperStuckhandleTick=0;
    bumperHandlerStatus = BumperStuckHandleStatus_t::OnBackOff;

    // wheelStuck
    wheelStuckLoopCount = 0;
    wheelStuckhandleTick = 0;
    wheelStuckHandlerStatus=WheelStuckHandlerStatus_t::OnBackOff;

    // mainBrushStuck
    mainBrushLoopCount=0;
    mainBrushHandleTick = 0;
    mainBrushStuckHandleStatus=MainBrushStuckHandleStatus_t::OnRotation;

    // side brush stuck
    sideBrushLoopCount=0;
    sideBrushHandleTick=0;
    sideBrushStuckHandleStatus=SideBrushStuckHandleStatus_t::OnRotation;
}

bool ExceptionHandlerTask_t::bothWheelsLiftUpHandle()
{
    return true;
}

bool ExceptionHandlerTask_t::bumperStuckHandle()
{
    bool ret = false;
    if (bumpHandleLoopCount < 2)
    {
        switch (bumperHandlerStatus)
        {
        case BumperStuckHandleStatus_t::OnBackOff:
        {
            // xlog->Info("antiRotation OnBackOff: bumperBackOffTick= %d", bumperBackOffTick);
            if (wheelAction(-0.1, 0, bumperStuckhandleTick, 25))
            {
                bumperStuckhandleTick=0;
                if(bumpHandleLoopCount%2 == 0)
                {
                    bumperHandlerStatus = BumperStuckHandleStatus_t::OnClockRotation;
                }
                else
                {
                    bumperHandlerStatus = BumperStuckHandleStatus_t::OnAntiClockRotation;
                }
            }
            else
            {
                ;
            }
        }
        break;
        case BumperStuckHandleStatus_t::OnClockRotation:
        {
            // xlog->Info("antiRotation OnClockRotation: bumperClockRotationTick= %d", bumperClockRotationTick);
            if (wheelAction(0, 90, bumperStuckhandleTick, 200))
            {
                bumperStuckhandleTick=0;
                if(bumpHandleLoopCount%2 == 0)
                {
                    bumperHandlerStatus = BumperStuckHandleStatus_t::OnAntiClockRotation;
                }
                else
                {
                    bumperHandlerStatus = BumperStuckHandleStatus_t::OnDone;                    
                }
            }
            else
            {
                ;
            }
        }
        break;
        case BumperStuckHandleStatus_t::OnAntiClockRotation:
        {
            // xlog->Info("antiRotation OnAntiClockRotation: bumperAntiClockRotationTick= %d", bumperAntiClockRotationTick);
            if (wheelAction(0, -90, bumperStuckhandleTick, 200))
            {
                bumperStuckhandleTick=0;
                if(bumpHandleLoopCount%2 == 0)
                {
                    bumperHandlerStatus = BumperStuckHandleStatus_t::OnDone;
                }
                else
                {
                    bumperHandlerStatus = BumperStuckHandleStatus_t::OnClockRotation;                    
                }                
            }
            else
            {
                ;
            }
        }
        break;
        case BumperStuckHandleStatus_t::OnDone:
        {
            bumperStuckhandleTick=0;
            bumperHandlerStatus = BumperStuckHandleStatus_t::OnBackOff;            
            bumpHandleLoopCount+=1;
        }
        break;
        default:
            break;
        }
    }
    else
    {
        xlog->Error("bumper stuck handle done");
        ret=true;
    }
    return ret;
}

bool ExceptionHandlerTask_t::wheelStuckHandle()
{
    bool ret = false;
    switch (wheelStuckHandlerStatus)
    {
    case WheelStuckHandlerStatus_t::OnBackOff:
    {
        // xlog->Info("wheelStuckHandle OnBackOff: wheelStuckhandleTick= %d", wheelStuckhandleTick);
        if (wheelAction(-0.3, 0, wheelStuckhandleTick, 30))
        {
            wheelStuckhandleTick=0;
            wheelStuckHandlerStatus = WheelStuckHandlerStatus_t::OnAdvance;
        }
        else
        {
            ;
        }
    }
    break;
    case WheelStuckHandlerStatus_t::OnAdvance:
    {
        // xlog->Info("wheelStuckHandle OnAdvance: wheelStuckhandleTick= %d", wheelStuckhandleTick);
        if (wheelAction(0.3, 0, wheelStuckhandleTick, 20))
        {
            wheelStuckhandleTick=0;
            wheelStuckHandlerStatus = WheelStuckHandlerStatus_t::OnClockRotation;
        }
        else
        {
            ;
        }
    }
    break;
    case WheelStuckHandlerStatus_t::OnClockRotation:
    {
        // xlog->Info("wheelStuckHandle OnClockRotation: wheelStuckhandleTick= %d", wheelStuckhandleTick);
        if (wheelAction(0, -90, wheelStuckhandleTick, 100))
        {
            wheelStuckhandleTick=0;
            wheelStuckHandlerStatus = WheelStuckHandlerStatus_t::OnAntiClockRotation;
        }
        else
        {
            ;
        }
    }
    break;
    case WheelStuckHandlerStatus_t::OnAntiClockRotation:
    {
        // xlog->Info("wheelStuckHandle OnAntiClockRotation: wheelStuckhandleTick= %d", wheelStuckhandleTick);
        if (wheelAction(0, 90, wheelStuckhandleTick, 100))
        {
            wheelStuckhandleTick=0;
            wheelStuckHandlerStatus = WheelStuckHandlerStatus_t::OnLeftRotation;
        }
        else
        {
            ;
        }
    }
    break;
    case WheelStuckHandlerStatus_t::OnLeftRotation:
    {
        // xlog->Info("wheelStuckHandle OnLeftRotation: wheelStuckhandleTick= %d", wheelStuckhandleTick);
        if (wheelAction(0.3, -90, wheelStuckhandleTick, 100))
        {
            wheelStuckhandleTick=0;
            wheelStuckHandlerStatus = WheelStuckHandlerStatus_t::OnRightRotation;
        }
        else
        {
            ;
        }
    }
    break;
    case WheelStuckHandlerStatus_t::OnRightRotation:
    {
        // xlog->Info("wheelStuckHandle OnRightRotation: wheelStuckhandleTick= %d", wheelStuckhandleTick);
        if (wheelAction(-0.3, -90, wheelStuckhandleTick, 100))
        {
            wheelStuckhandleTick=0;
            wheelStuckHandlerStatus = WheelStuckHandlerStatus_t::OnLoop;
            wheelStuckLoopCount+=1;
        }
        else
        {
            ;
        }
    }
    break;
    case WheelStuckHandlerStatus_t::OnLoop:
    {
        if (wheelStuckLoopCount < 3)
        {
            wheelStuckHandlerStatus = WheelStuckHandlerStatus_t::OnBackOff;
        }
        else
        {
            wheelStuckHandlerStatus = WheelStuckHandlerStatus_t::OnDone;
        }
    }
    break;
    case WheelStuckHandlerStatus_t::OnDone:
    {
        xlog->Error("wheelStuck handle done");
        ret = true;
    }
    break;
    default:
    {

    }
    break;
    }
    return ret;
}

bool ExceptionHandlerTask_t::mainBrushStuckHandle()
{
    bool ret = false;
    switch (mainBrushStuckHandleStatus)
    {
    case MainBrushStuckHandleStatus_t::OnRotation:
    {
        // xlog->Info("mainBrushStuckHandle OnRotation: mainBrushRotationTick= %d", mainBrushRotationTick);
        bridgePt->GetXBasePt()->setMainBrushDuty(60);
        if (wheelAction(0.1, 90, mainBrushHandleTick, 200))
        {
            mainBrushHandleTick=0;
            mainBrushStuckHandleStatus = MainBrushStuckHandleStatus_t::OnRotation2;
        }
        else
        {
            
        }
    }
    break;
    case MainBrushStuckHandleStatus_t::OnRotation2:
    {
        // xlog->Info("mainBrushStuckHandle OnRotation2: mainBrushRotationTick= %d", mainBrushRotationTick);
        bridgePt->GetXBasePt()->setMainBrushDuty(-60);
        if (wheelAction(-0.1, -90, mainBrushHandleTick, 200))
        {
            mainBrushHandleTick=0;
            mainBrushStuckHandleStatus = MainBrushStuckHandleStatus_t::Onloop;
            mainBrushLoopCount+=1;
        }
        else
        {
            
        }
    }
    break;
    case MainBrushStuckHandleStatus_t::Onloop:
    {
        if (mainBrushLoopCount < 7)
        {
            mainBrushStuckHandleStatus = MainBrushStuckHandleStatus_t::OnRotation;
        }
        else
        {
            mainBrushStuckHandleStatus = MainBrushStuckHandleStatus_t::OnDone;
        }
    }
    break;
    case MainBrushStuckHandleStatus_t::OnDone:
    {
        // xlog->Info("mainBrushStuckHandle OnDone");
        bridgePt->GetXBasePt()->setMainBrushDuty(0);
        ret = true;
    }
    break;
    default:
        break;
    }
    return ret;
}

bool ExceptionHandlerTask_t::sideBrushStuckHandle()
{
    bool ret = false;
    switch (sideBrushStuckHandleStatus)
    {
    case SideBrushStuckHandleStatus_t::OnRotation:
    {
        bridgePt->GetXBasePt()->setLeftBrushDuty(60);
        bridgePt->GetXBasePt()->setRighBrushDuty(60);
        if (wheelAction(0.1, 90, sideBrushHandleTick, 200))
        {
            sideBrushHandleTick=0;
            sideBrushStuckHandleStatus = SideBrushStuckHandleStatus_t::OnRotation2;
        }
        else
        {
            
        }
    }
    break;
    case SideBrushStuckHandleStatus_t::OnRotation2:
    {
        bridgePt->GetXBasePt()->setLeftBrushDuty(-60);
        bridgePt->GetXBasePt()->setRighBrushDuty(-60);
        if (wheelAction(-0.1, -90, sideBrushHandleTick, 200))
        {
            sideBrushHandleTick=0;
            sideBrushStuckHandleStatus = SideBrushStuckHandleStatus_t::Onloop;
            sideBrushLoopCount+=1;
        }
        else
        {
            
        }
    }
    break;
    case SideBrushStuckHandleStatus_t::Onloop:
    {
        if (sideBrushLoopCount < 7)
        {
            sideBrushStuckHandleStatus = SideBrushStuckHandleStatus_t::OnRotation;
        }
        else
        {
            sideBrushStuckHandleStatus = SideBrushStuckHandleStatus_t::OnDone;
        }
    }
    break;
    case SideBrushStuckHandleStatus_t::OnDone:
    {
        bridgePt->GetXBasePt()->setRighBrushDuty(0);
        ret = true;
    }
    break;
    default:
        break;
    }
    return ret;
}

bool ExceptionHandlerTask_t::noneExceptionHandle()
{
    return true;
}

// action
bool ExceptionHandlerTask_t::wheelAction(float v, float w, int &taskTick, uint8_t tick)
{
    w = dtor(w);
    if (taskTick < tick)
    {
        bridgePt->GetXBasePt()->setSpeed(v, w);
        taskTick = taskTick + 1;
    }
    else
    {
        taskTick = 0;
        return true;
    }
    return false;
}

// public
void ExceptionHandlerTask_t::SetExceptionType(ExceptionType_t type)
{
    fixedExceptionType = type;
    setFixedExceptionType = true;
    xlog->Debug("set exception type:%d",int(type));
}

ExceptionType_t ExceptionHandlerTask_t::GetExceptionType()
{
    return curExceptionType;
}

void ExceptionHandlerTask_t::DisableRelocalization()
{
    isRelocalizaitonEnable = false;
}

void ExceptionHandlerTask_t::EnableRelocalization()
{
    isRelocalizaitonEnable = true;
}

ExceptionType_t ExceptionHandlerTask_t::GetExceptionHandleResult()
{
    return handlerFinishResult;
}

BaseTask_t *CreatClass(NavBridge_t *navbridgePt, std::string name)
{
    return new ExceptionHandlerTask_t(navbridgePt, name);
}