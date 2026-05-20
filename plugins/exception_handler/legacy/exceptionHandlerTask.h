/*
 * @Description:
 * @version:
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2022-04-14 11:29:17
 * @LastEditors: jephy jephy.zhang@any-eye.com
 * @LastEditTime: 2023-07-31 16:52:40
 */

#ifndef __EXCEPTIONHANDLER_TASK_H__
#define __EXCEPTIONHANDLER_TASK_H__

#include "baseTask.h"
#include "Msg/NavMsg/InitialPoseCmd_t.hpp"
#include "Msg/RobotMsg/HackCmd_t.hpp"
#include <functional>
#include "ExceptionHandlerTask_version.h"
#include "Speaker/speakerService.h"

typedef std::function<bool(void)> ExceptionHandlerCb_t;
#define REGISTER_EXCEPTION_HANDLER(HandlerPool, exceptType, HandlerFunc) HandlerPool.emplace(exceptType, std::bind(&ExceptionHandlerTask_t::HandlerFunc, this))

enum class ExceptionType_t
{
    BothWheelLiftUp = 1, // p1
    Unbalance,           // p2
    CliffStuck,          // p3
    BumperStuck,         // p4
    WheelStuck,          // p5
    MainBrushStuck,      // p6
    SideBrushStuck,      // p7
    WheelSlide,          // p8
    None,                // p9
    LostLocation,
    MopAbsent,
    DustBoxMiss,
    Error,
          
};

enum class BumperStuckHandleStatus_t
{
    OnBackOff,
    OnClockRotation,
    OnAntiClockRotation,
    OnDone,
};

enum class WheelStuckHandlerStatus_t
{
    OnBackOff,
    OnAdvance,
    OnClockRotation,
    OnAntiClockRotation,
    OnLeftRotation,
    OnRightRotation,
    OnLoop,
    OnDone,
};

enum class MainBrushStuckHandleStatus_t
{
    OnRotation,
    OnRotation2,
    Onloop,
    OnDone,
};

enum class SideBrushStuckHandleStatus_t
{
    OnRotation,
    OnRotation2,
    Onloop,
    OnDone,
};

enum class WheelSlideHandleStatus_t
{
    OnBackOff,
    OnAdvance,
    OnClockRotation,
    OnAntiArcRush,
    OnAntiClockRotation,
    OnArcRush,
    OnCurveBackRush,
    OnCurveBackRush2,
    OnDone,
};

class ExceptionHandlerTask_t : public BaseTask_t
{
private:
    XLog *xlog;

    //task status
    ExceptionType_t curExceptionType;
    ExceptionType_t lastExceptionType;
    bool isExceptionhandleDone;
    bool isRelocalizaitonEnable;
    ExceptionType_t handlerFinishResult;

    // mainloop
    void updateExceptionType();
    void updateHandler();
    ExceptionHandlerCb_t* handlerPtr;

    //handler action status
    
    WheelSlideHandleStatus_t wheelSlideHandlerStatus;
    void resetHandler();    

    // handler parameters
    std::map<ExceptionType_t, ExceptionHandlerCb_t> handlerPool;

    bool bothWheelsLiftUpHandle();
    bool bumperStuckHandle();
    int bumpHandleLoopCount;
    int bumperStuckhandleTick;
    BumperStuckHandleStatus_t bumperHandlerStatus;
    bool wheelStuckHandle();
    int wheelStuckLoopCount;
    int wheelStuckhandleTick;
    WheelStuckHandlerStatus_t wheelStuckHandlerStatus;
    bool mainBrushStuckHandle();
    int mainBrushLoopCount;
    int mainBrushHandleTick;
    MainBrushStuckHandleStatus_t mainBrushStuckHandleStatus;
    bool sideBrushStuckHandle();
    int sideBrushLoopCount;
    int sideBrushHandleTick;
    SideBrushStuckHandleStatus_t sideBrushStuckHandleStatus;
    bool noneExceptionHandle();

    // action
    bool wheelAction(float v, float w, int &taskTick, uint8_t tick);

    // for test
    bool setFixedExceptionType;
    ExceptionType_t fixedExceptionType;

    //basetask
    void PreStart();
    void PreStop();
    void PreResume();
    void MainLoop();
    void PreSuspend();

public:
    ExceptionHandlerTask_t(NavBridge_t *navBridge, std::string name);
    ~ExceptionHandlerTask_t();

    void SetExceptionType(ExceptionType_t type);
    ExceptionType_t GetExceptionType();

    void DisableRelocalization();
    void EnableRelocalization();

    ExceptionType_t GetExceptionHandleResult();
};

#endif
