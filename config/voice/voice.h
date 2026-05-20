#ifndef     __ROBOT_VOICE_H__
#define     __ROBOT_VOICE_H__


enum class AudioName_t
{
    BackToCharge = 1,
    Charging,       //2
    StopLocalClean, //3
    StopRemoteCtrl, //4
    ShutDown,       //5
    ChargingAndUpgradeDone, //6
    ZoneCleanDoneAndBackToCharge,   //7
    IamHere,    //8
    GoToTargetPositionFailed,   //9
    TargetPositionArrived,      //10
    LocalClean,                 //11
    LostposeAndSearchCharger,   //12
    LocalizatingAndWait,        //13
    LostposeAndStartNewClean,   //14
    StartRemoteCtrl,            //15
    StartGoToTargetPos,         //16
    StartZoneClean,             //17
    StartSelectRoomClean,       //18
    StartClean,                 //19
    Pause,                      //20
    ResumeZoneClean,            //21
    CantFindChargerAndPutRobotOnCharger,    //22
    CleanDoneAndBackToCharge,   //23
    CleanDone,                  //24
    ResumeBackToCharge,         //25
    ResumeClean,                //26
    StartUpgradeAndBusy,        //27
    SelectZoneCleanDoneAndBackToCharge, //28
    UpgradeWarningInfo,         //29
    ResumeSelectRoomClean,      //30
    StopClean,                  //31
    ZoneCleanDone,              //32
    StopSelectRoomClean,        //33
    StopGoToPose,               //34
    StopZoneClean,              //35
    StartLocalClean,            //36
    LocalCleanDone,             //37
    SelectRoomCleanDone,        //38
    StartGoToPose,              //39
    ResumeLocalClean,           //40
    PowerOn,                    //41
    LowPower,                   //42
    DustBoxMissing,             //43
    DustBoxReady,               //44
    DeviceError,                //45
    CheckWaterbox,              //46
    OnMapping,                  //47
    SmartCleanDoneAndBackToCharge, //48
    StartBackToWash,            //  49
    ContinueToWash,             //  50
    StartToColletDust,          //  51
    PleaseMoveOutOfStationToPoweroff,   //  52
    PoseTrackedAndStartClean,       //  53
    MopCleanedAndBackToWash,        //  54
    CleanIsDoneAndBackToWash,       //  55
    CleanIsDoneAndBackToCharge,     //  56
    StartWashing,               //  57
    StopWashing,                //  58
    CliffStuck,                 //  59
    UnBalance,                  //  60
    WheelLiftUp,                //  61
    BumperStuck,                //  62
    SideBrushStuck,             //  63
    MainBrushStuck,             //  64
    WheelStuck,                 //  65
    WheelSlide,                 //  66
    LostLocation,               //  67
    MopLeft,                    //  68
    MopBack,                    //  69
    MopReplace,                 //  70
    StartBackwash,              //  71
    ReturnToClean,              //  72
    PauseRecharge,              //  73
    PauseBackwash,              //  74
    StartQuickMapCreation,      //  75
    UnableToCharge,             //  76
    PalletNotInPlace,           //  77
    TurnOffThePowerAndTryAgain, //  78
    MapReset,                   //  79
    WashingAndCharge,           //  80
    GetLogSuccess,              //  81
    RechargeFailedReturnTrayToStation,      // 82   
    UnableChargeTurnOnPowerSwitch,          // 83
    ReinstallMmopAndTryAgain,               // 84
    TrayStuckRemoveTrayAndCleanUp,          // 85
    MainbrushLiftingFailedPleaseCheck,      // 86
    MainbrushDescentFailedPleaseCheck,      // 87
    EnableSingleSweepMode,                  // 88
    EnableSingleDragMode,                   // 89
    EnableSweepAndDraggMode,                // 90
    TrayHasBeenReinstalled,                 // 91
    TrayHasBeenRemoved,                     // 92
    WiFiResetAndPairMode,                   // 93
    WiFiPairOkAndExitPairMode,              // 94
    ContinueMapping,                        // 95
    PositioningFailed,                      // 96
    HiHiHi,                                 // 97
    WheelTemperatureTooHigh,                // 98
    BatteryTooLowPleaseCharging,            // 99
    BatteryAbnormal,                        // 100
    LaserabnormalPleaseCheck,               // 101
    CommunicationAbnormalitiesCheckPower,   // 102
    LaserAbnormalityCheckWire,              // 103
    SuccessfullyOvercomeDifficulties,       // 104
    MachineTrapped,                         // 105
    ContinuePositioning,                    // 106

    ImuFault,                               // 107
    GyroFault,                              // 108
    TotalNum,                   
};

#endif