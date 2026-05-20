#ifndef     __TESTZHB_H__
#define     __TESTZHB_H__


#include "stdio.h"
#include "unistd.h"
#include <lcm/lcm-cpp.hpp>
#include "sys/fcntl.h"
#include "XCfg/xini.h"
#include "XLog/xlog.h"
#include "global.h"
#include "NavUtils.h"
#include "Msg/NavMsg/GridMapInfo_t.hpp"
#include "Msg/NavMsg/VelCmd_t.hpp"
#include "Msg/RobotMsg/RobotStatus_t.hpp"
#include "Msg/RobotMsg/MotorDutyCmd_t.hpp"
#include "Msg/RobotMsg/HairCutCmd_t.hpp"
#include "Msg/RobotMsg/BumperForSlam_t.hpp"
#include "Msg/RobotMsg/Power_t.hpp"
#include "Msg/SensorMsg/Bumper_t.hpp"
#include "Msg/SensorMsg/Cliff_t.hpp"
#include "Msg/SensorMsg/LaserData_t.hpp"
#include "Msg/NavMsg/Odom_t.hpp"
#include "Msg/RobotMsg/Power_t.hpp"
#include "Msg/SensorMsg/WallSensor_t.hpp"
#include "Msg/NavMsg/VelCmd_t.hpp"
#include "Msg/NavMsg/Odom_t.hpp"
#include "Msg/SensorMsg/Dock_t.hpp"
#include "Msg/AppMsg/CleanCmd_t.hpp"
#include "Msg/AppMsg/DutyCmd_t.hpp"
#include "Msg/AppMsg/SpdCmd_t.hpp"
#include "Msg/NavMsg/PoseArray_t.hpp"
#include "Msg/NavMsg/OriginTimer_t.hpp"
#include "Msg/NavMsg/SlamPoseInfo_t.hpp"
#include "Msg/SensorMsg/PointCloud_t.hpp"
#include "Msg/SensorMsg/ImuData_t.hpp"

#include "XBase/xbase.h"
#include "iomanip"

enum class TestMode_t
{
    none,
    case1,
    case2,
    case3,
    case4,
    case5,
    test1,
    test2,
    test3,
    test4_mBrush,
    totalCaseCount,
};

using caseFunPtr = std::function<void(void)>;
using testModeToCaseMap_t = std::map<TestMode_t,caseFunPtr>;

enum class RobKeyCodeTest_t
{
    PowerKeyShortPressed = 0,
    PowerKeyLongPressed = 1,
    FnKeyShortPressed = 2,
    FnKeyLongPressed = 3,
    FnPowerCombinedLongPressed = 4,
    None,
};

class testZhb_t
{
private:
    XLog* xlogPtr;
    XBase_t* xbasePtr;
    lcm::LCM* lcmPtr;
    ini::IniFile* robotCfgPtr;
    ini::IniFile* platformCfgPtr; 

    testModeToCaseMap_t testModeToCaseInitMap,testModeToCaseHandleMap;

    //case2     

public:
    testZhb_t();
    ~testZhb_t();

    XLog* getXlogPtr();

    void init(TestMode_t testMode);
    void handle(TestMode_t testMode);

    void case1Init();
    void case2Init();
    void case3Init();
    void case4Init();
    void case5Init();
    void test1Init();
    void test2Init();
    void test3Init();    
    void test4Init();

    void case1Handle();        
    void case2Handle();
    void case3Handle();
    void case4Handle();
    void case5Handle();
    void test1Handle();   
    void test2Handle();
    void test3Handle();
    void test4Handle();
    //case1
    void setLed1(uint16_t& writeKeyData,bool ledROn,bool ledBOn,bool ledYOn);    
    int fd;
    uint16_t writeKeyData;     
    bool ledROn;
    bool ledBOn;
    bool ledYOn;
    int8_t lastBumpStatus;
    std::vector<int> cliffInput;
    std::vector<int> cliffCount;
    int cliffTriggerThreshold;
    float cliffBackDistance;
    float cliffRotateSum;
    NavPose lastCliffTriggerPositionBack;
    NavPose lastCliffTriggerPositionRotation;
    bool cliffbackMode;//cliff back mode(1),or cliff rotate mode(-1)
    //error case:1 cliff trigger count > 3,2 trigger both front and back cliff,3 back distance > 0.5,4 rotate > 2*pi
    bool rotateAfterMove;
    float a1StartTime;
    int8_t* cliffTrigger;
    

    //case2
    RobKeyCodeTest_t getKeyboard();
    uint8_t readKeyData;
    bool case2Start;
    bool case2KeyPressed;

    //case3
    bool case3Start;
    bool case3KeyStart;
    bool case3KeyStop;
    double startTime3,endTime3,bumpTime3;
    bool isBumped;
    bool case3KeyPressed;

    //case 5
    bool case5Start;
    float startTime5;

    //test1
    float rollInitial;
    float pitchInitial;
    int pitchTick;
    float pitchDiff,rollDiff;
    float pitchDiffThreshold,rollDiffThreshold;
    int pitchCheckTick,rollCheckTick;
    bool pitchVaild,rollVaild;
    float pitchSum,rollSum;

    //test2
    bool test2Start;
    std::ofstream test2Outfile;
    std::map<int,int> batteryLevel;
    int smoothwindow;
    int smoothTick;
    int* smoothdata;
    bool isSmoothed;
    float smoothAvg;
    float batteryUpRate;
    float batteryDropRate;
    float batteryLevelOutput;

    //test3
    bool test3Start;
    std::ofstream test3Outfile;    
    std::vector<SensorMsg::ImuData_t> imuVector;
    float wRate,vRate;
    float wUpdate,vUpdate;
    float wLimit,vLimit;
    float wUnit;
    float accUnit;
    bool lastRotateDir;
    float rotateDirSave;
    int dirChangeCount;
    std::vector<int> currentSmoothDataLeft,currentSmoothDataRight;
    int currentSmoothTick;
    int currentSmoothwindowSize;
    float currentSmoothAvgLeft,currentSmoothAvgRight;
    bool isCurrentSmoothed;
    int leftErrorTick,rightErrorTick,errorTickThreshold;


    /* case-4 */
    bool case4Start = false;
    uint32_t loopTick = 0;
    uint32_t idleTick = 0;
    int32_t limitTick1 = 0;
    int32_t limitTick2 = 0;

/*    
private:
    //init


    //mcu
    ring_buf m_mcuDataBuf;
	RobotMsg::RobotStatus_t robotStatus;
	SensorMsg::ImuData_t imuData;
	std::vector<SensorMsg::ImuData_t> imuDataArray;   


    //xd1y
	SensorMsg::PointCloud_t xd1yData;
	uint8_t xd1y_raw_data[XD1Y_RAW_POINT_CNT];
	uint16_t xd1y_data[XD1Y_POINT_CNT];
	uint16_t xd1y_intensity[XD1Y_POINT_CNT];
    int16_t xd1y_dist_mm;
	float xd1y_install_height_mm;
	float xd1y_install_pitch_angle_degree;
	float xd1yCosPitch[XD1Y_POINT_CNT];
	float xd1ySinPitch[XD1Y_POINT_CNT];
    //xd1q
	SensorMsg::PointCloud_t xd1qData;
	float xd1qMountPitch;
	float xd1qMountHeight;
	float xd1qCosYaw[XD1Q_POINT_CNT];	
	float xd1qSinYaw[XD1Q_POINT_CNT];
	float xd1qCosPitch;		// pitch = 15.996 degree	
	float xd1qSinPitch;	 
    int xd1qReferArray[XD1Q_POINT_CNT];
    int xd1qThreshArray[XD1Q_POINT_CNT];       

    //laser
    SensorMsg::LaserData_t laserData;
    NavPose laserMountLoc;

    //keyboard
    int fd = -1;
    uint16_t writeKeyData = 0; 
    uint8_t readKeyData = 0;   

    //led

    //control
    SensorMsg::Bumper_t bumpData;

    //global
    XLog* xlogPtr;
    ini::IniFile* robotCfgPtr;
    ini::IniFile* platformCfgPtr;
    TestMode_t testMode;

    //hardware
    HWInputDataInterface hwInput;
    HWOutputDataInterface hwOutput;
 

    //error
	bool openMcuError;
	bool openXD1YError;
	bool openLaserError;
	bool updateMCUError;
	bool updateXD1YError;
	bool updateLaserError;

    //
    uint64_t st1, st2, et;



public:
    testZhb_t();
    ~testZhb_t();

    //init 
    void init(bool enxd1y,bool enLaser);
    void update(bool enxd1y,bool enLaser);

    //mcu
    void mcuOpen();
    void mcuUpdate();

    //xd1y
    void xd1yOpen();
    void xd1yUpdate();
    void setupLaser();

    //laser
    void laserOpen();
    void laserUpdate();

    //keyboard
    RobKeyCodeTest_t getKeyboard();

    //led
    void setLed(bool ledROn,bool ledBOn,bool ledYOn);

    //control
    BumpState getBumperState();
    void bumperAutoHandle(bool en);

    //test mode
    TestMode_t getTestMode();

    //global variable
    XLog* getXlogPtr();
*/

};














#endif