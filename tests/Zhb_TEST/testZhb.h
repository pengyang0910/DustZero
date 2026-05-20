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

#include "xbase.h"
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
    test4,
    test5,
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

void fftBase2(Complex *resource, Complex *result, int n)
{
	if(n>1) { 
		for(int i=0; i<n; i+=2) {
			result[i/2]=resource[i];
			result[i/2+n/2]=resource[i+1];
		}
		fftBase2( result,resource,n/2);
		fftBase2( result+n/2,resource,n/2 );

		for(int i=0;i<n/2;i++) {
			Complex omega(cos(2*M_PI*i/double(n)),-sin(2*M_PI*i/double(n)));
			Complex temp=omega*result[i+n/2];
			resource[i]=result[i]+temp;
			resource[i+n/2]=result[i]-temp;
		}
	}
}

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
    void test5Init();   

    void case1Handle();        
    void case2Handle();
    void case3Handle();
    void case4Handle();
    void case5Handle();
    void test1Handle();   
    void test2Handle();
    void test3Handle();
    void test4Handle();
    void test5Handle();

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
    int cliffTick;
    int smoothWindowSize;
    std::vector<std::vector<int>> cliffInputAll;
    std::vector<int> cliffSmoothValue;
    

    //case2
    RobKeyCodeTest_t getKeyboard();
    uint8_t readKeyData;
    bool case2Start;
    bool case2KeyPressed;
        //wheel current
    int wheelTick;
    std::vector<int> leftWheelCurrent,rightWheelCurrent;
    float leftWheelAvg,rightWheelAvg;
    int wheelCurrentWindowSize;

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
    //float accUnit;
    bool lastRotateDir;
    float rotateDirSave;
    int dirChangeCount;
    std::vector<int> currentSmoothDataLeft,currentSmoothDataRight;
    int currentSmoothTick;
    int currentSmoothwindowSize;
    float currentSmoothAvgLeft,currentSmoothAvgRight;
    bool isCurrentSmoothed;
    int leftErrorTick,rightErrorTick,errorTickThreshold;

    //test4
    bool test4Start;
    std::ofstream test4Outfile;
    double test4StartTs;
    int imuTick;
    int imuCalibrationTick;
    double imuCalibrationRotateSpeed;
    SensorMsg::ImuData_t lastImuData;
    std::vector<double> imuOdom;//0:ts,1:x,2:y,3:z,4:roll,5:pitch,6:yaw,7:vx,8:vy,9:vz,10:iroll,11:ipitch
    std::vector<double> rotateMatrix;
    std::vector<double> imuCalibrationDataSum;//0:gyro0,1:gyro1,2:gyro2,3:acc0,4:acc1,5:acc2
    std::vector<double> imuCalibrationData;//0:gyro0,1:gyro1,2:gyro2,3:acc0,4:acc1,5:acc2
    double accUnit;
    double gyroUnit;
    std::vector<double> initialRP;
    std::vector<double> matrixMultiplyMatrix(std::vector<double> matrix1,int row1,int column1,\
                                            std::vector<double> matrix2,int row2,int column2);
    std::vector<double> constantMultiplyMatrix(double num1,std::vector<double> matrix1);
    std::vector<double> matrixPlusMatrix(std::vector<double> matrix1,std::vector<double> matrix2);
    std::vector<double> matrixTranspose(std::vector<double> matrix,int row,int column);
    void displayMatrix(std::vector<double> matrix,int row,int column);
    std::vector<double> rotateMatrixToRPY(std::vector<double> matrix);
    std::vector<double> RPYToRotatematrix(std::vector<double> RPY);
    std::vector<double> accCombineSmooth;
    std::vector<double> accxSmooth;
    std::vector<double> accySmooth;
    std::vector<double> acczSmooth;
    std::vector<double> gyroCombineSmooth;
    std::vector<double> gyroxSmooth;
    std::vector<double> gyroySmooth;
    std::vector<double> gyrozSmooth;    
    int imuSmoothTick;
    int imuSmoothTickWindowSize;
    int imufftfreq1TriggerTick;
    int lastSpeedSetTick;
    int vwSetTriggerTick;

    //test 5
    bool test5Start;
    float test5StartTs;
    std::ofstream test5Outfile; 
    std::vector<double> odomData;   


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