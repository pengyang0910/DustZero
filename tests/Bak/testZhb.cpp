#include "testZhb.h"

#define XBASE_JITTER_MAX_US 7000

#define KEY_TIMEOUT 300    // 2s
#define KEY_SHORT_PRESS 10 // 0.1s
#define KEY_LONG_PRESS 300 // 3.0s

#define OUTGPIO_MASK_POW_ON 0x01
#define OUTGPIO_MASK_3V3_ON 0x02
#define OUTGPIO_MASK_5V_ON 0x04
#define OUTGPIO_MASK_LED_Y 0x08
#define OUTGPIO_MASK_LED_R 0x10
#define OUTGPIO_MASK_LED_B 0x20

#define INGPIO_MASK_PWR_KEY 0x40
#define INGPIO_MASK_FN_KEY 0x80


#ifdef RK3566_BUILD
#define OUTGPIO_MASK 0x013F
#define INGPIO_MASK 0x00C0
#define OUTGPIO_MASK_MCU_RST 0x0100
#else 
#define OUTGPIO_MASK 0x3F
#define INGPIO_MASK 0xC0
#define OUTGPIO_MASK_MCU_RST 0
#endif

testZhb_t::testZhb_t()
{
    //xlog
    xlogPtr = new XLog(true);
    xlogPtr->Initialise("testZhb.log");
    xlogPtr->SetThreshold(Type(4));
    xlogPtr->EnableCout(true);  
    lcmPtr=new lcm::LCM();
    robotCfgPtr=new ini::IniFile();
    robotCfgPtr->load("/app/fj212/Config/robot.cfg");
    platformCfgPtr=new ini::IniFile();
    platformCfgPtr->load("/userdata/platform.cfg");
    xbasePtr=new XBase_t(lcmPtr,robotCfgPtr,platformCfgPtr);

    testModeToCaseInitMap.emplace(TestMode_t::case1,std::bind(&testZhb_t::case1Init,this));
    testModeToCaseInitMap.emplace(TestMode_t::case2,std::bind(&testZhb_t::case2Init,this));
    testModeToCaseInitMap.emplace(TestMode_t::case3,std::bind(&testZhb_t::case3Init,this));
    testModeToCaseInitMap.emplace(TestMode_t::case4,std::bind(&testZhb_t::case2Init,this));
    testModeToCaseInitMap.emplace(TestMode_t::case5,std::bind(&testZhb_t::case5Init,this));
    testModeToCaseInitMap.emplace(TestMode_t::test1,std::bind(&testZhb_t::test1Init,this));
    testModeToCaseInitMap.emplace(TestMode_t::test2,std::bind(&testZhb_t::test2Init,this));
    testModeToCaseInitMap.emplace(TestMode_t::test3,std::bind(&testZhb_t::test3Init,this));
    testModeToCaseInitMap.emplace(TestMode_t::test4_mBrush,std::bind(&testZhb_t::test4Init,this));
    testModeToCaseHandleMap.emplace(TestMode_t::case1,std::bind(&testZhb_t::case1Handle,this));
    testModeToCaseHandleMap.emplace(TestMode_t::case2,std::bind(&testZhb_t::case2Handle,this));
    testModeToCaseHandleMap.emplace(TestMode_t::case3,std::bind(&testZhb_t::case3Handle,this));
    testModeToCaseHandleMap.emplace(TestMode_t::case4,std::bind(&testZhb_t::case2Handle,this));
    testModeToCaseHandleMap.emplace(TestMode_t::case5,std::bind(&testZhb_t::case5Handle,this)); 
    testModeToCaseHandleMap.emplace(TestMode_t::test1,std::bind(&testZhb_t::test1Handle,this)); 
    testModeToCaseHandleMap.emplace(TestMode_t::test2,std::bind(&testZhb_t::test2Handle,this));
    testModeToCaseHandleMap.emplace(TestMode_t::test3,std::bind(&testZhb_t::test3Handle,this)); 
    testModeToCaseHandleMap.emplace(TestMode_t::test4_mBrush,std::bind(&testZhb_t::test4Handle,this));   


}

testZhb_t::~testZhb_t()
{
    delete xlogPtr;
    delete lcmPtr;
    delete robotCfgPtr;
    delete platformCfgPtr;
    delete xbasePtr;
    delete smoothdata;
}

XLog* testZhb_t::getXlogPtr()
{
    return xlogPtr;
}

void testZhb_t::init(TestMode_t testMode)
{
    testModeToCaseMap_t::iterator iter1=testModeToCaseInitMap.find(testMode);
    testModeToCaseMap_t::iterator iter2=testModeToCaseHandleMap.find(testMode);
    if(iter1 == testModeToCaseInitMap.end() || iter2 == testModeToCaseHandleMap.end())
    {
        xlogPtr->Error("can not find test mode behaviour handler!");
        exit(-1);
    }    
    testModeToCaseInitMap[testMode]();
}

void testZhb_t::handle(TestMode_t testMode)
{
    testModeToCaseHandleMap[testMode]();
}

/*
//init 
void testZhb_t::init(bool enxd1y,bool enLaser)
{
    mcuOpen();

    if(enxd1y)
    {
        setupLaser();
        xd1yOpen();
    }
    if(enLaser)
    {
        setupLaser();
        laserOpen();
    }

	hwOutput.m_mr133_mcu.cmd.bit_field.bumper_manual_back = 0;
	//hwOutput.m_mr133_mcu.cmd.bit_field.rf_response_enable = 1;    
}

void testZhb_t::update(bool enxd1y,bool enLaser)
{
    st1 = getTimeStap_us();

    mcuUpdate();

    if(enxd1y)
    {
        xd1yUpdate();
    }
    if(enLaser)
    {
        laserUpdate();
    }
}

//mcu
void testZhb_t::mcuOpen()
{
    int temp;
    temp=uart_open(MCU_UART);
    if(temp != 0)
    {
        openMcuError=true;
        xlogPtr->Error("mcu uart open() failed");
        exit(-1);
    }
    else
    {
        openMcuError=false;
    }
}

void testZhb_t::mcuUpdate()
{
    static uint64_t previousTs = 0;
    static int mcuNotUpdateCount=0;
    int time_out_cnt = 10;
	bool lMcuUpdate = false;
	int valid_mcu_frame_cnt = 0;
	while (time_out_cnt--)
	{
		int lUartReceivedSize = 0;		
		while ((lUartReceivedSize = uart_read((char *)(m_mcuDataBuf.data + m_mcuDataBuf.in_idx), m_mcuDataBuf.bufferSize - m_mcuDataBuf.in_idx)) > 0)
		{
			m_mcuDataBuf.in_idx = (lUartReceivedSize + m_mcuDataBuf.in_idx) % m_mcuDataBuf.bufferSize;
		}

		int temp=hParseMCUFrame(m_mcuDataBuf, &hwInput.m_mcu_mr133);
		int count=1;
		//xlogPtr->Error("mcu buffer update count:%d",count);
		while (temp > 0)
		{
			imuData.name = "Imu";
			imuData.timestamp_us = getTimeStap_us();
			imuData.q0 = hwInput.m_mcu_mr133.q0;
			imuData.q1 = hwInput.m_mcu_mr133.q1;
			imuData.q2 = hwInput.m_mcu_mr133.q2;
			imuData.q3 = hwInput.m_mcu_mr133.q3;
			imuData.seq = hwInput.m_mcu_mr133.seq;
#if 1
			float q0 = imuData.q0 / 10000.0;
			float q1 = imuData.q1 / 10000.0;
			float q2 = imuData.q2 / 10000.0;
			float q3 = imuData.q3 / 10000.0;
#endif
#ifdef FJ212_PROTOCOL

			imuData.acc0 = hwInput.m_mcu_mr133.imu_raw_accel[0];
			imuData.acc1 = hwInput.m_mcu_mr133.imu_raw_accel[1];
			imuData.acc2 = hwInput.m_mcu_mr133.imu_raw_accel[2];
			imuData.gyro0 = hwInput.m_mcu_mr133.imu_raw_gyro[0];
			imuData.gyro1 = hwInput.m_mcu_mr133.imu_raw_gyro[1];
			imuData.gyro2 = hwInput.m_mcu_mr133.imu_raw_gyro[2];
#endif

			if (imuDataArray.size() < 100)
				imuDataArray.push_back(imuData);

			if ((hwInput.m_mcu_mr133.seq % 2) == 0)
			{
				lMcuUpdate = true;
				xlogPtr->Debug("lMcuUpdate = true");
				uint64_t ts = getTimeStap_us();
				// xlogPtr->Debug("[WQ][%3f][%d]\n", ts * 1000, hwInput.m_mcu_mr133.seq);
				if (ts - previousTs > 20000 + XBASE_JITTER_MAX_US)
				{
					xlogPtr->Error("[XBASE PERIOD JITTER %d] cost much time %d us, previousTs=%u cuurentTs=%u", getCPUID(), int32_t(ts - previousTs), (uint32_t)previousTs, (uint32_t)ts);
				}
				else if ((ts - previousTs < 20000 - XBASE_JITTER_MAX_US))
				{
					xlogPtr->Error("[XBASE PERIOD JITTER %d] cost less time %d us, previousTs=%u cuurentTs=%u", getCPUID(), int32_t(ts - previousTs), (uint32_t)previousTs, (uint32_t)ts);
				}
				previousTs = ts;
				//break; // !!!!! DON'T BREAK, Parse all available data
				valid_mcu_frame_cnt++;
			}
			temp=hParseMCUFrame(m_mcuDataBuf, &hwInput.m_mcu_mr133);
			count+=1;
			//xlogPtr->Error("mcu buffer update count:%d",count);
		}
		if (lMcuUpdate)
		{
			//xlogPtr->Debug("lMcuUpdate = true & break");
			if (valid_mcu_frame_cnt > 1)
			{
				xlogPtr->Warn("[XBASE MCU OVERRUN][%u] MCU frame overrun %d!!!!!!!!!!!", (uint32_t)getTimeStap_us(), valid_mcu_frame_cnt);
			}
			break; 
		}
		else
		{
			sleep_us(1000); // wait until parse vald mcu frame
		}
	}

	if (lMcuUpdate)
	{
		lMcuUpdate = false;

		st2 = getTimeStap_us(); // re-arrange start time
		mcuNotUpdateCount=0;
		updateMCUError=false;
	}
	else
	{
		mcuNotUpdateCount++;
		if(mcuNotUpdateCount > 50)
		{	
			updateMCUError=true;
			xlogPtr->Error("mcu not update");
		}
	}
}

//xd1y
void testZhb_t::xd1yOpen()
{
    int temp;
    temp=openXD1Y();
    if(temp == -1)
    {	
        openXD1YError=true;
        xlogPtr->Error("xd1y open failed!");
        exit(-1);
    }
    else
    {
        openXD1YError=false;
    }
    xd1y_load_intrinsic_file("/userdata/laserConfig/XD1Y_Calibration.bin");
}

void testZhb_t::xd1yUpdate()
{
	static int xd1yNotUpdateCount=0;

    if (updateXD1Y(xd1y_raw_data) > 0)
    {
        xd1y_transform(xd1y_raw_data, xd1y_data, xd1y_intensity, NULL);

        xd1y_dist_mm = xd1yPointCloudProcess(xd1y_data, fabs(xd1y_install_pitch_angle_degree), xd1y_install_height_mm);
        // xlogPtr->Debug("XD1Y wallsensor = %d ", xd1y_dist_mm);

        for (int i = 0; i < XD1Y_POINT_CNT; i++)
        {
            xd1yData.x[i] = xd1y_data[i] * xd1yCosPitch[i] / 10000.0;
            xd1yData.z[i] = xd1y_data[i] * xd1ySinPitch[i] / 10000.0;
        }

        xd1yData.timestamp_us = getTimeStap_us();
        xd1yNotUpdateCount=0;
        updateXD1YError=false;
    }
    else
    {
        xd1yNotUpdateCount++;
        if(xd1yNotUpdateCount > 50)
        {
            updateXD1YError=true;
            xlogPtr->Error("xd1y not update");
        }
    }
    //xlogPtr->Error("xd1y not update count:%d",xd1yNotUpdateCount);

    hwInput.m_mcu_mr133.proximity_dist_mm = xd1y_dist_mm;
    robotStatus.proximityDistMm = hwInput.m_mcu_mr133.proximity_dist_mm;

}

void testZhb_t::setupLaser()
{
	float centerRefer = 510 / sin(15.996 / 180.0 * M_PI);
	float centerThres = 100 / sin(15.996 / 180.0 * M_PI);
	for (int i = 0; i < XD1Q_POINT_CNT; i++)
	{
		xd1qReferArray[i] = centerRefer / cos((-51 + 0.15 * i) / 180.0 * M_PI);
		xd1qThreshArray[i] = centerThres / cos((-51 + 0.15 * i) / 180.0 * M_PI);
	}

	xlogPtr->Debug("X1C_POINT_CNT:%d", X1C_POINT_CNT);
	laserData.name = "LaserScan";
	laserData.range_num = X1C_POINT_CNT;
	laserData.ranges.resize(X1C_POINT_CNT);
	laserData.bearing.resize(X1C_POINT_CNT);
	laserData.max_range = 4.0;
	laserData.min_range = 0.15; // 0.15
	laserData.min_bearing = -M_PI / 2;
	laserData.max_bearing = M_PI / 2;
	laserData.resolution = M_PI / X1C_POINT_CNT;

	laserData.xPos = laserMountLoc.px;
	laserData.yPos = laserMountLoc.py;
	laserData.zPos = 0;
	laserData.roll = 0;
	laserData.pitch = 0;
	laserData.yaw = laserMountLoc.pa;

	for (size_t i = 0; i < X1C_POINT_CNT; i++)
	{
		laserData.bearing[i] = laserData.min_bearing + laserData.resolution * (i); // i
	}

	xd1qData.name = "XD1Q";
	xd1qData.pointsNum = XD1Q_POINT_CNT;
	xd1qData.x.resize(XD1Q_POINT_CNT);
	xd1qData.y.resize(XD1Q_POINT_CNT);
	xd1qData.z.resize(XD1Q_POINT_CNT);
	for (int i = 0; i < XD1Q_POINT_CNT; i++)
	{
		float yaw = (-51 + 0.15 * i) / 180.0 * M_PI;
		xd1qCosYaw[i] = cos(yaw);
		xd1qSinYaw[i] = sin(yaw);
		xd1qData.x[i] = 0;
		xd1qData.y[i] = 0;
		xd1qData.z[i] = 0;
	}
	xd1qCosPitch = cos(xd1qMountPitch / 180.0 * M_PI);
	xd1qSinPitch = sin(xd1qMountPitch / 180.0 * M_PI);

	xd1yData.name = "XD1Y";
	xd1yData.pointsNum = XD1Y_POINT_CNT;
	xd1yData.x.resize(XD1Y_POINT_CNT);
	xd1yData.y.resize(XD1Y_POINT_CNT);
	xd1yData.z.resize(XD1Y_POINT_CNT);
	for (int i = 0; i < XD1Y_POINT_CNT; i++)
	{
		float pitch = (60 + xd1y_install_pitch_angle_degree + (-1) * i) / 180.0 * M_PI; // up to down
		xd1yCosPitch[i] = cos(pitch);
		xd1ySinPitch[i] = sin(pitch);
		xd1yData.y[i] = 0;
		xd1yData.x[i] = 0;
		xd1yData.z[i] = 0;
	}    
}

//laser
void testZhb_t::laserOpen()
{
    int temp=openLaser();
    if(temp == -1)
    {
        openLaserError=true;
        xlogPtr->Error("open laser failed!");
        exit(-1);
    }
    else
    {
        openLaserError=false;
    }
}

void testZhb_t::laserUpdate()
{
	uint64_t st3 = getTimeStap_us();

	hwInput.laser_update = false;
	static int laserNotUpdateCount=0;
	int temp=updateLaser(hwInput.m_laserPoints, hwInput.m_laserIntensity, hwInput.m_xd1q_points, false, hwInput.bHomeMode, hwInput.m_home_beacon_points);
    if(temp < 0)
    {
        laserNotUpdateCount++;
        if(laserNotUpdateCount > 50)
        {
            updateLaserError=true;
            xlogPtr->Error("laser not update,%d",laserNotUpdateCount);
        }
    }
    else
    {
        updateLaserError=false;
        laserNotUpdateCount=0;
    }

	et = getTimeStap_us();

	uint32_t cost_us_1 = (et - st1);
	uint32_t cost_us_2 = (et - st2);
	if (cost_us_1 > (CONTROL_LOOP_XBASE * 1000 + XBASE_JITTER_MAX_US) || cost_us_2 > (CONTROL_LOOP_XBASE * 1000 - 2000 + XBASE_JITTER_MAX_US)) // add XBASE_JITTER_MAX_US margin
	{
		xlogPtr->Error("[XBase LOOP TO %d]: task cost too much time. %d us %d us, ts is %u", getCPUID(), cost_us_1, cost_us_2, (uint32_t)et);
		//xlogPtr->Error("[WQ][%d][st1 %u st2 %u us] cost %.3f %.3f %.3f %.3f %.3f ms, total cost %.3f ms", getCPUID(), (uint32_t)st1, (uint32_t)st2, (st2 - st1) / 1000.0, (st3 - st2) / 1000.0, (st4 - st3) / 1000.0, (st5 - st4) / 1000.0, (st6 - st5) / 1000.0, cost_us_1/1000.0);
	}
	else if (cost_us_1 < CONTROL_LOOP_XBASE * 1000 && cost_us_2 < (CONTROL_LOOP_XBASE * 1000 - 2000))
	{
		sleep_us(CONTROL_LOOP_XBASE * 1000 - 2000 - cost_us_2);
	}
	//xlogPtr->Debug("[WQ][%d][st1 %u st2 %u us] cost %.3f %.3f %.3f %.3f %.3f ms, total cost %.3f ms", getCPUID(), (uint32_t)st1, (uint32_t)st2, (st2 - st1) / 1000.0, (st3 - st2) / 1000.0, (st4 - st3) / 1000.0, (st5 - st4) / 1000.0, (st6 - st5) / 1000.0, cost_us_1/1000.0);
	//printf("[WQ][%d][st1 %u st2 %u us] cost %.3f %.3f %.3f %.3f %.3f ms, total cost %.3f ms\n", getCPUID(), (uint32_t)st1, (uint32_t)st2, (st2 - st1) / 1000.0, (st3 - st2) / 1000.0, (st4 - st3) / 1000.0, (st5 - st4) / 1000.0, (st6 - st5) / 1000.0, cost_us_1/1000.0);
}

//keyboard
RobKeyCodeTest_t testZhb_t::getKeyboard()
{
    int fnKey = -1;
    int powerKey = -1;
    static int fnKeyTick = 0;
    static int powerKeyTick = 0;
    static int bothPressTick = 0;
    bool fnKeyLongPressed = false;
    bool powerKeyLongPressed = false;
    bool fnKeyShortPressed = false;
    bool powerKeyShortPressed = false;
    bool bothKeyLongPressed = false;

    if(-1 == fd)
        return RobKeyCodeTest_t::None;
    
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
        return RobKeyCodeTest_t::FnPowerCombinedLongPressed;
    }
    else if(powerKeyLongPressed)
    {
        printf("Power Long!\r\n");
        return RobKeyCodeTest_t::PowerKeyLongPressed;
    }
    else if(fnKeyLongPressed)
    {
        printf("Fn Long!\r\n");
        return RobKeyCodeTest_t::FnKeyLongPressed;
    }
    else if(powerKeyShortPressed)
    {
        printf("Power Short!\r\n");
        return RobKeyCodeTest_t::PowerKeyShortPressed;
    }
    else if(fnKeyShortPressed)
    {
        printf("Fn Short!\r\n");
        return RobKeyCodeTest_t::FnKeyShortPressed;
    }
    else 
        return RobKeyCodeTest_t::None;
}

//led
void testZhb_t::setLed(bool ledROn,bool ledBOn,bool ledYOn)
{
    if(ledROn)
    {
        writeKeyData |= (OUTGPIO_MASK_LED_R);        
    }
    else
    {
        writeKeyData &= (~OUTGPIO_MASK_LED_R);
    }
    if(ledBOn)
    {
        writeKeyData |= (OUTGPIO_MASK_LED_B);        
    }
    else
    {
        writeKeyData &= (~OUTGPIO_MASK_LED_B);
    }
    if(ledYOn)
    {
        writeKeyData |= (OUTGPIO_MASK_LED_Y);        
    }
    else
    {
        writeKeyData &= (~OUTGPIO_MASK_LED_Y);
    }    
}

//control
BumpState testZhb_t::getBumperState()
{
	BumpState retState = BumpState::None;
	if (bumpData.rightBumped && bumpData.leftBumped)
		retState = BumpState::BothBumped;
	else if (bumpData.rightBumped)
		retState = BumpState::RightBumped;
	else if (bumpData.leftBumped)
		retState = BumpState::LeftBumped;
	else if(bumpData.isProcessing)
		retState = BumpState::BothBumped;
	else
		retState = BumpState::None;

	return retState;
}

void testZhb_t::bumperAutoHandle(bool en)
{
	if(en)
		hwOutput.m_mr133_mcu.cmd.bit_field.bumper_manual_back = 0;
	else 
		hwOutput.m_mr133_mcu.cmd.bit_field.bumper_manual_back = 1;
}

//test mode
TestMode_t testZhb_t::getTestMode()
{
    return testMode;    
}

//global variable
XLog* testZhb_t::getXlogPtr()
{
    return xlogPtr;
}




int main(int argc, char *argv[])
{
    testZhb_t* testZhbPtr=new testZhb_t();
    XLog* xlogPtr=testZhbPtr->getXlogPtr();
    TestMode_t cfgGetMode=testZhbPtr->getTestMode();
    bool enXD1Y=false;
    bool enLaser=false;
    testZhbPtr->init(enXD1Y,enLaser);
    testZhbPtr->bumperAutoHandle(false);

    bool isRunning=true;
    while(isRunning)
    {
        testZhbPtr->update(enXD1Y,enLaser);        
        //RobKeyCodeTest_t robotKey=testZhbPtr->getKeyboard();
        BumpState BumpState=testZhbPtr->getBumperState();
        bool ledROn=false;
        bool ledBOn=false;
        bool ledYOn=false;        
        switch(BumpState)
        {
            case BumpState::LeftBumped:
            {
                ledROn=true;
                break;
            }
            case BumpState::RightBumped:
            {
                ledBOn=true;                  
                break;
            }
            case BumpState::BothBumped:
            {
                ledROn=true;
                ledBOn=true;                   
                break;
            }
            case BumpState::None:
            {                     
                break;
            }
            default:
            {
                xlogPtr->Error("bump state error!");
                break;
            }      
            testZhbPtr->setLed(ledROn,ledBOn,ledYOn);               
        }
    }


    return 0;
}

*/


void testZhb_t::case1Init()
{
    xbasePtr->Init();
    xbasePtr->OpenLaserSensors(); 
    fd = open("/dev/xtgpio0", O_RDWR);
    writeKeyData = OUTGPIO_MASK_POW_ON | OUTGPIO_MASK_3V3_ON | OUTGPIO_MASK_5V_ON;      
    ledROn=false;
    ledBOn=false;
    ledYOn=false;
    lastBumpStatus=0; 
    cliffInput.clear();
    cliffInput.resize(6);
    cliffCount.clear();
    cliffCount.resize(6);
    cliffTriggerThreshold=30;
    cliffBackDistance=0;
    cliffRotateSum=0;
    lastCliffTriggerPositionBack={0,0,0};
    lastCliffTriggerPositionRotation={0,0,0};
    cliffbackMode=0;
    rotateAfterMove=false;
    a1StartTime=getTimeStap_ms()/1000.0;
    cliffTrigger=new int8_t(6);
}

void testZhb_t::case1Handle()
{
    xbasePtr->UpdateInput();      
    SensorMsg::Bumper_t Bumpdata=xbasePtr->getBumpData();
#if 1
    xbasePtr->MotorEnable(true);
    //xbasePtr->EnVelProfile(true);
    if(fabs(a1StartTime-getTimeStap_ms()/1000.0) < 5.0)
    {
        return;
    }
    const HWInputDataInterface* HWInputData=xbasePtr->getHWInput();
    cliffInput[0]=HWInputData->m_mcu_mr133.cliff_l_adc;
    cliffInput[1]=HWInputData->m_mcu_mr133.cliff_lf_adc;
    cliffInput[2]=HWInputData->m_mcu_mr133.cliff_ll_adc;
    cliffInput[3]=HWInputData->m_mcu_mr133.cliff_r_adc;
    cliffInput[4]=HWInputData->m_mcu_mr133.cliff_rf_adc;
    cliffInput[5]=HWInputData->m_mcu_mr133.cliff_rr_adc;  
    xlogPtr->Debug("[l,lf,ll,r,rf,rr]=[%3d,%3d,%3d,%3d,%3d,%3d]",cliffInput[0],cliffInput[1],cliffInput[2],cliffInput[3],cliffInput[4],cliffInput[5]);
    for(int i=0;i<cliffInput.size();i++)
    {
        cliffTrigger[i]=0;
    }
    //filter window size=3
    for(int i=0;i<cliffInput.size();i++)
    {
        if(cliffInput[i] <= cliffTriggerThreshold)
        {
            cliffCount[i]+=1;
            if(cliffCount[i] >= 3)
            {
                cliffCount[i]=3;
                cliffTrigger[i]=1;
            }
            else
            {
                cliffTrigger[i]=0;
            }
        }
        else
        {
            cliffCount[i]=0;
            cliffTrigger[i]=0;
        }
    }
    for(int i=0;i<cliffInput.size();i++)
    {
        xlogPtr->Debug("cliffTrigger:seq:%d,value:%02x",i,cliffTrigger[i]);
    }
    //debug for each cliff trigger
    int8_t cliffTriggerStatus=0x00;
    for(int i=0;i<cliffInput.size();i++)
    {
        cliffTriggerStatus = cliffTriggerStatus | (cliffTrigger[i] << (7-i));
    }
    xlogPtr->Debug("     cliffTriggerStatus:%02X",cliffTriggerStatus);
    //choose move back or move forward mode
    int8_t cliffFrontAndBackStatus=0x00;
    for(int i=0;i<cliffInput.size();i++)
    {
        if(i == 2 || i == 5)
        {
            cliffFrontAndBackStatus = cliffFrontAndBackStatus | (cliffTrigger[i] << 0);
        }
        else
        {
            cliffFrontAndBackStatus = cliffFrontAndBackStatus | (cliffTrigger[i] << 1);
        }
    }
    xlogPtr->Debug("cliffFrontAndBackStatus:%02X",cliffFrontAndBackStatus);

    //move forward
    float odomX=HWInputData->m_mcu_mr133.odometry_x_mm/1000.0;
    float odomY=HWInputData->m_mcu_mr133.odometry_y_mm/1000.0;
    float odomTheta=HWInputData->m_mcu_mr133.odometry_yaw_mrad/1000.0;
    float dist=0;
    float radian=0;
    float speedV=0;
    float speedW=0;
    float vFeedback=HWInputData->m_mcu_mr133.v_fdk_mm_s/1000.0;
    if(cliffFrontAndBackStatus == 0x03)
    {
        exit(-1);
        if(cliffTriggerStatus == 0x14 || cliffTriggerStatus == 0xa0)
        {
            lastCliffTriggerPositionBack.px=odomX;
            lastCliffTriggerPositionBack.py=odomY;
            lastCliffTriggerPositionBack.pa=odomTheta;
            cliffBackDistance=0;            
            lastCliffTriggerPositionRotation.px=odomX;
            lastCliffTriggerPositionRotation.py=odomY;
            lastCliffTriggerPositionRotation.pa=odomTheta;
            cliffRotateSum=0; 
            rotateAfterMove=true; 
            cliffbackMode=0;           
        }
        else
        {
            xlogPtr->Error("cliff front and back both trigger,but not in one side!");
            exit(-1);
        }
    }    
    else if(cliffFrontAndBackStatus == 0x00)
    {
        if(cliffbackMode == 0)
        {
            speedV=0.25;//wanderspeed
        }
        else
        {
            lastCliffTriggerPositionBack.px=odomX;
            lastCliffTriggerPositionBack.py=odomY;
            lastCliffTriggerPositionBack.pa=odomTheta;
            cliffBackDistance=0;            
            lastCliffTriggerPositionRotation.px=odomX;
            lastCliffTriggerPositionRotation.py=odomY;
            lastCliffTriggerPositionRotation.pa=odomTheta;
            cliffRotateSum=0; 
            rotateAfterMove=true;
        }
        cliffbackMode=0;
    }    
    else if(cliffFrontAndBackStatus == 0x01)
    {
        if(cliffbackMode == -1)
        {
            dist=sqrt((odomX-lastCliffTriggerPositionBack.px)*(odomX-lastCliffTriggerPositionBack.px)\
                        +(odomY-lastCliffTriggerPositionBack.py)*(odomY-lastCliffTriggerPositionBack.py));
            cliffBackDistance+=dist;
            xlogPtr->Debug("forward dist:%f",cliffBackDistance);
            if(cliffBackDistance > 0.3)
            {
                xlogPtr->Error("cliff move forward > 0.3m");
                exit(-1);
            }
            else
            {

            }
        }
        else if(cliffbackMode == 1)
        {
            xlogPtr->Debug("cliff move forward,may be move back too fast!");
            //need immidiate forward()
            cliffBackDistance=0;
        }
        else if(cliffbackMode == 0)
        {
            //need immidiate forward()
            cliffBackDistance=0;       
        }
        lastCliffTriggerPositionBack.px=odomX;
        lastCliffTriggerPositionBack.py=odomY;
        lastCliffTriggerPositionBack.pa=odomTheta;
        speedV=0.08;
        cliffbackMode == -1;
    }
    //move back
    else if(cliffFrontAndBackStatus == 0x02)
    {
        if(cliffbackMode == 1)
        {
            dist=sqrt((odomX-lastCliffTriggerPositionBack.px)*(odomX-lastCliffTriggerPositionBack.px)\
                        +(odomY-lastCliffTriggerPositionBack.py)*(odomY-lastCliffTriggerPositionBack.py));
            cliffBackDistance+=dist;
            xlogPtr->Debug("back dist:%f",cliffBackDistance);
            if(cliffBackDistance > 0.3)
            {
                xlogPtr->Error("cliff move back > 0.3m");
                exit(-1);
            }
            else
            {

            }
        }
        else if(cliffbackMode == -1)
        {
            xlogPtr->Debug("cliff move back,may be move forward too fast!");
            //need immidiate back()
            cliffBackDistance=0;
        }
        else if(cliffbackMode == 0)
        {
            //need immidiate back()
            cliffBackDistance=0;                          
        }
        lastCliffTriggerPositionBack.px=odomX;
        lastCliffTriggerPositionBack.py=odomY;
        lastCliffTriggerPositionBack.pa=odomTheta;        
        speedV=-0.08;
        cliffbackMode=1;
    }
    //rotate
    if(rotateAfterMove)
    {    
        radian=fabs(odomTheta-lastCliffTriggerPositionRotation.pa);
        if(radian > M_PI)
        {
            radian=2*M_PI-radian;
        } 
        cliffRotateSum+=radian;  
        if(cliffRotateSum > M_PI*2)
        {
            xlogPtr->Error("cliff rotate > 2*pi");
            exit(-1);
        }
        else if(cliffRotateSum > 15.0/180*M_PI)
        {
            xlogPtr->Debug("rotate 15 degree,start running");
            speedV=0.25;
            speedW=0;
            rotateAfterMove=false;
            cliffRotateSum=0;             
        }
        else
        {
            speedV=0;
            speedW=30.0/180*M_PI;
        }
        lastCliffTriggerPositionRotation.px=odomX;
        lastCliffTriggerPositionRotation.py=odomY;
        lastCliffTriggerPositionRotation.pa=odomTheta;         
    }
    xlogPtr->Debug("send speed:[%f,%f],vfeedback:%f",speedV,speedW,vFeedback);
    if(fabs(speedV+0.08) < 0.001 && vFeedback > 0)
    {
        xlogPtr->Debug("close vel profile,vFeedBack:%f",vFeedback);
        xbasePtr->EnVelProfile(false);
        speedV=-0.08;
    }
    xbasePtr->setSpeed(speedV,speedW);

#endif

#if 0          
    BumpState retState = BumpState::None;
    int a=0;
    int b=0;
    int c=0;
    int d=0;	if(lastBumpStatus && !Bumpdata.isProcessing)
	{
		a=lastBumpStatus^0x01;
		b=lastBumpStatus^0x02;
		c=lastBumpStatus^0x03;
		if(a == 0)
		{
			retState = BumpState::LeftBumped;
		}  
		else if(b == 0)
		{
			retState = BumpState::RightBumped;
		}
		else if(c == 0)
		{
			retState = BumpState::BothBumped;
		}
		else
		{
			retState = BumpState::None;
		}
        d=1;
	}
	else
	{
		retState = BumpState::None;
        d=0;
	}

	if(Bumpdata.isProcessing)
	{
		if(Bumpdata.leftBumped)
		{
			lastBumpStatus=lastBumpStatus|0x01;
		}
		if(Bumpdata.rightBumped)
		{
			lastBumpStatus=lastBumpStatus|0x02;
		}
	}
	else
	{
		lastBumpStatus=0;
	}
    xlogPtr->Debug("left,right,isprocessing=[%d,%d,%d],bump status:%d,trigger:%d,bump state:%d"\
                ,Bumpdata.leftBumped,Bumpdata.rightBumped,Bumpdata.isProcessing,int(lastBumpStatus),d,int(retState));
#endif

    ledROn=false;
    ledBOn=false;
    ledYOn=false;    
    writeKeyData = OUTGPIO_MASK_POW_ON | OUTGPIO_MASK_3V3_ON | OUTGPIO_MASK_5V_ON;
    if(Bumpdata.leftBumped && Bumpdata.rightBumped)
    {
        ledYOn=true;
    }
    else if(Bumpdata.leftBumped)
    {
        ledROn=true;
    }
    else if(Bumpdata.rightBumped)
    {
        ledBOn=true;
    }
    else
    {

    }
    setLed1(writeKeyData,ledROn,ledBOn,ledYOn);          
    if(fd > 0)
    {
        write(fd, &writeKeyData, sizeof(writeKeyData));
    }
    xbasePtr->UpdateOutput();  
}

void testZhb_t::setLed1(uint16_t& writeKeyData,bool ledROn,bool ledBOn,bool ledYOn)
{
    if(ledROn)
    {
        writeKeyData |= (OUTGPIO_MASK_LED_R);        
    }
    else
    {
        writeKeyData &= (~OUTGPIO_MASK_LED_R);
    }
    if(ledBOn)
    {
        writeKeyData |= (OUTGPIO_MASK_LED_B);        
    }
    else
    {
        writeKeyData &= (~OUTGPIO_MASK_LED_B);
    }
    if(ledYOn)
    {
        writeKeyData |= (OUTGPIO_MASK_LED_Y);        
    }
    else
    {
        writeKeyData &= (~OUTGPIO_MASK_LED_Y);
    }    
}

void testZhb_t::case2Init()
{
    xbasePtr->Init();
    xbasePtr->OpenLaserSensors();  
    xbasePtr->MotorEnable(true);
    fd = open("/dev/xtgpio0", O_RDWR);
    readKeyData = 0;
    case2Start = false;
    case2KeyPressed = false;
}

void testZhb_t::case2Handle()
{
    xbasePtr->UpdateInput();
    RobKeyCodeTest_t keyboard=getKeyboard();
    if(keyboard == RobKeyCodeTest_t::PowerKeyLongPressed || keyboard == RobKeyCodeTest_t::PowerKeyShortPressed)
    {
        if(!case2KeyPressed)
        {
            xlogPtr->Debug("robot start!");
            case2KeyPressed=true;
            case2Start=true;   
            xbasePtr->setRighBrushDuty(-52);
            xbasePtr->setMainBrushDuty(84);
            xbasePtr->mopLiftDown();
            xbasePtr->setMopDuty(87);
            xbasePtr->setFanDuty(22);        
            xbasePtr->startClean();
        }
        else
        {
            xlogPtr->Debug("robot stop!");
            case2KeyPressed=false;
            case2Start=false;   
            xbasePtr->stopClean(); 
            xbasePtr->mopLiftUp();          
        }
    }
    if(case2Start)
    {
        xbasePtr->setSpeed(0.3,0.0);  
    }
    else
    {
        xbasePtr->setSpeed(0.0,0.0);
    }
    xbasePtr->UpdateOutput(); 
}

RobKeyCodeTest_t testZhb_t::getKeyboard()
{
    int fnKey = -1;
    int powerKey = -1;
    static int fnKeyTick = 0;
    static int powerKeyTick = 0;
    static int bothPressTick = 0;
    bool fnKeyLongPressed = false;
    bool powerKeyLongPressed = false;
    bool fnKeyShortPressed = false;
    bool powerKeyShortPressed = false;
    bool bothKeyLongPressed = false;

    if(-1 == fd)
        return RobKeyCodeTest_t::None;
    
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
        return RobKeyCodeTest_t::FnPowerCombinedLongPressed;
    }
    else if(powerKeyLongPressed)
    {
        return RobKeyCodeTest_t::PowerKeyLongPressed;
    }
    else if(fnKeyLongPressed)
    {
        return RobKeyCodeTest_t::FnKeyLongPressed;
    }
    else if(powerKeyShortPressed)
    {
        return RobKeyCodeTest_t::PowerKeyShortPressed;
    }
    else if(fnKeyShortPressed)
    {
        return RobKeyCodeTest_t::FnKeyShortPressed;
    }
    else 
        return RobKeyCodeTest_t::None;
}



void testZhb_t::case3Init()
{
    xbasePtr->Init();
    //xbasePtr->OpenLaserSensors();  
    xbasePtr->MotorEnable(true);
    fd = open("/dev/xtgpio0", O_RDWR);
    readKeyData = 0;
    case3Start = false;
    case3KeyStart = false;
    case3KeyStop = false;
    startTime3=getTimeStap_ms()/1000.0;
    endTime3=getTimeStap_ms()/1000.0;
    bumpTime3=getTimeStap_ms()/1000.0;
    isBumped=false;
    case3KeyPressed=false;
}
void testZhb_t::case3Handle()
{
    RobKeyCodeTest_t keyboard=getKeyboard();    
    xbasePtr->UpdateInput();  
    if(keyboard == RobKeyCodeTest_t::PowerKeyLongPressed || keyboard == RobKeyCodeTest_t::PowerKeyShortPressed)
    {
        if(!case3KeyPressed)
        {
            xlogPtr->Debug("robot start!");
            case3KeyPressed=true;
            case3Start=true;   
            xbasePtr->setRighBrushDuty(-52);
            xbasePtr->setMainBrushDuty(84);
            xbasePtr->mopLiftDown();
            xbasePtr->setMopDuty(87);
            xbasePtr->setFanDuty(100);        
            xbasePtr->startClean();
            startTime3=getTimeStap_ms()/1000.0;
            if(!case3KeyStart)
            {
                case3KeyStart=true;
            }
        }
        else
        {
            xlogPtr->Debug("robot stop!");  
            xbasePtr->stopClean();
            xbasePtr->mopLiftUp();
            xbasePtr->setSpeed(0.0,0.0); 
            case3Start = false;
            case3KeyStart = false;
            startTime3=getTimeStap_ms()/1000.0;
            endTime3=getTimeStap_ms()/1000.0;
            bumpTime3=getTimeStap_ms()/1000.0;
            isBumped=false;
            case3KeyPressed=false;             
        }
        xbasePtr->UpdateOutput();
        return;           
    }
    if(!case3KeyStart)
    {
        xbasePtr->UpdateOutput();
        return;
    }
    SensorMsg::Bumper_t Bumpdata=xbasePtr->getBumpData(); 
    if(Bumpdata.leftBumped || Bumpdata.rightBumped || Bumpdata.isProcessing)
    {
        xlogPtr->Debug("bumped!,left:%d,right:%d,processing:%d",int(Bumpdata.leftBumped),int(Bumpdata.rightBumped),int(Bumpdata.isProcessing));
        isBumped=true;
        bumpTime3=getTimeStap_ms()/1000.0;
    }
    if(isBumped)
    {
        if((getTimeStap_ms()/1000.0-bumpTime3) < 1.0)
        {
            //xlogPtr->Debug("bump rotate!");                 
            xbasePtr->setSpeed(0.0,M_PI/3);
            xbasePtr->UpdateOutput();
            return;
        }    
        else
        {
            isBumped=false;
            xlogPtr->Debug("bump rotate done!");
        } 
    }
    if(case3Start)
    {       
        xbasePtr->setSpeed(0.3,0.0);
        if((getTimeStap_ms()/1000.0-startTime3) > 60*60)
        {
            xlogPtr->Debug("already running 60 minutes");
            case3Start=false;
            endTime3=getTimeStap_ms()/1000.0;
            xbasePtr->stopClean();
            xbasePtr->mopLiftUp();
            xbasePtr->setSpeed(0.0,0.0);
        }
    }
    else
    {
        if((getTimeStap_ms()/1000.0-endTime3) > 20*60)
        {
            xlogPtr->Debug("already stop 20 minutes");
            case3Start=true;   
            xbasePtr->setRighBrushDuty(-52);
            xbasePtr->setMainBrushDuty(84);
            xbasePtr->mopLiftDown();
            xbasePtr->setMopDuty(87);
            xbasePtr->setFanDuty(20);        
            xbasePtr->startClean();
            startTime3=getTimeStap_ms()/1000.0;
        }
    }
    xbasePtr->UpdateOutput(); 
}
void testZhb_t::case4Init()
{

}
void testZhb_t::case4Handle()
{

}
void testZhb_t::case5Init()
{
    xbasePtr->Init();
    xbasePtr->OpenLaserSensors();      
    xbasePtr->MotorEnable(true);
    fd = open("/dev/xtgpio0", O_RDWR);
    readKeyData = 0;
    case5Start = false;
    startTime5=getTimeStap_ms()/1000.0;
}
void testZhb_t::case5Handle()
{
    xbasePtr->UpdateInput();
    RobKeyCodeTest_t keyboard=getKeyboard();
    if(keyboard == RobKeyCodeTest_t::PowerKeyLongPressed || keyboard == RobKeyCodeTest_t::PowerKeyShortPressed)
    {
        bool temp=case5Start;
        case5Start=!temp;
        if(case5Start)
        {
            xlogPtr->Debug("robot start!");
            xbasePtr->setRighBrushDuty(-100);
            xbasePtr->setMainBrushDuty(100);
            xbasePtr->mopLiftDown();
            xbasePtr->setMopDuty(100);
            xbasePtr->setFanDuty(100);        
            xbasePtr->startClean(); 
            startTime5=getTimeStap_ms()/1000.0;        
        }
        else
        {
            xlogPtr->Debug("robot stop!");
            xbasePtr->stopClean();
            xbasePtr->mopLiftUp();
            xbasePtr->setSpeed(0.0,0.0); 
        }      
    }
    if(case5Start)
    {
        xbasePtr->setSpeed(0.3,0.0);      
    } 
    else
    {
        xbasePtr->stopClean();
        xbasePtr->mopLiftUp();
        xbasePtr->setSpeed(0.0,0.0);             
    }        
    xbasePtr->UpdateOutput(); 
}

void testZhb_t::test1Init()
{
    xbasePtr->Init();
    xbasePtr->MotorEnable(true);    
    rollInitial=0.0;
    pitchInitial=0.0;
    pitchTick=0;
    pitchDiff=0;
    rollDiff=0;
    pitchDiffThreshold=0.1;
    rollDiffThreshold=0.1;
    pitchCheckTick=0;
    rollCheckTick=0;
    pitchVaild=true;
    rollVaild=true;
    pitchSum=0;
    rollSum=0;
}

void testZhb_t::test1Handle()
{
    xbasePtr->UpdateInput();    
    const HWInputDataInterface* HWInputData=xbasePtr->getHWInput();
    float q0 = HWInputData->m_mcu_mr133.q0 / 10000.0;
    float q1 = HWInputData->m_mcu_mr133.q1 / 10000.0;
    float q2 = HWInputData->m_mcu_mr133.q2 / 10000.0;
    float q3 = HWInputData->m_mcu_mr133.q3 / 10000.0;      
    float rollTemp = atan2f(2.f * (q2 * q3 + q0 * q1), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
    float pitchTemp = asinf(2.f * (q0 * q2 - q1 * q3));
    xlogPtr->Debug("[roll,pitch]=[%f,%f]",rollTemp,pitchTemp);
    xlogPtr->Debug("initial [roll,pitch]=[%f,%f]",rollInitial,pitchInitial);

    if((pitchTick == 0) && (fabs(pitchTemp) > 0.1 || fabs(rollTemp) > 0.1))
    {
        xlogPtr->Warn("robot start in an incline that slopes at an angle,[roll pitch]=[%f,%f]",rollTemp,pitchTemp);
        return;
    }
    if(pitchTick > 99)
    {
        pitchDiff = fabs(pitchTemp - pitchInitial);
        if(pitchDiff > pitchDiffThreshold)
        {
            pitchCheckTick++;
            if(pitchCheckTick > 10)
            {
                pitchVaild=false;
                xlogPtr->Debug("set pitch invalid,tick:%d,pitch diff:%f ",pitchCheckTick,pitchDiff);
                xlogPtr->Debug("pitch initial:%f,current:%f ",pitchInitial,pitchTemp);
            }
        }
        else
        {
            pitchCheckTick = 0;
            pitchVaild=true;
            //xlog->Debug("pitch reset vaild ");
        }

        rollDiff = fabs(rollTemp - rollInitial);
        if(rollDiff > rollDiffThreshold)
        {
            rollCheckTick++;
            if(rollCheckTick > 10)
            {
                rollVaild=false;
                xlogPtr->Debug("set roll invalid,tick:%d,roll diff:%f ",rollCheckTick,rollDiff);
                xlogPtr->Debug("roll initial:%f,current:%f ",rollInitial,rollTemp);
            }
        }
        else
        {
            rollCheckTick = 0;
            rollVaild=true;
            //xlog->Debug("roll reset vaild ");
        }
    }
    else if(pitchTick < 99)
    {
        pitchTick++;
        pitchSum += pitchTemp;
        rollSum += rollTemp;
    }
    else
    {
        pitchTick=101;
        pitchInitial = (pitchSum + pitchTemp) / 100.0;
        rollInitial = (rollSum + rollTemp) / 100.0;
        xlogPtr->Debug("initial [roll,pitch]=[%f,%f]",rollInitial,pitchInitial);
    }
    xbasePtr->UpdateOutput();     
}

void testZhb_t::test2Init()
{
    xbasePtr->Init();
    xbasePtr->OpenLaserSensors();   
    fd = open("/dev/xtgpio0", O_RDWR);
    readKeyData = 0;
    test2Start = false;
    test2Outfile.open("/app/fj212/bin/amclTest.log");
    if(!test2Outfile.is_open())
    {
        xlogPtr->Error("fail to open file! ");
    }
    test2Outfile << std::fixed << std::setprecision(6);

    batteryLevel[75]=1;
    batteryLevel[76]=1; 
    batteryLevel[77]=1;         
    batteryLevel[78]=1; 
    batteryLevel[79]=1; 
    batteryLevel[80]=1;
    batteryLevel[81]=1; 
    batteryLevel[82]=1;         
    batteryLevel[83]=1; 
    batteryLevel[84]=1; 
    batteryLevel[85]=1;
    batteryLevel[86]=1; 
    batteryLevel[87]=1;         
    batteryLevel[88]=1; 
    batteryLevel[89]=1; 
    batteryLevel[90]=2;
    batteryLevel[91]=3; 
    batteryLevel[92]=4;         
    batteryLevel[93]=6; 
    batteryLevel[94]=8; 
    batteryLevel[95]=10;
    batteryLevel[96]=13; 
    batteryLevel[97]=17;         
    batteryLevel[98]=21; 
    batteryLevel[99]=26; 
    batteryLevel[100]=31;
    batteryLevel[101]=37; 
    batteryLevel[102]=44;
    batteryLevel[103]=50;
    batteryLevel[104]=56;
    batteryLevel[105]=61; 
    batteryLevel[106]=65;
    batteryLevel[107]=69;
    batteryLevel[108]=74;
    batteryLevel[109]=78; 
    batteryLevel[110]=83;
    batteryLevel[111]=88;
    batteryLevel[112]=93;
    batteryLevel[113]=97; 
    batteryLevel[114]=99;
    batteryLevel[115]=100;
    batteryLevel[116]=100;
    batteryLevel[117]=100; 
    batteryLevel[118]=100;
    batteryLevel[119]=100;
    batteryLevel[120]=100;

    smoothwindow=100;
    batteryUpRate=0.002;
    batteryDropRate=0.002;    
    smoothTick=-1;
    smoothdata=new int[smoothwindow];
    isSmoothed=false;
    smoothAvg=0;
    batteryLevelOutput=0;
}

void testZhb_t::test2Handle()
{
    xbasePtr->UpdateInput();
    const HWInputDataInterface* HWInputData1=xbasePtr->getHWInput();
    int adc1=HWInputData1->m_mcu_mr133.bat_adc;    
    int batteryLevel1=xbasePtr->GetBatteryLevel();
    xlogPtr->Debug("battery level:%d,adc:%d",batteryLevel1,adc1);
    xbasePtr->UpdateOutput();
    return;


    xbasePtr->UpdateInput();
    RobKeyCodeTest_t keyboard=getKeyboard();
    if(keyboard == RobKeyCodeTest_t::PowerKeyLongPressed || keyboard == RobKeyCodeTest_t::PowerKeyShortPressed)
    {
        bool temp=test2Start;
        test2Start=!temp;
        if(test2Start)
        {
            xlogPtr->Debug("robot start!");
            xbasePtr->setRighBrushDuty(-100);
            xbasePtr->setMainBrushDuty(100);
            xbasePtr->mopLiftDown();
            xbasePtr->setMopDuty(100);
            xbasePtr->setFanDuty(100);        
            xbasePtr->startClean();         
        }
        else
        {
            xlogPtr->Debug("robot stop!");
            xbasePtr->stopClean();
            xbasePtr->mopLiftUp(); 
            delete smoothdata;
            smoothdata=nullptr;
            smoothdata=new int[smoothwindow];
            smoothTick=-1;
            isSmoothed=false;
            batteryLevelOutput=0;        
        }      
    }

    const HWInputDataInterface* HWInputData=xbasePtr->getHWInput();
    int adc=HWInputData->m_mcu_mr133.bat_adc;
    int inputFanDuty=int(HWInputData->m_mcu_mr133.fan_duty_cycle);
    int isCharging=int(HWInputData->m_mcu_mr133.err_status.bit_field.power_is_chagering);
    int batteryLevelBias=std::ceil(abs(inputFanDuty)/20);
    if(adc < 60 || adc > 130)
    {
        xlogPtr->Error("battery adc error:%d",adc);
        xbasePtr->UpdateOutput();
        return;
    }    
    //smooth
    smoothTick=(smoothTick+1)%smoothwindow;
    smoothdata[smoothTick]=adc;
    if(!isSmoothed)
    {
        if(smoothTick == smoothwindow-1)
        {
            isSmoothed=true;
            float sum=0;
            for(int i=0;i<smoothwindow;i++)
            {
                sum+=smoothdata[i];
            }
            smoothAvg=sum/smoothwindow;
        }
        xbasePtr->UpdateOutput();
        return;        
    }
    else
    {
        float temp=0;
        for(int i=0;i<smoothwindow;i++)
        {
            temp+=smoothdata[i];
        }
        smoothAvg=temp/smoothwindow;
    }
    //bias
    if(isCharging)
    {
        batteryLevelBias-=2;
    }
    int smooth=int(smoothAvg)+batteryLevelBias;
    if(smooth < 60 || smooth > 130)
    {
        xlogPtr->Error("battery smooth error:%d",smooth);
        xbasePtr->UpdateOutput();
        return;
    }
    //drop and up limit
    int batteryLevelData=batteryLevel[smooth];
    if(fabs(batteryLevelOutput) < 1e-6)
    {
        batteryLevelOutput=batteryLevelData;
    }
    else
    {
        if(!isCharging)
        {
            if(batteryLevelOutput > batteryLevelData-0.1)
            {
                batteryLevelOutput=batteryLevelOutput-batteryDropRate;
            }
            else
            {

            }
        }
        else
        {
            if(batteryLevelOutput >= batteryLevelData-0.1)
            {

            }       
            else
            {
                batteryLevelOutput=batteryLevelOutput+batteryUpRate;
            }        

        }
    }
    test2Outfile<<getTimeStap_ms()/1000.0<<"  "<<adc<<"  "<<inputFanDuty<<"  "<<isCharging<<"  "<<smoothAvg<<"  "<<smooth<<"  "<<batteryLevelData<<"  "<<batteryLevelOutput<<"  "<<std::ceil(batteryLevelOutput)<<std::endl;
    
    xbasePtr->UpdateOutput(); 
}

void testZhb_t::test3Init()
{
    xbasePtr->Init();
    xbasePtr->MotorEnable(true);
    xbasePtr->bumperAutoHandle(true);
    xbasePtr->OpenLaserSensors();  
    fd = open("/dev/xtgpio0", O_RDWR);
    readKeyData = 0;
    test3Start = false;
    test3Outfile.open("/app/fj212/bin/amclTest2.log");
    if(!test3Outfile.is_open())
    {
        xlogPtr->Error("fail to open file! ");
    }
    test3Outfile << std::fixed << std::setprecision(6);
    imuVector.clear();
    imuVector.resize(0);
    wRate=0.005;
    wUpdate=M_PI/180*60;
    vUpdate=0.1;
    wLimit=M_PI/6;
    wUnit=16.44*180/M_PI;
    accUnit=17085/9.8;
    startTime3=getTimeStap_ms()/1000.0;  
    vRate=0.005;
    vUpdate=0.3;
    vLimit=0.3;
    lastRotateDir = true;
    rotateDirSave = 0.0;
    dirChangeCount = 0;
    isBumped=false;
    currentSmoothwindowSize=50;
    currentSmoothDataLeft.clear();
    currentSmoothDataLeft.resize(currentSmoothwindowSize);
    currentSmoothDataRight.clear();
    currentSmoothDataRight.resize(currentSmoothwindowSize);    
    currentSmoothTick=-1;
    currentSmoothAvgLeft=0;
    currentSmoothAvgRight=0;
    isCurrentSmoothed=false;
    leftErrorTick=0;
    rightErrorTick=0;
    errorTickThreshold=100;
}

void testZhb_t::test3Handle()
{
    xbasePtr->UpdateInput();
    RobKeyCodeTest_t keyboard=getKeyboard();
    if(keyboard == RobKeyCodeTest_t::PowerKeyLongPressed || keyboard == RobKeyCodeTest_t::PowerKeyShortPressed)
    {
        bool temp=test3Start;
        test3Start=!temp;
        if(test3Start)
        {
            xlogPtr->Debug("robot start!");
            xbasePtr->MotorEnable(true);
            xbasePtr->bumperAutoHandle(true);
            startTime3=getTimeStap_ms()/1000.0; 
            currentSmoothDataLeft.clear();
            currentSmoothDataLeft.resize(currentSmoothwindowSize);
            currentSmoothDataRight.clear();
            currentSmoothDataRight.resize(currentSmoothwindowSize);            
            currentSmoothTick=-1;
            currentSmoothAvgLeft=0;
            currentSmoothAvgRight=0;
            isCurrentSmoothed=false;   
            leftErrorTick=0;
            rightErrorTick=0;                        
        }
        else
        {
            xlogPtr->Debug("robot stop!");
        }      
    }
    if(test3Start)
    {
        if((getTimeStap_ms()/1000.0-startTime3) > 20.0)
        {
            test3Start=false;
            xbasePtr->setSpeed(0.0,0.0);
            xbasePtr->MotorEnable(false);
        }       
        // if(wRate > 0)
        // {
        //     if(wUpdate < wLimit)
        //     {
        //         wUpdate+=wRate;
        //     }
        //     else
        //     {
        //         wRate=wRate*-1;
        //     }
        // }
        // else
        // {
        //     if(wUpdate > wLimit*-1)
        //     {
        //         wUpdate+=wRate;
        //     }
        //     else
        //     {
        //         wRate=wRate*-1;
        //     }
        // }
        // xlogPtr->Debug("speed w:%f",wUpdate);

        if(vRate > 0)
        {
            if(vUpdate < vLimit)
            {
                vUpdate+=vRate;
            }
            else
            {
                vRate=vRate*-1;
            }
        }
        else
        {
            if(vUpdate > vLimit*-1)
            {
                vUpdate+=vRate;
            }
            else
            {
                vRate=vRate*-1;
            }
        }
        xlogPtr->Debug("speed v:%f",vUpdate);
        xbasePtr->setSpeed(vUpdate,0.0);
        
        // xbasePtr->setSpeed(0.2,0);

#if 0
        SensorMsg::Bumper_t Bumpdata=xbasePtr->getBumpData(); 
        if(Bumpdata.leftBumped || Bumpdata.rightBumped || Bumpdata.isProcessing)
        {
            xlogPtr->Debug("bumped!,left:%d,right:%d,processing:%d",int(Bumpdata.leftBumped),int(Bumpdata.rightBumped),int(Bumpdata.isProcessing));
            isBumped=true;
            bumpTime3=getTimeStap_ms()/1000.0;
        }
        if(isBumped)
        {
            if((getTimeStap_ms()/1000.0-bumpTime3) < 1.0)
            {
                //xlogPtr->Debug("bump rotate!");                 
                xbasePtr->setSpeed(0.0,M_PI/3);
                xbasePtr->UpdateOutput();
                return;
            }    
            else
            {
                isBumped=false;
                xlogPtr->Debug("bump rotate done!");
            } 
        }

        SensorMsg::LaserData_t laserData = xbasePtr->getLaserData();
        float rhoMin=10000;
        float thetaMin=0.0;
        int num=-1;
        for(int i=0;i<laserData.range_num;i++)
        {
            //xlog->Info("laserData range=%f,theta=%f \n",laserData.ranges[i],-1.0*M_PI_2+resolution*i);
            if((laserData.ranges[i] < 0.1) || (laserData.ranges[i] > 2.0))
            {
                //xlog->Info("skip laserData! \n");
                continue;
            }
            else
            {
                float tempRho=laserData.ranges[i];
                if(tempRho<rhoMin)
                {
                    rhoMin=tempRho;
                    thetaMin= laserData.bearing[i];
                    num=i;
                }
            }
        }
        if(num < 0)
        {
            xlogPtr->Warn("do not find any vaild laser in laserscan");
        }

        NavMsg::VelCmd_t velCmd;
        bool rotateDir=true;
        velCmd.timestamp_us = getTimeStap_us();
        velCmd.vSpd=0.0;
        velCmd.wSpd=0.0;    
        //w
        if(thetaMin>dtor(10))
        {
            rotateDir=false;
            lastRotateDir=false;
        }
        else if(thetaMin<dtor(-10))
        {
            rotateDir=true;
            lastRotateDir=true;
        }
        else
        {
            rotateDir=lastRotateDir;
        }
        //rotate direction
        float direction=0.0;
        if(rotateDir)
        {
            direction=1.0;
        }
        else
        {
            direction=-1.0;
        }
        //v
        //xlogPtr->Debug("rhoMoin:%f,thetaMin:%f",rhoMin,thetaMin);
        if(rhoMin>0.2)
        {
            if(fabs(thetaMin)>dtor(10))
            {
                velCmd.vSpd=0.2;
                velCmd.wSpd=0;
            }
            else
            {
                velCmd.vSpd=0;
                velCmd.wSpd=dtor(45)*direction;
            }
        }
        else
        {
            velCmd.vSpd=0;
            velCmd.wSpd=dtor(45)*direction;
        }
        //avoid oscillation
        if(velCmd.vSpd<0.001)
        {
            if(fabs(rotateDirSave)<0.001)
            {

            }
            else
            {
                if(fabs(rotateDirSave-direction)>1.5)
                {
                    dirChangeCount++;
                    if(dirChangeCount>2)
                    {
                        direction=-direction;
                        velCmd.wSpd=dtor(45)*direction;
                    }
                }
            }
            rotateDirSave=direction;
        }
        else
        {
            rotateDirSave=0.0;
            dirChangeCount=0;
        }
        xbasePtr->setSpeed(velCmd.vSpd,velCmd.wSpd);
        //xlogPtr->Debug("send speed v:%f,w:%f",velCmd.vSpd,velCmd.wSpd);        
#endif

        imuVector.clear();
        imuVector.resize(0);
        int size=xbasePtr->getImuDataArray(imuVector);
        //xlogPtr->Debug("get imu data size:%d",size);
        float accCombine=0;
        const HWInputDataInterface* HWInputData=xbasePtr->getHWInput();
        float vFeedback=HWInputData->m_mcu_mr133.v_fdk_mm_s/1000.0;
        float wFeedback=HWInputData->m_mcu_mr133.w_fdk_mrad_s/1000.0;
        float xFeedBack=HWInputData->m_mcu_mr133.odometry_x_mm/1000.0;
        float yFeedBack=HWInputData->m_mcu_mr133.odometry_y_mm/1000.0;
        float xthetaFeedBack=HWInputData->m_mcu_mr133.odometry_yaw_mrad/1000.0;
        float q0,q1,q2,q3;
        int wl=int(HWInputData->m_mcu_mr133.left_wheel_current);
        int wr=int(HWInputData->m_mcu_mr133.right_wheel_current);
        int i=0;
        float sumL=0;
        float sumR=0;
        int suspendLeft=HWInputData->m_mcu_mr133.err_status.bit_field.left_wheel_suspend;
        int suspendRight=HWInputData->m_mcu_mr133.err_status.bit_field.right_wheel_suspend;
        for(int i=0;i<size;i++)
        {
            q0=imuVector[i].q0/10000.0;
            q1=imuVector[i].q1/10000.0;
            q2=imuVector[i].q2/10000.0;
            q3=imuVector[i].q3/10000.0;
            float rollTemp = atan2f(2.f * (q2 * q3 + q0 * q1), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
            float pitchTemp = asinf(2.f * (q0 * q2 - q1 * q3));
            float xoy=std::acos(std::cos(rollTemp)*std::cos(pitchTemp));
            //xlogPtr->Debug("[roll,pitch,xoy]=[%f,%f,%f]",rollTemp,pitchTemp,xoy);

            test3Outfile
            <<getTimeStap_ms()/1000.0<<"  "
            <<wl<<"  "<<wr<<"  "
            //<<rollTemp<<"  "<<pitchTemp<<"  "<<xoy
            <<"  "<<imuVector[i].gyro0<<"  "<<imuVector[i].gyro1<<"  "<<imuVector[i].gyro2
            <<"  "<<imuVector[i].acc0<<"  "<<imuVector[i].acc1<<"  "<<imuVector[i].acc2
            <<"  "<<vFeedback<<"  "<<wFeedback<<"  "<<xFeedBack<<"  "<<yFeedBack<<"  "<<xthetaFeedBack
            <<std::endl;
        }
        // if(suspendLeft || suspendRight)
        // {
        //     xlogPtr->Error("wheel suspend=[%d,%d]",suspendLeft,suspendRight);
        //     xbasePtr->UpdateOutput();
        //     return;    
        // }

        // if(fabs(vFeedback) < 0.01 && fabs(wFeedback) < 0.1/180*M_PI)
        // {
        //     xlogPtr->Debug("robot not move");
        //     xbasePtr->UpdateOutput();
        //     return;    
        // }

        // if(wl < 0 || wl > 255 || wr < 0 || wr > 255)
        // {
        //     xlogPtr->Error("input wheel current error=[%d,%d]",wl,wr);
        //     xbasePtr->UpdateOutput();
        //     return;      
        // }

        // //smooth
        // currentSmoothTick=(currentSmoothTick+1)%currentSmoothwindowSize;
        // currentSmoothDataLeft[currentSmoothTick]=wl;
        // currentSmoothDataRight[currentSmoothTick]=wr;            
        // if(!isCurrentSmoothed)
        // {
        //     if(currentSmoothTick == currentSmoothwindowSize-1)
        //     {
        //         xlogPtr->Debug("currentSmoothTick:%d",currentSmoothTick);
        //         isCurrentSmoothed=true;
        //         sumL=0;
        //         sumR=0;
        //         for(i=0;i<currentSmoothwindowSize;i++)
        //         {
        //             sumL+=currentSmoothDataLeft[i];
        //             sumR+=currentSmoothDataLeft[i];
        //         }
        //         currentSmoothAvgLeft=sumL/currentSmoothwindowSize;
        //         currentSmoothAvgRight=sumR/currentSmoothwindowSize;
        //     }
        //     xbasePtr->UpdateOutput();
        //     return;                 
        // }
        // else
        // {
        //     sumL=0;
        //     sumR=0;
        //     for(i=0;i<currentSmoothwindowSize;i++)
        //     {
        //         sumL+=currentSmoothDataLeft[i];
        //         sumR+=currentSmoothDataLeft[i];
        //     }
        //     currentSmoothAvgLeft=sumL/currentSmoothwindowSize;
        //     currentSmoothAvgRight=sumR/currentSmoothwindowSize;              
        //     xlogPtr->Debug("average current=[%f,%f]",currentSmoothAvgLeft,currentSmoothAvgRight);
        // }
        // if(currentSmoothAvgLeft < 0 || currentSmoothAvgLeft > 255 || currentSmoothAvgRight < 0 || currentSmoothAvgRight > 255)
        // {
        //     xlogPtr->Error("average wheel current error=[%d,%d]",wl,wr);
        //     xbasePtr->UpdateOutput();
        //     return;      
        // }

        // //check
        // if(currentSmoothAvgLeft < 10)
        // {
        //     leftErrorTick=0;
        // }
        // else if(currentSmoothAvgLeft < 25)
        // {
        //     leftErrorTick+=1;
        // }
        // else if(currentSmoothAvgLeft < 50)
        // {
        //     leftErrorTick+=2;
        // }
        // else
        // {   
        //     leftErrorTick+=3;
        // }

        // if(currentSmoothAvgRight < 10)
        // {
        //     rightErrorTick=0;
        // }
        // else if(currentSmoothAvgRight < 25)
        // {
        //     rightErrorTick+=1;
        // }
        // else if(currentSmoothAvgRight < 50)
        // {
        //     rightErrorTick+=2;
        // }
        // else
        // {   
        //     rightErrorTick+=3;
        // }
        // xlogPtr->Debug("error tick:[%d,%d]",leftErrorTick,rightErrorTick);
        // if(leftErrorTick > errorTickThreshold || rightErrorTick > errorTickThreshold)
        // {
        //     leftErrorTick=errorTickThreshold;
        //     rightErrorTick=errorTickThreshold;
        //     xlogPtr->Error("slip trigger,[%d,%d]",leftErrorTick,rightErrorTick);
        // }
        
    }
    else
    {
        test3Start=false;
        xbasePtr->setSpeed(0.0,0.0);
        xbasePtr->MotorEnable(false);
    }   
    xbasePtr->UpdateOutput();
}


void testZhb_t::test4Init()
{
    xbasePtr->Init();
   // xbasePtr->OpenLaserSensors();   
    fd = open("/dev/xtgpio0", O_RDWR);
    xlogPtr->Debug("test4 init");
    xbasePtr->setRighBrushDuty(0);
    xbasePtr->setMainBrushDuty(0);
    xbasePtr->mopLiftDown();
    xbasePtr->setMopDuty(0);
    xbasePtr->setFanDuty(0);   
}
void testZhb_t::test4Handle()
{
    static uint32_t tick = 0;
    xbasePtr->UpdateInput();
    RobKeyCodeTest_t keyboard=getKeyboard();
    if(keyboard == RobKeyCodeTest_t::FnKeyShortPressed || keyboard == RobKeyCodeTest_t::PowerKeyShortPressed)
    {
        xlogPtr->Debug("t4: KeyPresed");
        tick = 0;
        xbasePtr->setRighBrushDuty(0);
        xbasePtr->setMBrushUpDownDuty(0);
        xbasePtr->setMopDuty(0);
        xbasePtr->setFanDuty(0);   
        if(!case4Start)
        {
            xlogPtr->Debug("robot start!");
            case4Start=true;
            xbasePtr->startClean();
        }
        else
        {
            xlogPtr->Debug("robot stop!");
            case4Start = false;  
            xbasePtr->stopClean();
            
        }
    }

    bool limit1 = false;
    bool limit2 = false;
    if(xbasePtr->getRobotStatus().mBrushLimit1)
        limitTick1++;
    else 
        limitTick1--;

    if(xbasePtr->getRobotStatus().mBrushLimit2)
        limitTick2++;
    else 
        limitTick2--;

    if(limitTick1 < -10)
    {
        limit1 = false;
        limitTick1 = -10;
    }  
    else if(limitTick1 > 10)
    {
        limit1 = true;
        limitTick1 = 10;
    }

    if(limitTick2 < -10)
    {
        limit2 = false;
        limitTick2 = -10;
    }
    else if(limitTick2 > 10)
    {
        limit2 = true;
        limitTick2 = 10;
    }


    if(loopTick / 2 == 10) // 10 loop, 1min idle
    {
        loopTick = 0;
        idleTick = 50 * 60; 
        xbasePtr->setMBrushUpDownDuty(0);
    }
        

    if(0 != idleTick) 
    {
        idleTick--;
        xbasePtr->setMBrushUpDownDuty(0);
    }

    if(case4Start && (0 == idleTick))
    {
        tick++;
        if(tick < 50 * 10)
        {
            if(!limit1)
            {
                if( tick > 50 * 10 - 5) // 4.9s
                {
                    //case4Start = false;
                    //system("aplay /app/fj212/resource/robot_voice/86.wav");
                    loopTick++;
                    xlogPtr->Error("tick = %d, loopTick=%d, duty = -90 timeout!", tick, loopTick);
                    tick = 50 * 10;
                    
                    //xbasePtr->stopClean();
                }
                else 
                {
                    //printf("tick=%d mBrushUpDownDuty =-90\r\n", tick);
                    xbasePtr->setMBrushUpDownDuty(-90);
                }
                    
            }
            else 
            {  
                loopTick++;
                xlogPtr->Debug("tick=%d loopTick=%d, change 90 by limiter!", tick, loopTick);
                tick = 50 * 10;
            }
        }
        else if(tick < 50 * 10 * 2)
        {
            if(!limit2)
            {
                if( tick > 50 * 10 * 2 - 5) 
                {
                    //case4Start = false;
                    //system("aplay /app/fj212/resource/robot_voice/87.wav");
                    loopTick++; 
                    xlogPtr->Error("tick = %d, loopTick=%d, duty = 90 timeout!", tick, loopTick);
                    
                    tick = 0;
                    //xbasePtr->stopClean();
                }
                else 
                {
                    //printf("tick=%d mBrushUpDownDuty =90\r\n", tick);
                    xbasePtr->setMBrushUpDownDuty(90);
                    
                }
                   
            }
            else 
            {
                
                loopTick++;
                xlogPtr->Debug("tick=%d loopTick=%d, change -90 by limiter!", tick, loopTick);
                tick = 0;
            }
        }
        else 
        {
            tick = 0; // reset
        }
    }
    else
    {
        tick = 0;
        loopTick = 0;
        xbasePtr->stopClean();
    }
    xbasePtr->UpdateOutput(); 
}

int main(int argc, char *argv[])
{
    testZhb_t* testZhbPtr=new testZhb_t();
    XLog* xlogPtr=testZhbPtr->getXlogPtr();
    //get test mode
    TestMode_t testMode=TestMode_t::none;
    if(argc != 2)
    {
        xlogPtr->Debug("test mode argc error!");
        exit(-1);
    }
    else
    {
        if(std::string(argv[1]) == std::string("a1"))
        {
            testMode=TestMode_t::case1;
        }
        else if(std::string(argv[1]) == std::string("a2"))
        {
            testMode=TestMode_t::case2;
        }
        else if(std::string(argv[1]) == std::string("a3"))
        {
            testMode=TestMode_t::case3;
        }
        else if(std::string(argv[1]) == std::string("a4"))
        {
            testMode=TestMode_t::case4;
        }
        else if(std::string(argv[1]) == std::string("a5"))
        {
            testMode=TestMode_t::case5;
        }
        else if(std::string(argv[1]) == std::string("test1"))
        {
            testMode=TestMode_t::test1;
        } 
        else if(std::string(argv[1]) == std::string("test2"))
        {
            testMode=TestMode_t::test2;
        }
        else if(std::string(argv[1]) == std::string("test3"))
        {
            testMode=TestMode_t::test3;
        }
        else if(std::string(argv[1]) == std::string("test4"))
        {
            testMode=TestMode_t::test4_mBrush;
        }                              
        else
        {
            xlogPtr->Debug("test mode argv[] error!"); 
            exit(-1);
        }      
    }

    testZhbPtr->init(testMode);
    sleep_ms(1000);

    bool isRunning=true;      
    while(isRunning)
    {
        testZhbPtr->handle(testMode);
    }

    delete testZhbPtr;
    return 0;
}





