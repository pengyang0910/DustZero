#include "relocalization_task.h"

void RelocalizationTask_t::PreStart()
{
    odomData=bridgePt->GetXBasePt()->getOdomData();
    laserData = bridgePt->GetXBasePt()->getLaserData();
    robotStatus=bridgePt->GetXBasePt()->getRobotStatus();    
    //global init
    taskStep=1;  
    
    //select position
    startPose=odomData;
    avoidStartTime=getTimeStap_ms()/1000.0;
    bumpDir=0;
    isbumped=false;
    bumpTs=getTimeStap_ms()/1000.0;
    isBestDirReached=false;
    isBestDirFind=false;
    bestDir=0;
    lastBestDirOdom=odomData;
    lastBumpDir=0;
    repeatedDirCount=0;
    isOscillation=false;
    oscillationTs=getTimeStap_ms()/1000.0;
    oscillationDir=0; 
    moveTs=getTimeStap_ms()/1000.0;

    //sampling
    samplingStartTime=getTimeStap_ms()/1000.0;  
    samplePointCloudCount=-1;
    sampleRotateCount=-1;
    sampleLastPose=odomData;
    sampleRotateSum=0.0;
    laserConcatenationSampleOrigin.resize(0);
    laserConcatenationResult.resize(0);
    fixedTheta=odomData.pa;

    //wait for result
    waitForResultStartTime=getTimeStap_ms()/1000.0;
    isAmclCoreHandleDone=false;
    isAmclCoreHandleSuccess=false;

    //success check
    successCheckStartTime=getTimeStap_ms()/1000.0; 
    successCheckStartPose.px=odomData.px;
    successCheckStartPose.py=odomData.py;
    successCheckStartPose.pa=odomData.pa;
    successCheckLastPose.px=odomData.px;
    successCheckLastPose.py=odomData.py;
    successCheckLastPose.pa=odomData.pa;  
    moveDist=0.0;  
    isScoreSampling=false;
    scoreSample.resize(0);
    ScoreCount=0;

    //global reentrant
    retryCount=0;    
  
    outfile2.open("/tmp/relocalizationTest2.log");
    if(!outfile2.is_open())
    {
        xlog->Error("fail to open /tmp/relocalizationTest.log  ! ");
    }
    outfile2 << std::fixed << std::setprecision(6);
    step2Tick=0;       

    xlog->Debug("relocalization task prestart");
}

void RelocalizationTask_t::PreStop()
{
    RobotMsg::HackCmd_t hackCmd;
    hackCmd.data=132;
    lcm_m.publish(LCM_CHANNEL_HACK, &hackCmd);
    xlog->Debug("relocalization task stop ahead,enable relocalization");     

    if(outfile2.is_open())
    {
        outfile2.close();
    }                 
}

void RelocalizationTask_t::PreResume()
{
}

void RelocalizationTask_t::PreSuspend()
{
}

void RelocalizationTask_t::MainLoop()
{    
    static int tick=0;
    lcm_m.handleTimeout(1);
    //global vaiable
    odomData=bridgePt->GetXBasePt()->getOdomData();
    laserData = bridgePt->GetXBasePt()->getLaserData();
    robotStatus=bridgePt->GetXBasePt()->getRobotStatus();

    if(retryCount >= retryCountThreshold)
    {
        xlog->Error("retry count >= %d,Stop()",retryCountThreshold);
        relocalizationFailedAndStop();
        return;        
    }
    else
    {
        //select position
        if(taskStep == 1)
        {
            if(selectPositionCheck())
            {
                selectPositionHandle();
            }
            return;
        }
        //sampling
        else if(taskStep == 2)
        {
            step2Tick+=1;
            quaternionVector[0]=robotStatus.q0;
            quaternionVector[1]=robotStatus.q1;
            quaternionVector[2]=robotStatus.q2;
            quaternionVector[3]=robotStatus.q3;
            calRPY(quaternionVector,rpyVector);
            //recordStep2ToFile();
            
            if(samplingCheck())
            {
                samplingHandle();
            }
        }
        //wait for result
        else if(taskStep == 3)
        {
            if(waitForResultCheck())
            {
                waitForResultHandle();
            }
        }
        //success check
        else if(taskStep == 4)
        {
            if(lastCheck())
            {
                lastHandle();
            }
        }
    }
}

void RelocalizationTask_t::teleopCmdUpdate(const lcm::ReceiveBuffer* rbuf,
                    const std::string &channel,
                    const RobotMsg::HackCmd_t *msg)
{
    switch (msg->data)
    {
    case 304:
        xlog->Debug("relocalization success 304");    
        isAmclCoreHandleDone=true;
        isAmclCoreHandleSuccess=true;
        break;   
    case 305:
        xlog->Debug("relocalization failed 305");   
        isAmclCoreHandleDone=true;
        isAmclCoreHandleSuccess=false;
        break;                          
    default:
        break;
    }
}

void RelocalizationTask_t::amclScoreUpdate(const lcm::ReceiveBuffer* rbuf, 
                    const std::string &channel, 
                    const NavMsg::SlamPoseInfo_t *msg)
{
    if(isScoreSampling)
    {
        scoreSample.push_back(msg->score);
        xlog->Debug("amcl score:%f",msg->score);
        if(msg->score >= 0.9)
        {
            ScoreCount+=3;
        }
        else if(msg->score >= successCheckScoreThreshold)
        {
            ScoreCount+=1;
        }
    }
}

void RelocalizationTask_t::calRPY(std::vector<float> &quaternion,std::vector<float> &rpy)
{   
    float q0 = quaternion[0] / 10000.0;
    float q1 = quaternion[1] / 10000.0;
    float q2 = quaternion[2] / 10000.0;
    float q3 = quaternion[3] / 10000.0;
    float X = 0, Y = 0, Z = 0;
    float sum = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 / sum;
    q1 = q1 / sum;
    q2 = q2 / sum;
    q3 = q3 / sum;
    const double Epsilon = 0.0009765625f;
    const double Threshold = 0.5f - Epsilon;
    double test = q3 * q1 - q0 * q2;
    if(test > Threshold || test < -Threshold) {
        int sign = 0;
        if (test > 0)
            sign = 1;
        else if (test < 0)
            sign = -1;
        X = -2*atan2(q0, q3) *sign;
        Y = (M_PI / 2.0) * sign;
        Z = 0;
    }else{
        X = -atan2(2*(q2*q3 + q0*q1), 1 - 2*(q1*q1 + q2*q2) );
        Y = asin(-2*(q0*q2 - q1*q3));
        Z = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3) );
    }
    rpy[0] = X;
    rpy[1] = Y;
    rpy[2] = Z;
}

void RelocalizationTask_t::relocalizationFailedAndStop()
{
    xlog->Debug("relocalization result failed");
    TuyaMsg::TaskFinishStatus_t taskFinishStatus;
    taskFinishStatus.key=(int)TuyaMsg::TaskFinishStatusKey_t::RelocalizationFaile;
    taskFinishStatus.value=true;
    lcm_m.publish("Localization2Nav", &taskFinishStatus);
    RobotMsg::HackCmd_t hackCmd;
    hackCmd.data=306;
    lcm_m.publish(LCM_CHANNEL_HACK, &hackCmd);              
    Stop();
}

void RelocalizationTask_t::relocalizationSuccessAndStop()
{
    xlog->Debug("relocalization result success");
    TuyaMsg::TaskFinishStatus_t taskFinishStatus;
    taskFinishStatus.key=(int)TuyaMsg::TaskFinishStatusKey_t::RelocalizationSuccess;
    taskFinishStatus.value=true;
    lcm_m.publish("Localization2Nav", &taskFinishStatus); 
    RobotMsg::HackCmd_t hackCmd;
    hackCmd.data=307;
    lcm_m.publish(LCM_CHANNEL_HACK, &hackCmd);
    Stop();
}

void RelocalizationTask_t::resetTask()
{
    xlog->Debug("reset task status");
    //init
    taskStep=1;  
    
    //select position
    startPose=odomData;
    avoidStartTime=getTimeStap_ms()/1000.0;
    bumpDir=0;
    isbumped=false;
    bumpTs=getTimeStap_ms()/1000.0;
    isBestDirReached=false;
    isBestDirFind=false;
    bestDir=0;
    lastBestDirOdom=odomData;
    lastBumpDir=0;
    repeatedDirCount=0;
    isOscillation=false;
    oscillationTs=getTimeStap_ms()/1000.0;
    oscillationDir=0; 
    moveTs=getTimeStap_ms()/1000.0; 

    //sampling
    samplingStartTime=getTimeStap_ms()/1000.0;  
    samplePointCloudCount=-1;
    sampleRotateCount=-1;
    sampleLastPose=odomData;
    sampleRotateSum=0.0;
    laserConcatenationSampleOrigin.resize(0);
    laserConcatenationResult.resize(0);
    fixedTheta=odomData.pa;

    //wait for result
    waitForResultStartTime=getTimeStap_ms()/1000.0;
    isAmclCoreHandleDone=false;
    isAmclCoreHandleSuccess=false;

    //success check
    successCheckStartTime=getTimeStap_ms()/1000.0; 
    successCheckStartPose.px=odomData.px;
    successCheckStartPose.py=odomData.py;
    successCheckStartPose.pa=odomData.pa;
    successCheckLastPose.px=odomData.px;
    successCheckLastPose.py=odomData.py;
    successCheckLastPose.pa=odomData.pa;    
    moveDist=0.0; 
    isScoreSampling=false;
    scoreSample.resize(0);
    ScoreCount=0;    
}

bool RelocalizationTask_t::selectPositionCheck()
{
    //overtime
    if(fabs(getTimeStap_ms()/1000.0-avoidStartTime) > avoidTimeoutThreshold)
    {
        retryCount+=1;
        xlog->Debug("relocalization select position over time,cost time:%f,retry count:%d"\
                ,fabs(getTimeStap_ms()/1000.0-avoidStartTime),retryCount);
        resetTask();
        return false;
    }
    else
    {
        float tempDist=sqrt((startPose.px-odomData.px)*(startPose.px-odomData.px)+(startPose.py-odomData.py)*(startPose.py-odomData.py));
        //xlog->Debug("move dist:%f",tempDist);
        if(tempDist > avoidMaxDist)
        {
            retryCount+=1;
            xlog->Debug("relocalization select position over range,move dist:%f,retry count:%d"\
                        ,tempDist,retryCount);                
            resetTask();
            return false;
        }
        else if(tempDist > avoidMinDist)
        {
            quaternionVector[0]=robotStatus.q0;
            quaternionVector[1]=robotStatus.q1;
            quaternionVector[2]=robotStatus.q2;
            quaternionVector[3]=robotStatus.q3;
            calRPY(quaternionVector,rpyVector);                    
            if(fabs(rpyVector[0]) > steadyRollPitchThreshold || fabs(rpyVector[1]) > steadyRollPitchThreshold)
            {
                xlog->Debug("roll pitch out range:[%f,%f]",rpyVector[0],rpyVector[1]);
            }
            else
            {
                xlog->Debug("select position success,start sampling");
                resetTask();
                taskStep=2;
                RobotMsg::HackCmd_t hackCmd;
                hackCmd.data=134;
                lcm_m.publish(LCM_CHANNEL_HACK, &hackCmd);
                xlog->Debug("reset amcl fixed point"); 
                return false;                   
            }
        }
        else
        {

        }
    }
    return true;
}

bool RelocalizationTask_t::samplingCheck()
{
    if(fabs(getTimeStap_ms()/1000.0-samplingStartTime) > samplingTimeoutThreshold)
    {
        retryCount+=1;
        xlog->Debug("relocalization sampling over time,cost time:%f,retry count:%d"\
                ,fabs(getTimeStap_ms()/1000.0-samplingStartTime),retryCount);
        resetTask();
        return false;                
    }
    else
    {
        if(samplePointCloudCount < 0)
        {

        }
        else if(samplePointCloudCount < sampleMinPointCloudCountThreshold)
        {
            retryCount+=1;
            xlog->Debug("relocalization sampling not enough,count:%d,retry count:%d"\
                    ,samplePointCloudCount,retryCount);
            resetTask();
            return false;                       
        }
        else
        {
            xlog->Debug("sampling success,wait for result");
            SensorMsg::PointCloud_t relocalizationPC;
            relocalizationPC.timestamp_us=getTimeStap_us();
            if(triggerMode == 1)
            {
                relocalizationPC.name="globalSearch";
            }
            else if(triggerMode == 2)
            {
                relocalizationPC.name="localSearch";
            }
            relocalizationPC.pointsNum=laserConcatenationResult.size();
            relocalizationPC.x.resize(laserConcatenationResult.size());
            relocalizationPC.y.resize(laserConcatenationResult.size());
            relocalizationPC.z.resize(laserConcatenationResult.size());
            for(int i=0;i<laserConcatenationResult.size();i++)
            {
                relocalizationPC.x[i]=laserConcatenationResult[i].v[0];
                relocalizationPC.y[i]=laserConcatenationResult[i].v[1];
                relocalizationPC.z[i]=laserConcatenationResult[i].v[2];
            }
            bridgePt->GetLcmPt()->publish("relocalizationPC",&relocalizationPC);  
            resetTask();
            taskStep=3;             
            return false;                          
        }
    }    
    return true;
}

bool RelocalizationTask_t::waitForResultCheck()
{
    if(fabs(getTimeStap_ms()/1000.0-waitForResultStartTime) > waitForResultTimeoutThreshold)
    {
        retryCount+=1;
        xlog->Debug("relocalization amcl core handle over time,cost time:%f,retry count:%d"\
                ,fabs(getTimeStap_ms()/1000.0-waitForResultStartTime),retryCount);
        resetTask();
        return false;                
    }
    else
    {
        if(!isAmclCoreHandleDone)
        {

        }
        else
        {
            if(isAmclCoreHandleSuccess)
            {
                resetTask();
                if(isSkipLastCheck)
                {   
                    relocalizationSuccessAndStop();
                }
                else
                {
                    taskStep=4;
                    isScoreSampling=true;                  
                    return false;
                }
            }
            else
            {
                retryCount+=1;
                xlog->Debug("relocalization match failed,retry count:%d",retryCount);
                resetTask();
                return false;                           
            }
        }
    }
    return true;
}

void RelocalizationTask_t::selectPositionHandle()
{
    //step1,bumped
    SensorMsg::Bumper_t bumpdata=bridgePt->GetXBasePt()->getBumpData();
    if(bumpdata.leftBumped && bumpdata.rightBumped)
    {
        bumpDir=1;
    }
    else if(bumpdata.leftBumped)
    {
        bumpDir=-1;
    }
    else if(bumpdata.rightBumped)
    {
        bumpDir=1;
    }
    else
    {
        bumpDir=bumpDir;
    }

    if(bumpdata.leftBumped || bumpdata.rightBumped)
    {
        if(!isbumped)
        {
            //first bump
        }
        bumpTs=getTimeStap_ms()/1000.0;
        isbumped=true;
    }
    if(isbumped)
    {
        moveTs=getTimeStap_ms()/1000.0;
        if((getTimeStap_ms()/1000.0-bumpTs) < 0.5)//over oscillation threshold
        {
            float w=M_PI/4;
            if(bumpDir == -1)
            {
                w=-w;
            }
            bridgePt->GetXBasePt()->setSpeed(0.0,w);//to min resolution
            return;
        }
        else
        {
            int currentBumpDir=bumpDir;
            if(currentBumpDir*lastBumpDir < 0)
            {
                repeatedDirCount+=1;
                if(repeatedDirCount > 4)
                {
                    repeatedDirCount=0;
                    isOscillation=true;
                    oscillationTs=getTimeStap_ms()/1000.0;
                    oscillationDir=currentBumpDir;
                }
            }
            else
            {
                repeatedDirCount=0;
            }
            lastBumpDir=currentBumpDir;
            isbumped=false;
            xlog->Debug("bump rotate done");
        }
    }
    if(isOscillation)
    {
        moveTs=getTimeStap_ms()/1000.0;
        if((getTimeStap_ms()/1000.0-oscillationTs) < 3.0)
        {
            if(oscillationDir > 0)
            {
                bridgePt->GetXBasePt()->setSpeed(0.0,M_PI/3);
            }
            else
            {
                bridgePt->GetXBasePt()->setSpeed(0.0,-M_PI/3);
            }
            return;
        }
        else
        {
            isOscillation=false;
            xlog->Debug("oscillation rotate done");
        }
    }
    //step2,find direction
    if(!isBestDirReached)
    {
        if(isBestDirFind)
        {
            float reachableDirRotateTheta=ClipAngle(odomData.pa-lastBestDirOdom.pa);
            //xlog->Debug("odom theta:%f,rotate sum:%f",odomData.pa,reachableDirRotateTheta);
            if(fabs(reachableDirRotateTheta) > fabs(bestDir))
            {
                xlog->Debug("reach best direction done");
                isBestDirReached=true;
            }
            else
            {
                if(bestDir > 0)
                {
                    bridgePt->GetXBasePt()->setSpeed(0.0,M_PI/4);
                }
                else
                {
                    bridgePt->GetXBasePt()->setSpeed(0.0,-M_PI/4);
                }
            }
            return;
        }
        else
        {
            isBestDirFind=calDirectionWithLaser(bestDir,laserData);
            bridgePt->GetXBasePt()->setSpeed(0.0,M_PI/3); 
        }
    }
    else
    {
        if((getTimeStap_ms()/1000.0-moveTs) > 1.0)
        {
            repeatedDirCount=0;
            moveTs=getTimeStap_ms()/1000.0;
        }
        bridgePt->GetXBasePt()->setSpeed(0.15,0);
    }
}

void RelocalizationTask_t::samplingHandle()
{
    if(sampleRotateCount < laserPartNum-1)
    {
        if(fabs(odomData.pa-sampleLastPose.pa) < 5.0)
        {
            sampleRotateSum+=fabs(odomData.pa-sampleLastPose.pa);
        }
        else
        {
            sampleRotateSum+=M_PI*2-fabs(odomData.pa-sampleLastPose.pa);
        }
        if(sampleRotateSum >= M_PI*2.0/laserPartNum)
        {
            //1 rotate
            sampleRotateCount+=1;
            xlog->Debug("rotate count:%d,odom theta:%f,rotate sum:%f",sampleRotateCount,odomData.pa,sampleRotateSum);
            sampleLastPose=odomData;
            sampleRotateSum=0.0;

            //2 buffer laser with odom
            {
                std::pair<SensorMsg::LaserData_t,pf_vector_t> tempLaserwithOdom;
                tempLaserwithOdom.first=laserData;
                pf_vector_t tempOdomv3;
                tempOdomv3.v[0]=0;
                tempOdomv3.v[1]=0;
                tempOdomv3.v[2]=ClipAngle(odomData.pa-fixedTheta);
                tempLaserwithOdom.second=tempOdomv3;
                laserConcatenationSampleOrigin.push_back(tempLaserwithOdom);
            }
        }
    }
    else
    {
        //// laser concatenation
        //3 cal cut radian
        if(laserConcatenationSampleOrigin.size() <= 1)
        {
            xlog->Error("laser part num <= 1");
            exit(1);
        }
        std::vector<float> cutRadian(laserConcatenationSampleOrigin.size(),0.0);
        pf_vector_t laserMountV;
        laserMountV.v[0]=bridgePt->GetXBasePt()->getLaserMountPose().px;
        laserMountV.v[1]=bridgePt->GetXBasePt()->getLaserMountPose().py;
        laserMountV.v[2]=bridgePt->GetXBasePt()->getLaserMountPose().pa;
        for(int i=0;i<laserConcatenationSampleOrigin.size();i++)
        {
            pf_vector_t leftOdom=laserConcatenationSampleOrigin[i].second;
            pf_vector_t rightOdom;
            if(i != laserConcatenationSampleOrigin.size()-1)
            {
                rightOdom=laserConcatenationSampleOrigin[i+1].second;
            }
            else
            {
                rightOdom=laserConcatenationSampleOrigin[0].second;
            }
            if(leftOdom.v[2] > rightOdom.v[2])
            {
                float la=M_PI-leftOdom.v[2];
                float ra=M_PI+rightOdom.v[2];
                if(la > ra)
                {
                    cutRadian[i]=(leftOdom.v[2]+rightOdom.v[2])*0.5+M_PI;
                }
                else
                {
                    cutRadian[i]=(leftOdom.v[2]+rightOdom.v[2])*0.5-M_PI;
                }
            }
            else
            {
                cutRadian[i]=(leftOdom.v[2]+rightOdom.v[2])*0.5;
            }

        }
        for(int i=0;i<laserConcatenationSampleOrigin.size();i++)
        {
            xlog->Debug("seq:%d,origin odom [%f,%f,%f]",i,\
                laserConcatenationSampleOrigin[i].second.v[0],laserConcatenationSampleOrigin[i].second.v[1],laserConcatenationSampleOrigin[i].second.v[2]);
        }
        for(int i=0;i<cutRadian.size();i++)
        {
            xlog->Debug("seq:%d,right part radian:%f",i,cutRadian[i]);
        }

        //4 cal laser coordinate in each cut radian part
        outfile2<<"start process laser"<<std::endl;
        for(int i=0;i<laserConcatenationSampleOrigin.size();i++)
        {
            SensorMsg::LaserData_t tempLaser=laserConcatenationSampleOrigin[i].first;
            pf_vector_t tempPose=laserConcatenationSampleOrigin[i].second;
            tempPose=pf_vector_coord_add(laserMountV,tempPose);
            int vaildLaserCount=0;

            //split
            float leftTheta;
            float rightTheta;
            bool isSplit=false;
            if(i != 0)
            {
                leftTheta=cutRadian[i-1];
                rightTheta=cutRadian[i];
            }
            else
            {
                leftTheta=cutRadian.back();
                rightTheta=cutRadian[i];
            }
            if(leftTheta > rightTheta)
            {
                isSplit=true;
            }
            xlog->Debug("left:%f,right:%f",leftTheta,rightTheta);

            outfile2<<"laserdata"<<" "<<i<<" "<<tempLaser.range_num<<" "<<laserConcatenationSampleOrigin[i].first.timestamp_us<<" "<<tempPose.v[0]<<" "<<tempPose.v[1]<<" "<<tempPose.v[2]<<std::endl;
            for(int j=0;j<tempLaser.range_num;j++)
            {
                outfile2<<tempLaser.ranges[j]<<" "<<tempLaser.bearing[j]<<std::endl; 
                pf_vector_t tempPose2;
                // tempPose.v[0]=0;
                // tempPose.v[1]=0;
                tempPose2.v[0]=tempPose.v[0]+tempLaser.ranges[j]*cos(tempPose.v[2]+tempLaser.bearing[j]);
                tempPose2.v[1]=tempPose.v[1]+tempLaser.ranges[j]*sin(tempPose.v[2]+tempLaser.bearing[j]);
                tempPose2.v[2]=atan2(tempPose2.v[1]-laserConcatenationSampleOrigin[i].second.v[1],\
                                    tempPose2.v[0]-laserConcatenationSampleOrigin[i].second.v[0]);
            //    outfile2<<tempPose2.v[0]<<" "<<tempPose2.v[1]<<" "<<tempPose2.v[2]<<std::endl;              
            }
            outfile2<<"valid laser"<<std::endl;
            int invalidCnt=0;

            for(int j=0;j<tempLaser.range_num;j++)
            {
                if(/* tempLaser.ranges[j] != tempLaser.ranges[j] ||  */tempLaser.ranges[j] < 0.15)
                {
                    invalidCnt++;
                    continue;
                }
                pf_vector_t tempPose2;
                // tempPose.v[0]=0;
                // tempPose.v[1]=0;
                tempPose2.v[0]=tempPose.v[0]+tempLaser.ranges[j]*cos(tempPose.v[2]+tempLaser.bearing[j]);
                tempPose2.v[1]=tempPose.v[1]+tempLaser.ranges[j]*sin(tempPose.v[2]+tempLaser.bearing[j]);
                tempPose2.v[2]=atan2(tempPose2.v[1]-laserConcatenationSampleOrigin[i].second.v[1],\
                                    tempPose2.v[0]-laserConcatenationSampleOrigin[i].second.v[0]);


                //filter
                //if((!isSplit && tempPose2.v[2] > leftTheta && tempPose2.v[2] < rightTheta) || \
                //    (isSplit && (tempPose2.v[2] > leftTheta || tempPose2.v[2] < rightTheta)))
                if (tempLaser.bearing[j]>=-0.7&&tempLaser.bearing[j]<=0.7)
                {
                    outfile2<<tempPose2.v[0]<<" "<<tempPose2.v[1]<<" "<<tempPose2.v[2]<<std::endl; 
                    laserConcatenationResult.push_back(tempPose2);
                    vaildLaserCount++;
                }
                //xlog->Debug("seq:%d,valid count index:%d,[%f,%f,%f]",j,vaildLaserCount,tempPose2.v[0],tempPose2.v[1],tempPose2.v[2]);
            }
            xlog->Debug("part:%d,valid laser count:%d invalid is %d",i,vaildLaserCount,invalidCnt);
        }
        outfile2<<"end process laser"<<std::endl;
        xlog->Debug("total size:%d",laserConcatenationResult.size());
        
        int laserNum=2400;
        float* laserScan=(float*) malloc(laserNum*sizeof(float));
        float laserResolution=2.0*M_PI/laserNum;
        for(int i=0;i<2400;i++)
        {
            laserScan[i]=0;
        }

        int index=0;
        float rho=0;
        for(int i=0;i<laserConcatenationResult.size();i++)
        {
            if(laserConcatenationResult[i].v[2] < 0.0)
            {
                laserConcatenationResult[i].v[2]+=M_PI*2.0;
            }
            if(laserConcatenationResult[i].v[2] >= M_PI*2.0)
            {
                laserConcatenationResult[i].v[2]-=M_PI*2.0;
            }
            index=laserConcatenationResult[i].v[2]/laserResolution;
            if(index < 0 && index >= 2400)
                continue;
            rho=sqrt(laserConcatenationResult[i].v[0]*laserConcatenationResult[i].v[0]+laserConcatenationResult[i].v[1]*laserConcatenationResult[i].v[1]);
            laserScan[index]=rho;
        } 

        int validCnt =0;
        for (size_t i = 0; i < 2400; i++)
        {
          if(laserScan[i]!=0)
          {
             validCnt++;
          }
        }
        xlog->Debug("sample size:%d",validCnt);
        //samplePointCloudCount=laserConcatenationResult.size();
        samplePointCloudCount = validCnt;
        bridgePt->GetXBasePt()->setSpeed(0,0);
        return;
    }
    sampleLastPose=odomData;
    float sampleWSpeed=M_PI/180.0*75.0;
    bridgePt->GetXBasePt()->setSpeed(0,sampleWSpeed);
}

void RelocalizationTask_t::waitForResultHandle()
{

}

bool RelocalizationTask_t::lastCheck()
{
    if(fabs(getTimeStap_ms()/1000.0-successCheckStartTime) > successCheckTimeThreshold)
    {
        retryCount+=1;
        xlog->Debug("relocalization last check handle over time,cost time:%f,retry count:%d"\
                ,fabs(getTimeStap_ms()/1000.0-successCheckStartTime),retryCount);
        resetTask();
        return false;   
    }
    float transDist=sqrt((odomData.px-successCheckStartPose.px)*(odomData.px-successCheckStartPose.px) \
                        +(odomData.py-successCheckStartPose.py)*(odomData.py-successCheckStartPose.py));
    moveDist+=sqrt((odomData.px-successCheckLastPose.px)*(odomData.px-successCheckLastPose.px) \
                        +(odomData.py-successCheckLastPose.py)*(odomData.py-successCheckLastPose.py));
    successCheckLastPose=odomData;                    
    if(transDist > successCheckTransDistThreshold &&  moveDist > successCheckMoveDistThreshold)
    {
        xlog->Debug("trans:%f,move:%f,count:%d",transDist,moveDist,ScoreCount);
        if(ScoreCount > 20)
        {
            relocalizationSuccessAndStop();
        }
        else
        {
            retryCount+=1;
            xlog->Debug("amcl low score,retry count:%d",retryCount);
            resetTask();
        }
        return false; 
    }
    return true;
}

void RelocalizationTask_t::lastHandle()
{
    //step1,bumped
    xlog->Debug("last check!");
    SensorMsg::Bumper_t bumpdata=bridgePt->GetXBasePt()->getBumpData();
    if(bumpdata.leftBumped && bumpdata.rightBumped)
    {
        bumpDir=1;
    }
    else if(bumpdata.leftBumped)
    {
        bumpDir=-1;
    }
    else if(bumpdata.rightBumped)
    {
        bumpDir=1;
    }
    else
    {
        bumpDir=bumpDir;
    }

    if(bumpdata.leftBumped || bumpdata.rightBumped)
    {
        if(!isbumped)
        {
            //first bump
        }
        bumpTs=getTimeStap_ms()/1000.0;
        isbumped=true;
    }
    if(isbumped)
    {
        moveTs=getTimeStap_ms()/1000.0;
        if((getTimeStap_ms()/1000.0-bumpTs) < 0.5)//over oscillation threshold
        {
            float w=M_PI/4;
            if(bumpDir == -1)
            {
                w=-w;
            }
            bridgePt->GetXBasePt()->setSpeed(0.0,w);//to min resolution
            return;
        }
        else
        {
            int currentBumpDir=bumpDir;
            if(currentBumpDir*lastBumpDir < 0)
            {
                repeatedDirCount+=1;
                if(repeatedDirCount > 4)
                {
                    repeatedDirCount=0;
                    isOscillation=true;
                    oscillationTs=getTimeStap_ms()/1000.0;
                    oscillationDir=currentBumpDir;
                }
            }
            else
            {
                repeatedDirCount=0;
            }
            lastBumpDir=currentBumpDir;
            isbumped=false;
            xlog->Debug("bump rotate done");
        }
    }
    if(isOscillation)
    {
        moveTs=getTimeStap_ms()/1000.0;
        if((getTimeStap_ms()/1000.0-oscillationTs) < 3.0)
        {
            if(oscillationDir > 0)
            {
                bridgePt->GetXBasePt()->setSpeed(0.0,M_PI/3);
            }
            else
            {
                bridgePt->GetXBasePt()->setSpeed(0.0,-M_PI/3);
            }
            return;
        }
        else
        {
            isOscillation=false;
            xlog->Debug("oscillation rotate done");
        }
    }
    //step2,find direction
    if(!isBestDirReached)
    {
        if(isBestDirFind)
        {
            float reachableDirRotateTheta=ClipAngle(odomData.pa-lastBestDirOdom.pa);
            //xlog->Debug("odom theta:%f,rotate sum:%f",odomData.pa,reachableDirRotateTheta);
            if(fabs(reachableDirRotateTheta) > fabs(bestDir))
            {
                xlog->Debug("reach best direction done");
                isBestDirReached=true;
            }
            else
            {
                if(bestDir > 0)
                {
                    bridgePt->GetXBasePt()->setSpeed(0.0,M_PI/4);
                }
                else
                {
                    bridgePt->GetXBasePt()->setSpeed(0.0,-M_PI/4);
                }
            }
            return;
        }
        else
        {
            isBestDirFind=calDirectionWithLaser(bestDir,laserData);
            bridgePt->GetXBasePt()->setSpeed(0.0,M_PI/3); 
        }
    }
    else
    {
        if((getTimeStap_ms()/1000.0-moveTs) > 1.0)
        {
            repeatedDirCount=0;
            moveTs=getTimeStap_ms()/1000.0;
        }
        bridgePt->GetXBasePt()->setSpeed(0.15,0);
    }
}

bool RelocalizationTask_t::calDirectionWithLaser(float &direction,const SensorMsg::LaserData_t &laser)
{
    std::vector<float> laserAvgDist(laser.range_num,0);
    float maxLaserAvgDist=-1;
    int maxLaserIndex=-1;      
    for(int i=0;i<laser.range_num;i+=10)
    {
        //xlog->Debug("laser sep:%d,range:%f",i,laser.ranges[i]);
        if(i < findBestDirWindowSize || i > (laser.range_num-findBestDirWindowSize) )
        {
            laserAvgDist[i]=0;
        }
        else
        {
            float sum=0;
            for(int j=i-findBestDirWindowSize/2;j<i+findBestDirWindowSize/2;j++)
            {
                if(laser.ranges[j] > 1.0)
                {
                    sum=sum+1.0;
                }
                else
                {
                    sum=sum+laser.ranges[j];
                }
            }
            laserAvgDist[i]=sum/float(findBestDirWindowSize);
            if(laserAvgDist[i] > bestDirDistThreshold*2.0)
            {
                maxLaserAvgDist=laserAvgDist[i];
                maxLaserIndex=i;
                break;
            }                    
            if(laserAvgDist[i] > maxLaserAvgDist)
            {
                maxLaserAvgDist=laserAvgDist[i];
                maxLaserIndex=i;
            }
        }
    }
    xlog->Debug("max dist:%f,index:%d",maxLaserAvgDist,maxLaserIndex);
    if(maxLaserAvgDist > bestDirDistThreshold)
    {
        float mountX=0.1825;
        float tempY=maxLaserAvgDist*std::cos(maxLaserIndex*0.15/180*M_PI);
        float tempX=mountX+maxLaserAvgDist*std::sin(maxLaserIndex*0.15/180*M_PI);
        direction=-atan2(tempY,tempX);
        lastBestDirOdom=odomData;
        xlog->Debug("max dist:%f,index:%d,direction:%f",maxLaserAvgDist,maxLaserIndex,direction);
        return true;
    }
    else
    {
        xlog->Debug("cant find reachableDir");
    }  
    return false;     
}

RelocalizationTask_t::RelocalizationTask_t( NavBridge_t *brigdePt, std::string name ):
BaseTask_t(brigdePt, name)
{
    xlog = new XLog(true);
    xlog->Initialise("relocalization.log");
    xlog->SetThreshold(XLOG_LEVEL(4));
    xlog->EnableCout(true);    
    xlog->Debug("relocalization task initialized ");
    mainVersionInfo = XT212_DEPENDENCE_VERSION;

#ifndef _WIN32
	prctl(PR_SET_NAME, "ILcmRelocalization");
	bindCpuCore(BIND_CPU_ID_INNER_LCM);
#endif
    teleopHackCmd=lcm_m.subscribe(LCM_CHANNEL_HACK, &RelocalizationTask_t::teleopCmdUpdate, this);
    teleopHackCmd->setQueueCapacity(1);
    amclScoreSub=lcm_m.subscribe(LCM_CHANNEL_AmclPoseInfo, &RelocalizationTask_t::amclScoreUpdate, this);
    amclScoreSub->setQueueCapacity(1);    
#ifndef _WIN32
	prctl(PR_SET_NAME, "main");
	bindCpuCore(BIND_CPU_ID_MISC);
#endif

    retryCountThreshold=2;

    odomData=bridgePt->GetXBasePt()->getOdomData();
    laserData = bridgePt->GetXBasePt()->getLaserData();
    robotStatus=bridgePt->GetXBasePt()->getRobotStatus();    

    //select position
    avoidMinDist=0.4;
    avoidMaxDist=0.6;
    avoidTimeoutThreshold=30.0;
    quaternionVector.clear();
    quaternionVector.resize(4);
    rpyVector.clear();
    rpyVector.resize(3);
    steadyRollPitchThreshold=0.1;
    findBestDirWindowSize=300;
    bestDirDistThreshold=findBestDirWindowSize*0.002;     

    //sampling
    samplingTimeoutThreshold=20.0;
    sampleMinPointCloudCountThreshold=1500;
    laserPartNum=10;

    //wait for result
    waitForResultTimeoutThreshold=300.0;

    //success check
    successCheckTransDistThreshold=0.4;
    successCheckMoveDistThreshold=1.0;
    successCheckTimeThreshold=20.0;  
    successCheckScoreThreshold=0.8; 

    triggerMode=1; 

    isSkipLastCheck=false;

    ini::IniFile cfg = (brigdePt->GetRobotCfg());
    isSkipLastCheck = cfg["Relocalization"][cfg.GetKey("isSkipLastCheck_b")].as<bool>();
    sampleMinPointCloudCountThreshold = cfg["Relocalization"][cfg.GetKey("sampleMinPointCloudCountThreshold_i")].as<int>();
}

void RelocalizationTask_t::setTriggerMode(int mode)
{
    assert(mode == 1 || mode == 2);
    triggerMode=mode;
}

void RelocalizationTask_t::recordStep2ToFile()
{
    // outfile1<<step2Tick<<std::endl;
    // outfile1<<"odom "<<odomData.timestamp_us<<","<<odomData.name<<","<<odomData.fkVSpd<<","<<odomData.fkWSpd<<","
    //         <<odomData.tarVSpd<<","<<odomData.tarWSpd<<","
    //         <<odomData.px<<","<<odomData.py<<","<<odomData.pa<<","<<odomData.score<<std::endl;
    // outfile1<<"ellar "<<rpyVector[0]<<" "<<rpyVector[1]<<" "<<rpyVector[2]<<std::endl;
    // outfile1<<"laserInfo "<<laserData.timestamp_us<<","<<laserData.name<<","<<laserData.xPos<<","<<laserData.yPos<<","<<laserData.zPos<<","
    //         <<laserData.roll<<","<<laserData.pitch<<","<<laserData.yaw<<","<<laserData.range_num<<","<<laserData.min_bearing<<","<<laserData.max_bearing<<","
    //         <<laserData.min_range<<","<<laserData.max_range<<","<<laserData.resolution<<std::endl;
    // outfile1<<"laserData "<<laserData.range_num<<" "<<fixedTheta<<std::endl;
    // for(int i=0;i<laserData.range_num;i++)
    // {
    //     outfile1<<laserData.ranges[i]<<" "<<laserData.bearing[i]<<std::endl;
    // }            
}

RelocalizationTask_t::~RelocalizationTask_t()
{
    lcm_m.unsubscribe(teleopHackCmd);
    lcm_m.unsubscribe(amclScoreSub);
   
}

BaseTask_t* CreatClass(NavBridge_t* navbridgePt, std::string name)
{
    return new RelocalizationTask_t(navbridgePt, name);
}
