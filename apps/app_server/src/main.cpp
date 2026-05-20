/*
 * @Description: 
 * @version: 
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2022-02-16 12:50:57
 * @LastEditors: Jephy Zhang
 * @LastEditTime: 2022-02-18 10:22:22
 */
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include "appServer.h"
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif


int main(int argc, char **argv)
{
    AppServer_t *appServerPt = new AppServer_t();
    appServerPt->Start();

    bindCpuCore(BIND_CPU_ID_APP);
#ifndef WIN32
	prctl(PR_SET_NAME, "AppStartUp");
#endif
    while (true)
    {
#ifdef _WIN32
		Sleep(1000);
#else
        sleep(1);

        if (reset_end ==1&&reset_end ==1)
        {
            reset_start = 0;
            if (appServerPt->checkIsNeedStart())
            {
                reset_end = 0;
                reset_end1 = 0;
                wifiReset();
                s_ipc_sdk_started =false;
                appServerPt->reStart();
            }
            
        }
        
#endif
        //std::cout<<"app task loop..."<<std::endl;
    }
    
}