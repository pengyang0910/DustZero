/*
 * @Description: 
 * @version: 
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2020-09-04 10:04:48
 * @LastEditors: Jephy Zhang
 * @LastEditTime: 2021-09-30 12:32:00
 */
#include "viewServer.h"
#include "xutils.h"
#include "global.h"

int main()
{
    bindCpuCore(BIND_CPU_ID_ViewServer);
    ViewServer_t viewSrv;
    viewSrv.start();
	viewSrv.mThread.join();
    
}