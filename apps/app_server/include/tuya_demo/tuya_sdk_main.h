#ifndef _TUYA_SDK_MAIN_HANDLER_H
#define _TUYA_SDK_MAIN_HANDLER_H
#ifdef __cplusplus
extern "C" {
#endif
#include "tuya_ipc_sdk_simple_start.h"

extern FILE *appFile;

typedef struct 
{
    int64_t    timestamp_us;
    int32_t    seq;
    int32_t    keyNum;
    int32_t    *keyArray;
    int32_t    valueNum;
    int32_t    *valueArray;
    int32_t    virWallNum;
    float      *virWallPoses;
    int32_t    mopNum;
    float      *mopPoses;
    int32_t    bothNum;
    float      *bothPoses;
    int32_t    zoneNum;
    float      *zonePoses;
    int32_t    spotNum;
    float      *spotPoses;
    int32_t    roomSplitId;
    int32_t    roomSplitLineDataLen;
    float      *roomSplitLineData;
    int32_t    roomMergeIdNum;
    int32_t    *roomMergeIdArray;
    int8_t     roomReset;
    int32_t    roomPropertyNum;
    int32_t    *roomProperties;
    int32_t    roomNameNum;
    int32_t    *roomNameId;
    char*      *roomNameArray;
    int32_t    roomCleanNum;
    int32_t    *roomCleanSeq;
    ///  Robot Cmd 
    int8_t     Rob_3dObsAvoidEnable;
    int32_t    Robo_ErrCode;

    ///  Station Cmd 
    int32_t    Sta_Status;
    int32_t    Sta_WashMopMode;
    int8_t     Sta_AutoDustCollection;
    int32_t    Sta_AutoDustCollectionInternal;
    int8_t     Sta_AutoDry;
    int32_t    Sta_AutoWashWithPara;
    int8_t     Sta_AutoCarpetTurbo;
    int8_t     Sta_AutoFluid;
    int8_t     Sta_KeyLedOn;
    int8_t     Sta_ScreenLedOn;
    int8_t     Sta_KidLockOn;
    int32_t    Sta_Sn;
    int32_t    Sta_Version;
    int32_t    Sta_ErrCode;

    uint8_t   *Vwall_data;
    uint8_t   *ForbidMop_data;  
    uint8_t   *ForbidBoth_data;
    uint8_t   *Raw_data;
    uint8_t   Raw_data_num;
}SDK_TuyaXtPDO_t;

extern SDK_TuyaXtPDO_t tuya_XtPDO;
extern int reset_start;
extern int reset_end;
extern int reset_end1;
extern bool s_ipc_sdk_started;

void wifiReset();
void gwReset();

int sdkStart();
int getLcmSeq();
uint64_t getTimeStap_us_c();
void set_s_ipc_pid(char data[64]);
void set_s_ipc_uuid(char data[64]);
void set_s_ipc_authkey(char data[64]);
void set_s_sdk_version(char data[64]);
void set_origin(int16_t x, int16_t y, int mw);
int16_t getoriginPx();
int16_t getoriginPy();
int getMapWidth();
void setTasktatus(bool status);
bool getTaskStatus();

void CmdDataSet(TY_RECV_OBJ_DP_S * data);
TY_RECV_OBJ_DP_S * CmdDataGet();
bool actCmdActGet();
bool actCmdActSet(bool rc);

void RawDataSet(TY_RECV_RAW_DP_S * data);
TY_RECV_RAW_DP_S * RawDataGet();
bool rawCmdActGet();
bool rawCmdActSet(bool rc);

void thread_album_send_new();

extern int tuya_ipc_sdk_start(IN CONST TUYA_IPC_SDK_RUN_VAR_S * pRunInfo);

#ifdef __cplusplus
}
#endif

#endif  /*_TUYA_IPC_MGR_HANDLER_H*/