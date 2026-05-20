#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "tuya_sdk_main.h"
#include "tuya_ipc_dp.h"
#include "tuya_iot_com_api.h"
#include "tuya_iot_sweeper_api.h"

bool s_ipc_sdk_started = false;

static TY_RECV_OBJ_DP_S g_obj_dp;
static TY_RECV_RAW_DP_S g_raw_dp;
static bool g_obj_active = false;
static bool g_raw_active = false;
static int16_t g_origin_x = 0;
static int16_t g_origin_y = 0;
static int g_map_width = 0;
static bool g_task_status = false;

void wifiReset(void) {}
void gwReset(void) {}
int sdkStart(void)
{
    s_ipc_sdk_started = true;
    return 0;
}
int getLcmSeq(void) { return 0; }
uint64_t getTimeStap_us_c(void) { return 0; }
void set_s_ipc_pid(char data[64]) { (void)data; }
void set_s_ipc_uuid(char data[64]) { (void)data; }
void set_s_ipc_authkey(char data[64]) { (void)data; }
void set_s_sdk_version(char data[64]) { (void)data; }
void set_origin(int16_t x, int16_t y, int mw)
{
    g_origin_x = x;
    g_origin_y = y;
    g_map_width = mw;
}
int16_t getoriginPx(void) { return g_origin_x; }
int16_t getoriginPy(void) { return g_origin_y; }
int getMapWidth(void) { return g_map_width; }
void setTasktatus(bool status) { g_task_status = status; }
bool getTaskStatus(void) { return g_task_status; }
void CmdDataSet(TY_RECV_OBJ_DP_S *data)
{
    if (data) {
        g_obj_dp = *data;
        g_obj_active = true;
    }
}
TY_RECV_OBJ_DP_S *CmdDataGet(void) { return &g_obj_dp; }
bool actCmdActGet(void) { return g_obj_active; }
bool actCmdActSet(bool rc)
{
    g_obj_active = rc;
    return true;
}
void RawDataSet(TY_RECV_RAW_DP_S *data)
{
    if (data) {
        g_raw_dp = *data;
        g_raw_active = true;
    }
}
TY_RECV_RAW_DP_S *RawDataGet(void) { return &g_raw_dp; }
bool rawCmdActGet(void) { return g_raw_active; }
bool rawCmdActSet(bool rc)
{
    g_raw_active = rc;
    return true;
}
void thread_album_send_new(void) {}
void ex_location_toMap(float data, BYTE_T *loc0, BYTE_T *loc1, bool isX)
{
    (void)isX;
    int value = (int)(data * 10.0f);
    if (loc0) {
        *loc0 = (BYTE_T)((value >> 8) & 0xff);
    }
    if (loc1) {
        *loc1 = (BYTE_T)(value & 0xff);
    }
}

OPERATE_RET dev_report_dp_json_async(const CHAR_T *dev_id, const TY_OBJ_DP_S *dp_data, UINT_T cnt)
{
    (void)dev_id;
    (void)dp_data;
    (void)cnt;
    return OPRT_OK;
}

OPERATE_RET dev_report_dp_raw_sync_extend(const CHAR_T *dev_id, BYTE_T dpid,
                                          const BYTE_T *data, UINT_T len,
                                          UINT_T timeout, BOOL_T enable_auto_retrans)
{
    (void)dev_id;
    (void)dpid;
    (void)data;
    (void)len;
    (void)timeout;
    (void)enable_auto_retrans;
    return OPRT_OK;
}

OPERATE_RET tuya_ipc_dp_report(const CHAR_T *dev_id, BYTE_T dp_id,
                               DP_PROP_TP_E type, VOID *pVal, UINT_T cnt)
{
    (void)dev_id;
    (void)dp_id;
    (void)type;
    (void)pVal;
    (void)cnt;
    return OPRT_OK;
}

OPERATE_RET tuya_iot_map_upload_files(const CHAR_T *bitmap_file,
                                      const CHAR_T *datamap_file,
                                      const CHAR_T *descript,
                                      const UINT_T *map_id)
{
    (void)bitmap_file;
    (void)datamap_file;
    (void)descript;
    (void)map_id;
    return OPRT_OK;
}

OPERATE_RET tuya_iot_map_record_upload_buffer(INT_T map_id, const BYTE_T *buffer,
                                              UINT_T len, const CHAR_T *descript)
{
    (void)map_id;
    (void)buffer;
    (void)len;
    (void)descript;
    return OPRT_OK;
}
