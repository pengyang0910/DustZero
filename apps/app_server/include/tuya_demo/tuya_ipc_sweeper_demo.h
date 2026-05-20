/*
 * tuya_ipc_p2p_demo.h 
 *  Created on: 2020年5月19日
 *      Author: 02426
 * Copyright(C),2018-2020, 涂鸦科技 www.tuya.comm
 */

#ifndef DEMOS_DEMO_TUYA_IPC_INCLUDE_TUYA_IPC_SWEERPER_DEMO_H_
#define DEMOS_DEMO_TUYA_IPC_INCLUDE_TUYA_IPC_SWEERPER_DEMO_H_



#include<stdbool.h>
INT_T tuya_ipc_sweeper_album_cb(IN CONST TRANSFER_EVENT_E event, IN CONST PVOID_T args);
void* thread_album_send(void* arg);

VOID multi_map_oper(VOID);


#endif /* DEMOS_DEMO_TUYA_IPC_INCLUDE_TUYA_IPC_P2P_DEMO_H_ */
