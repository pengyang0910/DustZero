/*
 * @Author: jephy jephy.zhang@any-eye.com
 * @Date: 2023-08-03 09:13:30
 * @LastEditors: jephy jephy.zhang@any-eye.com
 * @LastEditTime: 2023-08-04 13:07:54
 * @FilePath: /Alpha/SlamPro/slamRpc.cpp
 */

#include "slam/slamRpc.h"

SlamRpc_t::SlamRpc_t(/* args */)
{
    mapWidth =0;
}

SlamRpc_t::~SlamRpc_t()
{

}

std::string SlamRpc_t::Hello(rpc_conn conn, std::string callerName)
{
    std::string ret = "Hello "+ callerName + "," +" This is SlamRpc!";
    return ret;
}

void SlamRpc_t::setMapInfo(int16_t _map_ox,int16_t _map_oy,uint16_t _mapWidth)
{ 
     map_ox = _map_ox*1;
     map_oy = _map_oy*1;
     mapWidth = _mapWidth*10;
}

// exchange virtual info to navClean -- Mickel
float SlamRpc_t::ex_vitrual_location(uint8_t loc0, uint8_t loc1, bool isX)
{
    float res = 0.00;

    int16_t ex;
    // BYTE_T *pt = (BYTE_T *)&ex; 
    // pt[0] = loc1;
    // pt[1] = loc0;
    ex = ( loc0 * 256 ) + loc1;

    if( isX )
    {
        res = mapWidth - ex - map_ox ; 
    }
    else
    {
        res = -ex - map_oy ;
    }

    //printf(" map %d %d %d %d %d %f %d  ",loc0,loc1,map_ox,map_oy,mapWidth,res,ex);

    return res / 200 * 1.00;
}


bool SlamRpc_t::UpdateRobot(rpc_conn conn, RobotData_t robot_)
{
    std::lock_guard<std::mutex> lock(mtx);
    robot = robot_;

    return true;
}


bool SlamRpc_t::getAppCmd(rpc_conn conn, APP_CMD_DP_S cmd)
{
    int dpid = cmd.dpid;
    cmd_dpid = cmd.dpid;
    printf("update cmd id is %d\n ",dpid);
    return true;
}

bool SlamRpc_t::getAppRawCmd(rpc_conn conn, APP_RAW_DP_S  raw)
{
    if (mapWidth==0)
    {
        return false;
    }
    
    int dpid = raw.dpid;
    raw_dpid = raw.dpid;
    printf("update raw cmd id is %d\n ",dpid);
    for (int i = 0; i < raw.len; i++)
    {
        uint8_t data = raw.data[i];
        printf("data is %d\n",data);
    }

    if (raw_dpid == 0x12)  //virwall
    {
        
        int num = raw.data[2];
        int arrLen = 8;
        edit_data_s.virWallNum =  4 * num + 1;
        edit_data_s.virWallPoses.resize( edit_data_s.virWallNum );
        edit_data_s.virWallPoses[0] = 1.11; // data Type , useful type > 0
        for (int i = 0; i < num; i++)
        {
            for (int j = 0; j < 4; j++) // wall 2 points, fixed
            {
                bool isX = true;
                if( j % 2 == 0 )
                {
                    isX = true;
                }else{
                    isX = false;
                }  
                uint8_t loc0 = raw.data[arrLen*i + j*2 + 3];
                uint8_t loc1 = raw.data[arrLen*i + j*2 + 4];
                edit_data_s.virWallPoses[i*4 + j +1] = ex_vitrual_location(loc0, loc1, isX);   
                printf("pose is %f \n",edit_data_s.virWallPoses[i*4 + j +1]);
            }                   
        }
    }
    else if( raw_dpid == 0x24 )// set room name 
    {
        int roomNums = raw.data[2];

        if( roomNums != 0 )
        {               
            int arrLen = 21; 
            //lcm_data.roomNameNum = roomNums;
            //std::vector<std::pair<int, std::string>> roomNames;
            
            for(int i = 0; i < roomNums; i++)
            {              
                std::pair<int, std::string> currRoomNames;
                char roomNameBytes[19];
                currRoomNames.first = raw.data[3+arrLen*i]; // id

                for (int j = 0; j < 19; j++)
                {
                    roomNameBytes[j] = raw.data[5+j+arrLen*i];
                }     
                currRoomNames.second = roomNameBytes;

                edit_data_s.roomNames.push_back(currRoomNames);

                //lcm_data.roomNameArray.push_back(roomName);                       
            }
        } 
    }
    else if( raw_dpid == 0x38 )// set mop both forbidden area
    {
        int arrLen = 38;
        bool isX =false;

        if( raw.data[3] == 0 )
        {
            edit_data_s.mopNum = 1;
            edit_data_s.mopPoses.push_back(-1.3);

            edit_data_s.bothNum = 1;
            edit_data_s.bothPoses.push_back(-1.3);
        }
        else
        {   
            int mopType = 0;
            int bothType = 0;
            for (int i = 0; i < raw.data[3]; i++) // areas number
            {
                // forbid type
                if( raw.data[arrLen*i+4] == 0x00 ) 
                {
                    bothType += 1;
                }else if( raw.data[arrLen*i+4] == 0x02 )
                {
                    mopType += 1;
                }
            }

            if( mopType != 0 )
            {
                edit_data_s.mopNum = mopType * 8 + 1;
                edit_data_s.mopPoses.resize( edit_data_s.mopNum );
            }
            else
            {
                edit_data_s.mopNum = 0;
                edit_data_s.mopPoses.clear();
            }

            if( bothType != 0 )
            {
                edit_data_s.bothNum = bothType * 8 + 1;
                edit_data_s.bothPoses.resize( edit_data_s.bothNum );
            }
            else
            {
                edit_data_s.bothNum = 0;
                edit_data_s.bothPoses.clear();
            }

            int both_key = 0;
            int mop_key = 0;
            for (int i = 0; i < raw.data[3]; i++)
            {
                if( raw.data[arrLen*i+4] == 0x00 ) 
                {
                    edit_data_s.bothPoses[0] = 1.1;  
                }
                else if( raw.data[arrLen*i+4] == 0x02 )
                {
                    edit_data_s.mopPoses[0] = 1.2;                   
                }
                
                for (int j = 1; j <= 8; j++) // area 4 points, fixed
                {
                    uint8_t loc0 = raw.data[arrLen*i + (j-1)*2 + 6]; // 0
                    uint8_t loc1 = raw.data[arrLen*i + (j-1)*2 + 7]; // 1
                    if( j % 2 == 0 )
                    {
                        isX = false;
                    }else{
                        isX = true;
                    } 
                    if( raw.data[arrLen*i+4] == 0x00 ) 
                    {
                        edit_data_s.bothPoses[both_key*8 + j] = ex_vitrual_location(loc0, loc1, isX);  
                        printf("bothPoses is %f \n",edit_data_s.bothPoses[both_key*8 + j]);                        
                    }
                    if( raw.data[arrLen*i+4] == 0x02 ) 
                    {
                        edit_data_s.mopPoses[mop_key*8 + j] = ex_vitrual_location(loc0, loc1, isX);
                        printf("mopPoses is %f \n",edit_data_s.mopPoses[mop_key*8 + j]);  
                    }
                }

                if( raw.data[arrLen*i+4] == 0x00 ) 
                {
                    both_key++; 
                }
                if( raw.data[arrLen*i+4] == 0x02 )
                {
                    mop_key++;                
                }          
            }
        }
    } 
    else if( raw_dpid == 0x1C )// split
    {
        int roomSplitId = raw.data[2];
        edit_data_s.roomSplitPoses.resize(5);
        edit_data_s.roomSplitPoses[0] = roomSplitId;
        //xlog->Debug("roomSplitLineData roomSplitId %d", lcm_data.roomSplitId);  
        bool isX =false;
        for (int j = 0; j < 4; j++) // pose 2 points, fixed
        {
            if( j % 2 == 0 )
            {
                isX = true;
            }else{
                isX = false;
            } 
            uint8_t loc0 = raw.data[j*2 + 3];
            uint8_t loc1 = raw.data[j*2 + 4];

            edit_data_s.roomSplitPoses[j+1] = ex_vitrual_location(loc0, loc1, isX); 
            printf("split pose is %f \n",edit_data_s.roomSplitPoses[j+1]);  
            //xlog->Debug("roomSplitLineData loc trans to slam %d: %f", j, lcm_data.roomSplitLineData[j]);                    
        }   
    }
    else if( raw_dpid == 0x1E)// merge
    {
        std::pair<int, int> mergeId;
        mergeId.first = raw.data[2];
        mergeId.second = raw.data[3];
        edit_data_s.roomMergeId.clear();
        edit_data_s.roomMergeId.push_back(mergeId);
    }
    else if( raw_dpid == 0x26 )// clean order
    {
        int roomsNum = raw.data[2];
        if( roomsNum != 0 )
        {
            int arr_len = 1;
            edit_data_s.roomClearOrder.resize(roomsNum);
            for (int i = 0; i < roomsNum; i++)
            {
                edit_data_s.roomClearOrder[i] = raw.data[3+arr_len*i];    // clean order id
                //xlog->Debug("clean order data trans to slam %d", raw_dp->data[5+arr_len*i]);                    
            }
        }
    }
    else if( raw_dpid == 0x22 )// set room property 
    {
        int roomsNum = raw.data[2];
        if( roomsNum != 0 )
        {
            int arr_len = 5;
            for (int i = 0; i < roomsNum; i++)
            {
                std::vector<uint8_t> currRoomProperty;
                currRoomProperty.resize(arr_len);
                for (int j = 0; j < arr_len; j++) // pose 2 points, fixed
                {
                    currRoomProperty[j] = raw.data[3+arr_len*i+j]; 
                }
                edit_data_s.roomPropertyData.push_back(currRoomProperty);
                //edit_data_s.roomPropertyData[i] = raw.data[3+arr_len*i];    // clean order id                 
            }
        }
    }
    else if( raw_dpid == 0x1C )// reset
    {
        
    }
    else if( raw_dpid == 0x32)// reset
    {
        
    }
    else if( raw_dpid == 100 )// save map
    {
        printf("save map  \n ");
    }
    
    return true;
}

void SlamRpc_t::ResetCmdId()
{
    cmd_dpid = 0;
    raw_dpid = 0;
}

void SlamRpc_t::resetRoomEditData()
{
    edit_data_s.bothNum=0;
    edit_data_s.mopNum=0;
    edit_data_s.virWallNum=0;
    edit_data_s.virWallPoses.clear();
    edit_data_s.mopPoses.clear();
    edit_data_s.bothPoses.clear();
    edit_data_s.roomClearOrder.clear();
    edit_data_s.roomPropertyData.clear();
    edit_data_s.roomNames.clear();
    edit_data_s.roomMergeId.clear();
    edit_data_s.roomSplitPoses.clear();

}


void SlamRpc_t::main()
{
    rpc_server server(RpcPort_SlamPro, 1);
    server.register_handler("Hello", &SlamRpc_t::Hello, this);
    server.register_handler("RpcApi_APPCMD", &SlamRpc_t::getAppCmd, this);
    server.register_handler("RpcApi_APPRAWCMD", &SlamRpc_t::getAppRawCmd, this);
    server.set_network_err_callback(
      [](std::shared_ptr<connection> conn, std::string reason)
      {
        std::cout << "SLAMPRO RPC_SERVER ERROR: remote client address: " << conn->remote_address()
                  << " networking error, reason: " << reason << "\n";
      });

    server.async_run();

    while(isRunning)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

void SlamRpc_t::Start()
{
    if(!isRunning)
    {   t1 = createBindThread(ProSlamName+"Rpc", std::bind(&SlamRpc_t::main, this), BIND_CPU_ID_RPC);
        //t1 = std::thread(&SlamRpc_t::main, this);
        isRunning = true;
    }
}

void SlamRpc_t::Stop()
{
    
}

bool SlamRpc_t::IsRunning()
{
    return isRunning;
}