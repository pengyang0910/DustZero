#include <iostream>
#include "Socket/tlv.h"
#include "Socket/tcp_server.h"
#include "global.h"
#include "xutils.h"
#include "unistd.h"
#include <vector>
#include "hmi.h"

#define  DEBUG_FACTORY_PORT     9191
#define  HEADER                 0xFEFEFEFE
#define  SOCKET_RX_BUF_LEN      1024

enum class ReqKey_t
{
    LED = 1,
    VOICE = 2,
    MODE = 3,
    BindSta = 4,
};

typedef struct
{
	uint32_t frame_header;
	uint32_t seq;
	/*
	 1：灯显；2语音包；3：模式切换 4. 基站绑定
	*/
	uint32_t key;   
	
	/*
		灯显 1：白；2：绿：3：黄；0：默认值
		语音包：1：播报语音
		模式切换：1：切换至产测模式；2：切换至出厂模式；0：默认值
        基站绑定：序列号
	*/
	uint32_t value;   

}req_t;

typedef struct
{
	uint32_t frame_header;
	uint32_t seq;                   //丢包使用
	uint32_t hard_version;          //硬件版本号 默认携带
	uint32_t soft_version;          //软件版本号 默认携带
	uint32_t robot_model;           //机器型号   默认携带
	uint32_t key_type;              //b0-key1, b1-key2
	uint32_t light_type;            //b0-led1, b1-led2, b2-led3
	uint32_t wifi_cont;             //WIFI 1：连接成功；0：默认值 
	uint32_t voice;                 //语音包：1：语音已播报；0：默认值 
	uint32_t mode_switch;           //模式：1：产测模式1；2：出厂模式；0：默认值
    uint32_t staSN;
    uint32_t robotSN;
}res_t;




int main()
{
    bindCpuCore(BIND_CPU_ID_ViewServer);
    int server_fd = tcp_server_open(DEBUG_FACTORY_PORT);
    int fd_client = -1;
	char buf[SOCKET_RX_BUF_LEN];
    unsigned int cnt = 0;
    bool isRunning = true;
    int rxIndex = 0;
    std::vector<char> rxByteArray;
    req_t req;
    res_t res;
    res.key_type = 0;
    res.light_type = 1;
    res.mode_switch = 2;
    res.robot_model = 3;
    res.soft_version = 0x12341234;
    res.voice = 4;
    res.wifi_cont = 0;
    res.hard_version = 0x56785678;
    res.robotSN = 0x12121212;
    res.staSN = 0x23232323;

    int fd = open("/dev/xtgpio0", O_RDWR);
    uint8_t writeKeyData = OUTGPIO_MASK_POW_ON | OUTGPIO_MASK_3V3_ON | OUTGPIO_MASK_5V_ON;
    uint8_t readKeyData = 0;

    // get wifi state
    FILE* pp = popen("wifi_scan_results_test | grep rickWiFi", "r");
    char tmp[128] = {0};
    memset(tmp, 0, sizeof(tmp));
    while (fgets(tmp, sizeof(tmp), pp) != NULL)
    {
        char *pIPStart = strstr(tmp, " rickWiFi");
        if(pIPStart != NULL)
        {
            res.wifi_cont = 1;
            break;
        }
    }
    pclose(pp);

    while (isRunning && server_fd > 0)
    {
        uint64_t sts_us = 0;
        sts_us = getTimeStap_us();

        // update key
        if(-1 != fd)
            read(fd, &readKeyData, 1);

        if(readKeyData & INGPIO_MASK_FN_KEY) 
        {
            res.key_type |= 0x00000001;
        }
        else 
        {
            res.key_type &= ~(0x00000001);
        }
        
        if (readKeyData & INGPIO_MASK_PWR_KEY)
        {
            res.key_type |= 0x00000002;
        }
        else 
        {
            res.key_type &= ~(0x00000002);
        }


        if (fd_client < 0)
        {
            is_requested_connections(server_fd, fd_client);

        }

        if (fd_client > 0)
        {
            //std::cout<<"Connected!"<<std::endl;
            if (!is_lost_connection(fd_client))
            {
                int cnt = tcp_server_read_timeout(fd_client, buf, SOCKET_RX_BUF_LEN, 0, 50);
                if (cnt <= 0)
                {
                    usleep(5000);                    
                }
                else if (cnt > 0)
                {
                    for (int i = 0; i < cnt; i++)
                    {
                        /* code */
                        printf("%x ", buf[i]);
                        rxByteArray.push_back(buf[i]);
                    }
                    printf("  receive %d bytes!\r\n", rxByteArray.size());
                    
                    if(rxByteArray.size() >= sizeof(req_t))
                    {
                        // handle frame
                        int state = 0;
                        uint32_t index = 0;
                        uint32_t len = rxByteArray.size() / sizeof(uint32_t);
                        uint32_t* ptr = (uint32_t*)rxByteArray.data();
                        bool foundFrame = false;
                        printf("*ptr=%x\r\n", *ptr);
                        for (int i = 0; i < len; i++)
                        {
                            /* code */
                            printf("i=%d *ptr=%x\r\n", i, *ptr);
                            if(*ptr == HEADER)
                            {
                                // rm data before HEADER
                                foundFrame = true;
                                break;
                            }
                            i++;
                            ptr++;
                        }
                        
                        if(foundFrame)
                        {
                            req = *(req_t*)ptr;
                            int i = ptr - (uint32_t*)rxByteArray.data();
                            rxByteArray.erase(rxByteArray.begin(), rxByteArray.begin()+i*4+sizeof(req_t));
                            std::cout<<"seq="<<req.seq <<" key="<<req.key<<" value="<<req.value<<std::endl;
                            res.frame_header = req.frame_header;
                            res.seq = req.seq;
                          
                            switch (ReqKey_t(req.key))
                            {
                            case ReqKey_t::LED :
                                /* code */
                                res.light_type = req.value;
                                if(res.light_type & 0x01)
                                {
                                    writeKeyData |= (OUTGPIO_MASK_LED_Y);
                                }
                                else
                                {
                                    writeKeyData &= ~(OUTGPIO_MASK_LED_Y);
                                }
                                if(res.light_type & 0x02)
                                {
                                    writeKeyData |= (OUTGPIO_MASK_LED_R);
                                }
                                else
                                {
                                    writeKeyData &= ~(OUTGPIO_MASK_LED_R);
                                }
                                if(res.light_type & 0x04)
                                {
                                    writeKeyData |= (OUTGPIO_MASK_LED_B);
                                }
                                else 
                                {
                                    writeKeyData &= ~(OUTGPIO_MASK_LED_B);
                                }
                                break;
                            case ReqKey_t::VOICE:
                                res.voice = 1;
                                #ifdef RK3566_BUILD
                                system("aplay /app/fj212/resource/robot_voice/1.wav");
                                #else 
                                system("aplay /mnt/UDISK/fj212/resource/robot_voice/1.wav");
                                #endif
                                
                                break;
                            case ReqKey_t::MODE:
                            {
                                res.mode_switch = req.value;
                                switch (res.mode_switch)
                                {
                                case 1: // Factory Mode
                                    /* code */
                                    system("/root/killall.sh");
                                    system("/root/start_x1c.sh");
                                    break;
                                case 2: // User Mode
                                    system("/root/kill_x1c.sh");
                                    system("/root/startFj212.sh");
                                    break;
                                default:
                                    break;
                                }
                                break;
                            }

                            case ReqKey_t::BindSta:
                                res.staSN = req.value;
                                break;
                            default:
                                break;
                            }

                            if(-1 != fd)
                                write(fd, &writeKeyData, 1);
                            tcp_server_write(fd_client, (char*)&res, sizeof(res));
                        }
                    }    
                    else
                    {
                        std::cout<<"Server Recieve "<< cnt << " bytes!" << std::endl;
                    }
                        
                    
                }
            }
            else
            {

            }
        }
        else
            usleep(40000);
    }
}