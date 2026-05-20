#include "Msg/NavMsg/Point_t.hpp"
#include <iostream>
#include <zcm/zcm-cpp.hpp>
#include <time.h>
#include <unistd.h>

int main()
{
    uint32_t tick = 0;
    zcm::ZCM zcm("ipc");
    NavMsg::Point_t point;
    point.x = 0.11;
    point.y = 0.22;
    point.z = 0.33;
    
    while (true)
    {
        point.x += 1;
        zcm.publish("zcm_point_pub", &point);
        usleep(1000000);
        std::cout<<"pub "<<tick<<std::endl;
    }
    

    return 0;
}