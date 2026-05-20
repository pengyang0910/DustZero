#include "Msg/NavMsg/Point_t.hpp"
#include <iostream>
#include <zcm/zcm-cpp.hpp>
#include <time.h>
#include <unistd.h>

NavMsg::Point_t point;
class Test_Sub_t
{
private:
    /* data */
    NavMsg::Point_t point;
public:
    Test_Sub_t(/* args */);
    ~Test_Sub_t();

    void pointCb(const zcm::ReceiveBuffer *rbuf,
                                    const std::string &channel,
                                    const NavMsg::Point_t *msg)
    {
        point = *msg;
        std::cout<<"sub "<< point.x<<" "<<point.y<<" "<<point.z<<std::endl;
    }
};

Test_Sub_t::Test_Sub_t(/* args */)
{
}

Test_Sub_t::~Test_Sub_t()
{
}

void pointCb(const zcm::ReceiveBuffer *rbuf,
            const std::string &channel,
            const NavMsg::Point_t *msg)
{
    point = *msg;
    std::cout<<"sub "<< point.x<<" "<<point.y<<" "<<point.z<<std::endl;
}

int main()
{
    uint32_t tick = 0;
    zcm::ZCM zcm("ipc");
    Test_Sub_t sub;
    zcm.subscribe("zcm_point_pub", &Test_Sub_t::pointCb, &sub);

    while (true)
    {
        zcm.handleNonblock();
        usleep(500);
    }
    

    return 0;
}