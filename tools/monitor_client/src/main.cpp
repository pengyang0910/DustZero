#include "MonitorTask.h"




int main(int argc, char** argv) {
    ros::init(argc, argv,"monitor",ros::init_options::NoSigintHandler);
    LCM_TO_ROS::MonitorTask_t mt;
    mt.Start();

    ros::spin();
    return 0;
}
