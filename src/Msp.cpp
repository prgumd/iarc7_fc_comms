 ////////////////////////////////////////////////////////////////////////////
//
// Msp flight controller node.
//
// This node connects to the FC using the MSP protocol.
//
////////////////////////////////////////////////////////////////////////////

 #include <ros/ros.h>
 #include "MspFcComms.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "FC_comms");

    ROS_INFO("FC_comms begin");

    MspFcComms fc;

    (void)fc.init();

    if(fc.run(argc, argv) != MspFcComms::kReturnOk)
    {
        ROS_ERROR("FC Node exited with error");
        return 1;
    }

    // All is good.
    return 0;
}