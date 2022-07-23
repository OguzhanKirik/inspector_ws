#include "ros/ros.h"

#include<std_msgs/Int64.h>
#include<std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include "frank/UpdateDrillTCP.h"
#include "frank/UpdateRackLayout.h"

bool update_drill_tcp_handler(frank::UpdateDrillTCP::Request &newCalib,
                                frank::UpdateDrillTCP::Response &UpdateResult)
{
    
    std::string Init = "/DTool_ref";
    std::string DrillBit = std::to_string(newCalib.DrillBitNumber);
    DrillBit= Init + DrillBit;
    ros::param::set(DrillBit+"/x", double(newCalib.NewTCP.position.x));
    ros::param::set(DrillBit+"/y", double(newCalib.NewTCP.position.y));
    ros::param::set(DrillBit+"/z", double(newCalib.NewTCP.position.z));
    //ros::param::set(DrillBit+"/q1", double(newCalib.NewTCP.orientation.w));
    //ros::param::set(DrillBit+"/q2",double(newCalib.NewTCP.orientation.x));
    //ros::param::set(DrillBit+"/q3", double(newCalib.NewTCP.orientation.y));
    //ros::param::set(DrillBit+"/q1", double(newCalib.NewTCP.orientation.z));

    system("rosparam dump /home/c301/Desktop/ROS-Frank_robot_control/catkin_ws/src/frank/config/Reference_systems.yaml");
    UpdateResult.Result = "TPC of Drill bit "+ DrillBit + " updated!";
    ROS_INFO("Driller TCP updated on configuration file");
    return true;
}

bool update_rack_dbitpos_handler(frank::UpdateRackLayout::Request &newRack,
                                frank::UpdateRackLayout::Response &UpdateResult)
{
    

    ros::param::set("/DrillBit/1_pos", int(newRack.DrillBit1pos));
    ros::param::set("/DrillBit/2_pos", int(newRack.DrillBit2pos));
    ros::param::set("/DrillBit/3_pos", int(newRack.DrillBit3pos));
    ros::param::set("/DrillBit/4_pos", int(newRack.DrillBit4pos));
    ros::param::set("/DrillBit/5_pos", int(newRack.DrillBit5pos));
    


    system("rosparam dump /home/c301/Desktop/ROS-Frank_robot_control/catkin_ws/src/frank/config/Reference_systems.yaml");
    
    ros::param::set("/DrillBit/actual", int(newRack.DrillBitOn));
    UpdateResult.Result = "Rack Layout updated!";
    ROS_INFO("Rack layout updated on configuration file");
    return true;
}





int main(int argc,char **argv)
{
    ros::init(argc, argv, "Drill_Config_server");
    ros::NodeHandle n;
    ros::ServiceServer UpRackL = n.advertiseService("update_drillbits_rack", update_rack_dbitpos_handler);
    ROS_INFO("Ready to Update DrillBits rack layout...");
    ros::ServiceServer UpDrillTCP = n.advertiseService("update_driller_tcp", update_drill_tcp_handler);
    ROS_INFO("Ready to Update Driller TCP with calibration data...");
    ros::spin();
    return 0;
}