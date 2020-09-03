#include "lanechange_manager.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lanechange_manager");
    ros::NodeHandle core_nh("~");
    double main_frequency, change_frequency;
    if(!core_nh.getParam("main_frequency", main_frequency)) throw std::runtime_error("set main_frequency");
   
    CalcAroundWaypoints calc_waypoints;
    ros::Rate loop_rate(main_frequency);
    while(ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
        if(calc_waypoints.checkOverlapLanes()) continue;
        --calc_waypoints.wait;
        if(calc_waypoints.isReady() && calc_waypoints.wait < 0) calc_waypoints.run();
    }
    return 0;
}