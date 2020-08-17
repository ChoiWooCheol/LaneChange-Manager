#include "lanechange_manager.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lanechange_manager");
    ros::NodeHandle core_nh("~");
    double main_frequency, change_frequency;
    if(!core_nh.getParam("main_frequency", main_frequency)) throw std::runtime_error("set main_frequency");
    if(!core_nh.getParam("change_frequency", change_frequency)) throw std::runtime_error("set change_frequency");
    ros::Rate change_rate(change_frequency);
    CalcAroundWaypoints calc_waypoints(&change_rate);
    ros::Rate loop_rate(main_frequency);
    while(ros::ok()){
        ros::spinOnce();
        if(calc_waypoints.isReady()){
            calc_waypoints.run();
        }
        loop_rate.sleep();
    }
    return 0;
}