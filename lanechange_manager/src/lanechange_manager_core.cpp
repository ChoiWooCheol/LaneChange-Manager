#include "lanechange_manager.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lanechange_manager");
    CalcAroundWaypoints calc_waypoints;
    ros::Rate loop_rate(3);
    while(ros::ok()){
        ros::spinOnce();
        if(calc_waypoints.isReady()){
            calc_waypoints.run();
        }
        loop_rate.sleep();
    }
    return 0;
}