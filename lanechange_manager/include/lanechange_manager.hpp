#ifndef __LANECHANGE_MANAGER__
#define __LANECHANGE_MANAGER__

#include<ros/ros.h>
#include<autoware_msgs/LaneArray.h>
#include<autoware_msgs/Lane.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/TwistStamped.h>
#include<std_msgs/String.h>

#include<cmath>
#include<cstdlib>
#include<vector>
#include<string>

class CalcAroundWaypoints{
public:
    CalcAroundWaypoints()
    : cur_x(0.0)
    , cur_y(0.0)
    , cur_z(0.0)
    , cur_vel(0.0)
    , prev_x(0.0)
    , prev_y(0.0)
    , lane_x(0.0)
    , lane_y(0.0)
    , check_seq(0)
    , subscribe_ok(false)
    , lane_check_ok(false)
    , change_done(true)
    , private_nh("~")
    , left_check(false)
    , right_check(false)
    , left_change(false)
    , right_change(false)
    {
        if(!private_nh.getParam("dist_threshold", DIST_THRESHOLD))
            throw std::runtime_error("set dist_threshold");
        if(!private_nh.getParam("current_dist_threshold", CURRENT_DIST_THRESHOLD))
            throw std::runtime_error("set current_dist_threshold");      
        if(!private_nh.getParam("default_margin", DEFAULT_MARGIN))
            throw std::runtime_error("set default_margin");
        if(!private_nh.getParam("rollin", ROLLIN))
            throw std::runtime_error("set rollin");

        lane_sub = nh.subscribe("/lane_waypoints_array2", 10, &CalcAroundWaypoints::callbackLaneWaypointsArray, this);
        cur_pose_sub = nh.subscribe("/current_pose", 1, &CalcAroundWaypoints::callbackCurrentPose, this);
        sub_LocalWeightTrajectories = nh.subscribe("/local_weighted_trajectories", 1, &CalcAroundWaypoints::callbackLocalWeightTrajectories, this);

        cur_vel_sub = nh.subscribe("/current_velocity", 1, &CalcAroundWaypoints::callbackCurrentVelocity, this);

        lane_pub = nh.advertise<autoware_msgs::LaneArray>("lane_waypoints_array", 10, true);
        sub_decision_state = nh.subscribe("/decision_maker/state", 1, &CalcAroundWaypoints::callbackDecisionMakerState, this);
        check_same.clear();
        check_same.resize(0);
    }

    bool DistanceCheck(double x, double y, double z);
    bool CurrentLaneCheck(double x, double y, double z);
    bool OntoLaneCheck(double x, double y, double z);
    bool LaneChangeDone(autoware_msgs::Lane& next_lane);
    void callbackLaneWaypointsArray(const autoware_msgs::LaneArray::ConstPtr& in_lane);
    void callbackCurrentPose(const geometry_msgs::PoseStamped::ConstPtr& in_pose);
    void callbackLocalWeightTrajectories(const autoware_msgs::LaneArray::ConstPtr& in_trajectory);
    bool isReady();
    bool checkSameLane(std::vector<int>& cur_lanes, std::vector<int>& prev_lanes);
    void callbackCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr& in_velocity);
    void callbackDecisionMakerState(const std_msgs::StringConstPtr& input_msg);
    void SetNearLane(autoware_msgs::LaneArray& out_lane, autoware_msgs::Lane& lane,
                    std::vector<int>& check_same_tmp, int& seq);
    double calcRollInMargin();
    void run();


private:
    ros::NodeHandle nh, private_nh;
    ros::Subscriber lane_sub;
    ros::Subscriber cur_pose_sub;
    ros::Subscriber sub_decision_state;
    ros::Subscriber cur_vel_sub;
    ros::Subscriber sub_LocalWeightTrajectories;

    ros::Publisher lane_pub;

    autoware_msgs::LaneArray tmp_lane;
    autoware_msgs::Lane changed_lane;
    std::vector<int> check_same;

    unsigned int check_seq;

    bool subscribe_ok, lane_check_ok;
    bool left_check, right_check;
    bool left_change, right_change;
    bool change_done;

    double cur_x, cur_y, cur_z, cur_vel;
    double lane_x, lane_y;
    double prev_x, prev_y;

    double DEFAULT_MARGIN;
    double DIST_THRESHOLD;
    double CURRENT_DIST_THRESHOLD;
    int ROLLIN;
};

#endif