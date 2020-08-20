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

#include "lanechange_manager_state.hpp"

class CalcAroundWaypoints{
public:
    ~ CalcAroundWaypoints(){}
    CalcAroundWaypoints()
    : cur_vel(0.0)
    , prev_x(0.0)
    , prev_y(0.0)
    , lane_x(0.0)
    , lane_y(0.0)
    , check_seq(0)
    , pose_seq(0)
    , subscribe_ok(false)
    , lane_check_ok(true)
    , change_done(true)
    , private_nh("~")
    , left_check(false)
    , right_check(false)
    , left_change(false)
    , right_change(false)
    , wait(0)
    , decision_wait(0)
    {
        if(!private_nh.getParam("mainlane_dist_threshold", MAINLANE_DIST_THRESHOLD)) throw std::runtime_error("set mainlane_dist_threshold");
        if(!private_nh.getParam("dist_threshold", DIST_THRESHOLD)) throw std::runtime_error("set dist_threshold");
        if(!private_nh.getParam("current_dist_threshold", CURRENT_DIST_THRESHOLD)) throw std::runtime_error("set current_dist_threshold");      
        if(!private_nh.getParam("default_margin", DEFAULT_MARGIN)) throw std::runtime_error("set default_margin");
        if(!private_nh.getParam("main_wait", wait_time)) throw std::runtime_error("set main_wait_time");
        if(!private_nh.getParam("decision_wait", decision_wait_time)) throw std::runtime_error("set decision_wait_time");
        if(!private_nh.getParam("global_lane_topic", global_lane_topic_name)) throw std::runtime_error("set global_lane_topic");
        sub_cur_pose = nh.subscribe("/current_pose", 1, &CalcAroundWaypoints::callbackCurrentPose, this);
        sub_local_weight_trajectories = nh.subscribe("/local_weighted_trajectories", 1, &CalcAroundWaypoints::callbackLocalWeightTrajectories, this);
        sub_cur_vel = nh.subscribe("/current_velocity", 1, &CalcAroundWaypoints::callbackCurrentVelocity, this);
        sub_global_path = nh.subscribe(global_lane_topic_name.c_str(), 10,  &CalcAroundWaypoints::callbackGlobalPath, this);
        sub_decision_state = nh.subscribe("change_decision", 1, &CalcAroundWaypoints::callbackChangeDecision, this);

        lane_pub = nh.advertise<autoware_msgs::LaneArray>("lane_waypoints_array", 10, true);
        pub_state = nh.advertise<std_msgs::String>("state_cmd", 1);
        pub_change_sign = nh.advertise<std_msgs::String>("change_sign", 1);
        setClearVector(check_same);
    }

    template <typename T>
    void setClearVector(std::vector<T>& in_vector);

    bool checkDistance(const double x, const double y, const double z, const double in_dist);
    bool checkLaneChangeDone(const autoware_msgs::Lane& next_lane);
    // void callbackLaneWaypointsArray(const autoware_msgs::LaneArray::ConstPtr& in_lane);
    void callbackCurrentPose(const geometry_msgs::PoseStamped::ConstPtr& in_pose);
    void callbackLocalWeightTrajectories(const autoware_msgs::LaneArray::ConstPtr& in_trajectory);
    void callbackGlobalPath(const autoware_msgs::LaneArray::ConstPtr& global_lanes);
    bool isReady();
    bool checkSameLane(const std::vector<int>& cur_lanes, const std::vector<int>& prev_lanes);
    void callbackCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr& in_velocity);
    void onSign(std::string in_sign);
    void callbackChangeDecision(const std_msgs::String::ConstPtr& msg);
    int setWaitTime(const uint sec);
    void setState(std::string& data, const uint curr_idx);
    void setNearLane(autoware_msgs::LaneArray& out_lane, const autoware_msgs::Lane& lane,
                    std::vector<int>& check_same_tmp, int& seq);
    void doFollowMainLane(const autoware_msgs::LaneArray& global_lane, const uint main_lane_idx, const uint curr_idx);
    int getGlobalMainLaneIndex(const autoware_msgs::LaneArray& global_lanes);
    int calcLaneEquationDist(const double x, const double y);
    void publishFirstCurrentLane(const autoware_msgs::LaneArray& global_lanes);
    double calcRollInMargin();
    void run();

    double prev_pose_x = 0.0, prev_pose_y = 0.0;
    double cur_x = 0.0, cur_y = 0.0, cur_z = 0.0;
    int wait, decision_wait;
private:
    ros::NodeHandle nh, private_nh;

    ros::Subscriber sub_cur_pose;
    ros::Subscriber sub_decision_state;
    ros::Subscriber sub_cur_vel;
    ros::Subscriber sub_local_weight_trajectories;
    ros::Subscriber sub_global_path;

    ros::Publisher lane_pub;
    ros::Publisher pub_state;
    ros::Publisher pub_change_sign;
    
    autoware_msgs::LaneArray tmp_lane;
    autoware_msgs::LaneArray global_lane;
    autoware_msgs::Lane changed_lane;
    std::vector<int> check_same;

    unsigned int check_seq;
    int main_lane_idx;
    uint curr_lane_idx;
    uint pose_seq;
    bool subscribe_ok, lane_check_ok;
    bool left_check, right_check;
    bool left_change, right_change;
    bool change_done;

    int wait_time, decision_wait_time;

    double cur_vel;
    double lane_x, lane_y;
    double prev_x, prev_y;

    double DEFAULT_MARGIN;
    double DIST_THRESHOLD;
    double CURRENT_DIST_THRESHOLD;
    double MAINLANE_DIST_THRESHOLD;
    std::string global_lane_topic_name;
    int ROLLIN; // getparam 으로 수정해야함. op common params 에서 발행. (rollOutsNumber + 1)와 같아야함.

    LaneChangeStateMachine manager_state;
};

#endif