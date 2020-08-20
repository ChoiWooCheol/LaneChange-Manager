#include<ros/ros.h>
#include<autoware_msgs/LaneArray.h>
#include<autoware_msgs/Lane.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/TwistStamped.h>
#include<std_msgs/String.h>

#include<math.h>
#include<cmath>
#include<cstdlib>
#include<vector>
#include<string>
#include <unistd.h>

#include "lanechange_manager.hpp"

// Some dot(x,y,z) to current_pose distance check (distance threshold is in_dist).
bool CalcAroundWaypoints::checkDistance(const double x, const double y, const double z, const double in_dist)
{
       double dist;
       dist = 
           sqrt(pow(cur_x - x, 2) 
               + pow(cur_y - y, 2));
       if(dist > in_dist)
           return false;
       else
           return true;
}

// Some target lane's waypoint to current pose distance checking. Using 'checkDistance' function.
bool CalcAroundWaypoints::checkLaneChangeDone(const autoware_msgs::Lane& next_lane){
    manager_state.tryNextState("is_target_lane_no");
    bool change_done_ok = false;
    for(auto& waypoint : next_lane.waypoints){
        if(checkDistance(waypoint.pose.pose.position.x
                        ,waypoint.pose.pose.position.y
                        ,waypoint.pose.pose.position.z
                        , 0.5))
        {
            manager_state.tryNextState("is_target_lane_yes");
            change_done_ok = true;
            manager_state.tryNextState("lane_change_is_done");
            break;
        }
    }
    return change_done_ok;
}

/* if you use waypoint loader, check off this session.
void CalcAroundWaypoints::callbackLaneWaypointsArray(const autoware_msgs::LaneArray::ConstPtr& in_lane){
    tmp_lane.id = in_lane->id;
    tmp_lane.lanes = in_lane->lanes;
    manager_state.tryNextState("received_global_path");
    if (tmp_lane.lanes.size() != NULL) { 
        subscribe_ok = true;
        ROS_INFO("Subscribe OK!");
        ROS_INFO("Total Lane size : %d", tmp_lane.lanes.size());
    }
    manager_state.tryNextState("received_multi_global_path");
}
*/

void CalcAroundWaypoints::callbackCurrentPose(const geometry_msgs::PoseStamped::ConstPtr& in_pose){
    cur_x = in_pose->pose.position.x;
    cur_y = in_pose->pose.position.y;
    cur_z = in_pose->pose.position.z;
    ++pose_seq;
    if (pose_seq > 3) {
        pose_seq = 0;
        prev_pose_x = cur_x;
        prev_pose_y = cur_y;
    }
}

// lane change의 조건이 만족되었는지 체크
bool CalcAroundWaypoints::isReady(){
    return subscribe_ok;
}

bool CalcAroundWaypoints::checkSameLane(const std::vector<int>& cur_lanes, const std::vector<int>& prev_lanes){
    if(cur_lanes.size() != prev_lanes.size()) 
        return false;

    for(int i = 0; i < cur_lanes.size(); i++){
        if(cur_lanes[i] != prev_lanes[i])
            return false;
    }

    return true;
}

int CalcAroundWaypoints::getGlobalMainLaneIndex(const autoware_msgs::LaneArray& global_lanes){
    uint index = 0;
    uint main_index;
    uint main_gid;

    if (global_lanes.lanes.size() < 2) {
        manager_state.tryNextState("not_need_lane_change");
        return index;
    }
    else {
        manager_state.tryNextState("received_multi_global_path");
        for (auto& lane : global_lanes.lanes){
            if (index == 0) {
                main_gid = lane.waypoints[0].gid;
                main_index = index;
            }
            else {
                if (main_gid < lane.waypoints[0].gid){
                    main_gid = lane.waypoints[0].gid;
                    main_index = index;
                }
            }
            ++index;
        }
        return main_index;
    }
    return -1;
}

void CalcAroundWaypoints::publishFirstCurrentLane(const autoware_msgs::LaneArray& global_lanes) {
    uint index = 0;
    uint curr_index;
    uint curr_gid;
    autoware_msgs::LaneArray curr_lanes;
    curr_lanes.id = global_lanes.id;
    if (global_lanes.lanes.size() < 2) {
        lane_pub.publish(global_lanes);
    }
    else {
        for (auto& lane : global_lanes.lanes){
            if (index == 0) {
                curr_gid = lane.waypoints[0].gid;
                curr_index = index;
            }
            else {
                if (curr_gid > lane.waypoints[0].gid){
                    curr_gid = lane.waypoints[0].gid;
                    curr_index = index;
                }
            }
            ++index;
        }
        curr_lanes.lanes.emplace_back(global_lanes.lanes[curr_index]);
        lane_pub.publish(curr_lanes);
        change_done = true;
    }
}

int CalcAroundWaypoints::setWaitTime(const uint sec) {
    double main_freq;
    private_nh.getParam("main_frequency", main_freq);
    return int(sec * main_freq);
}

void CalcAroundWaypoints::callbackGlobalPath(const autoware_msgs::LaneArray::ConstPtr& global_lanes){
    left_change = false;
    right_change = false;
    left_check = false;
    right_check = false;
    manager_state.tryNextState("received_global_path");
    global_lane.id = global_lanes->id;
    global_lane.lanes = global_lanes->lanes;
    publishFirstCurrentLane(global_lane);
    onSign("straight");
    if (global_lane.lanes.size() != 0) {
        subscribe_ok = true;
        ROS_INFO("Received New Global Lanes!");
        ROS_INFO("Global Lanes size : %d", global_lane.lanes.size());
        wait = setWaitTime(wait_time);
    }
}

void CalcAroundWaypoints::callbackCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr& in_velocity){
    cur_vel = in_velocity->twist.linear.x;
}

void CalcAroundWaypoints::callbackLocalWeightTrajectories(const autoware_msgs::LaneArray::ConstPtr& in_trajectory){
    int blocked_count = 0;
    int rollOutsNumber = 0;
    nh.getParam("/op_common_params/rollOutsNumber", rollOutsNumber);
    ROLLIN = rollOutsNumber + 1;
    if (in_trajectory->lanes.size() == ROLLIN) {
        lane_check_ok = true;
    }
    else if (in_trajectory->lanes.size() > ROLLIN){
        for(int i = 0; i < ROLLIN; i++){
            if(in_trajectory->lanes[ROLLIN + i].is_blocked)
                ++blocked_count;
        }
    }

    if(blocked_count > ROLLIN * 0.6){
        lane_check_ok = false;
    }
    else {
        lane_check_ok = true;
    }
}

void CalcAroundWaypoints::callbackChangeDecision(const std_msgs::String::ConstPtr& msg){
	if (msg->data.find("CheckLeftLane") != std::string::npos || msg->data.find("ChangeToLeft") != std::string::npos)
	{
		left_check = true;
		right_check = false;

        if(check_seq <= 0){
            check_seq++;
            prev_x = cur_x;
            prev_y = cur_y;
        }
	}
	else if (msg->data.find("CheckRightLane") != std::string::npos || msg->data.find("ChangeToRight") != std::string::npos)
	{
		left_check = false;
		right_check = true;
        if(check_seq <= 0){
            check_seq++;
            prev_x = cur_x;
            prev_y = cur_y;
        }
	}
	else
	{
		left_check = false;
		right_check = false;
        
        check_seq = 0;
	}    

    if(lane_check_ok){
        if (msg->data.find("ChangeToLeft") != std::string::npos)
		{
            ROS_ERROR("in change to left");
            manager_state.tryNextState("can_excute_lane_change");
			left_change = true;
			right_change = false;
		}
		else if (msg->data.find("ChangeToRight") != std::string::npos)
		{
            ROS_ERROR("in change to right");
            manager_state.tryNextState("can_excute_lane_change");
			left_change = false;
			right_change = true;
		}
		else
		{
			left_change = false;
			right_change = false;
		}            
    }
    else{
        manager_state.tryNextState("can_not_excute_lane_change");
        ROS_WARN("obstacle is detected!");
    }
}

void CalcAroundWaypoints::onSign(std::string in_sign){
    std_msgs::String sign;
    sign.data = in_sign;
    pub_change_sign.publish(sign);
}

int CalcAroundWaypoints::calcLaneEquationDist(const double x, const double y) {
    // vertical_equation's a b c
    double a = 0.0 , b = 0.0, c = 0.0;
    if (cur_y != prev_pose_y) a = (cur_x - prev_pose_x) / (cur_y - prev_pose_y);
    else return -1;
    b = 1;
    c = -1 * (prev_pose_y + ((cur_x - prev_pose_x) / (cur_y - prev_pose_y)) * prev_pose_x);

    double dist = fabs(a * x + b * y + c) / sqrt(pow(a, 2) + pow(b, 2));

    if (dist < 0.5) {
        return 1;
    }
    else return -1;
}

void CalcAroundWaypoints::doFollowMainLane(const autoware_msgs::LaneArray& global_lane_, const uint main_lane_idx_, const uint curr_idx){
    std_msgs::String sign;
    std::string decision;
    double lane_x_vector = 0.0;
    double lane_y_vector = 0.0;
    double target_x_vector = 0.0;
    double target_y_vector = 0.0;
    double outer_product = 0.0;
    if(change_done && (main_lane_idx_ != curr_idx)){
        for(auto& wp : global_lane_.lanes[main_lane_idx_].waypoints){
            if(checkDistance(wp.pose.pose.position.x
                            ,wp.pose.pose.position.y
                            ,wp.pose.pose.position.z
                            ,MAINLANE_DIST_THRESHOLD))
            {
                if(calcLaneEquationDist(wp.pose.pose.position.x, wp.pose.pose.position.y) == 1){
                    lane_x_vector = cur_x - prev_pose_x;
                    lane_y_vector = cur_y - prev_pose_y;
                    target_x_vector = wp.pose.pose.position.x - prev_pose_x;
                    target_y_vector = wp.pose.pose.position.y - prev_pose_y;
                    outer_product = (lane_x_vector * target_y_vector) - (lane_y_vector * target_x_vector);
                    break;
                }
            }
        }
            
        if (outer_product > 0){ // left
            onSign("left");
            decision_wait = setWaitTime(decision_wait_time);
        }
        else if (outer_product < 0){ // right
            onSign("right");
            decision_wait = setWaitTime(decision_wait_time);
        }
        else onSign("straight");
    }
    else{
        // Driving in main lane
        // if you want to add some function that is main lane state's actions, write in this session.
    }
}

void CalcAroundWaypoints::setNearLane(autoware_msgs::LaneArray& out_lane, const autoware_msgs::Lane& lane,
                                        std::vector<int>& check_same_tmp, int& seq){
    const double lane_vector_x = cur_x - prev_x;
    const double lane_vector_y = cur_y - prev_y;
    const double side_vector_x = lane_x - prev_x;
    const double side_vector_y = lane_y - prev_y;
    const double outer_product = (lane_vector_x * side_vector_y) - (lane_vector_y * side_vector_x);

    if(left_check && outer_product > 0){
        if(left_change){
            nh.setParam("/op_trajectory_generator/samplingOutMargin", calcRollInMargin());
            setClearVector(out_lane.lanes);
            setClearVector(check_same_tmp);
            changed_lane = lane;
            change_done = false;
        }
        out_lane.lanes.emplace_back(lane);
        check_same_tmp.emplace_back(seq);
    }
    else if(right_check && outer_product < 0){
        if(right_change){
            nh.setParam("/op_trajectory_generator/samplingOutMargin", calcRollInMargin());
            setClearVector(out_lane.lanes);
            setClearVector(check_same_tmp);
            changed_lane = lane;
            change_done = false;
        }
        out_lane.lanes.emplace_back(lane);
        check_same_tmp.emplace_back(seq);
    }
}

double CalcAroundWaypoints::calcRollInMargin(){
    if(cur_vel < 5)
        return DEFAULT_MARGIN;
    else
        return 8 + (0.8*cur_vel) + (2.5 * sqrt(cur_vel - 4));
}

template <typename T>
void CalcAroundWaypoints::setClearVector(std::vector<T>& in_vector) {
    in_vector.clear();
    in_vector.resize(0);
}

void CalcAroundWaypoints::run(){
    autoware_msgs::LaneArray output_lane;

    output_lane.id = global_lane.id;
    setClearVector(output_lane.lanes);
    std::vector<int> check_same_tmp;
    setClearVector(check_same_tmp);

    main_lane_idx = getGlobalMainLaneIndex(global_lane);
    bool is_near = false;
    int cur_index;
    
    if(change_done){
        onSign("straight");
        nh.setParam("/op_trajectory_generator/samplingOutMargin", DEFAULT_MARGIN);
        for(int i = 0; i < global_lane.lanes.size(); i++){
            for(auto& waypoint : global_lane.lanes[i].waypoints){
                if(checkDistance(waypoint.pose.pose.position.x
                                ,waypoint.pose.pose.position.y
                                ,waypoint.pose.pose.position.z
                                ,CURRENT_DIST_THRESHOLD))
                {
                    output_lane.lanes.push_back(global_lane.lanes[i]);
                    check_same_tmp.emplace_back(i);
                    cur_index = i;
                    break;
                }
            }
        }
        if (cur_index == main_lane_idx) manager_state.tryNextState("current_lane_is_main");
//        if (wait < 0){
        for(int i = 0; i < global_lane.lanes.size(); i++){
            if(i != cur_index){
                for(auto& waypoint : global_lane.lanes[i].waypoints){
                    if(checkDistance(waypoint.pose.pose.position.x
                                    ,waypoint.pose.pose.position.y
                                    ,waypoint.pose.pose.position.z
                                    ,DIST_THRESHOLD))
                    {
                        is_near = true;
                        lane_x = waypoint.pose.pose.position.x;
                        lane_y = waypoint.pose.pose.position.y;
                        break;
                    }
                }
                if(is_near){
                    if(left_check || right_check){
                        setNearLane(output_lane, global_lane.lanes[i], check_same_tmp, i);
                    }
                    is_near = false;
                }
            }
        }
//        }

        decision_wait--;
        if (decision_wait < 0) doFollowMainLane(global_lane, main_lane_idx, cur_index);

    }
    else{
        change_done = checkLaneChangeDone(changed_lane);
        setClearVector(output_lane.lanes);
        output_lane.lanes.emplace_back(changed_lane);
    }

    if(!checkSameLane(check_same_tmp,check_same) && output_lane.lanes.size() != 0){
        lane_pub.publish(output_lane);
        check_same = check_same_tmp;
    }
}