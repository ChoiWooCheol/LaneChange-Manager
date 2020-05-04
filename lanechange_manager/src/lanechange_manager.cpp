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

#include "lanechange_manager.hpp"

bool CalcAroundWaypoints::DistanceCheck(double x, double y, double z)
{
       double dist;
       dist = 
           sqrt(pow(cur_x - x, 2) 
               + pow(cur_y - y, 2));
       if(dist > DIST_THRESHOLD)
           return false;
       else
           return true;
}

bool CalcAroundWaypoints::CurrentLaneCheck(double x, double y, double z){
    double dist;

     dist = 
        sqrt(pow(cur_x - x, 2) 
            + pow(cur_y - y, 2));

     if(dist > CURRENT_DIST_THRESHOLD)
        return false;
    else
        return true;
}

bool CalcAroundWaypoints::LaneChangeDone(autoware_msgs::Lane& next_lane){
    bool change_done_ok = false;
    for(auto& waypoint : next_lane.waypoints){
        if(DistanceCheck(waypoint.pose.pose.position.x
                        ,waypoint.pose.pose.position.y
                        ,waypoint.pose.pose.position.z))
        {
            change_done_ok = true;
            break;
        }
    }
    return change_done_ok;
}

void CalcAroundWaypoints::callbackLaneWaypointsArray(const autoware_msgs::LaneArray::ConstPtr& in_lane){
    tmp_lane.id = in_lane->id;
    tmp_lane.lanes = in_lane->lanes;

    if (tmp_lane.lanes.size() != NULL) {
        subscribe_ok = true;
        ROS_INFO("Subscribe OK!");
        ROS_INFO("Total Lane size : %d", tmp_lane.lanes.size());
    }
}

void CalcAroundWaypoints::callbackCurrentPose(const geometry_msgs::PoseStamped::ConstPtr& in_pose){
    cur_x = in_pose->pose.position.x;
    cur_y = in_pose->pose.position.y;
    cur_z = in_pose->pose.position.z;
}

bool CalcAroundWaypoints::isReady(){
    return subscribe_ok;
}

bool CalcAroundWaypoints::checkSameLane(std::vector<int>& cur_lanes, std::vector<int>& prev_lanes){
    if(cur_lanes.size() != prev_lanes.size()) 
        return false;

    for(int i = 0; i < cur_lanes.size(); i++){
        if(cur_lanes[i] != prev_lanes[i])
            return false;
    }

    return true;
}

void CalcAroundWaypoints::callbackCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr& in_velocity){
    cur_vel = in_velocity->twist.linear.x;
}

void CalcAroundWaypoints::callbackDecisionMakerState(const std_msgs::StringConstPtr& input_msg)
{
	if (input_msg->data.find("Drive\n") != std::string::npos && input_msg->data.find("VehicleReady\n") != std::string::npos)
	{
		if (input_msg->data.find("CheckLeftLane\n") != std::string::npos || input_msg->data.find("ChangeToLeft\n") != std::string::npos)
		{
			left_check = true;
			right_check = false;

            if(check_seq <= 0){
                check_seq++;
                prev_x = cur_x;
                prev_y = cur_y;
            }
		}
		else if (input_msg->data.find("CheckRightLane\n") != std::string::npos || input_msg->data.find("ChangeToRight\n") != std::string::npos)
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

        if (input_msg->data.find("ChangeToLeft\n") != std::string::npos)
		{
			left_change = true;
			right_change = false;
		}
		else if (input_msg->data.find("ChangeToRight\n") != std::string::npos)
		{
			left_change = false;
			right_change = true;
		}
		else
		{
			left_change = false;
			right_change = false;
		}
	}
}

void CalcAroundWaypoints::SetNearLane(autoware_msgs::LaneArray& out_lane, autoware_msgs::Lane& lane,
                                        std::vector<int>& check_same_tmp, int& seq){
    double lane_vector_x = cur_x - prev_x;
    double lane_vector_y = cur_y - prev_y;
    double side_vector_x = lane_x - prev_x;
    double side_vector_y = lane_y - prev_y;
    double outer_product;
    outer_product = (lane_vector_x * side_vector_y) - (lane_vector_y * side_vector_x);

    if(left_check && outer_product > 0){
        if(left_change){
            nh.setParam("/op_trajectory_generator/samplingOutMargin", calcRollInMargin());
            out_lane.lanes.clear();
            out_lane.lanes.resize(0);
            check_same_tmp.clear();
            check_same_tmp.resize(0);
            changed_lane = lane;
            change_done = false;
        }
        ROS_INFO("set left lane!");
        out_lane.lanes.emplace_back(lane);
        check_same_tmp.emplace_back(seq);
    }
    else if(right_check && outer_product < 0){
        if(right_change){
            nh.setParam("/op_trajectory_generator/samplingOutMargin", calcRollInMargin());
            out_lane.lanes.clear();
            out_lane.lanes.resize(0);
            check_same_tmp.clear();
            check_same_tmp.resize(0);
            changed_lane = lane;
            change_done = false;
        }
        ROS_INFO("set right lane!");
        out_lane.lanes.emplace_back(lane);
        check_same_tmp.emplace_back(seq);
    }
    lane_check_ok = true;
}

double CalcAroundWaypoints::calcRollInMargin(){
    double margin;
    if(cur_vel < 5)
        margin = DEFAULT_MARGIN;
    else
        margin = 8 + (0.8*cur_vel) + (2.5 * sqrt(cur_vel - 4));

    // if(margin <= DEFAULT_MARGIN) return DEFAULT_MARGIN;
    // else return margin; 
    ROS_INFO("Current v : %f", cur_vel);
    ROS_INFO("OutMargin : %f", margin);
    return margin;
}

void CalcAroundWaypoints::run(){
    autoware_msgs::LaneArray output_lane;
    output_lane.id = tmp_lane.id;
    output_lane.lanes.clear();
    output_lane.lanes.resize(0);

    std::vector<int> check_same_tmp;
    check_same_tmp.clear();
    check_same_tmp.resize(0);
    
    bool is_near = false;

    int cur_index;
    if(change_done){
        nh.setParam("/op_trajectory_generator/samplingOutMargin", DEFAULT_MARGIN);
        for(int i = 0; i < tmp_lane.lanes.size(); i++){
            for(auto& waypoint : tmp_lane.lanes[i].waypoints){
                if(CurrentLaneCheck(waypoint.pose.pose.position.x
                                ,waypoint.pose.pose.position.y
                                ,waypoint.pose.pose.position.z))
                {
                    output_lane.lanes.push_back(tmp_lane.lanes[i]);
                    check_same_tmp.emplace_back(i);
                    cur_index = i;
                    break;
                }
            }
        }
            
        for(int i = 0; i < tmp_lane.lanes.size(); i++){
            if(i != cur_index){
                for(auto& waypoint : tmp_lane.lanes[i].waypoints){
                    if(DistanceCheck(waypoint.pose.pose.position.x
                                    ,waypoint.pose.pose.position.y
                                    ,waypoint.pose.pose.position.z))
                    {
                        is_near = true;
                        lane_x = waypoint.pose.pose.position.x;
                        lane_y = waypoint.pose.pose.position.y;
                        break;
                    }
                }

                if(is_near){
                    if(left_check || right_check){
                        SetNearLane(output_lane, tmp_lane.lanes[i], check_same_tmp, i);
                    }
                    is_near = false;
                }
            }
        }
    }
    else{
        ROS_INFO("change_done = false");
        change_done = LaneChangeDone(changed_lane);
        output_lane.lanes.clear();
        output_lane.lanes.resize(0);
        output_lane.lanes.emplace_back(changed_lane);
    }

    if(!checkSameLane(check_same_tmp,check_same)){
        lane_pub.publish(output_lane);
        check_same = check_same_tmp;
        ROS_INFO("Changed around lanes!");
        lane_check_ok = false;
    }
}