#ifndef __LANECHANGE_MANAGER_STATE__
#define __LANECHANGE_MANAGER_STATE__

#include <map>
#include <vector>
#include <string>

#define STATE_FOLLOW 0
#define STATE_NEED_LANE_CHANGE 1
#define STATE_CHECK_NEXT_LANE 2
#define STATE_EXCUTE_LANE_CHANGE 3
#define STATE_LANE_CHANGE_DONE 4

/* transition
    * not_need_lane_change
    * received_global_path
    * lane_change_is_done
    * can_not_excute_lane_change
    * can_excute_lane_change
    * received_multi_global_path
    * is_target_lane_yes
    * is_target_lane_no
    * current_lane_is_main
*/
class LaneChangeStateMachine{
public:
    LaneChangeStateMachine(){
        transition_map_follow.insert(std::make_pair("received_global_path",                  STATE_NEED_LANE_CHANGE)); // follow -> need_lane_change
        transition_map_follow.insert(std::make_pair("current_lane_is_main",                  STATE_FOLLOW));           // follow -> follow
        transition_map_need_lane_chanege.insert(std::make_pair("not_need_lane_change",       STATE_FOLLOW));           // need_lane_change -> follow
        transition_map_need_lane_chanege.insert(std::make_pair("received_multi_global_path", STATE_CHECK_NEXT_LANE));  // need_lane_change -> check_next_lane
        transition_map_check_next_lane.insert(std::make_pair("can_not_excute_lane_change",   STATE_CHECK_NEXT_LANE));  // check_next_lane -> check_next_lane
        transition_map_check_next_lane.insert(std::make_pair("can_excute_lane_change",       STATE_EXCUTE_LANE_CHANGE)); // check_next_lane -> excute_lane_change
        transition_map_check_next_lane.insert(std::make_pair("current_lane_is_main",         STATE_FOLLOW));           // check_next_lane -> follow
        transition_map_excute_lane_change.insert(std::make_pair("is_target_lane_no",         STATE_EXCUTE_LANE_CHANGE)); // excute_lane_change -> excute_lane_change
        transition_map_excute_lane_change.insert(std::make_pair("is_target_lane_yes",        STATE_LANE_CHANGE_DONE)); // excute_lane_change -> lane_change_done
        transition_map_lane_change_done.insert(std::make_pair("lane_change_is_done",         STATE_NEED_LANE_CHANGE)); // lane_change_done -> need_lane_change
        transition_map.emplace_back(transition_map_follow);
        transition_map.emplace_back(transition_map_need_lane_chanege);
        transition_map.emplace_back(transition_map_check_next_lane);
        transition_map.emplace_back(transition_map_excute_lane_change);
        transition_map.emplace_back(transition_map_lane_change_done);
        state = STATE_FOLLOW;
    }
    ~LaneChangeStateMachine(){}

    void onUpdate(){
        // stateCallBack()
    }

/*
    void stateCallBack(){
        // state 마다 수행해야하는 별도의 동작 정의
        // ex) lane change 하는 state 일 때 replanning 못하도록 global planner에 메세지전송
    }
*/

    void tryNextState(std::string transition){
        uint prev_state = state;
        if (transition_map[state].count(transition) == 0){
            state = state;
        }
        else{
            // onUpdate();
            state = transition_map[state].at(transition);
            if(prev_state != state){
                std::string prev, curr;
                if (prev_state == STATE_FOLLOW) prev = "follow";
                else if (prev_state == STATE_NEED_LANE_CHANGE) prev = "need lane change";
                else if (prev_state == STATE_CHECK_NEXT_LANE) prev = "check next lane";
                else if (prev_state == STATE_EXCUTE_LANE_CHANGE) prev = "excute lane change";
                else if (prev_state == STATE_LANE_CHANGE_DONE) prev = "lane change done";

                if (state == STATE_FOLLOW) curr = "follow";
                else if (state == STATE_NEED_LANE_CHANGE) curr = "need lane change";
                else if (state == STATE_CHECK_NEXT_LANE) curr = "check next lane";
                else if (state == STATE_EXCUTE_LANE_CHANGE) curr = "excute lane change";
                else if (state == STATE_LANE_CHANGE_DONE) curr = "lane change done";

                ROS_WARN("Changed State [%s]->[%s]", prev.c_str(), curr.c_str());
            }
        }
        
    }

    void getCurrentState(){
        return state;
    }

    bool isCurrentState(uint state_){
        return state_ == state;
    }

private:
    uint state;
    std::map<std::string, uint> transition_map_follow;
    std::map<std::string, uint> transition_map_need_lane_chanege;
    std::map<std::string, uint> transition_map_check_next_lane;
    std::map<std::string, uint> transition_map_excute_lane_change;
    std::map<std::string, uint> transition_map_lane_change_done;
    std::vector<std::map<std::string, uint>> transition_map;
};


#endif