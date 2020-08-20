#include <ros/ros.h>
#include <std_msgs/String.h>

class EchoDecisionPublisher{
public:
    EchoDecisionPublisher()
    : pub_echo_decision(nh.advertise<std_msgs::String>("change_decision", 10))
    , sub_request(nh.subscribe("change_sign", 1, &EchoDecisionPublisher::callbackChangeSign, this))
    , private_nh("~")
    {
        if(!private_nh.getParam("decision_sleep_sec", sleep_sec)) throw std::runtime_error("set decision_sleep_sec");
    }
    ~EchoDecisionPublisher() {}

    double getSleepFrequency() {
        if (sleep_sec < 0) return 20.0;
        else return 1.0 / sleep_sec;
    } 
    
    void callbackChangeSign(const std_msgs::String::ConstPtr& in_sign) {
        ros::Rate loop_rate(getSleepFrequency());

        std_msgs::String decision;
        if (in_sign->data == "right"){
            decision.data = "CheckRightLane";
            pub_echo_decision.publish(decision);
            ROS_INFO("CheckRightLane published!");
            loop_rate.sleep();
            decision.data = "ChangeToRight";
            pub_echo_decision.publish(decision);
            ROS_INFO("ChangeToRight published!");
            loop_rate.sleep();
            decision.data = "Straight";
            pub_echo_decision.publish(decision);
        }
        else if (in_sign->data == "left"){
            decision.data = "CheckLeftLane";
            pub_echo_decision.publish(decision);
            ROS_INFO("CheckLeftLane published!");
            loop_rate.sleep();
            decision.data = "ChangeToLeft";
            pub_echo_decision.publish(decision);
            ROS_INFO("ChangeToLeft published!");
            loop_rate.sleep();
            decision.data = "Straight";
            pub_echo_decision.publish(decision);
        }
        else {
            decision.data = "Straight";
            pub_echo_decision.publish(decision);
        }
    }

private:
    ros::NodeHandle nh, private_nh;
    ros::Publisher pub_echo_decision;
    ros::Subscriber sub_request;

    double sleep_sec;
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "echo_decision_publisher");
    EchoDecisionPublisher echo_decision_publisher;
    ros::spin();
    return 0;
}