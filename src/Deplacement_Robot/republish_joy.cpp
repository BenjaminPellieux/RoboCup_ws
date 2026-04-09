#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

class JoyNode {
private:
	ros::NodeHandle nh;
	ros::Publisher joy_pub;
	ros::Subscriber joy_sub;
	ros::Timer timer;
	sensor_msgs::Joy last_joy_msg;
	bool is_first_message;

public:
	JoyNode() {
		joy_pub = nh.advertise<sensor_msgs::Joy>("/joy_out", 10);
		joy_sub = nh.subscribe("/joy", 10, &JoyNode::joyCallback, this);
		timer = nh.createTimer(ros::Duration(0.2), &JoyNode::timerCallback, this);
		is_first_message = true;
	}

	void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
		if (is_first_message || *msg != last_joy_msg) {
			last_joy_msg = *msg;
			is_first_message = false;
		}
	}

	void timerCallback(const ros::TimerEvent&) {
		joy_pub.publish(last_joy_msg);
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "republish_joy_node");
	JoyNode joy_node;
	ros::spin();
	return 0;
}