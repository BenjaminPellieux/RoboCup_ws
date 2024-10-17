#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16MultiArray.h>


class JoyNode {
private:
    ros::NodeHandle nh;
    ros::Publisher joy_pub;
    ros::Publisher consigne_pub;    
    ros::Subscriber joy_sub;
    ros::Timer timer;
    sensor_msgs::Joy last_joy_msg;
    bool is_first_message;
    std_msgs::Int16MultiArray msg_int;

public:
    JoyNode() {
        joy_pub = nh.advertise<sensor_msgs::Joy>("/joy_out", 10);
        joy_sub = nh.subscribe("/joy", 10, &JoyNode::joyCallback, this);
        timer = nh.createTimer(ros::Duration(0.2), &JoyNode::timerCallback, this);
        
        
          consigne_pub = nh.advertise<std_msgs::Int16MultiArray >("/consigne", 1);



	
	// set up dimensions for multiArray
	msg_int.layout.dim.push_back(std_msgs::MultiArrayDimension());
	msg_int.layout.dim[0].size = 2;
	msg_int.layout.dim[0].stride = 1;
	msg_int.layout.dim[0].label = "consigne"; // or whatever name you typically use to index vec1
	
	// save space
	msg_int.data.clear();
	msg_int.data.push_back(0);
	msg_int.data.push_back(0);
	msg_int.data.push_back(0);
	msg_int.data.push_back(0);
	msg_int.data.push_back(0);
	msg_int.data.push_back(0);
	msg_int.data.push_back(0);
	msg_int.data.push_back(0);
	msg_int.data.push_back(0);
						
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {

        
	//moteur 
	int xAxis_m = msg->axes[0]*127;
	int yAxis_m = msg->axes[1]*127;
	
	//flipper ar
	int xAxis2_f_ar = msg->axes[2]*127;
	
	//flipper droit av
	int button1_f_av_d = msg->buttons[0]*127;
	int button2_f_av_d = msg->buttons[1]*127;
	
	//flipper gauche av
	int button4_f_av_g = msg->buttons[3]*127;
	int button3_f_av_g = msg->buttons[2]*127;
	
	//camera
	int button_camera = msg->buttons[5]*127;
	int button_camera_2 = msg->buttons[7]*127;	
	
	
	
	msg_int.data[0]=xAxis_m;
	msg_int.data[1]=yAxis_m;
	msg_int.data[2]=xAxis2_f_ar;
	msg_int.data[3]=button1_f_av_d;
	msg_int.data[4]=button2_f_av_d;
	msg_int.data[5]=button4_f_av_g;
	msg_int.data[6]=button3_f_av_g;
	msg_int.data[7]=button_camera;
	msg_int.data[8]=button_camera_2;	
	
    }

    void timerCallback(const ros::TimerEvent&) {
        consigne_pub.publish(msg_int);
    }
    

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "republish_joy_node");
    JoyNode joy_node;
    ros::spin();
    return 0;
}
