/*

Programme réalisé dans le cadre du projet RoboCup Rescue de MT5 de l'ISTY    
Année 2024
Groupe de Marya Pastukhova, Tanguy Foret, Elian Dussart, Tom Roudaut, Clément Cat Baptiste Giraud, Angélique Da Costa, Aurélien Lelay.
Professeur encadrant : Pierre Blazevic, Olivier Snoeck 

Commande pour lancer le programme et son contenu : roslaunch ROBOCUP2024 Rescue.launch 
*/ 
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/LaserScan.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/tf.h"
#include <fstream>
#include <sensor_msgs/Imu.h>

#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>
#include <vector>

#include "opencv2/imgcodecs.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>

#include <unistd.h>
#include <sys/stat.h> //Pour créer des dossiers s'ils ne sont pas déjà existants.
#include <sstream>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void AcceleroCallback(const geometry_msgs::Vector3::ConstPtr& msg);

using namespace cv;
using namespace std;

geometry_msgs::Vector3* Rotation_Accelero = NULL;
geometry_msgs::Vector3 retour_accelero;
bool passage_accelero = false;
double x_accelero;
double y_accelero;
double z_accelero;
double offset_Z = 0.0;

void AcceleroCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	// Allocate memory if not already done
    if (!Rotation_Accelero)
        Rotation_Accelero = new geometry_msgs::Vector3();
    // Access the x, y, and z accelerations from the Vector3 message
    if(!passage_accelero)
    {
    	offset_Z = msg->z;
    	passage_accelero = true;
    }
    double adjusted_z = msg->z - offset_Z;
    if (adjusted_z > 180)
        adjusted_z -= 360;
    else if (adjusted_z < -180)
        adjusted_z += 360;

    // Update the global variable
    Rotation_Accelero->z = adjusted_z;

    // Print the received accelerations
    //ROS_INFO("Received acceleration - x: [%f], y: [%f], z: [%f]", x, y, z);
    cout << "ROTATION_ACCELERO EN Z : " << Rotation_Accelero->z << endl;
}


int main(int argc, char **argv)
{  
	ros::init(argc, argv, "ROBOCUP2024_Initialisation_Accelero_node");
	if (Rotation_Accelero)
	{
       		 delete Rotation_Accelero;
       	}
	ros::Publisher consigne_send;
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/Pub_Accelero", 1000, AcceleroCallback);
	ros::Rate loop_rate(1);

	
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
