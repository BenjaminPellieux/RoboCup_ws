/*

Programme réalisé dans le cadre du projet RoboCup Rescue de MT5 de l'ISTY  
Année 2024
Groupe de Marya Pasukhova, Tanguy Foret, Elian Dussart, Tom Roudaut, Baptiste Giraud, Angélique Da Costa, Aurélien Lelay.
Professeur encadrant : Pierre Blazevic, Olivier Snoeck

Commande pour lancer le programme et son contenu : roslaunch ROBOCUP2024 Rescue.launch

*/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>

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

/* Déclaration des prototypes */

int Rotation_Images(cv_bridge::CvImagePtr cv_ptr,  image_transport::Publisher pub_rotated);

/*________________________________________*/

using namespace cv;
using namespace std;

//geometry_msgs::Vector3 Co;

image_transport::Subscriber image_sub_;
image_transport::Subscriber image_sub2_;
image_transport::Subscriber image_sub3_;
image_transport::Subscriber image_sub4_;
image_transport::Publisher image_pub_;
sensor_msgs::ImagePtr img_msg;
cv::Mat image, gray, edges, rotated_image;
image_transport::Publisher image_rotation;
image_transport::Publisher image_rotation2;
image_transport::Publisher image_rotation3;
image_transport::Publisher image_rotation4;


/*__________________________________________________________________________*/

int i = 1;

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{

	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	image_pub_.publish(cv_ptr->toImageMsg());
	
	/*PrisePhotos(cv_ptr);
	PrisePhotosNoirEtBlanc(cv_ptr);
	PrisePhotosAretes(cv_ptr);*/
	Rotation_Images(cv_ptr, image_rotation);
	
   
    	//std::string filepath = std::string(dir) + "/captured_image2.jpg";
    	//cv::imwrite(filepath, cv_ptr->image);
    	//cv::imwrite("/home/ros/catkin_ws/src/ROBOCUP2024/src/captured_image.jpg", cv_ptr->image); //Sauvegarde une image de ce que voit la caméra.
};

void imageCb2(const sensor_msgs::ImageConstPtr& msg)
{

	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	image_pub_.publish(cv_ptr->toImageMsg());
	
	/*PrisePhotos(cv_ptr);
	PrisePhotosNoirEtBlanc(cv_ptr);
	PrisePhotosAretes(cv_ptr);*/
	Rotation_Images(cv_ptr, image_rotation2);
	
   
    	//std::string filepath = std::string(dir) + "/captured_image2.jpg";
    	//cv::imwrite(filepath, cv_ptr->image);
    	//cv::imwrite("/home/ros/catkin_ws/src/ROBOCUP2024/src/captured_image.jpg", cv_ptr->image); //Sauvegarde une image de ce que voit la caméra.
};

void imageCb3(const sensor_msgs::ImageConstPtr& msg)
{

	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	image_pub_.publish(cv_ptr->toImageMsg());
	
	/*PrisePhotos(cv_ptr);
	PrisePhotosNoirEtBlanc(cv_ptr);
	PrisePhotosAretes(cv_ptr);*/
	Rotation_Images(cv_ptr, image_rotation3);
	
   
    	//std::string filepath = std::string(dir) + "/captured_image2.jpg";
    	//cv::imwrite(filepath, cv_ptr->image);
    	//cv::imwrite("/home/ros/catkin_ws/src/ROBOCUP2024/src/captured_image.jpg", cv_ptr->image); //Sauvegarde une image de ce que voit la caméra.
};

void imageCb4(const sensor_msgs::ImageConstPtr& msg)
{

	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	image_pub_.publish(cv_ptr->toImageMsg());
	
	/*PrisePhotos(cv_ptr);
	PrisePhotosNoirEtBlanc(cv_ptr);
	PrisePhotosAretes(cv_ptr);*/
	Rotation_Images(cv_ptr, image_rotation4);
	
   
    	//std::string filepath = std::string(dir) + "/captured_image2.jpg";
    	//cv::imwrite(filepath, cv_ptr->image);
    	//cv::imwrite("/home/ros/catkin_ws/src/ROBOCUP2024/src/captured_image.jpg", cv_ptr->image); //Sauvegarde une image de ce que voit la caméra.
};

int Rotation_Images(cv_bridge::CvImagePtr cv_ptr, image_transport::Publisher pub_rotated)
{
//-----------------------------------Création du / des dossiers ---------------------------------//
	cv::rotate(cv_ptr->image, rotated_image, cv::ROTATE_180);
	cv_ptr->image = rotated_image;

        // Output modified video stream
	pub_rotated.publish(cv_ptr->toImageMsg());
	
return 0;
}

int main(int argc, char **argv)
{  
	ros::init(argc, argv, "Robocup2024_PaysBas_node");
  
	ros::NodeHandle n;
	image_transport::ImageTransport it_(n);
  
	/* ===Attribution des noeuds et de leur nom respectif pour les flux vidéo===*/
	image_rotation = it_.advertise("/output_image_topic", 1);
	image_rotation2 = it_.advertise("/output_image_topic2", 1);
	image_rotation3 = it_.advertise("/output_image_topic3", 1);
	image_rotation4 = it_.advertise("/output_image_topic4", 1);
	image_sub_ = it_.subscribe("/cv_camera/image_raw", 1, imageCb);
	image_sub2_ = it_.subscribe("/cv_camera2/image_raw", 1, imageCb2);
	image_sub3_ = it_.subscribe("/cv_camera4/image_raw", 1, imageCb3);
	image_sub4_ = it_.subscribe("/cv_camera6/image_raw", 1, imageCb4);
	
	ros::Rate loop_rate(300);
	
		
	
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
