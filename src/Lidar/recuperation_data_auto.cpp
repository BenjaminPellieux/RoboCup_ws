/*

Programme réalisé dans le cadre du projet RoboCup Rescue de MT5 de l'ISTY  
Année 2024
Groupe de Marya Pasukhova, Tanguy Foret, Elian Dussart, Tom Roudaut, Baptiste Giraud, Angélique Da Costa, Aurélien Lelay.
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
#include <sensor_msgs/Joy.h>

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
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
void trajectoryCallback(const nav_msgs::Path::ConstPtr& msg); 

/*________________________________________*/

#define NUM_SCANS 1
#define NUM_VALUES 682

using namespace cv;
using namespace std;

ros::Time start_time;
ros::Duration n_precedent;
ros::Duration n;
sensor_msgs::Joy::ConstPtr lastJoyMsg;
std::ofstream outfile;
std::ofstream outfile_scan;
std::ofstream outfile_joy;
bool verification = false;
bool verification2 = false;
int test = 0;

/*__________________________________________________________________________*/

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) 
{
	// Traitement des données du scan
	// scan->ranges[] contient les distances mesurées
	// Temps entre chaque scan : 0.000148701872s
	
	std::vector<float> scan_data[5000];
	
	ros::Duration elapsed_time = ros::Time::now() - start_time;
	outfile_scan << elapsed_time << ";;;";
	for (int i = 0; i < NUM_SCANS; i++) 
	{
		
    		scan_data[i] = scan->ranges;
    		
    		for(int j = 0; j < NUM_VALUES; j++)//trajectory_data
    		{
    			//std::cout<<"scan_data["<<i<<"]["<<j<<"] = "<<scan_data[i][j] << " ; ";
    		//std::cout<<endl;	
    			outfile_scan << scan_data[i][j] << "; ";
    		}	
	}
	//std::cout<< ++test << " = " << elapsed_time << endl;
	
	outfile_scan << std::endl;
	
}
	

void trajectoryCallback(const nav_msgs::Path::ConstPtr& msg) 
{
	int i = 0;
	double x[100000] = {0};
	double y[100000] = {0};
	double yaw[100000] = {0};
	double qw[100000] = {0};
	double roll, pitch;
	
	std::vector<float> scan_data[5000];
	
	for (const geometry_msgs::PoseStamped& pose_stamped : msg->poses) 
	{
		i++;
		// Récupérer la position
		x[i] = pose_stamped.pose.position.x;
		y[i] = pose_stamped.pose.position.y;
		qw[i] = pose_stamped.pose.orientation.w;
		
		// Récupérer l'orientation sous forme de quaternion
		geometry_msgs::Quaternion quat = pose_stamped.pose.orientation;
		
		// Convertir le quaternion en angles de roulis, tangage et lacet (roll, pitch, yaw)
		tf::Quaternion tf_quat;
		tf::quaternionMsgToTF(quat, tf_quat);
		tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw[i]);
		
		// Afficher ou utiliser les valeurs x, y, roll, pitch, yaw
		/*ROS_INFO("Elapsed Time: %f seconds", elapsed_time.toSec());
		ROS_INFO("Position: x = %f, y = %f", x, y);
		ROS_INFO("Orientation: roll = %f, pitch = %f, yaw = %f", roll, pitch, yaw);*/
		
		//ROS_INFO("qw : %f\n", qw);
		
	}
	std::cout<<n<<endl;
	if(verification == true)
	{
		verification2 = true;
		n_precedent = n;
	}
	verification = true;
	//std::cout<<i<<endl;
	n = ros::Time::now() - start_time;
	//std::cout<<n_precedent<<endl;
	std::cout<<n<<endl;
	//if((verification2 == true)&&(n_precedent.toSec() > 2.5))
	
	for(int k = 0; k < i; k++)
	{
		if((verification2 == true)&&(ros::Time::now().toSec() - start_time.toSec() + ros::Duration(static_cast<double>(k)*((static_cast<double>(n.toSec()) - static_cast<double>(n_precedent.toSec()))/static_cast<double>(i))).toSec() > 4))
		{
			for(int j = 0; j < i; j++)
			{
				//std::cout<<static_cast<double>(4)/static_cast<double>(i)<<endl;
				//std::cout<<(static_cast<double>(n.toSec()) - static_cast<double>(n_precedent.toSec()))<<endl;
				outfile << ros::Time::now() - start_time + ros::Duration(static_cast<double>(j)*((static_cast<double>(n.toSec()) - static_cast<double>(n_precedent.toSec()))/static_cast<double>(i))) << " " << x[j] << " " << y[j] << " " << qw[j] << std::endl;
			}
			outfile.flush();
			verification2 = false;
		}
	}
	
	return;
}

// Fonction de callback pour le topic /joy
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    lastJoyMsg = joy;
    
}

// Fonction de callback pour le timer
void timerCallback(const ros::TimerEvent&)
{
    if (lastJoyMsg)
    {
        	std::stringstream ss, sa;
        	ss << "Buttons : ";
        	sa << "Axes : ";
	
        	for(size_t i = 0; i < lastJoyMsg->buttons.size(); ++i) {
            	ss << lastJoyMsg->buttons[i] << " ";
        	}
        	for(size_t i = 0; i < lastJoyMsg->axes.size(); ++i) {
            	sa << lastJoyMsg->axes[i] << " ";
        	}
	
        	ROS_INFO_STREAM(ss.str());
        	ROS_INFO_STREAM(sa.str());
        	
        	n = ros::Time::now() - start_time;
    	//std::cout<<n_precedent<<endl;
    	std::cout<<n<<endl;
    	outfile_joy << n << " ";
    	for(size_t i = 0; i < lastJoyMsg->buttons.size(); ++i) 
    	{
            	outfile_joy << lastJoyMsg->buttons[i] << " ";
    	}
    	//outfile_joy << std::endl;
    	//outfile_joy << n << " ";
    	for(size_t i = 0; i < lastJoyMsg->axes.size(); ++i) 
    	{
            	outfile_joy << lastJoyMsg->axes[i] << " ";
    	}
    	outfile_joy << std::endl;
    	outfile.flush();
    }
}

int main(int argc, char **argv)
{  
	
	ros::init(argc, argv, "ROBOCUP2024_recuperation_data_auto_node");
  
	ros::NodeHandle n;
	start_time = ros::Time::now();
	/*const char* dir = "/home/ros/catkin_ws/src/ROBOCUP2024/src/trajectoire"; //déclare 
	if (access(dir, F_OK) != -1) 
	{
		// fichier existe
		std::cout << "Le fichier existe déjà." << std::endl;
	} 
	else 
	{
		// fichier n'existe pas
		mkdir(dir, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH); // crée le dossier avec les permissions appropriées
	}
	outfile.open(std::string(dir) + "/trajectory_data_test.txt", std::ios::out | std::ios::app);
	if (!outfile.is_open()) 
	{
		ROS_ERROR("Le fichier n'est pas ouvert pour l'écriture.");
	}
	else
	{
		chmod((std::string(dir) + "/trajectory_data_test.txt").c_str(), 0644);
	}
	
	const char* dir_scan = "/home/ros/catkin_ws/src/ROBOCUP2024/src/scan"; //déclare 
	if (access(dir_scan, F_OK) != -1) 
	{
		// fichier existe
		std::cout << "Le fichier existe déjà." << std::endl;
	} 
	else 
	{
		// fichier n'existe pas
		mkdir(dir_scan, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH); // crée le dossier avec les permissions appropriées
	}
	outfile_scan.open(std::string(dir_scan) + "/scan_data.txt", std::ios::out | std::ios::app);
	if (!outfile_scan.is_open()) 
	{
		ROS_ERROR("Le fichier n'est pas ouvert pour l'écriture.");
	}
	else
	{
		chmod((std::string(dir_scan) + "/scan_data.txt").c_str(), 0644);
	}*/
	
	
	const char* dir_joy = "/home/ros/catkin_ws/src/ROBOCUP2024/src/commandes_manette"; //déclare 
	if (access(dir_joy, F_OK) != -1) 
	{
		// fichier existe
		std::cout << "Le fichier existe déjà." << std::endl;
	} 
	else 
	{
		// fichier n'existe pas
		mkdir(dir_joy, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH); // crée le dossier avec les permissions appropriées
	}
	outfile_joy.open(std::string(dir_joy) + "/commandes_manette.txt", std::ios::out | std::ios::app);
	if (!outfile_joy.is_open()) 
	{
		ROS_ERROR("Le fichier n'est pas ouvert pour l'écriture.");
	}
	else
	{
		chmod((std::string(dir_joy) + "/commandes_manette.txt").c_str(), 0644);
	}
	/* ===Attribution des noeuds et de leur nom respectif pour les flux vidéo===*/
	//ros::Subscriber sub = n.subscribe("/trajectory", 1000, trajectoryCallback);
	//ros::Subscriber sub2 = n.subscribe("/scan", 1000, scanCallback);
	ros::Subscriber sub = n.subscribe<sensor_msgs::Joy>("/joy", 10, joyCallback);
	ros::Timer timer = n.createTimer(ros::Duration(0.05), timerCallback);
	ros::Rate loop_rate(300);
		
	
	while (ros::ok())
	{
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	outfile.close();
	outfile_scan.close();
	outfile_joy.close();
	return 0;
}
