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

int PrisePhotos(cv_bridge::CvImagePtr Image);
int PrisePhotosNoirEtBlanc(cv_bridge::CvImagePtr cv_ptr);
int PrisePhotosAretes(cv_bridge::CvImagePtr cv_ptr);
int Rotation_Images(cv_bridge::CvImagePtr cv_ptr);

/*________________________________________*/

using namespace cv;
using namespace std;

//geometry_msgs::Vector3 Co;

image_transport::Subscriber image_sub_;
image_transport::Publisher image_pub_;
sensor_msgs::ImagePtr img_msg;
cv::Mat image, gray, edges, rotated_image;
image_transport::Publisher image_rotation;


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
	
	PrisePhotos(cv_ptr);
	/*PrisePhotosNoirEtBlanc(cv_ptr);
	PrisePhotosAretes(cv_ptr);*/
	//Rotation_Images(cv_ptr);
	
   
    	//std::string filepath = std::string(dir) + "/captured_image2.jpg";
    	//cv::imwrite(filepath, cv_ptr->image);
    	//cv::imwrite("/home/ros/catkin_ws/src/ROBOCUP2024/src/captured_image.jpg", cv_ptr->image); //Sauvegarde une image de ce que voit la caméra.
};

//Déclaration fonction prise de photos
int PrisePhotos(cv_bridge::CvImagePtr cv_ptr)
{
//-----------------------------------Création du / des dossiers ---------------------------------//
	const char* dir = "/home/ros/catkin_ws/src/ROBOCUP2024/src/images"; //déclare 
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
	
	
	if(i < 10000000)
	{
		// Convertir 'i' en std::string
		std::string index = std::to_string(i);
		std::string filepath = std::string(dir) + "/captured_image" + index + ".jpg";
		if (access(filepath.c_str(), F_OK) != -1)  // Concaténer avec le préfixe pour créer le nom du fichier
		{
			// Le fichier existe
			i += 1;
			std::cout << i << std::endl;
		} 
		else 
		{
			// Le fichier n'existe pas
			// Afficher ou utiliser le nom du fichier
			std::cout << filepath << std::endl;
			//cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2GRAY);
			cv::imwrite(filepath, cv_ptr->image);
		}
		
		//double milliseconds = 1; // remplacez 1000 par le nombre de millisecondes souhaité
		//ros::Duration duration_in_ms(milliseconds / 1000.0);
		// Attend (sleep) pendant la durée spécifiée
		//duration_in_ms.sleep();
		//cv_ptr.reset();
		/*cv_ptr.reset();
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);*/	
	}
return 0;
}

int Rotation_Images(cv_bridge::CvImagePtr cv_ptr)
{
//-----------------------------------Création du / des dossiers ---------------------------------//
	cv::rotate(cv_ptr->image, rotated_image, cv::ROTATE_90_CLOCKWISE);
	cv_ptr->image = rotated_image;
	cv::rotate(cv_ptr->image, rotated_image, cv::ROTATE_90_CLOCKWISE);
	cv_ptr->image = rotated_image;

        // Output modified video stream
	image_rotation.publish(cv_ptr->toImageMsg());
	
return 0;
}


int PrisePhotosAretes(cv_bridge::CvImagePtr cv_ptr)
{
//-----------------------------------Création du / des dossiers ---------------------------------//
	const char* dir = "/home/ros/catkin_ws/src/ROBOCUP2024/src/images3"; //déclare 
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
	
	
	if(i < 10000000)
	{
		// Convertir 'i' en std::string
		std::string index = std::to_string(i);
		std::string filepath = std::string(dir) + "/captured_image" + index + ".jpg";
		if (access(filepath.c_str(), F_OK) != -1)  // Concaténer avec le préfixe pour créer le nom du fichier
		{
			// Le fichier existe
			i += 1;
			std::cout << i << std::endl;
		} 
		else 
		{
			// Le fichier n'existe pas
			// Afficher ou utiliser le nom du fichier
			std::cout << filepath << std::endl;
			cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2GRAY);
			cv::Canny(cv_ptr->image, edges, 50, 150);
			cv::imwrite(filepath, edges);
		}
		
		//double milliseconds = 1; // remplacez 1000 par le nombre de millisecondes souhaité
		//ros::Duration duration_in_ms(milliseconds / 1000.0);
		// Attend (sleep) pendant la durée spécifiée
		//duration_in_ms.sleep();
		//cv_ptr.reset();
		/*cv_ptr.reset();
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);*/	
	}
return 0;
}

int PrisePhotosNoirEtBlanc(cv_bridge::CvImagePtr cv_ptr)
{
//-----------------------------------Création du / des dossiers ---------------------------------//
	const char* dir = "/home/ros/catkin_ws/src/ROBOCUP2024/src/images2"; //déclare 
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
	
	
	if(i < 10000000)
	{
		// Convertir 'i' en std::string
		std::string index = std::to_string(i);
		std::string filepath = std::string(dir) + "/captured_image" + index + ".jpg";
		if (access(filepath.c_str(), F_OK) != -1)  // Concaténer avec le préfixe pour créer le nom du fichier
		{
			// Le fichier existe
			i += 1;
			std::cout << i << std::endl;
		} 
		else 
		{
			// Le fichier n'existe pas
			// Afficher ou utiliser le nom du fichier
			std::cout << filepath << std::endl;
			cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
			cv::imwrite(filepath, gray);
		}
		
		//double milliseconds = 1; // remplacez 1000 par le nombre de millisecondes souhaité
		//ros::Duration duration_in_ms(milliseconds / 1000.0);
		// Attend (sleep) pendant la durée spécifiée
		//duration_in_ms.sleep();
		//cv_ptr.reset();
		/*cv_ptr.reset();
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);*/	
	}
return 0;
}

// Callback function to handle joystick messages
/*void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
//buttons 5 et 7 : L et R droite.
    ROS_INFO("Axes: [%f, %f, %f, %f, %f, %f, %f, %f]",
             joy->axes[0], joy->axes[1], joy->axes[2], joy->axes[3],
             joy->axes[4], joy->axes[5], joy->axes[6], joy->axes[7]);

    ROS_INFO("Buttons: [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d]",
             joy->buttons[0], joy->buttons[1], joy->buttons[2], joy->buttons[3],
             joy->buttons[4], joy->buttons[5], joy->buttons[6], joy->buttons[7],
             joy->buttons[8], joy->buttons[9], joy->buttons[10], joy->buttons[11],
             joy->buttons[12], joy->buttons[13], joy->buttons[14], joy->buttons[15], joy->buttons[16]);
}*/

sensor_msgs::Joy::ConstPtr lastJoyMsg;

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
        //Axe 2 : Mouvement Joystick Gauche Avant / Arrière (positif ou négatif)

        ROS_INFO_STREAM(ss.str());
        ROS_INFO_STREAM(sa.str());
    }
}


int main(int argc, char **argv)
{  
	ros::init(argc, argv, "ROBOCUP2024_node");
  
	ros::NodeHandle n;
	image_transport::ImageTransport it_(n);
  
	/* ===Attribution des noeuds et de leur nom respectif pour les flux vidéo===*/
	//image_rotation = it_.advertise("/output_image_topic", 1);
	image_sub_ = it_.subscribe("/cv_camera/image_raw", 1, imageCb);
	//ros::Subscriber sub = n.subscribe<sensor_msgs::Joy>("/joy", 10, joyCallback);
	//ros::Timer timer = n.createTimer(ros::Duration(0.05), timerCallback);
	
	
	ros::Rate loop_rate(300);
	
		
	
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
