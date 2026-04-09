/*

Programme réalisé dans le cadre du projet RoboCup Rescue de MT5 de l'ISTY  
Année 2024
Groupe de Marya Pasukhova, Tanguy Foret, Elian Dussart, Tom Roudaut, Baptiste Giraud, Angélique Da Costa, Aurélien Lelay.
Professeur encadrant : Pierre Blazevic, Olivier Snoeck

Commande pour lancer le programme et son contenu : roslaunch ROBOCUP2024 Rescue.launch

*/
#include <cstdlib>
#define nullptr NULL

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
#include <string>

#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>
#include <vector>

#include "opencv2/imgcodecs.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <cstdlib>

#include <unistd.h>
#include <sys/stat.h> //Pour créer des dossiers s'ils ne sont pas déjà existants.
#include <sstream>
#include <sensor_msgs/Joy.h>

/* Déclaration des prototypes */
void trajectoryCallback(const nav_msgs::Path::ConstPtr& msg); 
int Copie_joy();

/*________________________________________*/

using namespace cv;
using namespace std;

int test = 0;
int j = 0;
const char* dir = "/home/ros/catkin_ws/src/ROBOCUP2024/src/trajectoire"; //déclare 
std::ifstream file(std::string(dir) + "/trajectory_data.txt"); // Remplacez "monfichier.txt" par le nom de votre fichier
const char* dir_joy = "/home/ros/catkin_ws/src/ROBOCUP2024/src/commandes_manette"; //déclare 
std::ifstream file_joy(std::string(dir_joy) + "/commandes_manette.txt"); // Remplacez "monfichier.txt" par le nom de votre fichier
std::vector<std::string> col0; //Timer
std::vector<std::string> col1; //Bouton 1 (appuyé == 1)
std::vector<std::string> col2; //Bouton 2 (appuyé == 1)
std::vector<std::string> col3; //Bouton 3 (appuyé == 1)
std::vector<std::string> col4; //Bouton 4 (appuyé == 1)
std::vector<std::string> col5; //Bouton L1 (appuyé == 1)
std::vector<std::string> col6; //Bouton R1 (appuyé == 1)
std::vector<std::string> col7; //Bouton L2 (appuyé == 1)
std::vector<std::string> col8; //Bouton R2 (appuyé == 1)
std::vector<std::string> col9; //Bouton Select (appuyé == 1)
std::vector<std::string> col10; //Bouton Start (appuyé == 1)
std::vector<std::string> col11; //Enfonçage Joystick Gauche (appuyé == 1)
std::vector<std::string> col12; //Enfonçage Joystick Droite (appuyé == 1)
std::vector<std::string> col13; //Gauche / Droite Joystick gauche (droite == -1)
std::vector<std::string> col14; //Avant / Arrière Joystick gauche (arrière == -1)
std::vector<std::string> col15; //Avant / Arrière Joystick droite (arrière == -1)
std::vector<std::string> col16; //Gauche / Droite Joystick droite (droite == -1)
std::vector<std::string> col17; //Gauche / Droite Croix (droite == -1)
std::vector<std::string> col18; //Haut / Bas Croix (bas == -1)
std::string val0, val1, val2, val3, val4, val5, val6, val7, val8, val9, val10, val11, val12, val13, val14, val15, val16, val17, val18;
int i = 0;
int k[6] = {0};
double recuperation_x = 0;
double recuperation_y = 0;
double recuperation_qw = 0;
double somme_x = 0;
double somme_y = 0;
double somme_qw = 0;
int compteur_for = 0;
float value2 = 0;
float value1 = 0;
size_t q = 0;

/*__________________________________________________________________________*/
	

void trajectoryCallback(const nav_msgs::Path::ConstPtr& msg) 
{
	std::string line;
	if(j == 0)
	{
		while (getline(file, line)) 
		{
        			std::istringstream iss(line);
        			
        			iss >> val1 >> val2 >> val3 >> val4;
        			col1.push_back(val1);
        			col2.push_back(val2);
        			col3.push_back(val3);
        			col4.push_back(val4);
    		}
		
    		file.close();
		
    		// Afficher les valeurs pour vérification
    		for (size_t i = 0; i < col1.size(); ++i) 
    		{
        			//std::cout << col1[i] << " " << col2[i] << " " << col3[i] << " " << col4[i] << std::endl;
    		}
		j++;
	}
	for (q; q < msg->poses.size(); ++q) 
	{
    		const geometry_msgs::PoseStamped& pose_stamped = msg->poses[q];
		// Récupérer la position
			
			double x = pose_stamped.pose.position.x;
			double y = pose_stamped.pose.position.y;
			double qw = pose_stamped.pose.orientation.w;
			
			float transitoire1 = 0;
			float transitoire2 = 0;
			float transitoire3 = 0;
			float transitoire4 = 0;
			
			// Récupérer l'orientation sous forme de quaternion
			geometry_msgs::Quaternion quat = pose_stamped.pose.orientation;
			
			// Convertir le quaternion en angles de roulis, tangage et lacet (roll, pitch, yaw)
			tf::Quaternion tf_quat;
			tf::quaternionMsgToTF(quat, tf_quat);
			double roll, pitch, yaw;
			tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
			int condition = 0;
		while((i < col1.size())&&(condition!=1))//&&(compteur_for == 10))
    		{
			
    			transitoire2 = std::stof(col2[i]);
    			transitoire3 = std::stof(col3[i]);
    			transitoire4 = std::stof(col4[i]);
				std::cout<< "Position: x = "<<x<<", y = "<<y<<endl;
				std::cout<<"Orientation: qw = "<<qw<<endl;
				std::cout << "Sortie fichier : " << col1[i] << " " << col2[i] << " " << col3[i] << " " << col4[i] << std::endl;
				if( x > transitoire2 + 0.05 )
				{
					std::cout<<"Reculer"<<endl;
					k[1]++;
					
				}
				else if( x < transitoire2 - 0.05 )
				{
					std::cout<<"Avancer"<<endl;
					k[2]++;
				}
				else if( y > transitoire3 + 0.05 )
				{
					std::cout<<"Gauche"<<endl;
					k[3]++;
				}
				else if( y < transitoire3 - 0.05 )
				{
					std::cout<<"Droite"<<endl;
					k[4]++;
				}
				else if( (qw > transitoire4 + 0.1)&&(transitoire4!=0) )
				{
					std::cout<<"Rotation Gauche"<<endl;
					//i--;
					//break;
					k[5]++;
				}
				else if( (qw  < transitoire4 - 0.1)&&(transitoire4!=0) )
				{
					std::cout<<"Rotation Droite"<<endl;
					//i--;
					//break;
					k[6]++;
				}
				else
				{
					i++;
				}
				ros::Duration(0.005).sleep();
				for(int y = 0; y < 7; y++)
				{
				
					if(k[y] >= 5)
					{
						condition = 1;
						compteur_for++;
						//q = i;
						std::cout << "K == 5" << endl;
						for(int t = 0; t < 6; t++)
						{
							k[t] = 0;
						}
					}
				}
		}
		
		
		if(compteur_for == 10)
		{
			compteur_for = 0;
		}
	}
	
	return;
}

int Copie_joy()
{
	std::string line;
	
	while (getline(file_joy, line)) 
	{
		std::istringstream iss(line);
		//for (size_t i = 0; i < col1.size(); ++i) 
		if (iss >> val0 >> val1 >> val2 >> val3 >> val4 >> val5 >> val6 >> val7 >> val8 >> val9 >> val10 >> val11 >> val12 >> val13 >> val14 >> val15 >> val16 >> val17 >> val18)
		{
			col0.push_back(val0);
        			col1.push_back(val1);
        			col2.push_back(val2);
        			col3.push_back(val3);
        			col4.push_back(val4);
        			col5.push_back(val5);
        			col6.push_back(val6);
        			col7.push_back(val7);
        			col8.push_back(val8);
        			col9.push_back(val9);
        			col10.push_back(val10);
        			col11.push_back(val11);
        			col12.push_back(val12);
        			col13.push_back(val13);
        			col14.push_back(val14);
        			col15.push_back(val15);
        			col16.push_back(val16);
        			col17.push_back(val17);
        			col18.push_back(val18);
        			std::cout << "Sortie fichier : " << col0[i] << " " << col1[i] << " " << col2[i] << " " << col3[i] << " " << col4[i] << " " << col5[i] << " " << col6[i] << " " << col7[i] << " " << col8[i] << " " << col9[i] << " " << col10[i] << " " << col11[i] << " " << col12[i] << " " << col13[i] << " " << col14[i] << " " << col15[i] << " " << col16[i] << " " << col17[i] << " " << col18[i] << " " << std::endl;
        			//std::cout << "Sortie Val : " << val13 << " " << val14 << " " << val15 << " " << val16 << " " << val17 << " " << val18 << endl;
        		}
        		
        		if(col14[i] == "1")
        		{
        			std::cout<<"Avancement Flipper"<<endl;
        		}
        		else if(col14[i] == "-1")
        		{
        			std::cout<<"Reculement Flipper"<<endl;
        		}
        		else
        		{
        			std::cout << "Pas de Mouvement Flipper" << endl;
        		}
        		if(col15[i] == "1")
        		{
        			std::cout<<"Avancement Robot"<<endl;
        		}
        		else if(col15[i] == "-1")
        		{
        			std::cout<<"Reculement Robot"<<endl;
        		}
        		else
        		{
        			std::cout << "Pas de Déplacement Robot" << endl;
        		}
        		if(col16[i] == "1")
        		{
        			std::cout<<"Rotation Gauche Robot"<<endl;
        		}
        		else if(col16[i] == "-1")
        		{
        			std::cout<<"Rotation Droite Robot"<<endl;
        		}
        		else
        		{
        			std::cout << "Pas de Rotation Robot" << endl;
        		}
        		
        		value1 = std::atof(col0[i].c_str());
        		if(((i-1)!=0) && (i!=0))
    			value2 = std::atof(col0[i-1].c_str());
    		else
    			value2 = value1;
    		float time_difference = value1 - value2;
    		
		ros::Duration(time_difference).sleep();
        		i++;
        			
    	}
	
    	file_joy.close();
    	return 0;
}

int main(int argc, char **argv)
{  
	
	ros::init(argc, argv, "ROBOCUP2024_copie_trajet_node");
  
	ros::NodeHandle n;
	
	/* ===Attribution des noeuds et de leur nom respectif pour les flux vidéo===*/
	//ros::Subscriber sub = n.subscribe("/trajectory", 1000, trajectoryCallback);
	Copie_joy();
    		
		
	ros::Rate loop_rate(300);
	while (ros::ok())
	{
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
