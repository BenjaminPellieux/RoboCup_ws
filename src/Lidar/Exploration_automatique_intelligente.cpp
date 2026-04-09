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


#define NUM_SCANS 1
#define NUM_VALUES 682

struct Point_Espace {
    bool Cul_de_Sac;
    bool Chemin;
    double X;
    double Y;
};

/* Déclaration des prototypes */
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
void trajectoryCallback(const nav_msgs::Path::ConstPtr& msg); 
double deplacement_automatique(double (*scan_data)[NUM_VALUES]);
bool Verification_Tourner_Droite();

/*________________________________________*/



using namespace cv;
using namespace std;

double (*pscan)[NUM_VALUES] = NULL;
double *rotation_Lidar = NULL;
ros::Time start_time;
std::ofstream outfile;
std::ofstream outfile_scan;
size_t q = 0;
int deblocage_gauche = 0;
int deblocage_droite = 0;
Point_Espace Position_Points_dans_Espace; //Création du point pour répertorier le point vu
std::vector<Point_Espace> points;
double* x = nullptr;
double* y = nullptr;

/*__________________________________________________________________________*/

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) 
{
	// Traitement des données du scan
	// scan->ranges[] contient les distances mesurées
	// Temps entre chaque scan : 0.000148701872s
	
	std::vector<float> scan_data[5000];
	double scan_data_recuperation[NUM_SCANS][NUM_VALUES] = {0};
	ros::Duration elapsed_time = ros::Time::now() - start_time;
	for (int i = 0; i < NUM_SCANS; i++) 
	{
		
    		scan_data[i] = scan->ranges;
    		
    		for(int j = 0; j < NUM_VALUES; j++)
    		{
    			//std::cout<<"scan_data["<<i<<"]["<<j<<"] = "<<scan_data[i][j] << " ; ";
    		//std::cout<<endl;	
    			scan_data_recuperation[i][j] = scan_data[i][j];
    			if(isnan(scan_data_recuperation[i][j]) == true)
    			{
    				scan_data_recuperation[i][j] = 0;
    			}
    		}	
	}
	pscan = scan_data_recuperation; //pointeur du scan
	deplacement_automatique(scan_data_recuperation);
}
	

void trajectoryCallback(const nav_msgs::Path::ConstPtr& msg) 
{
	ros::Time starting = ros::Time::now();
	geometry_msgs::Vector3 Com;
	std::string line;
	double recuperation_plus_grande_valeur = 0;
	double Rotation_Lidar = 0;
	std::vector<double> x_values(msg->poses.size()); // Stocke les valeurs modifiées de x
	std::vector<double> y_values(msg->poses.size()); // Stocke les valeurs modifiées de y
	
	for (size_t q = 0; q < msg->poses.size(); ++q) 
	{
    			const geometry_msgs::PoseStamped& pose_stamped = msg->poses[q];
    			// Récupérer la position et stocker les adresses
    			x_values[q] = pose_stamped.pose.position.x;
    			x = &(x_values[q]); // Adresse de x
    			y_values[q] = pose_stamped.pose.position.y * (-1); // Modifier la valeur de y et la stocker
    			y = &(y_values[q]);//Droite ou gauche //Positif vers la droite
			double qx = pose_stamped.pose.orientation.x;
			double qy = pose_stamped.pose.orientation.y;
			double qw = pose_stamped.pose.orientation.w;
			double z = pose_stamped.pose.orientation.z;
			// RÃ©cupÃ©rer l'orientation sous forme de quaternion
			geometry_msgs::Quaternion quat = pose_stamped.pose.orientation;
			
			// Convertir le quaternion en angles de roulis, tangage et lacet (roll, pitch, yaw)
			tf::Quaternion tf_quat;
			tf::quaternionMsgToTF(quat, tf_quat);
			double roll, pitch, yaw;
			tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
			
			while(q < msg->poses.size())
			{
				q++; //permet de sauter jusqu'au dernier message du paquet de donnÃ©es.
			}
			
			std::cout<< "Position: x = "<<x<<", y = "<<y<<endl;
			std::cout<<"Orientation: qw = "<<qw<<endl;
			double siny_cosp = 2 * (qw * z + qx * qy);
    			double cosy_cosp = 1 - 2 * (qy * qy + z * z);
    			Rotation_Lidar = (atan2(siny_cosp, cosy_cosp))*180/(M_PI)*(-1); //Fois -1 pour avoir un angle positif lorsque je tourne Ã  droite
    			rotation_Lidar = &Rotation_Lidar;
	}
	return;
}

double deplacement_automatique(double (*scan_data)[NUM_VALUES])
{	
	int test = 0;
	int compteur_passage_boucle = 0;
	int compteur_somme = 0;
	int somme_repere_distance_scan[NUM_SCANS][227] = {0};
	std::cout<<test++<<endl;
	
	double calcul_distance_point_scan_x[NUM_SCANS][NUM_VALUES] = {0};
	double calcul_distance_point_scan_y[NUM_SCANS][NUM_VALUES] = {0};
	double angle_point_scan = 0.0;
	double angle_point_scan_radian = 0.0;
	
	double rotation_lidar_radian = 0.0;
	
	if(pscan != NULL)
	{
		for(int i = 0; i < NUM_SCANS; i++)
		{
			for(int j = 0; j < NUM_VALUES; j++)
			{
				if(j!=0 && (pscan[i][j] < pscan[i][j-1] + 0.1))// && j > 170 && j < 510) //Permet d'esquiver les NaN
				{
					
					if(j<=NUM_VALUES/2)
					{
						angle_point_scan = ((NUM_VALUES/2)-j)*0.3519;
					}
					else
					{
						angle_point_scan = -(j-(NUM_VALUES/2))*0.3519;
					}
					if(rotation_Lidar != NULL)
					{
						rotation_lidar_radian = *rotation_Lidar*(M_PI)/180;
						
						angle_point_scan_radian = angle_point_scan*(M_PI)/180; //Calcul Angle Lidar / Point (Radian)
						
						if(&x != NULL && &y != NULL)
						{
							calcul_distance_point_scan_x[i][j] = std::round((pscan[i][j]*cos(angle_point_scan_radian+rotation_lidar_radian)+*x)*100)/100; //Calcul des valeurs métriques X
							calcul_distance_point_scan_y[i][j] = std::round((pscan[i][j]*sin(angle_point_scan_radian+rotation_lidar_radian)+*y)*100)/100; //Calcul des valeurs métriques X
						
						
							
							Position_Points_dans_Espace.X = calcul_distance_point_scan_x[i][j]; //Attribution des valeurs métriques X
							Position_Points_dans_Espace.Y = calcul_distance_point_scan_y[i][j]; //Attribution des valeurs métriques Y
							Position_Points_dans_Espace.Chemin = false; //Chemin == false tant que le robot ne passe pas dessus
							Position_Points_dans_Espace.Cul_de_Sac = false; //Attribution false
							points.push_back(Position_Points_dans_Espace); //Sauvegarde dans le vecteur
						}
					}
				}
			}
		}
	}
	
	for(int i = 0; i < NUM_SCANS; i++)
	{
		
		for(int j = 1; j < NUM_VALUES ; j++)
		{
			
			
			if(j%3 == 0)
			{
				compteur_somme++;
				somme_repere_distance_scan[i][compteur_somme] = 0;
				somme_repere_distance_scan[i][compteur_somme] = scan_data[i][j] + scan_data[i][j-1] + scan_data[i][j-2];
				
			}
		}
		compteur_somme = 0;
		
	}
	
	int somme_valeurs_gauche = 0;
	int somme_valeurs_centre = 0;
	int somme_valeurs_droite = 0;
	
	for(int i = 0; i < NUM_SCANS; i++)
	{
		for(int j = 0; j < 227; j++)
		{
			
			if(j < 75)
			{
				somme_valeurs_droite += somme_repere_distance_scan[i][j];
			}
			else if((j >= 75)&&(j < 151))
			{
				somme_valeurs_centre += somme_repere_distance_scan[i][j];
			}
			else
			{
				somme_valeurs_gauche += somme_repere_distance_scan[i][j];
			}
		}
	}
	
	std::cout<<"somme_valeurs_gauche = " << somme_valeurs_gauche<<endl;
	std::cout<<"somme_valeurs_centre = " << somme_valeurs_centre<<endl;
	std::cout<<"somme_valeurs_droite = " << somme_valeurs_droite<<endl;
	
	//std::cout<<"absolue(somme_valeurs_gauche,somme_valeurs_droite) = " << absolue(somme_valeurs_gauche,somme_valeurs_droite)<<endl;
	//std::cout<<"absolue(somme_valeurs_gauche,somme_valeurs_centre) = " << absolue(somme_valeurs_gauche,somme_valeurs_centre)<<endl;
	//std::cout<<"absolue(somme_valeurs_centre,somme_valeurs_droite) = " << absolue(somme_valeurs_centre,somme_valeurs_droite)<<endl;
	
	if(((std::abs(somme_valeurs_gauche - somme_valeurs_droite) > 3000)&&((std::abs(somme_valeurs_gauche - somme_valeurs_centre) > 3000))&&((std::abs(somme_valeurs_centre - somme_valeurs_droite) > 3000)))||((somme_valeurs_gauche < 20000) || (somme_valeurs_centre < 20000) || (somme_valeurs_droite < 20000)))
	{
		if((somme_valeurs_gauche > somme_valeurs_centre)&&(somme_valeurs_gauche > somme_valeurs_droite))
		{
			if(Verification_Tourner_Droite() == true)
			{
				std::cout<<"Tourner à droite de 90°"<<endl; // à remplacer par tourner de X degrés en faisant le calcul par rapport à lendroit du scan où on a vu le point.
				std::cout<<"Puis avancer pendant 3s"<<endl; // à remplacer par : si le point détecté est à droite et que le robot n'a pas encore marché dessus
				deblocage_droite++;
			}
			else
			{
				std::cout<<"Tourner à gauche"<<endl;
				deblocage_gauche++;
			}
					
			
		}
		else if((somme_valeurs_centre > somme_valeurs_gauche)&&(somme_valeurs_centre > somme_valeurs_droite))
		{
			if(Verification_Tourner_Droite() == true)
			{
				std::cout<<"Tourner à droite de 90°"<<endl;
				std::cout<<"Puis avancer pendant 3s"<<endl;
				deblocage_droite++;
			}
			else
			{
				std::cout<<"Continuer tout droit"<<endl;
				deblocage_droite = 0;
				deblocage_gauche = 0;
			}
			
		}
		else
		{
			std::cout<<"Tourner à droite"<<endl;
			deblocage_droite++;
		}
		std::cout<<"deblocage_droite : "<<deblocage_droite<<endl;
		std::cout<<"deblocage_gauche : "<<deblocage_gauche<<endl;
		if((deblocage_droite != 0 && deblocage_gauche != 0 && std::abs(deblocage_gauche + deblocage_droite > 4)))
		{
			std::cout<<"Tourner à droite de 180°"<<endl; //dans une boucle while
			deblocage_droite = 0;
			deblocage_gauche = 0;
		}
	}
	else
	{
		std::cout<<"Tourner de 90° à droite (pour se décoincer)"<<endl;
	}
			
				
	compteur_somme = 0;
	
			
	return 0;
}



bool Verification_Tourner_Droite()
{
	double Somme_droite = 0;
	int compteur_boucle = 0;
	for(int i = 0; i < NUM_SCANS; i++)
	{
		for(int j = 45; j < 85; j++)
		{
			if(pscan[i][j] > 0.2)
			{
				Somme_droite += pscan[i][j];
				compteur_boucle++;
			}
		}
		std::cout<<"Some_Droite : "<<Somme_droite / compteur_boucle<<endl;
		if(Somme_droite / compteur_boucle > 3.5)
			return true;
	}
	return false;
}

int main(int argc, char **argv)
{  
	
	ros::init(argc, argv, "ROBOCUP2024_Exploration_automatique_intelligente_node");
  
	ros::NodeHandle n;
	start_time = ros::Time::now();
	/* ===Attribution des noeuds et de leur nom respectif pour les flux vidéo===*/
	//ros::Subscriber sub = n.subscribe("/trajectory", 1000, trajectoryCallback);
	ros::Subscriber sub2 = n.subscribe("/scan", 1000, scanCallback);
	ros::Rate loop_rate(300);
		
	
	while (ros::ok())
	{
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
