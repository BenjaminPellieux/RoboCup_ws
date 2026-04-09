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


#define NUM_SCANS 1
#define NUM_VALUES 682

struct Point_Espace {
    bool Mur;
    int Recompense;
    double X;
    double Y;
};



/* Déclaration des prototypes */
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
void trajectoryCallback(const nav_msgs::Path::ConstPtr& msg); 
double Reward(double (*scan_data)[NUM_VALUES], double x, double y, double x_fichier, double y_fichier, double rotation_Lidar, double x_precedent);
bool comparePoints(const Point_Espace& a, const Point_Espace& b);
bool Existence_Point(const std::vector<Point_Espace>& points, double x, double y, double tolerance);
bool operator==(const Point_Espace& lhs, const Point_Espace& rhs);
int calculerRecompense(double x, double y, double x_fichier, double y_fichier);
geometry_msgs::Vector3 deplacement_vers_point(double x, double y, double x_fichier, double y_fichier, const Point_Espace& a, double rotation_Lidar);
/*________________________________________*/

/*Définition du publisher !! :*/


//consigne_send_= nh_.advertise<geometry_msgs::Point >("direction", 1);

using namespace cv;
using namespace std;

int compteur_effacement = 0;
ros::Time start_time;
std::ofstream outfile;
std::ofstream outfile_scan;
int test = 0;
geometry_msgs::Vector3 Com;
geometry_msgs::Vector3* p_Commande = &Com;
int j = 0;
int p = 0;
const char* dir = "/home/ros/catkin_ws/src/ROBOCUP2024/src/trajectoire"; //dÃ©clare 
std::ifstream file(std::string(dir) + "/trajectory_data.txt"); // Remplacez "monfichier.txt" par le nom de votre fichier
int i = 0;
int changement_coord_fichier = 0;
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
std::vector<std::string> col0; //Timer
std::vector<std::string> col1; //Bouton 1 (appuyÃ© == 1)
std::vector<std::string> col2; //Bouton 2 (appuyÃ© == 1)
std::vector<std::string> col3; //Bouton 3 (appuyÃ© == 1)
std::vector<std::string> col4; //Bouton 4 (appuyÃ© == 1)
std::string val0, val1, val2, val3, val4;
double (*pscan)[NUM_VALUES] = NULL;
bool* position_atteignable = new bool;
double moyennage_point_dans_scan = 0;
double somme_point_dans_scan = 0;
double compteur_point_scan = 0.0;
std::vector<Point_Espace> points;
double rotation_Lidar_precedent = -180;
double x_precedent = -9999999;
int wait = 0;
bool passage_scan = false;
int progression_avance = 0;
int progression_droite = 0;
int progression_gauche = 0;

sensor_msgs::Imu Commande_Retour;
//sensor_msgs::Joy Retour_Test;



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
    		pscan = scan_data_recuperation; //pointeur du scan	
	}
	
	//std::cout<<Commande.x<<" Dans le scan"<<Commande.y<<endl;
	//*p_Commande=Commande;
	//cout << "passage scan" <<endl;
	passage_scan = true;
	return ;
}

void tfCallback(const tf2_msgs::TFMessageConstPtr& msg)
{
    double x_fichier = 0;
    double y_fichier = 0;
    std::string line;
    double Rotation_Lidar = 0;
    
    if(p == 0)
	{
		while (getline(file, line)) 
		{
        			std::istringstream iss(line);
        			
        			iss >> val1 >> val2 >> val3 >> val4; //RÃ©cupÃ©ration de 4 valeurs dans le fichier (sÃ©parÃ©es par un espace)
        			col1.push_back(val1); //attribution de la premiÃ¨re valeur. 
        			col2.push_back(val2);
        			col3.push_back(val3);
        			col4.push_back(val4);
    		}
		
    		file.close();
		
    		// Afficher les valeurs pour vÃ©rification
		p++; //permet de ne pas revenir dans cette boucle une fois la lecture faite.
	}
    // Les messages TFMessage contiennent un vecteur de TransformStamped
    
    
    /*for (int last = 0; last < msg->transforms.size(); ++last)
    {
   	 while (last < msg->transforms.size())
    	{
       	 	last++; // permet de sauter jusqu'au dernier message du paquet de données.
    	}
        const geometry_msgs::TransformStamped& transformStamped = msg->transforms[last];
	
        // Position
        double x = transformStamped.transform.translation.x;
        double y = transformStamped.transform.translation.y*(-1);
        double z = transformStamped.transform.translation.z;

        // Orientation en quaternion
        tf2::Quaternion q;
        tf2::convert(transformStamped.transform.rotation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        double qw = q.w();
        double qx = roll;
        double qy = pitch;
        double qz = yaw;
cout << "passage : "<<last<< endl;*/
    
    int sdf = 0;
    
    //for (const auto& transformStamped : msg->transforms)
    //{
    const geometry_msgs::TransformStamped& transformStamped = msg->transforms[0];
        // Position
        sdf++;
        //cout << "sdf : "<<sdf<<endl;
        double x = transformStamped.transform.translation.x;
        double y = transformStamped.transform.translation.y*(-1);
        double z = transformStamped.transform.translation.z;

        // Orientation en quaternion
        tf2::Quaternion q;
        tf2::convert(transformStamped.transform.rotation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        double qw = q.w();
        double qx = roll;
        double qy = pitch;
        double qz = yaw;

	if(x!=0&&y!=0)
	{
	//ros::Duration(0.7).sleep();
        // Affichage des informations
       // ROS_INFO("Position -> x: %f, y: %f, z: %f", x, y, z);
       // ROS_INFO("Rotation -> roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);
	x_fichier = std::atof(col2[changement_coord_fichier].c_str());
	y_fichier = std::atof(col3[changement_coord_fichier].c_str());
	double siny_cosp = 2 * (qw * qz + qx * qy);
    	double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    	Rotation_Lidar = (atan2(siny_cosp, cosy_cosp))*180/(M_PI)*(-1); //Fois -1 pour avoir un angle positif lorsque je tourne Ã  droite
    	//std::cout<<"Rotation Lidar = " << Rotation_Lidar<<endl;
    	//std::cout<<rotation_Lidar_precedent<<endl;
			if((pscan != NULL))//&&(std::abs(int(Rotation_Lidar) - int(rotation_Lidar_precedent)) < 2)) 
			{
				Reward(pscan, x, y, x_fichier, y_fichier, Rotation_Lidar, x_precedent);
			}
			rotation_Lidar_precedent = Rotation_Lidar;
			
			std::sort(points.begin(), points.end(), [](const Point_Espace& a, const Point_Espace& b) 
			{
    		return a.Recompense > b.Recompense; // Utilisation de > pour l'ordre dÃ©croissant.
			});
			
			int compteur12345 = 0;
			if((pscan!= NULL))//&&(std::abs(int(Rotation_Lidar) - int(rotation_Lidar_precedent)) < 2))//&&(x!=x_precedent))
			{
					/*if (points.size() > 10) { //Permet de constamment jeter la plus forte valeur (en récompense), afin de se débarasser des potentielles erreurs (qui elles ne reviennent pas).
    					points.erase(points.begin(), points.begin() + 1);
					} else {
    					points.clear(); // Si moins de 10 Ã©lÃ©ments, efface tout
					}*/
					for (const auto& Point_Espace : points) 
					{
						if(Point_Espace.Recompense >= 0)
						{
							std::cout << "Point: X=" << Point_Espace.X << ", Y=" << Point_Espace.Y << ", Mur = " << Point_Espace.Mur << ", RÃ©compense = " << Point_Espace.Recompense << std::endl;
							
							*p_Commande=deplacement_vers_point(x, y, x_fichier, y_fichier, Point_Espace, Rotation_Lidar);
							//ros::Duration(2.0).sleep();
							
							}
							compteur12345++;
						
							break;
						
					}
						
			}
			/*else
				{
					Com.x = 0;
					Com.y = 0;
				}*/
				
				x_precedent = x;
		}		
	//}
	passage_scan = false;
	return;
}

bool Existence_Point(const std::vector<Point_Espace>& points, double x, double y, double tolerance) {
    for (const auto& point : points) {
        // VÃ©rifie si le point donnÃ© est Ã  l'intÃ©rieur de la zone de tolÃ©rance pour X et Y.
        if (std::abs(point.X - x) <= tolerance && std::abs(point.Y - y) <= tolerance) {
            return true; // Le point existe dÃ©jÃ  dans le vecteur.
        }
    }
    return false; // Aucun point correspondant n'a Ã©tÃ© trouvÃ© dans le vecteur.
}


bool comparePoints(const Point_Espace& a, const Point_Espace& b) {
    if (a.X < b.X) return true;
    if (a.X > b.X) return false;
    return a.Y < b.Y;
}

bool operator==(const Point_Espace& lhs, const Point_Espace& rhs) {
    // DÃ©finissez ici la logique de comparaison des points
    return (lhs.X == rhs.X) && (lhs.Y == rhs.Y); // Exemple de comparaison basÃ©e sur les coordonnÃ©es X et Y
}

int calculerRecompense(double x, double y, double x_fichier, double y_fichier) {
	
	double distance_x = 0;
   	double distance_y = 0;
   	double pourcentage_x = 0;
   	double pourcentage_y = 0;
   	double distance = std::sqrt((x - x_fichier) * (x - x_fichier) + (y - y_fichier) * (y - y_fichier));
    
    	// Calcul de la rÃ©compense proportionnelle Ã  l'inverse de la distance
    	double proportion = 1.0 / (distance + 1);  // Ajout de 1 pour Ã©viter une division par zÃ©ro
	
    	// Conversion de la rÃ©compense en entier (vous pouvez ajuster si nÃ©cessaire)
    	int recompense = static_cast<int>(proportion * 1000);
	
    	return recompense*1000000/((std::abs(x-x_fichier)+std::abs(y-y_fichier))*1000);
}

double Reward(double (*scan_data)[NUM_VALUES], double x, double y, double x_fichier, double y_fichier, double rotation_Lidar, double x_precedent)
{
	geometry_msgs::Vector3 Commande;
	//std::cout<<"x = "<<x<<endl;
	//std::cout<<"y = "<<y<<endl;
	double calcul_distance_point_scan_x[NUM_SCANS][NUM_VALUES] = {0};
	double calcul_distance_point_scan_y[NUM_SCANS][NUM_VALUES] = {0};
	double angle_point_scan = 0.0;
	double angle_point_scan_radian = 0.0;
	Point_Espace Position_Points_dans_Espace; //CrÃ©ation du point pour rÃ©pertorier le point vu
	double rotation_lidar_radian = 0.0;
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
				rotation_lidar_radian = rotation_Lidar*(M_PI)/180;
				angle_point_scan_radian = angle_point_scan*(M_PI)/180; //Calcul Angle Lidar / Point (Radian)
				calcul_distance_point_scan_x[i][j] = std::round((pscan[i][j]*cos(angle_point_scan_radian+rotation_lidar_radian)+x)*10)/10; //Calcul des valeurs mÃ©triques X
				calcul_distance_point_scan_y[i][j] = std::round((pscan[i][j]*sin(angle_point_scan_radian+rotation_lidar_radian)+y)*10)/10; //Calcul des valeurs mÃ©triques X
				
					
				Position_Points_dans_Espace.X = calcul_distance_point_scan_x[i][j]; //Attribution des valeurs mÃ©triques X
				Position_Points_dans_Espace.Y = calcul_distance_point_scan_y[i][j]; //Attribution des valeurs mÃ©triques Y
				Position_Points_dans_Espace.Mur = true; //Mur car dÃ©tection d'obstacle
				Position_Points_dans_Espace.Recompense = -99999; //Attribution rÃ©compense aberrante car Mur
				points.push_back(Position_Points_dans_Espace); //Sauvegarde dans le vecteur
			}
		}
	}
	
	
	for (const auto& Point_Espace : points) 
	{
		double OriginalX = Position_Points_dans_Espace.X;
		double OriginalY = Position_Points_dans_Espace.Y;
		if(Position_Points_dans_Espace.Mur == true)
		{
			if(Position_Points_dans_Espace.X > 0)
			{
				while(Position_Points_dans_Espace.X > OriginalX - 0.5)
				{	
					Position_Points_dans_Espace.X = Position_Points_dans_Espace.X - 0.01;
					//std::cout<<Position_Points_dans_Espace.X<<endl;
					Position_Points_dans_Espace.Mur = false;
					Position_Points_dans_Espace.Recompense = -10000;
					points.push_back(Position_Points_dans_Espace);
				}
			}
			else
			{
				while(Position_Points_dans_Espace.X > OriginalX + 0.5)
				{	
					Position_Points_dans_Espace.X = Position_Points_dans_Espace.X + 0.01;
					//std::cout<<Position_Points_dans_Espace.X<<endl;
					Position_Points_dans_Espace.Mur = false;
					Position_Points_dans_Espace.Recompense = -10000;
					points.push_back(Position_Points_dans_Espace);
				}
			}
			if(Position_Points_dans_Espace.Y > 0)
			{
				while(Position_Points_dans_Espace.Y > OriginalY - 0.5)
				{	
					Position_Points_dans_Espace.Y = Position_Points_dans_Espace.Y - 0.01;
					Position_Points_dans_Espace.Mur = false;
					Position_Points_dans_Espace.Recompense = -10000;
					points.push_back(Position_Points_dans_Espace);
				}
			}
			else
			{
				while(Position_Points_dans_Espace.Y > OriginalY + 0.5)
				{	
					Position_Points_dans_Espace.Y = Position_Points_dans_Espace.Y + 0.01;
					Position_Points_dans_Espace.Mur = false;
					Position_Points_dans_Espace.Recompense = -10000;
					points.push_back(Position_Points_dans_Espace);
				}
			}
		}
		
		//std::cout << "Point: X=" << Point_Espace.X << ", Y=" << Point_Espace.Y << ", Mur = " << Point_Espace.Mur << ", RÃ©compense = " << Point_Espace.Recompense << std::endl;
	}
	
	for(int i = 0; i < NUM_SCANS; i++)
	{
		for(int j = 0; j < NUM_VALUES; j++)
		{
			if(j != 0 && pscan[i][j] != 0 && (pscan[i][j] < pscan[i][j-1] + 0.1))// && j > 170 && j < 510) //Permet d'esquiver les NaN
			{	
				double distance_changeante = 0;
				double tolerance = 0;
				if(j<=NUM_VALUES/2)
				{
					angle_point_scan = ((NUM_VALUES/2)-j)*0.3519;
				}
				else
				{
					angle_point_scan = -(j-(NUM_VALUES/2))*0.3519;
				}
				rotation_lidar_radian = rotation_Lidar*(M_PI)/180;
				angle_point_scan_radian = angle_point_scan*(M_PI)/180; //Calcul Angle Lidar / Point (Radian)
				
				while(distance_changeante < pscan[i][j] - 0.5)// && distance_changeante <= 1.4)
				{
					calcul_distance_point_scan_x[i][j] = std::round((distance_changeante*cos(angle_point_scan_radian+rotation_lidar_radian)+x)*10)/10; //Calcul des valeurs mÃ©triques X
					
					calcul_distance_point_scan_y[i][j] = std::round((distance_changeante*sin(angle_point_scan_radian+rotation_lidar_radian)+y)*10)/10; //Calcul des valeurs mÃ©triques X
					/*if(pscan[i][j]>1)
					{
						std::cout<<"calcul_distance_point_scan_x[i][j] = "<<calcul_distance_point_scan_x[i][j]<<endl;
						std::cout<<"calcul_distance_point_scan_y[i][j] = "<<calcul_distance_point_scan_y[i][j]<<endl;
						std::cout<<"pscan = "<<pscan[i][j]<<endl;
					}*/
					distance_changeante = distance_changeante + 0.1;
					//std::cout<<distance_changeante<<endl;
					
					//for (auto& Point_Espace : points) 
	//{
					//if(calcul_distance_point_scan_x[i][j] < 0)	
						//std::cout<<"calcul_distance_point_scan_x[i][j] : "<<calcul_distance_point_scan_x[i][j]<<endl;
					Position_Points_dans_Espace.X = calcul_distance_point_scan_x[i][j];
					Position_Points_dans_Espace.Y = calcul_distance_point_scan_y[i][j];
					//std::cout<<"Position_Points_dans_Espace.X : "<<Position_Points_dans_Espace.X<<endl;
					if(Existence_Point(points, Position_Points_dans_Espace.X, Position_Points_dans_Espace.Y, tolerance) == false)
					{
					//std::cout<<Position_Points_dans_Espace.X<<endl;
						Position_Points_dans_Espace.Mur = false;
						//Position_Points_dans_Espace.Recompense = 0;
						
						Position_Points_dans_Espace.Recompense = calculerRecompense(Position_Points_dans_Espace.X, Position_Points_dans_Espace.Y, x_fichier, y_fichier);
						points.push_back(Position_Points_dans_Espace);
						
					}
					//std::cout << "Point: X=" << Point_Espace.X << ", Y=" << Point_Espace.Y << ", Mur = " << Point_Espace.Mur << ", RÃ©compense = " << Point_Espace.Recompense << std::endl;
				}
			}
		}
	}
	
	std::sort(points.begin(), points.end(), comparePoints); //Permet de les trier dans l'ordre X, puis dans l'ordre Y si X = X.
	points.erase(std::unique(points.begin(), points.end()), points.end());
	
	std::sort(points.begin(), points.end(), [](const Point_Espace& a, const Point_Espace& b) 
	{
    return a.Recompense > b.Recompense; // Utilisation de > pour l'ordre dÃ©croissant.
	});
	
	int compteur12345 = 0;
	
	for (auto& Point_Espace : points) 
	{
		if((Point_Espace.X < x + 1)&&(Point_Espace.Y < y + 1)&&(Point_Espace.X > x - 1)&&(Point_Espace.Y > y - 1))
		{
			//Point_Espace.Recompense -= 50000; 
			Point_Espace.Recompense /= 100;
		}
		compteur12345++;
	}
	//std::cout<<"compteur12345 : "<<compteur12345<<endl;
	std::sort(points.begin(), points.end(), [](const Point_Espace& a, const Point_Espace& b) 
	{
    return a.Recompense > b.Recompense; // Utilisation de > pour l'ordre dÃ©croissant.
	});
	
	compteur12345 = 0;
	
	/*if(std::round(x*10)/10 == std::round(x_precedent*10)/10)
	{
		points.erase(points.begin(), points.begin() + 10);
		
		if (++compteur_effacement > 3) 
		{
    		for (auto& Point_Espace : points) 
    		{
        		if (Point_Espace.Recompense >= 0)
        		{
            		Point_Espace.Recompense = -2;
            		break; // Sort de la boucle aprÃ¨s avoir trouvÃ© et mis Ã  jour le premier Ã©lÃ©ment correspondant
        		}
    		}
    		compteur_effacement = 0; // RÃ©initialisation du compteur
		}
	}*/
	
	std::sort(points.begin(), points.end(), [](const Point_Espace& a, const Point_Espace& b) 
	{
    return a.Recompense > b.Recompense; // Utilisation de > pour l'ordre dÃ©croissant.
	});
	
	
	
	//std::cout<<"x : "<<x<<endl;
	//std::cout<<"y : "<<y<<endl;
	std::cout<<"x_fichier_trajectoire : "<<x_fichier<<endl;
	std::cout<<"y_fichier_trajectoire : "<<y_fichier<<endl;
	//std::cout<<"rotation Lidar : "<<rotation_Lidar<<endl; 
	//cout<<"changement_coord_fichier = "<<changement_coord_fichier<<endl;

	return 0.0;
}

geometry_msgs::Vector3 deplacement_vers_point(double x, double y, double x_fichier, double y_fichier, const Point_Espace& a, double rotation_Lidar)
{
	geometry_msgs::Vector3 Commande;
	
	int compteur_droite = 0;
	int compteur_tout_droit = 0;
	int compteur_gauche = 0;
	
	double angleToTarget = std::atan2(a.Y - y, a.X - x) * 180 / M_PI;

    // Correction de l'angle en fonction de la rotation actuelle du Lidar
    double angleRelative = angleToTarget - rotation_Lidar;

    // Normalisation de l'angle entre -180 et 180 degrÃ©s
    angleRelative = fmod(angleRelative + 360, 360);
    if (angleRelative > 180) angleRelative -= 360;

    // Analyse des donnÃ©es Lidar pour vÃ©rifier la prÃ©sence d'obstacles dans la direction de la cible
    
    double somme_verif = 0;
    double somme_verif_gauche = 0;
    double somme_verif_droite = 0;
    
    
 	
 for (int i = 0; i < NUM_SCANS; i++) {
 		for(int j = 100; j < 200; j++)
 		{
 			if(pscan[i][j]!=0)
 			{
 				somme_verif_droite+=pscan[i][j];
 				compteur_droite++;
 			}
 		}
        for (int j = 281; j < 381; j++) {
        	if(pscan[i][j]!=0)
 			{
        		somme_verif+=pscan[i][j];
        		compteur_tout_droit++;
        	}
        	else
        	{
        		somme_verif+=1;
        		compteur_tout_droit++;
        	}
        }
        for(int j = 480; j < 583; j++)
 		{
 			if(pscan[i][j]!=0)
 			{
 				somme_verif_gauche+=pscan[i][j];
 				compteur_gauche++;
 			}
 		}
        somme_verif/=compteur_tout_droit;
        somme_verif_droite/=compteur_droite;
        somme_verif_gauche/=compteur_gauche;
    }
       
       //std::cout<<"somme_verif : "<<somme_verif<<endl;
       //std::cout<<"somme_verif_droite : "<<somme_verif_droite<<endl;
       //std::cout<<"somme_verif_gauche : "<<somme_verif_gauche<<endl;
       
    // DÃ©cision de dÃ©placement
    if (angleRelative < -5 && somme_verif_gauche > 0.7) {
    	progression_gauche++;
    	if(progression_gauche == 5)
    	{
        		std::cout << "Tourner Ã  gauche" << std::endl;  
        		Commande.x=-1;
		Commande.y=1;
		progression_avance = 0;
		progression_droite = 0;
		progression_gauche = 0;
		//std::cout<<"x_fichier_trajectoire : "<<x_fichier<<endl;
	//std::cout<<"y_fichier_trajectoire : "<<y_fichier<<endl;
	}
	
    } else if (somme_verif_droite > 0.7 && angleRelative > 5) {
    	progression_droite++;
    	if(progression_droite == 5)
    	{
        		std::cout << "Tourner Ã  droite" << std::endl;
        		Commande.x=1;
		Commande.y=-1;
		progression_avance = 0;
		progression_droite = 0;
		progression_gauche = 0;
		//std::cout<<"x_fichier_trajectoire : "<<x_fichier<<endl;
	//std::cout<<"y_fichier_trajectoire : "<<y_fichier<<endl;
	}
    } else if (somme_verif>=0.7) {
   	 progression_avance++;
    	if(progression_avance == 5)
    	{
        		std::cout << "Avancer" << std::endl;   
        		Commande.x=1;
		Commande.y=1;
		progression_avance = 0;
		progression_droite = 0;
		progression_gauche = 0;
		//std::cout<<"x_fichier_trajectoire : "<<x_fichier<<endl;
	//std::cout<<"y_fichier_trajectoire : "<<y_fichier<<endl;
	}
    }
    else
    {
    	std::cout << "Reculer" <<endl;
    	Commande.x=-1;
	Commande.y=-1;
	//std::cout<<"x_fichier_trajectoire : "<<x_fichier<<endl;
	//std::cout<<"y_fichier_trajectoire : "<<y_fichier<<endl;
}
   
    if((pscan[0][341] > 0.05 && pscan[0][341] < 0.3) || (pscan[0][208] > 0.05 && pscan[0][208] < 0.3) || (pscan[0][464] > 0.05 && pscan[0][464] < 0.3))
    {
    	std::cout<<"Endroit inferieur a 30cm"<<endl;
    	std::cout<<"Devant : "<<pscan[0][341]<<endl;
    	std::cout<<"Gauche : "<<pscan[0][208]<<endl;
    	std::cout<<"Droite : "<<pscan[0][464]<<endl;
    	//std::cout<<"x_fichier_trajectoire : "<<x_fichier<<endl;
	//std::cout<<"y_fichier_trajectoire : "<<y_fichier<<endl;
    	if((pscan[0][341] != 0 && pscan[0][341] < 0.3) && (Commande.x == -1 || Commande.y == -1))
    	{
    	}
    	else if(pscan[0][208] != 0 && pscan[0][208] < 0.3 && Commande.x == - 1)
    	{
    	}
    	else if(pscan[0][464] != 0 && pscan[0][464] < 0.3 && Commande.y == - 1)
    	{
    	}
    	else
    	{
    		Commande.x = -1;
    		Commande.y = -1;
    	}
    }
   // std::cout<<"x = "<<x<<endl;
    //std::cout<<"y = "<<y<<endl;
    //std::cout<<"y_fichier = "<<y_fichier<<endl;
    //std::cout<<"std::abs(x-a.X)+std::abs(y-a.Y) = "<<std::abs(x-x_fichier)+std::abs(y-y_fichier)<<endl;
    if(std::abs(x-x_fichier)+std::abs(y-y_fichier) < 1)
    {
    	std::cout<<"arrive au point"<<endl; 
    	points.clear();
    	Commande.x = 0;
    	Commande.y = 0;
    	changement_coord_fichier++;
    }
	return Commande;
}

int main(int argc, char **argv)
{  
	
	
	ros::init(argc, argv, "ROBOCUP2024_Exploration_Publisher_node");
  
	
	ros::Publisher consigne_send;
	
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<geometry_msgs::Vector3>("chatter", 1000);
	start_time = ros::Time::now();
	/* ===Attribution des noeuds et de leur nom respectif pour les flux vidéo===*/
	//ros::Subscriber sub = n.subscribe("/trajectory", 1000, trajectoryCallback);
	//std::cout<<"Avant scanCallback"<<endl;
	ros::Subscriber sub2 = n.subscribe("/scan", 1000, scanCallback);
	//ros::Subscriber sub = n.subscribe("/trajectory", 1000, trajectoryCallback);
	ros::Subscriber sub3 = n.subscribe("/tf", 1000, tfCallback);
	//cout <<"valeur scan = " <<passage_scan<<endl;
	
	//std::cout<<"Après scanCallback"<<endl;
	ros::Rate loop_rate(300);

	
	while (ros::ok())
	{
		//std::cout<<"Dans le ros::ok"<<endl;
		//std::cout<<Com.x<<" "<<Com.y<<endl;
		
		ros::spinOnce();
		loop_rate.sleep();	
	    	chatter_pub.publish(Com);
	    	//ros::Duration(0.5).sleep();
	    	/*if(Com.x == 1 && Com.y == 1)
	    	{
	    		//ros::Duration(2).sleep();
	    		Com.x = 0;
	    		Com.y = 0;
	    		chatter_pub.publish(Com);
	    		//ros::Duration(0.5).sleep();
	    	}
	    	if(Com.x == -1 || Com.y == -1)
	    	{
	    		//ros::Duration(0.5).sleep();
	    		Com.x = 0;
	    		Com.y = 0;
	    		chatter_pub.publish(Com);
	    		ros::Duration(0.5).sleep();
	    	}*/
	    
	    //chatter_pub.publish(Com);
	    
	}
	return 0;
}
