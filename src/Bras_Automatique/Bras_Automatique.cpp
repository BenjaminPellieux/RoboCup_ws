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
#include "snpi5_V3.h"
#include "Finder.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <unistd.h>
#include <sys/stat.h> 
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <random>
#include "ROBOCUP2024/Traitement_HSV.h"
#include "ROBOCUP2024/Pas.h"  
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#define HEIGHT_IMAGE 480
#define WIDTH_IMAGE 640
#define CENTER_X_IMAGE WIDTH_IMAGE/2
#define CENTER_Y_IMAGE HEIGHT_IMAGE/2
#define DECALAGE_TRANSLATION 10
#define DECALAGE_ROTATION 10

using namespace cv;
using namespace std;

geometry_msgs::Vector3 Co;

/*~~~~~~~Déclaration des prototypes~~~~~~~*/
void imageCb(const sensor_msgs::ImageConstPtr& msg);
cv::Mat Thresholded(cv_bridge::CvImagePtr cv_ptr, int iLowH,int iHighH,int iLowS,int iHighS,int iLowV,int iHighV,int k);
std::vector<std::vector<cv::Point_<int>>> Blopblop(cv::Mat imageMono,cv_bridge::CvImagePtr cv_ptr,int iBlopVoulu,int iR, int iG,int iB);
std::vector<cv::Point_<int>> Approximation_ligne(cv_bridge::CvImagePtr cv_ptr,std::vector<cv::Point_<int>>Contour,int nombre_decoupage, int iR, int iG, int iB);
int IntersectionPoly(cv_bridge::CvImagePtr Image_conducteur,std::vector<cv::Point_<int>> Route,std::vector<cv::Point_<int>> Obstacle,int indice);
/*________________________________________*/
/* Prototype des fonctions de service*/
static bool serv_Traitement_HSV(ROBOCUP2024::Traitement_HSV::Request& req, ROBOCUP2024::Traitement_HSV::Response& res); //ROBOCUP2024_CAT_node
static bool serv_Pas(ROBOCUP2024::Pas::Request& req, ROBOCUP2024::Pas::Response& res);
/*________________________________________*/

struct Position_Objet {
    double x; // Position en x
    double y; // Position en y
    double z; // Position en z
    double rotX; // Rotation autour de l'axe X
    double rotY; // Rotation autour de l'axe Y
    double rotZ; // Rotation autour de l'axe Z
};

class RobotArm {
public:
    void moveToPosition(const Position_Objet& objet) {
        // Implémenter la logique de déplacement du bras
        if (objet.x != CENTER_X_IMAGE) {
            if (objet.x < (CENTER_X_IMAGE - DECALAGE_TRANSLATION)) {
                moveLeft();
            } else if (objet.x > (CENTER_X_IMAGE + DECALAGE_TRANSLATION)) {
                moveRight();
            }
        }
        if (objet.y != CENTER_Y_IMAGE) {
            if (objet.y < (CENTER_Y_IMAGE - DECALAGE_TRANSLATION)) {
                moveDown();
            } else if (objet.y > (CENTER_Y_IMAGE + DECALAGE_TRANSLATION)) {
                moveUp();
            }
        }
        cout << "Moving to position: " << objet.x << ", " << objet.y << ", " << objet.z << endl;
    }

    void rotate(const Position_Objet& objet) {
        // Implémenter la logique de rotation du bras
        if (objet.rotY != 0) {
            if (objet.rotY < -DECALAGE_ROTATION) {
                rotateLeft();
            } else if (objet.rotY > DECALAGE_ROTATION) {
                rotateRight();
            }
        }
        if (objet.rotX != 0) {
            if (objet.rotX < -DECALAGE_ROTATION) {
                rotateUp();
            } else if (objet.rotX > DECALAGE_ROTATION) {
                rotateDown();
            }
        }
        cout << "Rotating to: " << objet.rotX << ", " << objet.rotY << ", " << objet.rotZ << endl;
    }

private:
    void moveLeft() {
        cout << "Moving left" << endl;
        // Code réel pour déplacer le bras à gauche
    }

    void moveRight() {
        cout << "Moving right" << endl;
        // Code réel pour déplacer le bras à droite
    }

    void moveUp() {
        cout << "Moving up" << endl;
        // Code réel pour déplacer le bras vers le haut
    }

    void moveDown() {
        cout << "Moving down" << endl;
        // Code réel pour déplacer le bras vers le bas
    }

    void rotateLeft() {
        cout << "Rotating left" << endl;
        // Code réel pour faire pivoter le bras à gauche
    }

    void rotateRight() {
        cout << "Rotating right" << endl;
        // Code réel pour faire pivoter le bras à droite
    }

    void rotateUp() {
        cout << "Rotating up" << endl;
        // Code réel pour faire pivoter le bras vers le haut
    }

    void rotateDown() {
        cout << "Rotating down" << endl;
        // Code réel pour faire pivoter le bras vers le bas
    }
};

class ImageProcessor {
public:
    ImageProcessor(image_transport::ImageTransport& it) {
        image_pub_ = it.advertise("/image_converter/output_Obstacle", 1);
        image_pub2_ = it.advertise("/image_converter/Traitement_Bleu", 1);
        image_pub3_ = it.advertise("/image_converter/Traitement_Rouge", 1);
        image_pub4_ = it.advertise("/image_converter/Traitement_Ligne", 1);
        image_pub_circles_ = it.advertise("/image_converter/Traitement_Cercles", 1);
        image_sub_ = it.subscribe("/cv_camera/image_raw", 1, &ImageProcessor::imageCb, this);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg) {
        int k = 0;
        float Angle = 0.0;
        float Largeur = 0.0;
        float Hauteur = 0.0;
        float Moyenne0X = 0.0;
        float Moyenne0Y = 0.0;
        vector <vector<Point2i>> ContourBleu;
        vector <vector<Point2i>> ContourRouge;
        std::vector<cv::Point_<int>> Moyenne0;
        std::vector<cv::Point_<int>> Moyenne1;
        std::vector<cv::Point_<int>> Direction(100);
        std::vector<cv::Point_<int>> Poly;
        Scalar color(255,255,0);
        Point_<int> zero;
        zero.x = 0;
        zero.y = 0;
        int i = 0;

        cv::Mat img_Test, img_Obstacle;
        cv_bridge::CvImagePtr cv_ptr, testimage;

        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            testimage = cv_ptr;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        readParam();
        /*Traitement HSV*/
        img_Test = Thresholded(cv_ptr, iLowH, iHighH, iLowS, iHighS, iLowV, iHighV, 1);
        img_Obstacle = Thresholded(cv_ptr, iLowH_Obstacle, iHighH_Obstacle, iLowS_Obstacle, iHighS_Obstacle, iLowV_Obstacle, iHighV_Obstacle, 0);

        /*Traitement Detection des contours*/
        ContourRouge = Blopblop(img_Obstacle, cv_ptr, 1, 255, 0, 0);
        ContourBleu = Blopblop(img_Test, cv_ptr, 2, 0, 0, 255);

        // Publication des images de contours
        image_pub_.publish(cv_ptr->toImageMsg());

        if (ContourBleu.size() > 1) { // Assurez-vous qu'il y a au moins deux contours bleus
        	vector<int> stock = {0, 0};
        	vector<int> compar = {0, 0};
        	for (size_t i = 0; i < ContourBleu.size(); i++) {
            	double area = contourArea(ContourBleu[i]);
            	if (compar[0] < area) {
                	compar[1] = compar[0];
                	stock[1] = stock[0];
                	compar[0] = area;
                	stock[0] = i;
            	} else if (compar[1] < area) {
                	compar[1] = area;
                	stock[1] = i;
            	}
        	}
	
        	Moyenne0 = Approximation_ligne(testimage, ContourBleu[stock[0]], 20, 0, 255, 0);
        	Moyenne1 = Approximation_ligne(testimage, ContourBleu[stock[1]], 20, 0, 255, 0);
	
        	cout << Moyenne0.size() << endl;
        	for (size_t i = 1; i < Moyenne0.size(); i++) {
            	Direction[i].x = static_cast<int>((Moyenne0[i].x + Moyenne1[i].x) / 2);
            	Direction[i].y = static_cast<int>((Moyenne0[i].y + Moyenne1[i].y) / 2);
	
            	if ((Direction[i].y < 420) && (Direction[i].y > 0)) {
                	Moyenne0X += Direction[i].x;
                	Moyenne0Y += Direction[i].y;
                	k++;
            	}
        	}
	
        	if (k > 0) {
            	cout << "MoyenneOX = " << Moyenne0X / k << endl;
            	cout << "MoyenneOY = " << Moyenne0Y / k << endl;
	
            	Largeur = std::abs(320 - Moyenne0X / k);
            	Hauteur = Moyenne0Y / k;
            	Angle = std::atan(Largeur / Hauteur) * 180 / 3.1415 / 4;
	
            	if ((Moyenne0X / k) < 320) {
                	Angle = -Angle;
            	}
            	cout << "Angle = " << Angle << endl;
        	}
	
        	Co.x = Angle;
	
        	Moyenne0X = 0;
        	Moyenne0Y = 0;
	
        	for (size_t i = 1; i < (Moyenne0.size() - 1); i++) {
            	cv::arrowedLine(cv_ptr->image, Direction[i], Direction[i + 1], color, 1, 8, 0, 0.1);
        	}
	
        	std::reverse(Moyenne1.begin(), Moyenne1.end());
        	Poly.reserve(Moyenne0.size() + Moyenne1.size());
        	Poly.insert(Poly.end(), Moyenne0.begin(), Moyenne0.end());
        	Poly.insert(Poly.end(), Moyenne1.begin(), Moyenne1.end());
	
        	cv::polylines(cv_ptr->image, Poly, 1, Scalar(255, 0, 255), 1, 8, 0);
	
        	for (size_t i = 0; i < ContourRouge.size(); i++) {
            	IntersectionPoly(cv_ptr, Poly, ContourRouge[i], i);
        	}
        }
        

        // Détection de cercles dans l'image
        std::vector<cv::Vec3f> circles = detectCircles(cv_ptr->image);
        for (size_t i = 0; i < circles.size(); i++) {
            cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            // Dessine le cercle
            cv::circle(cv_ptr->image, center, radius, Scalar(0, 255, 0), 3, 8, 0);
        }

        // Publication de l'image avec les cercles détectés
        image_pub_circles_.publish(cv_ptr->toImageMsg());
    }

    cv::Mat Thresholded(cv_bridge::CvImagePtr cv_ptr, int iLowH, int iHighH, int iLowS, int iHighS, int iLowV, int iHighV, int k) {
        sensor_msgs::ImagePtr img_msg;
        cv::Mat imgHSV, imggray, thresh;
        cv::Mat imgThresholded;

        cv::cvtColor(cv_ptr->image, imgHSV, cv::COLOR_BGR2HSV); // Convert the captured frame from BGR to HSV  
        inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); // Etalonnage de l'image   

        erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
        dilate(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

        dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

        img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imgThresholded).toImageMsg();

        if (k == 1) {
            image_pub2_.publish(img_msg); // pub bleu
        } else {
            image_pub3_.publish(img_msg); // pub rouge
        }

        return imgThresholded;
    }

    std::vector<std::vector<cv::Point_<int>>> Blopblop(cv::Mat imageMono, cv_bridge::CvImagePtr cv_ptr, int iBlopVoulu, int iR, int iG, int iB) {
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;

        findContours(imageMono, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);

        Scalar color(iB, iG, iR); // en BGR
        drawContours(cv_ptr->image, contours, -1, color, 2);
        // cout<<"contours : "<< contours.size() <<endl;//affichage pour l'utilisateur
        image_pub_.publish(cv_ptr->toImageMsg());
        return contours;
    }

    std::vector<cv::Vec3f> detectCircles(cv::Mat& image) {
        std::vector<cv::Vec3f> circles;
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, gray, cv::Size(9, 9), 2, 2);
        cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, gray.rows / 8, 200, 50, 0, 0);
        return circles;
    }

    std::vector<cv::Point_<int>> Approximation_ligne(cv_bridge::CvImagePtr cv_ptr, std::vector<cv::Point_<int>> Contour, int nombre_decoupage, int iR, int iG, int iB) {
        // Implémenter la logique de l'approximation de ligne ici
        // Ceci est un exemple simplifié
        std::vector<cv::Point_<int>> approximatedLine;
        // Ajoutez votre logique pour l'approximation ici
        return approximatedLine;
    }

    int IntersectionPoly(cv_bridge::CvImagePtr Image_conducteur, std::vector<cv::Point_<int>> Route, std::vector<cv::Point_<int>> Obstacle, int indice) {
    	int Intersection = -1;
    	Point_<int> pt;
    	Scalar color(255, 255, 0);

    	if (!Obstacle.empty()) {
        		for (const auto& point : Obstacle) {
            		pt += point;
        		}
        		pt.x = static_cast<int>(pt.x / Obstacle.size());
        		pt.y = static_cast<int>(pt.y / Obstacle.size());
		
        		if (pointPolygonTest(Route, pt, false) >= 0) {
            		Intersection = indice;
            		putText(Image_conducteur->image, "Badaboum", pt, FONT_HERSHEY_SIMPLEX, 1, color, 1, LINE_8, false);
            		image_pub4_.publish(Image_conducteur->toImageMsg());
        		}
    	}
    	return Intersection;
    }


private:
    image_transport::Publisher image_pub_;
    image_transport::Publisher image_pub2_;
    image_transport::Publisher image_pub3_;
    image_transport::Publisher image_pub4_;
    image_transport::Publisher image_pub_circles_;
    image_transport::Subscriber image_sub_;
};


class Controller {
public:
    Controller(RobotArm& arm, ImageProcessor& processor) : arm_(arm), processor_(processor) {}

    void controlArm() {
        // Récupérer les coordonnées et l'angle d'inclinaison de l'objet
        auto objectCoordinates = detectObject();
        double inclinationAngle = findInclinationAngle();

        // Calculer la commande pour le bras
        Position_Objet objet = calculateArmCommand(objectCoordinates, inclinationAngle);

        // Déplacer et orienter le bras
        arm_.moveToPosition(objet);
        arm_.rotate(objet);
    }

private:
    RobotArm& arm_;
    ImageProcessor& processor_;

    Position_Objet calculateArmCommand(const pair<int, int>& objectCoordinates, double inclinationAngle) {
        Position_Objet objet;
        // Calcul de la commande en fonction des coordonnées de l'objet et de l'angle d'inclinaison
        // Ces valeurs sont des exemples et devraient être calculées en fonction de la configuration réelle du robot et de la caméra
        objet.x = objectCoordinates.first;
        objet.y = objectCoordinates.second;
        objet.z = 1.0; // Distance fixe
        objet.rotX = 0.0;
        objet.rotY = -inclinationAngle;
        objet.rotZ = 0.0;
        return objet;
    }

    pair<int, int> detectObject() {
        // Générer des coordonnées x et y aléatoires
        random_device rd;
        mt19937 gen(rd());
        uniform_int_distribution<> dis_x(0, WIDTH_IMAGE);
        uniform_int_distribution<> dis_y(0, HEIGHT_IMAGE);

        int x = dis_x(gen);
        int y = dis_y(gen);

        cout << "Detecting object at: " << x << ", " << y << endl;
        return {x, y};
    }

    double findInclinationAngle() {
        // Implémenter la recherche de l'angle d'inclinaison de l'objet
        cout << "Finding inclination angle..." << endl;
        return 0.0; // Exemple d'angle
    }
};

class ROSNode {
public:
    ROSNode() : it_(n_), processor_(it_), controller_(arm_, processor_) {
        // Déclaration des services
        service_Traitement_HSV_ = n_.advertiseService("Traitement_HSV", &ROSNode::serv_Traitement_HSV, this);
        service_Pas_ = n_.advertiseService("Pas", &ROSNode::serv_Pas, this);
    }

    void spin() {
        ros::Rate loop_rate(10);
        while (ros::ok()) {
            controller_.controlArm();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    ros::NodeHandle n_;
    image_transport::ImageTransport it_;
    RobotArm arm_;
    ImageProcessor processor_;
    Controller controller_;
    ros::ServiceServer service_Traitement_HSV_;
    ros::ServiceServer service_Pas_;

    bool serv_Traitement_HSV(ROBOCUP2024::Traitement_HSV::Request& req, ROBOCUP2024::Traitement_HSV::Response& res) {
        ROS_INFO("Request : LowH : %ld HighH : %ld LowS : %ld HighS : %ld LowV : %ld HighV : %ld \n", req.LowH, req.HighH, req.LowS, req.HighS, req.LowV, req.HighV);

        iLowH = req.LowH;
        iHighH = req.HighH;
        iHighS = req.HighS;
        iLowS = req.LowS;
        iLowV = req.LowV;
        iHighV = req.HighV;

        return true;
    }

    bool serv_Pas(ROBOCUP2024::Pas::Request& req, ROBOCUP2024::Pas::Response& res) {
        if (req.tolerance < 1) {
            ROS_INFO("Merci de saisir une valeur entière supérieur 0\n");
        } else {
            ROS_INFO("Request : tolerance : %ld \n", req.tolerance);
            Co.z = req.tolerance;
        }

        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ROBOCUP2024_Bras_Automatique_node");
    ROSNode node;
    node.spin();
    return 0;
}
