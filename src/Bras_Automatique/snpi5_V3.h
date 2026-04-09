#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/Joy.h>
#include "snpi5/Image.h" // service pour traiter l'image
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

/*static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_IMAGE_RESULTAT = "Image résultat";
static const std::string OPENCV_IMAGE_ORTHO = "Image redressée";*/

static const string FILE_NAME = "/home/ros/catkin_ws/src/projet_cmt/HSV_Config.txt";

struct myPoint {
    float x;
    float y;
};

/*Mat imgHSV;
Mat imgThresholded;
Mat imgThresholdedTarget_1;
Mat imgThresholdedTarget_2;
Mat im_with_keypoints;
// perspective image.
Mat im_with_keypointsOrtho;

Mat mask;
Mat res;

/* couleur des marqueurs latéraux */

int iLowH = 0;
int iHighH = 255;

int iLowS = 0;
int iHighS = 255;

int iLowV = 0;
int iHighV = 255;

int iLowH_Obstacle = 0;
int iHighH_Obstacle = 3;


int iHighS_Obstacle = 255;
int iLowS_Obstacle = 121; 

int iLowV_Obstacle = 0;
int iHighV_Obstacle = 255;

/* couleur de la cible de type 1 */

/*int iLowHTarget_1 = 0;
int iHighHTarget_1 = 255;

int iLowSTarget_1 = 0;
int iHighSTarget_1 = 255;

int iLowVTarget_1 = 0;
int iHighVTarget_1 = 255;


/* couleur de la cible de type 2 */

/*int iLowHTarget_2 = 0;
int iHighHTarget_2 = 255;

int iLowSTarget_2 = 0;
int iHighSTarget_2 = 255;

int iLowVTarget_2 = 0;
int iHighVTarget_2 = 255;


std::vector<geometry_msgs::Point> list_keypointTargetPosTrans_1;
std::vector<geometry_msgs::Point> list_keypointTargetPosTrans_2;*/


void readParam(void)

	{
		std::ifstream infile;
		string type;
		infile.open(FILE_NAME, std::ofstream::in); // append instead of overwrite
		
		

		infile >> type;
		infile >> iLowH >> iHighH >> iLowS >> iHighS >> iLowV >> iHighV;
  		cout <<type <<" "<<iLowH<<" "<< iHighH << " "<<iLowS<<" "<< iHighS << " "<<iLowV<<" "<< iHighV <<std::endl; 
		
		infile >> type;
		infile >> iLowH_Obstacle >> iHighH_Obstacle >> iLowS_Obstacle >> iHighS_Obstacle >> iLowV_Obstacle >> iHighV_Obstacle;

  		cout <<type <<" "<<iLowH_Obstacle<<" "<< iHighH_Obstacle << " "<<iLowS_Obstacle<<" "<< iHighS_Obstacle << " "<<iLowV_Obstacle<<" "<< iHighV_Obstacle <<std::endl; 

		/*infile >> type;
		infile >> iLowHTarget_2 >> iHighHTarget_2 >> iLowSTarget_2 >> iHighSTarget_2 >> iLowVTarget_2 >> iHighVTarget_2;

  		cout <<type <<" "<<iLowHTarget_2<<" "<< iHighHTarget_2 << " "<<iLowSTarget_2<<" "<< iHighSTarget_2 << " "<<iLowVTarget_2<<" "<< iHighVTarget_2 <<std::endl; 
*/

  		infile.close();
//  		exit(0);

}
