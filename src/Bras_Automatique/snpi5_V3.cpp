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
#include "snpi5_V3.h"
#include "fonctions.h"
#include <vector>



using namespace cv;
using namespace std;

/* Classe principale */

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

  public:
 
     cv::Point2f objectivePoints[4]; 
     ros::ServiceServer service;
 
    ImageConverter(): it_(nh_)
    {
      // Subscrive to input video feed and publish output video feed
      image_sub_ = it_.subscribe("/cv_camera/image_raw", 1, &ImageConverter::imageCb, this);
      image_pub_ = it_.advertise("/image_converter/output_video", 1);
      
      service = nh_.advertiseService("image", serv_image);
    
    

      //namedWindow(OPENCV_WINDOW);
      readParam();
        
      
    }
    ~ImageConverter()
    {
      //destroyWindow(OPENCV_WINDOW);
    }


	static bool serv_image(snpi5::Image::Request &req,
 		 snpi5::Image::Response &res)
	{
	  // creating the vector
  

  		res.nTarget_1=list_keypointTargetPosTrans_1.size();
   		res.nTarget_2=list_keypointTargetPosTrans_2.size(); 		

  		res.pointTarget_1.clear();
  		
  		res.pointTarget_1 = list_keypointTargetPosTrans_1;

  		res.pointTarget_2.clear();
  		
  		res.pointTarget_2 = list_keypointTargetPosTrans_2;
  		return true;
	}

    /* Fonction appelée à chaque nouvelle image */

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
      /* conversion topic ROS en format image OpenCV */
      cv_bridge::CvImagePtr cv_ptr;

      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception & e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      // Draw an example circle on the video stream
      //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      //circle(cv_ptr->image, Point(50, 50), 10, CV_RGB(255,0,0));

      /* traitement à proprement parler */

      /* 1ere etape conversion BRG en HSV (teinte / Saturation / Luminosité ) plus simple pour détection couleur */

      cvtColor(cv_ptr->image, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

      /* détection de la couleur des marqueurs résultat image seuillée imgThresholded */
      inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
      /* détection de la couleur des cibles résultat image seuillée imgThresholdedTarget */

      inRange(imgHSV, Scalar(iLowHTarget_1, iLowSTarget_1, iLowVTarget_1), Scalar(iHighHTarget_1, iHighSTarget_1, iHighVTarget_1), imgThresholdedTarget_1); //Threshold the image

      inRange(imgHSV, Scalar(iLowHTarget_2, iLowSTarget_2, iLowVTarget_2), Scalar(iHighHTarget_2, iHighSTarget_2, iHighVTarget_2), imgThresholdedTarget_2); //Threshold the image


      // Bitwise-AND of mask and purple only image - only used for display
      // il faut remettre à 0 la matrice de résultat à chaque fois.
      // on met 255 comme cela le masque est bon
      // on pourra utiliser or et la couleur typique.
      res = 0;
      //bitwise_and(cv_ptr->image, 255, res,imgThresholded); // couleur naturelle
      //bitwise_or(cv_ptr->image, CV_RGB(255,0,0), res,imgThresholded); // couleur prédéfinie

      /* traitement des marqueurs */
      // morphological opening (remove small objects from the foreground)
      erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
      dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

      // morphological closing (fill small holes in the foreground)
      dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
      erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
      
      
      /* traitement de la cible _1*/
      // morphological opening (remove small objects from the foreground)
      erode(imgThresholdedTarget_1, imgThresholdedTarget_1, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
      dilate( imgThresholdedTarget_1, imgThresholdedTarget_1, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

      // morphological closing (fill small holes in the foreground)
      dilate( imgThresholdedTarget_1, imgThresholdedTarget_1, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
      erode(imgThresholdedTarget_1, imgThresholdedTarget_1, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

/* traitement de la cible 2*/
      // morphological opening (remove small objects from the foreground)
      erode(imgThresholdedTarget_2, imgThresholdedTarget_2, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
      dilate( imgThresholdedTarget_2, imgThresholdedTarget_2, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

      // morphological closing (fill small holes in the foreground)
      dilate( imgThresholdedTarget_2, imgThresholdedTarget_2, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
      erode(imgThresholdedTarget_2, imgThresholdedTarget_2, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

      // Paramètre de la détection de Blob

      
      // réglage des paramètres du détecteur des 4 mires

      SimpleBlobDetector::Params params;
      mireParams(&params);

      // réglage des paramètres du détecteur des cibles (commun aux 2)

      SimpleBlobDetector::Params paramsTarget;
     targetParams(&paramsTarget);
      
      
      /* Définition des taches identifiées */
      vector<KeyPoint> keypoints;
      vector<KeyPoint> keypointsTarget_1;
      vector<KeyPoint> keypointsTarget_2;

      // Set up detector with params
      // Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
      // detector->detect( im, keypoints);

      // création des détecteurs de composantes connexes ou blob à partir des paramètres

      Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
      Ptr<SimpleBlobDetector> detectorTarget = SimpleBlobDetector::create(paramsTarget);
      // application des détecteurs sur les images seuillées et récupération des points
      detector->detect( imgThresholded, keypoints );
      detectorTarget->detect( imgThresholdedTarget_1, keypointsTarget_1 );
	  detectorTarget->detect( imgThresholdedTarget_2, keypointsTarget_2 );


      //the total no of blobs detected are:
      int nbMire = keypoints.size();
      //the total no of blobs detected are:
      int nbTarget_1 = keypointsTarget_1.size();
      int nbTarget_2 = keypointsTarget_2.size();

      cout << "Nombre total de Mires detectés : " << nbMire << " Objects type 1 " << nbTarget_1 << " Objects type 2 " << nbTarget_2 << endl;

      //the total no of blobs detected are:
      bool erreurNombreBlob;
      //Affichage image brut

      image_pub_.publish(cv_ptr->toImageMsg());
      imshow(OPENCV_WINDOW, cv_ptr->image);
       waitKey(3);

      if (nbMire == 4)
      {
        bool erreurNombreBlob = false;
        
        // trace les keypoints marqueurs en vert sur l'image  im_with_keypoints à partir de l'image de base
        drawKeypoints(cv_ptr->image, keypoints, im_with_keypoints, Scalar(0, 255, 0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
        // trace les keypoints cibles en rouge sur l'image  im_with_keypoints à partir de l'image im_with_keypoints
        drawKeypoints(im_with_keypoints, keypointsTarget_1, im_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
        drawKeypoints(im_with_keypoints, keypointsTarget_2, im_with_keypoints, Scalar(255, 0, 0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

        char num[4][2] = {"1", "2", "3", "4"};
        
        cv::Point2f keypointPos[4];
        
        orderKeypoints(keypointPos,keypoints);
        
        
		
        
        for (int i = 0; i < 4; i++)
        {

          cv::putText(im_with_keypoints, //target image
                      num[i], //text
                      keypointPos[i], //top-left position
                      cv::FONT_HERSHEY_DUPLEX,
                      1.0,
                      CV_RGB(0, 0, 0), //font color
                      2);
        }
        
        cv::Point2f keypointTargetPos_1[nbTarget_1];
        for (int i = 0; i < nbTarget_1; i++)
        {
          keypointTargetPos_1[i] = keypointsTarget_1.at(i).pt;
        }

        cv::Point2f keypointTargetPos_2[nbTarget_2];
        for (int i = 0; i < nbTarget_2; i++)
        {
          keypointTargetPos_2[i] = keypointsTarget_2.at(i).pt;
        }


		
		objectiveDefinition(objectivePoints);


		
        // taille de l'image résultat

        int centre_x = max_width / 2;
        int centre_y = max_height / 2;

	// coefficients nécessaire pour centrer et mettre à l'échelle l'image
        double coef = 0.6*max_width/largeur; // à voir
        double tx = max_width/2;
        double ty = max_height/2;
        double tix = (objectivePoints[1].x-objectivePoints[0].x)/2;
        double tiy = (objectivePoints[3].y-objectivePoints[0].y)/2;
        
	// matrices de transformation - on centre les points objectifs, on centre et on met à l'échelle
        cv::Mat t1 = (cv::Mat_<double>(3, 3) << 1, 0, -tx, 0, 1, -ty, 0, 0, 1);
        cv::Mat t2 = (cv::Mat_<double>(3, 3) << 1/coef, 0, 0, 0, 1/coef, 0, 0, 0, 1);
        cv::Mat t3 = (cv::Mat_<double>(3, 3) << 1, 0, tix, 0, 1, tiy, 0, 0, 1);

        //calcul de la matrice de transformation entre l'espace réel et l'image
        cv::Mat transform = cv::getPerspectiveTransform(objectivePoints, keypointPos);
        
        //transformation des positions des points détecté
        cv::Point2f keypointTargetPosTrans_1[nbTarget_1];
		cv::Point2f keypointTargetPosTrans_2[nbTarget_2];
        //necessaire pour utiliser les libraires opencv de calcul matricielle

        //calcul de la transformation inverse
        cv::Mat Inv = transform.inv();

       cout<< endl << "position des "<< nbTarget_1<<" objets 1" <<endl;

	calculPoints(keypointTargetPos_1, keypointTargetPosTrans_1, nbTarget_1, Inv);
       	cout<< endl << "position des "<< nbTarget_2<<" objets 2" <<endl;
	calculPoints(keypointTargetPos_2, keypointTargetPosTrans_2, nbTarget_2, Inv);
	
		
	list_keypointTargetPosTrans_1.clear();
	list_keypointTargetPosTrans_2.clear();
		
		int i = 0;
       	geometry_msgs::Point point;

    	for (i=0; i<nbTarget_1; i++) 
    	{
        	point.x = keypointTargetPosTrans_1[i].x;
        	point.y = keypointTargetPosTrans_1[i].y;
        	point.z = 0;
        	list_keypointTargetPosTrans_1.push_back(point);
    	}
		
	   	for (i=0; i<nbTarget_2; i++) 
    	{
        	point.x = keypointTargetPosTrans_2[i].x;
        	point.y = keypointTargetPosTrans_2[i].y;
        	point.z = 0;
        	list_keypointTargetPosTrans_2.push_back(point);
    	}


        // perspective.
        cv::warpPerspective(im_with_keypoints,
                            im_with_keypointsOrtho,
                            transform*t3*t2*t1,
                            cv::Size(max_width, max_height),
                            cv::INTER_LINEAR | cv::WARP_INVERSE_MAP);

        // Update GUI Window seulement si les 4 points de mire avaient été identifiés
        imshow(OPENCV_IMAGE_RESULTAT, im_with_keypoints);
        imshow(OPENCV_IMAGE_ORTHO, im_with_keypointsOrtho);


      }
      else if (nbMire != 4)
      {
        erreurNombreBlob = true;
        printf("Erreur nombre de blob detecte\n");
      }
   }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

