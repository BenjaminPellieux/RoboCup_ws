#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>

using namespace cv;
using namespace std;

//static const string OPENCV_WINDOW = "Image window";
static const string OPENCV_CONTROL_WINDOW = "control window";
static const string OPENCV_IMAGE_THRESHOLD = "threshold window";


 
const int max_thresh = 255;

// Set up the detector with default parameters.
//	SimpleBlobDetector detector;

// Detect blobs.
//	std::vector<KeyPoint> keypoints;
//	detector.detect( im, keypoints);

Mat imgHSV_serv;
Mat imgThresholded_serv;

 int iLowH_serv = 0;
int iHighH_serv = 179;

int iLowS_serv = 0; 
int iHighS_serv = 255;

int iLowV_serv = 0;
int iHighV_serv = 255;

int iLowH_Obstacle = 0;
int iHighH_Obstacle = 3;


int iHighS_Obstacle = 255;
int iLowS_Obstacle = 121; 

int iLowV_Obstacle = 0;
int iHighV_Obstacle = 255;

int iButton_serv = 0;

class ImageConverter
{
 	ros::NodeHandle nh_;
  	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_serv;
	image_transport::Publisher image_pub_serv;

	public:
	  	

	ImageConverter()
	: it_(nh_)
	{
    // Subscrive to input video feed and publish output video feed
		image_sub_serv = it_.subscribe("/cv_camera/image_raw", 1,  &ImageConverter::imageCb_serv, this);
		image_pub_serv = it_.advertise("/image_converter/output_video", 1);

    		//namedWindow(OPENCV_WINDOW);
    		namedWindow("Control",WINDOW_AUTOSIZE); //create a window called "Control"
		namedWindow(OPENCV_IMAGE_THRESHOLD);


 //Create trackbars in "Control" window
   		createTrackbar("LowH", "Control", &iLowH_serv, 255); //Hue (0 - 179)
		createTrackbar("HighH", "Control", &iHighH_serv, 255);
		createTrackbar("LowS", "Control", &iLowS_serv, 255); //Saturation (0 - 255)
    	createTrackbar("HighS", "Control", &iHighS_serv, 255);
		createTrackbar("LowV", "Control", &iLowV_serv, 255); //Value (0 - 255)
		createTrackbar("HighV", "Control", &iHighV_serv, 255);
		
		
		

		createTrackbar("Save", "Control", &iButton_serv, 1, ImageConverter::buttonCb1,NULL);
		createTrackbar("Quit", "Control", &iButton_serv, 1, ImageConverter::buttonCb2,NULL);

	

	 }

	~ImageConverter()
  	{
    	//destroyWindow(OPENCV_WINDOW);
		cv::destroyWindow(OPENCV_CONTROL_WINDOW);
 		cv::destroyWindow(OPENCV_IMAGE_THRESHOLD);
  	}


	static void buttonCb1(int position, void *userData)

	{
		std::ofstream outfile;
		outfile.open(FILE_NAME, std::ofstream::app); // append instead of overwrite
    	auto t = std::time(nullptr);
    	auto tm = *std::localtime(&t);
		outfile << "--------------------------------------------------"<<std::endl;
    	outfile << std::endl << std::put_time(&tm, "%d-%m-%Y %H-%M-%S") << std::endl;
  		outfile << "Param "<<iLowH_serv<<" "<< iHighH_serv << " "<<iLowS_serv<<" "<< iHighS_serv << " "<<iLowV_serv<<" "<< iHighV_serv <<std::endl; 
  		outfile.close();
	}

	static void buttonCb2(int position, void *userData)

	{
  		exit(0);
	}

  	void imageCb_serv(const sensor_msgs::ImageConstPtr& msg)
  	{
    		cv_bridge::CvImagePtr cv_ptr_serv;
    		try
    		{
 			cv_ptr_serv = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    		}
    		catch (cv_bridge::Exception& e)
    		{
      			ROS_ERROR("cv_bridge exception: %s", e.what());
      			return;
    		}

  		cvtColor(cv_ptr_serv->image, imgHSV_serv, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
 
		inRange(imgHSV_serv, Scalar(iLowH_serv, iLowS_serv, iLowV_serv), Scalar(iHighH_serv, iHighS_serv, iHighV_serv), imgThresholded_serv); //Threshold the image
      
 // morphological opening (remove small objects from the foreground)
		erode(imgThresholded_serv, imgThresholded_serv, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		dilate( imgThresholded_serv, imgThresholded_serv, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

 // morphological closing (fill small holes in the foreground)
		dilate( imgThresholded_serv, imgThresholded_serv, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
		erode(imgThresholded_serv, imgThresholded_serv, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );






    // Draw an example circle on the video stream
		//if (cv_ptr_serv->image.rows > 60 && cv_ptr_serv->image.cols > 60)
			//circle(cv_ptr_serv->image, Point(50, 50), 10, CV_RGB(255,0,0));



	
    // Update GUI Window
		//imshow(OPENCV_WINDOW, cv_ptr_serv->image);
		imshow(OPENCV_IMAGE_THRESHOLD, imgThresholded_serv);

		waitKey(3);
//	cvtColor(cv_ptr_serv, imgGrayscale, CV_BGR2GRAY);   
    // Output modified video stream
    		image_pub_serv.publish(cv_ptr_serv->toImageMsg());
	}
};

