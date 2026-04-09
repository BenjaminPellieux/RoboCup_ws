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

static const string OPENCV_WINDOW = "Image window";
static const string OPENCV_CONTROL_WINDOW = "control window";
static const string OPENCV_IMAGE_THRESHOLD = "threshold window";

static const string FILE_NAME = "/home/ros/catkin_ws/config/opencv3_threshold.txt";
 
const int max_thresh = 255;

// Set up the detector with default parameters.
//	SimpleBlobDetector detector;

// Detect blobs.
//	std::vector<KeyPoint> keypoints;
//	detector.detect( im, keypoints);

Mat imgHSV;
Mat imgThresholded;

 int iLowH = 0;
int iHighH = 179;

int iLowS = 0; 
int iHighS = 255;

int iLowV = 0;
int iHighV = 255;

int iButton = 0;

class ImageConverter
{
 	ros::NodeHandle nh_;
  	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

	public:
	  	

	ImageConverter()
	: it_(nh_)
	{
    // Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/cv_camera/image_raw", 1,  &ImageConverter::imageCb, this);
		image_pub_ = it_.advertise("/image_converter/output_video", 1);

    		namedWindow(OPENCV_WINDOW);
    		namedWindow("Control",WINDOW_AUTOSIZE); //create a window called "Control"
		namedWindow(OPENCV_IMAGE_THRESHOLD);


 //Create trackbars in "Control" window
   		createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
		createTrackbar("HighH", "Control", &iHighH, 179);
		createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    	createTrackbar("HighS", "Control", &iHighS, 255);
		createTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
		createTrackbar("HighV", "Control", &iHighV, 255);

		createTrackbar("Save", "Control", &iButton, 1, ImageConverter::buttonCb1,NULL);
		createTrackbar("Quit", "Control", &iButton, 1, ImageConverter::buttonCb2,NULL);

	

	 }

	~ImageConverter()
  	{
    	destroyWindow(OPENCV_WINDOW);
		destroyWindow(OPENCV_CONTROL_WINDOW);
 		destroyWindow(OPENCV_IMAGE_THRESHOLD);
  	}


	static void buttonCb1(int position, void *userData)

	{
		std::ofstream outfile;
		outfile.open(FILE_NAME, std::ofstream::app); // append instead of overwrite
    	auto t = std::time(nullptr);
    	auto tm = *std::localtime(&t);
		outfile << "--------------------------------------------------"<<std::endl;
    	outfile << std::endl << std::put_time(&tm, "%d-%m-%Y %H-%M-%S") << std::endl;
  		outfile << "Param "<<iLowH<<" "<< iHighH << " "<<iLowS<<" "<< iHighS << " "<<iLowV<<" "<< iHighV <<std::endl; 
  		outfile.close();
	}

	static void buttonCb2(int position, void *userData)

	{
  		exit(0);
	}

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

  		cvtColor(cv_ptr->image, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
 
		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
      
 // morphological opening (remove small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

 // morphological closing (fill small holes in the foreground)
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );






    // Draw an example circle on the video stream
		if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
			circle(cv_ptr->image, Point(50, 50), 10, CV_RGB(255,0,0));



	
    // Update GUI Window
		imshow(OPENCV_WINDOW, cv_ptr->image);
		imshow(OPENCV_IMAGE_THRESHOLD, imgThresholded);

		waitKey(3);
//	cvtColor(cv_ptr, imgGrayscale, CV_BGR2GRAY);   
    // Output modified video stream
    		image_pub_.publish(cv_ptr->toImageMsg());
	}
};

