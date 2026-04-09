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
static const string OPENCV_IMAGE_THRESHOLD_Obstacle = "obstacle window";
static const string OPENCV_IMAGE_THRESHOLD_Ligne = "ligne window";

static const string FILE_NAME = "/home/ros/catkin_ws/src/projet_cmt/HSV_Config.txt";

 
const int max_thresh = 255;

// Set up the detector with default parameters.
//	SimpleBlobDetector detector;

// Detect blobs.
//	std::vector<KeyPoint> keypoints;
//	detector.detect( im, keypoints);

Mat imgHSV;
Mat imgThresholded_Ligne,imgThresholded_Obstacle;

 int iLowH = 3;
int iHighH = 96;

int iLowS = 123; 
int iHighS = 227;

int iLowV = 182;
int iHighV = 255;


int iLowH_Obstacle = 19;
int iHighH_Obstacle = 93;

int iLowS_Obstacle = 120; 
int iHighS_Obstacle = 172;

int iLowV_Obstacle = 122;
int iHighV_Obstacle = 255;

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

    		//namedWindow(OPENCV_WINDOW);
    		namedWindow("Control",WINDOW_AUTOSIZE); //create a window called "Control"
			namedWindow(OPENCV_IMAGE_THRESHOLD_Obstacle);
			namedWindow(OPENCV_IMAGE_THRESHOLD_Ligne);


 //Create trackbars in "Control" window
   		createTrackbar("LowH", "Control", &iLowH, 255); //Hue (0 - 179)
		createTrackbar("HighH", "Control", &iHighH, 255);
		createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    		createTrackbar("HighS", "Control", &iHighS, 255);
		createTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
		createTrackbar("HighV", "Control", &iHighV, 255);


		createTrackbar("LowH_Obstacle", "Control", &iLowH_Obstacle, 255); //Hue (0 - 179)
		createTrackbar("HighH_Obstacle", "Control", &iHighH_Obstacle, 255);
		createTrackbar("LowS_Obstacle", "Control", &iLowS_Obstacle, 255); //Saturation (0 - 255)
    		createTrackbar("HighS_Obstacle", "Control", &iHighS_Obstacle, 255);
		createTrackbar("LowV_Obstacle", "Control", &iLowV_Obstacle, 255); //Value (0 - 255)
		createTrackbar("HighV_Obstacle", "Control", &iHighV_Obstacle, 255);

		createTrackbar("Save", "Control", &iButton, 1, ImageConverter::buttonCb1,NULL);
		createTrackbar("Quit", "Control", &iButton, 1, ImageConverter::buttonCb2,NULL);

	

	 }

	~ImageConverter()
  	{
    	//destroyWindow(OPENCV_WINDOW);
		destroyWindow(OPENCV_CONTROL_WINDOW);
 		destroyWindow(OPENCV_IMAGE_THRESHOLD_Obstacle);
 		destroyWindow(OPENCV_IMAGE_THRESHOLD_Ligne);
  	}


	static void buttonCb1(int position, void *userData)

	{
		std::ofstream outfile;
		system("rm -r /home/ros/catkin_ws/src/projet_cmt/HSV_Config.txt");
		outfile.open(FILE_NAME, std::ofstream::app); // append instead of overwrite
  		outfile << "ligne "<<iLowH<<" "<< iHighH << " "<<iLowS<<" "<< iHighS << " "<<iLowV<<" "<< iHighV <<std::endl; 
  		outfile << "Obstacle "<<iLowH_Obstacle<<" "<< iHighH_Obstacle << " "<<iLowS_Obstacle<<" "<< iHighS_Obstacle << " "<<iLowV_Obstacle<<" "<< iHighV_Obstacle <<std::endl; 
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
 
		inRange(imgHSV, Scalar(iLowH_Obstacle, iLowS_Obstacle, iLowV_Obstacle), Scalar(iHighH_Obstacle, iHighS_Obstacle, iHighV_Obstacle), imgThresholded_Obstacle); //Threshold the image
		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded_Ligne);
      
 // morphological opening (remove small objects from the foreground)
		erode(imgThresholded_Obstacle, imgThresholded_Obstacle, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		dilate( imgThresholded_Obstacle, imgThresholded_Obstacle, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

 // morphological closing (fill small holes in the foreground)
		dilate( imgThresholded_Obstacle, imgThresholded_Obstacle, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
		erode(imgThresholded_Obstacle, imgThresholded_Obstacle, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		
		// morphological opening (remove small objects from the foreground)
		erode(imgThresholded_Ligne, imgThresholded_Ligne, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		dilate( imgThresholded_Ligne, imgThresholded_Ligne, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

 // morphological closing (fill small holes in the foreground)
		dilate( imgThresholded_Ligne, imgThresholded_Ligne, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
		erode(imgThresholded_Ligne, imgThresholded_Ligne, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );






    // Draw an example circle on the video stream
		if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
			circle(cv_ptr->image, Point(50, 50), 10, CV_RGB(255,0,0));



	
    // Update GUI Window
		//imshow(OPENCV_WINDOW, cv_ptr->image);
		imshow(OPENCV_IMAGE_THRESHOLD_Obstacle, imgThresholded_Obstacle);
		imshow(OPENCV_IMAGE_THRESHOLD_Ligne, imgThresholded_Ligne);

		waitKey(3);
//	cvtColor(cv_ptr, imgGrayscale, CV_BGR2GRAY);   
    // Output modified video stream
    		image_pub_.publish(cv_ptr->toImageMsg());
	}
};

int main(int argc, char** argv)
{
 	cout << "OpenCV version : " << CV_VERSION << endl;
	cout << "Major version : " << CV_MAJOR_VERSION << endl;
	cout << "Minor version : " << CV_MINOR_VERSION << endl;
	cout << "Subminor version : " << CV_SUBMINOR_VERSION << endl;
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;

/*
     namedWindow("Name", WINDOW_NORMAL);
    setWindowProperty ("Name", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);

   getWindowImageRect("Name")
    printf("%d %d\n",screen.height, screen.width);
 VideoCapture cap;
    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    if(!cap.open(0))
        return 0;
    for(;;)
    {
          Mat frame;
          cap.read(frame);
          if( frame.empty() ) break; // end of video stream

//resize(frame,frame, Size(image.cols/2, image.rows/2));

          imshow("Name", frame);
          if( waitKey(10) == 27 ) break; // stop capturing by pressing ESC 
    }
    // the camera will be closed automatically upon exit
    // cap.close();
    return 0;

*/

	ros::spin();
	return 0;
}
