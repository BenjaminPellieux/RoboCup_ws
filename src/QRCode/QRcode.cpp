#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        // Convertir l'image ROS en une image OpenCV
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

        // Créer le détecteur QR Code (OpenCV 4.0+ requis)
        cv::QRCodeDetector qrDecoder;

        // Détecter et décoder le QR Code
        std::string data;
        std::vector<cv::Point> points;
        if (qrDecoder.detect(cv_ptr->image, points))
        {
            data = qrDecoder.decode(cv_ptr->image, points);
            if (!data.empty())
            {
                // Afficher les données et dessiner le cadre autour du QR Code
                ROS_INFO("QR Code Data: %s", data.c_str());
                for (size_t i = 0; i < points.size(); i++)
                {
                    cv::line(cv_ptr->image, points[i], points[(i+1)%points.size()], cv::Scalar(0,0,255), 2);
                }
            }
        }

        // Afficher l'image avec OpenCV (optionnel)
        cv::imshow("QR Code Detector", cv_ptr->image);
        cv::waitKey(3);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_qr_code_detector");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/cv_camera/image_raw", 1, imageCallback);
    ros::spin();
    return 0;
}
