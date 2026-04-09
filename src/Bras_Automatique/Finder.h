#ifndef FINDER_H
#define FINDER_H

#include <opencv2/core.hpp>
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
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <sstream>

std::pair<cv::Point_<int>, cv::Point_<int>> getMinMaxPointsX(const std::vector<cv::Point_<int>>& points);
std::pair<cv::Point_<int>, cv::Point_<int>> getMinMaxPointsY(const std::vector<cv::Point_<int>>& points);
std::pair<cv::Point_<int>, cv::Point_<int>> getMinMaxPoints(const std::vector<cv::Point_<int>>& points);

std::vector<cv::Point_<int>> MoyenneVecteurs(const std::vector<cv::Point_<int>>& vecteur1, const std::vector<cv::Point_<int>>& vecteur2);
std::vector<cv::Point_<int>> Approximation_ligne(cv_bridge::CvImagePtr testimage,std::vector<cv::Point_<int>>Contour,int nombre_decoupage, int iR, int iG, int iB);
#endif // MINMAXPOINTS_H
