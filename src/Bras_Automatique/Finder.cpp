#include "Finder.h"
using namespace cv;
using namespace std;

std::pair<cv::Point_<int>, cv::Point_<int>> getMinMaxPoints(const std::vector<cv::Point_<int>>& points)
{
    auto[minPointX, maxPointX] = getMinMaxPointsX(points);
    auto[minPointY, maxPointY] = getMinMaxPointsY(points);
     cv::Point_<int> minPoint = points[0], maxPoint = points[0];
    if (points.empty())
    {
        return std::make_pair(cv::Point_<int>(), cv::Point_<int>());
    }
    minPoint.x=minPointX.x;
    minPoint.y=minPointY.y;
    maxPoint.x=maxPointX.x;
    maxPoint.y=maxPointY.y;  
    return std::make_pair(minPoint, maxPoint);
}






std::pair<cv::Point_<int>, cv::Point_<int>> getMinMaxPointsX(const std::vector<cv::Point_<int>>& points) {
    if (points.empty()) {
        return std::make_pair(cv::Point_<int>(), cv::Point_<int>());
    }
    cv::Point_<int> minPoint = points[0], maxPoint = points[0];
    for (const auto& point : points) {
        if (point.x < minPoint.x || (point.x == minPoint.x && point.y < minPoint.y)) {
            minPoint = point;
        }
        if (point.x > maxPoint.x || (point.x == maxPoint.x && point.y > maxPoint.y)) {
            maxPoint = point;
        }
    }
    return std::make_pair(minPoint, maxPoint);
}



std::pair<cv::Point_<int>, cv::Point_<int>> getMinMaxPointsY(const std::vector<cv::Point_<int>>& points) {
    if (points.empty()) {
        return std::make_pair(cv::Point_<int>(), cv::Point_<int>());
    }
    cv::Point_<int> minPoint = points[0], maxPoint = points[0];
    for (const auto& point : points) {
        if (point.y < minPoint.y || (point.y == minPoint.y && point.x < minPoint.x)) {
            minPoint = point;
        }
        if (point.y > maxPoint.y || (point.y == maxPoint.y && point.x > maxPoint.x)) {
            maxPoint = point;
        }
    }
    return std::make_pair(minPoint, maxPoint);
}


std::vector<cv::Point_<int>> MoyenneVecteurs(const std::vector<cv::Point_<int>>& vecteur1, const std::vector<cv::Point_<int>>& vecteur2)
{
    std::vector<cv::Point_<int>> resultat;
    resultat.reserve(vecteur1.size());
    for (int i = 0; i < vecteur1.size(); i++)
    {
        resultat[i].x = static_cast<int>((vecteur1[i].x + vecteur2[i].x) / 2);
        resultat[i].y = static_cast<int>((vecteur1[i].y + vecteur2[i].y) / 2);
    }
    return resultat;
}



/*______________________________________________---=====Approximation_ligne=====---_____________________________________*/
/*												*/									/*												*/
/*		cv_bridge::CvImagePtr cv_ptr ==> Image sur laquelle on va venir appliqué les seuils		*/

/*______________________________________________________________________________________________________________________*/

std::vector<cv::Point_<int>> Approximation_ligne(cv_bridge::CvImagePtr testimage,std::vector<cv::Point_<int>>Contour,int nombre_decoupage, int iR, int iG, int iB)
{
	int isize = Contour.size();
	int k =0,y=0,i=0;
	std::vector<cv::Point_<int>> Pointmoyen(nombre_decoupage);
	cv::Point_<int> Point_min,Point_max;
	
	
	Point_<int> zero;
	zero.x=0;
	zero.y=0;
	
	Scalar color( iB, iG, iR );// en BGR
	auto[minPoint, maxPoint] = getMinMaxPoints(Contour);
	
	if(static_cast<int>((maxPoint.y-minPoint.y)/(nombre_decoupage-1)))
	{
		
		k =static_cast<int>((maxPoint.y-minPoint.y)/(nombre_decoupage-1));//on determine le pas

			Pointmoyen[i]=zero;
			y=minPoint.y;
			
			for(i=0;i<nombre_decoupage;i++)
			{
				
				Pointmoyen[i]=zero;
				Point_min.x = minPoint.x;
				Point_min.y = y;
				Point_max.x = maxPoint.x;
				Point_max.y = y;
				
				while(pointPolygonTest(Contour,Point_min,false)==-1 && Point_min.x <= maxPoint.x)
				{
					Point_min.x+=5;
					//cout<<"Good bye my friend   "<<Point_min.x<<endl;
				}
				while(pointPolygonTest(Contour,Point_max,false)==-1 && Point_max.x >= minPoint.x)
				{
					Point_max.x-=5;
					//cout<<"Good bye my lover  "<<Point_max.x<<endl;
				}
				Pointmoyen[i].x=static_cast<int>((Point_max.x+Point_min.x)/2);
				Pointmoyen[i].y=y;
				y+=k;

			}
			
		}
	return Pointmoyen;
}
