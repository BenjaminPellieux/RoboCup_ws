/*

Programme réaliser dans le cadre des projets Robotique de MT3 de l'ISTY 
Année 2022
Groupe de Marya Pasukhova, Tanguy Foret, Clément Cat
Professeur encadrant : Pierre Blazevic, Olivier Snoeck

commande pour lancer le programme et son contenu : roslaunch projet_cmt Tata.launch

*/
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
//#include "Clement_opencv.h"



#include "opencv2/imgcodecs.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>


#include <sstream>

/*Librairie créé pour les services*/
#include "projet_cmt/Traitement_HSV.h" 
#include "projet_cmt/Pas.h" 
/*________________________________________*/
/*~~~~~~~Déclaration des prototypes~~~~~~~*/
void imageCb(const sensor_msgs::ImageConstPtr& msg);
cv::Mat Thresholded(cv_bridge::CvImagePtr cv_ptr, int iLowH,int iHighH,int iLowS,int iHighS,int iLowV,int iHighV,int k);
std::vector<std::vector<cv::Point_<int>>> Blopblop(cv::Mat imageMono,cv_bridge::CvImagePtr cv_ptr,int iBlopVoulu,int iR, int iG,int iB);
std::vector<cv::Point_<int>> Approximation_ligne(cv_bridge::CvImagePtr cv_ptr,std::vector<cv::Point_<int>>Contour,int nombre_decoupage, int iR, int iG, int iB);
int IntersectionPoly(cv_bridge::CvImagePtr Image_conducteur,std::vector<cv::Point_<int>> Route,std::vector<cv::Point_<int>> Obstacle,int indice);
int absolue(float a);

/*________________________________________*/


using namespace cv;
using namespace std;

int image_width =320;
int image_height =240;

int distancex=0;
int distancey=0;

int iButton =0;


geometry_msgs::Vector3 Co;



image_transport::Subscriber image_sub_;
image_transport::Publisher image_pub_;
image_transport::Publisher image_pub2_;
image_transport::Publisher image_pub3_;
image_transport::Publisher image_pub4_;


/* Prototype des fonctions de service*/
static bool serv_Traitement_HSV(projet_cmt::Traitement_HSV::Request& req, projet_cmt::Traitement_HSV::Response& res); //projet_cmt_CAT_node
static bool serv_Pas(projet_cmt::Pas::Request& req, projet_cmt::Pas::Response& res);


ros::ServiceServer service_Traitement_HSV;
ros::ServiceServer service_Pas;



/* ===Définition des fonction de service===*/
static bool serv_Traitement_HSV(projet_cmt::Traitement_HSV::Request &req,projet_cmt::Traitement_HSV::Response &res)
{

/*Ce service permet de venir modifer le traitement HSV de l'image en cas de changement des conditions lumineuse ou autre 
on vient faire l'appel du service avec la commande : rosservice call /Traitement_HSV V1 V2 V3 V4 V5 V6*/
    ROS_INFO("Request : LowH : %ld HighH : %ld LowS : %ld HighS : %ld LowV : %ld HighV : %ld \n",req.LowH,req.HighH,req.LowS,req.HighS,req.LowV,req.HighV);
    
    
    
    iLowH = req.LowH;
    iHighH = req.HighH;
    iHighS = req.HighS;
    iLowS = req.LowS; 
    iLowV = req.LowV;
    iHighV = req.HighV;

    return true;
}


static bool serv_Pas(projet_cmt::Pas::Request& req, projet_cmt::Pas::Response& res)
{
    /* Ce servie permet d'envoyer la tolérance à avoir pour les servo motor par rapport a la commande d'aservissement on sera donc a plus ou moins la tolérance
    on vient faire l'appel du service avec la commande : rosservice call /Pas V1    */
    if(req.tolerance <1)
    {
        ROS_INFO("Merci de saisir une valeur entière supérieur 0\n" );
    }
    else
    {
        ROS_INFO("Request : tolerance : %ld \n",req.tolerance);
        Co.z=req.tolerance;
    }
    
    return true;
};

/*__________________________________________________________________________*/

	
	
	
	
	
	
/*__________________________________________________________________________*/	

void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
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
  	zero.x=0;
	zero.y=0;
	int i = 0;
  	
  	cv::Mat img_Test,img_Obstacle;
    cv_bridge::CvImagePtr cv_ptr,testimage;
    
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      testimage = cv_ptr;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
	
	readParam();
	/*Traitement HSV*/
	img_Test =Thresholded(cv_ptr, iLowH,iHighH,iLowS,iHighS,iLowV,iHighV,1);
	img_Obstacle =Thresholded(cv_ptr, iLowH_Obstacle,iHighH_Obstacle,iLowS_Obstacle,iHighS_Obstacle,iLowV_Obstacle,iHighV_Obstacle,0);
	
	/*Traitement Detection des contours*/
	ContourRouge=Blopblop(img_Obstacle,cv_ptr,1,255,0,0);
	ContourBleu=Blopblop(img_Test,cv_ptr,2,0,0,255);
	
	if(ContourBleu.size() != 0)
	{
		vector <int> stock = {0,0};
		vector <int> compar = {0,0};
		for(int i =0; i<ContourBleu.size();i++)
		{
			if(compar[0]< contourArea(ContourBleu[i]))
			{
				compar[1] = compar[0] ;
				stock[1] = stock[0];
				
				compar[0] = contourArea(ContourBleu[i]);;
				stock[0] =i;
			}
			else
			{
				if(compar[1]< contourArea(ContourBleu[i]))
				{
					cout<<"aire :" <<contourArea(ContourBleu[i])<<endl;
					compar[1] = contourArea(ContourBleu[i]);;
					stock[1] =i;
				}
				else
				{
			
				}
			}
		}
		
		Moyenne0=Approximation_ligne(testimage,ContourBleu[stock[0]],20, 0,255,0);//Voir Finder pour la definition
		Moyenne1=Approximation_ligne(testimage,ContourBleu[stock[1]],20, 0,255,0);//Voir Finder pour la definition



		cout<<Moyenne0.size()<<endl;
		for(int i=1; i<(Moyenne0.size());i++)
		{
			Direction[i].x=static_cast<int>((Moyenne0[i].x+Moyenne1[i].x)/2);
			Direction[i].y=static_cast<int>((Moyenne0[i].y+Moyenne1[i].y)/2);
			
			if ((Direction[i].y < 420)&&(Direction[i].y > 0))
			{
				Moyenne0X += Direction[i].x;
				Moyenne0Y += Direction[i].y;
				k++;	
			}
			//cv::rectangle(cv_ptr->image,(0,318),(640,360),(0,255,0));
		}
		
		if(k>0)
		{
			cout << "MoyenneOX = " << Moyenne0X/k << endl;
			cout << "MoyenneOY = " << Moyenne0Y/k << endl;		
		
			Largeur = absolue(320-Moyenne0X/k);
			Hauteur = Moyenne0Y/k;
			Angle = std::atan(Largeur/Hauteur)*180/3.1415/4;
			
			if((Moyenne0X/k)<320)
			{
				Angle = -Angle;
			}
			else{}
			cout << "Angle = " << Angle << endl;
			
		}
		
		Co.x = Angle;
		
		Moyenne0X = 0;
		Moyenne0Y = 0;
		
		for(int i = 1; i<(Moyenne0.size()-1);i++)
		{
			cv::arrowedLine(cv_ptr->image,Direction[i],Direction[i+1],color,1,8,0,0.1);
		}

    		
    		
		std::reverse(Moyenne1.begin(), Moyenne1.end());
		Poly.reserve(Moyenne0.size() + Moyenne0.size());
    		Poly.insert(Poly.end(), Moyenne0.begin(), Moyenne0.end());
    		Poly.insert(Poly.end(), Moyenne1.begin(), Moyenne1.end());
    		/*for(i=0;i<Poly.size();i++)
    		{
    			cout<<"Poly "<<i<<" x : "<<Poly[i].x<<", y : "<<Poly[i].y<<endl;
    		}*/
		
		
		
		
		cv::polylines(cv_ptr->image,Poly,1,Scalar(255,0,255),1,8,0);
			
		for(int i=0;i<ContourRouge.size();i++)
		{
			IntersectionPoly(cv_ptr,Poly,ContourRouge[i],i);
		}
	
	 	}	
	 	image_pub4_.publish(cv_ptr->toImageMsg());
	 };
	 
	 /*__________________________________________________________________________*/










/*_____________________________________________---=====IntersectionPoly=====---_________________________________________*/
/*												*/
/* 			Cette fonction check si il y a un obstacle dans la route				*/
/*	std::vector<cv::Point_<int>> Route == Tableau de point qui represente le polynome du chemin à suivre	*/
/*	std::vector<cv::Point_<int>> Obstacle ==  tableau du contour de l'obstacle à vérifier			*/
/*	int indice == indice du contour de l'obstacle testé retourné si intersection 				*/
/*______________________________________________________________________________________________________________________*/


int IntersectionPoly(cv_bridge::CvImagePtr Image_conducteur,std::vector<cv::Point_<int>> Route,std::vector<cv::Point_<int>> Obstacle,int indice)
{
	int Intersection = -1;
	Point_<int> pt;
	Scalar color(255,255,0);
	
	if(Obstacle.size()>0)
	{
		for(int i =0; i<Obstacle.size();i++)
		{
			pt += Obstacle[i];
		}
		pt.x=static_cast<int> (pt.x/Obstacle.size());
		pt.y=static_cast<int> (pt.y/Obstacle.size());
	
		if( pointPolygonTest(Route,pt,false)==1)
		{
			Intersection = indice;
			
			//drawContours( Image_conducteur->image, Obstacle, -1, color,2 );
			putText(Image_conducteur->image,"Badaboum",pt,FONT_HERSHEY_SIMPLEX ,1,color,1,LINE_8,false);
			image_pub4_.publish(Image_conducteur->toImageMsg());
			
		}
	}
	return Intersection;
}
/*__________________________________________________________________________*/




	 
/*_____________________________---=====BlopBlop=====---_____________________________*/
/*																					*/
/*	cv::Mat imageMono ==> Image reférence qui va servir a la détection de contour 	*/
/*	cv_bridge::CvImagePtr cv_ptr ==> Image de base sur laquelle on va venir faire 	*/
/*							des ajouts de contours									*/
/*			int iBlopVoulu ==> Nombre de blop que l'on cherche au total				*/
/*	   int iR, int iG,int iB ==> Fait passer les parametres de couleur en RGB		*/
/*																					*/
/*__________________________________________________________________________________*/

std::vector<std::vector<cv::Point_<int>>> Blopblop(cv::Mat imageMono,cv_bridge::CvImagePtr cv_ptr,int iBlopVoulu,int iR, int iG,int iB)
	{


	/*cv::arrowedLine(cv_ptr->image,cv::Point(x1, y1),cv::Point(x2, y2),Scalar(0, 255, 0),1,8,0,0.1);
    cv::circle(cv_ptr->image, cv::Point(image_width/2, image_height/2), 2, CV_RGB(255,0,0));*/
    
     vector <vector<Point>> contours;
     vector <Vec4i> hierarchy;
    
    findContours( imageMono, contours, hierarchy,RETR_TREE, CHAIN_APPROX_NONE /*CHAIN_APPROX_SIMPLE*/ );
    
    
    Scalar color( iB, iG, iR );// en BGR
    drawContours( cv_ptr->image, contours, -1, color,2 );
    //cout<<"contours : "<< contours.size() <<endl;//affichage pour l'utilisateur
    image_pub_.publish(cv_ptr->toImageMsg());
    return contours;
};











/*__________________________________________________________________________*/


/*______________________________________---=====Thresholded=====---_____________________________*/
/*																								*/
/*		cv_bridge::CvImagePtr cv_ptr ==> Image sur laquelle on va venir appliqué les seuils		*/
/*	int iLowH,int iHighH,int iLowS,int iHighS,int iLowV,int iHighV ==> Valeurs des seuils HSV	*/
/*									int k ==> Parametre de débug								*/
/*																								*/
/*______________________________________________________________________________________________*/


cv::Mat Thresholded(cv_bridge::CvImagePtr cv_ptr, int iLowH,int iHighH,int iLowS,int iHighS,int iLowV,int iHighV,int k)
{
	sensor_msgs::ImagePtr img_msg;
	cv::Mat imgHSV,imggray,thresh;
	cv::Mat imgThresholded;
	
	/*cv::cvtColor(cv_ptr->image, imggray, cv::COLOR_BGR2GRAY);
	threshold(imggray, thresh, 150, 255, THRESH_BINARY);
	
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(thresh, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
	// draw contours on the original image
	Mat image_copy = cv_ptr->image.clone();
	drawContours(image_copy, contours, -1, Scalar(0, 255, 0), 2);
	imshow("None approximation", image_copy);
	imshow("Binary mage", thresh);
	waitKey(0);
	destroyAllWindows();*/
	
	
    cv::cvtColor(cv_ptr->image, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV  
	inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); //		     etalonage de l'image   
	
	erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
      erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    
    img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8",imgThresholded ).toImageMsg();
    

    
    if(k==1)
    {
    	image_pub2_.publish(img_msg);//pub bleu
    }
    else
    {
		image_pub3_.publish(img_msg);//pub rouge
    }

    
    return imgThresholded;
};
     
     
     
     
     
     
     
     
     
     
     
/*__________________________________________________________________________*/


void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
}

/*__________________________________________________________________________*/



int absolue(float a)
{
	if(a<0)
	{
		a = -a;
	}
	else{}
	return a;
}










int main(int argc, char **argv)
{
  Co.z=0;
  
  ros::init(argc, argv, "cat_node");
  
  ros::NodeHandle n;
  image_transport::ImageTransport it_(n);
  
  /*===Déclation des node de service avec leurs callback respectif===*/
  service_Traitement_HSV = n.advertiseService("Traitement_HSV",serv_Traitement_HSV);
  service_Pas = n.advertiseService("Pas",serv_Pas);
  /* ===Attribution des noeuds et de leur nom respectif pour les flux vidéo===*/

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
 image_pub_ = it_.advertise("/image_converter/output_Obstacle", 1);
 image_pub2_ = it_.advertise("/image_converter/Traitement_Bleu", 1);
 image_pub3_ = it_.advertise("/image_converter/Traitement_Rouge", 1);
 image_pub4_ = it_.advertise("/image_converter/Traitement_Ligne", 1);
    
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  image_sub_ = it_.subscribe("/cv_camera/image_raw", 1, imageCb);
  
  /*===Attribution du topic de publication pour l'envoi à l'arduino===*/
  ros::Publisher CooBlop_pub = n.advertise< geometry_msgs::Vector3 >("Coo_blop_node",1000);



  ros::Rate loop_rate(10);
   

  while (ros::ok())
  {
        
        if (Co.z ==1)
	{
		//ImageConverter ic;
	}
        
        //ROS_INFO(" x = %f , y= %f ,z= %f",Co.x,Co.y,Co.z); 
        
    CooBlop_pub.publish(Co);// publication de la variable de type vector3 dans le noeud Coo_blop_node
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}




/*
std::vector<cv::Point_<int>> Approximation_ligne(cv_bridge::CvImagePtr testimage,std::vector<cv::Point_<int>>Contour,int nombre_decoupage, int iR, int iG, int iB)
{
	int isize = Contour.size();
	int k =0;
	std::vector<cv::Point_<int>> Pointmoyen(nombre_decoupage);
	
	Point_<int> zero;
	zero.x=0;
	zero.y=0;
	
	Scalar color( iB, iG, iR );// en BGR
	if(isize > nombre_decoupage)
	{
		for(int i=1; i<=nombre_decoupage;i++)
		{
			Pointmoyen[i-1]=zero;
		
			for(k =0;k < static_cast<int>(((isize/nombre_decoupage)*i));k++)
			{

				Pointmoyen[i-1]=Pointmoyen[i-1]+Contour[k];
			}
			Pointmoyen[i-1].x=static_cast<int>(Pointmoyen[i-1].x/k);
			Pointmoyen[i-1].y=static_cast<int>(Pointmoyen[i-1].y/k);

		}

		/*Permet de faire afficher les lignes moyennes de compositions d'un contour*/
		/*for(int i=0; i<nombre_decoupage-1;i++)
		{
			//cout<<"point x: "<< Pointmoyen[i].x<<" point y: "<<Pointmoyen[i].y <<endl;//affichage pour l'utilisateur
			cv::arrowedLine(testimage->image,Pointmoyen[i],Pointmoyen[i+1],color,1,8,0,0.1);
		}
	
		image_pub4_.publish(testimage->toImageMsg());
	
	}
	return Pointmoyen;
	};
*/
