#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int16MultiArray.h"
#include <sensor_msgs/JointState.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>


int Mapping(float angle);
struct vector{
//Types associés
   float x;	
   float y;
   float z;
   };
   
bool CalcIk(vector* position, vector* orientation, float anc_theta[]);
const float L1 = 0.3f;// Valeur originale : 0.3f -> 0.01 premier test
const float L2 = 0.3f;
const float L3 = 0.2f;
const float L4 = 0.2f;
const float L5 = 0.1f;
const float L6 = 0.1f;  
float theta[6];
float anc_theta[6];
vector position;
vector orientation;
vector gripreboot;
vector test_p;
vector test_o;
vector test_g;

float PI=3.1415;
//




int MappingMX(float angle)
{
    int theta_int =0;
    theta_int=angle *4095;
    return theta_int;
}

int MappingAX(float angle)
{
    int theta_int =0;
    theta_int=angle *1023;
    return theta_int;
}
    
void chatterCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
 ROS_INFO("I heard: [%f ,%f, %f, %f, %f, %f, %f, %f \n]",msg->data[0],msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5],msg->data[6],msg->data[7]);
  
  struct vector* pposition;
  struct vector* porientation;
  struct vector* pgripreboot;  
  pgripreboot = new vector;
  pposition = new vector;
  porientation = new vector;	
  pposition->x=msg->data[0];
  pposition->y=msg->data[1];
  pposition->z=msg->data[2];
  porientation->x=msg->data[3];
  porientation->y=msg->data[4];//Theta[5] correspond directement à la rotation y de la cam
  porientation->z=msg->data[5];
  pgripreboot->x=msg->data[6];
  pgripreboot->y=msg->data[7];
  position.x=msg->data[0];
  test_p.x=msg->data[0];
  test_p.y=msg->data[1];
  test_p.z=msg->data[2];
  test_o.x=msg->data[3];
  test_o.y=msg->data[4];
  test_o.z=msg->data[5];
  test_g.x=msg->data[6];
  test_g.y=msg->data[7];
  gripreboot.x=msg->data[6];
  gripreboot.y=msg->data[7];
  
//ROS_INFO("Okay");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("data", 1000, chatterCallback);// On appelle notre callback et on veut qu'il actualise orientation et position
  
  ros::Publisher pub_joints = n.advertise<sensor_msgs::JointState>("joint_states", 1000);
  ros::Publisher pub_angle  = n.advertise<std_msgs::Int16MultiArray>("retour_angles", 1000);
  //ROS_INFO("Okay");
  sensor_msgs::JointState jointState; //create an object of type "jointState", 
  jointState.name.push_back("base_link_to_double_shoulder");
  jointState.name.push_back("double_shoulder_to_arm");
  jointState.name.push_back("arm_to_upper_forearm");
  jointState.name.push_back("upper_forearm_to_lower_forearm");
  jointState.name.push_back("lower_forearm_to_wrist");
  jointState.name.push_back("wrist_to_gripper_body");
  
   for (int i=0;i<6;i++)
        jointState.position.push_back(0.0); // allocate memory and initialize joint values to 0
  struct vector* position;
  struct vector* orientation;
  struct vector* gripreboot;
  position = new vector;
  orientation = new vector;
  gripreboot = new vector;
  //Quaternion rotation;
  //float *theta;
  //theta = new float[6];    
  //angle of the joints
  float *anc_theta;
  anc_theta= new float[6];    
  //angle of the joints
  position->x=0;
  position->y=(L1+L2+L3+L4+L5+L6)/2;
  position->z=0;
  //printf("position i : %f",position->y);
  
  orientation->x=0;
  orientation->y=0;//Theta[5] correspond directement à la rotation y de la cam
  orientation->z=0;
  for (int i = 0; i < 6; i++)
    theta[i] = anc_theta[i] = 0.0f;
  CalcIk(position, orientation, anc_theta);
  position->y=1.2;
  orientation->x=0;
  ros::Rate r(10);
  while(ros::ok())
  {
     ros::spinOnce();  // Permettre la gestion des événements ROS 
     
     //printf("\n test = %f ",test_p.x);
     //printf("\n test = %f ",test_p.y);
     
     position->x=test_p.x;
     position->y=test_p.y;
     position->z=test_p.z;
     orientation->x=test_o.x;
     orientation->y=test_o.y;//Theta[5] correspond directement à la rotation y de la cam
     orientation->z=test_o.z;
     gripreboot->x=test_g.x;
     gripreboot->y=test_g.y;
     
     jointState.header.stamp = ros::Time::now();// Permet de mettre le temps dans l'header pour que la TF puisse se faire !
     CalcIk(position, orientation, anc_theta);
     //ROS_INFO("post_calcIk ok");
     
     int theta_int[6];
     
     for(int i=0;i<6;i++)
     {       


        if(i==2)
        {
            jointState.position[2] = (theta[2]-90)*PI/180;
            //theta[2]=(theta[2]-90)*PI/180;
            //ROS_INFO("LA MODIF DE COMPIL EST PASSEE");
        }    
        else
        {
                  jointState.position[i] = theta[i]*PI/180;                  
                   
        }

     /*  if(i==2)
        {
            theta[2] = (theta[2]-90)*PI/180;
            //theta[2]=(theta[2]-90)*PI/180;
            ROS_INFO("LA MODIF DE COMPIL EST PASSEE");
        }    
        else
        {
            theta[i]=theta[i]*PI/180;
            }*/
            theta[i]=jointState.position[i];
     }
     
       
        
     //jointState.position[0] = theta[0]*PI/180;
     // angle-> 0val 
     // 0->2048     180 ->4095 -180>=0
     // (val-2048) * 
     theta_int[0]=MappingMX(position->x);
     //ROS_INFO("J0ok");
     //jointState.position[1] = theta[1]*PI/180;
     theta_int[1]=MappingMX(position->y);
     //ROS_INFO("J1ok");
     //jointState.position[2] = theta[2]*PI/180;
     theta_int[2]=MappingMX(position->z);
     //ROS_INFO("J2ok");
     //jointState.position[3] = theta[3]*PI/180;
     theta_int[3]=MappingMX(orientation->x);
     //ROS_INFO("J3ok");
     //jointState.position[4] = theta[4]*PI/180;
     theta_int[4]=MappingMX(orientation->y);
     //ROS_INFO("J4ok");
     //jointState.position[5] = theta[5]*PI/180;
     theta_int[5]=MappingAX(orientation->z);
     //ROS_INFO("J5ok");; 
     theta_int[6]=MappingAX(gripreboot->x);

     theta_int[7]=gripreboot->y;
     
     for(int i=0;i<8;i++)
     {
        printf("theta_int[%d]: %d  \n",i, theta_int[i]);
     }
     
     std_msgs::Int16MultiArray retour_angle;
     
     
     
     retour_angle.data={theta_int[0],theta_int[1],theta_int[2],theta_int[3],theta_int[4],theta_int[5], theta_int[6], theta_int[7]};
     //printf("%f",theta[0]);
     pub_joints.publish(jointState);
     pub_angle.publish(retour_angle);
     
     r.sleep();
  }
  
  return 0;
}
 
bool CalcIk(vector* position, vector* orientation, float anc_theta[])
{


// Création du publisher 
//ros::NodeHandle n;
// 
// Calculs
   float px, py, pz;
   float rx, ry, rz;
   float ax, ay, az, bx, by, bz;
   float asx, asy, asz, bsx, bsy, bsz;
   float p5x, p5y, p5z;
   float C1, C23, S1, S23,C2;
   float A, B, N, M;        
   
	//Position de l'organe terminal
    px = position->x;
    py = position->y;
    pz = position->z;
           
    rx = -orientation->x * M_PI / 180.0f;
    ry = -orientation->y * M_PI / 180.0f; // angle en x rotation en x dans unity
    rz = orientation->z * M_PI / 180.0f; // -angle en y

       /**********************************************
         * projection du vecteur directeur de l'orientation
         * 
         * Les angles rx,ry,rz sont en degrés
         * ry rotation autour de l'axe y en premier
         * rz rotation autour z
         */
		 
        ax = -sin(rz);
        ay = cos(rx) * cos(rz);
        az = cos(rz) * sin(rx);

        bx = cos(ry) * cos(rz);
        by = sin(rx) * sin(ry) + cos(rx) * cos(ry) * sin(rz);
        bz = cos(ry) * sin(rx) * sin(rz) - cos(rx) * sin(ry);

        az = -az;
        bz = -bz;


        /**********************************************
         * 
         * On part de l'extrémité (px,py,pz) et on calcule la projection restante
        */
        p5x = px - (L5 + L6) * ax;
        p5y = py - (L5 + L6) * ay;
        p5z = pz - (L5 + L6) * az;

        theta[0] = atan2(p5x, p5z);
        A = pow(p5x * p5x + p5z * p5z, 0.5f);
        B = p5y - L1;
        C2 = (pow(A, 2) + pow(B, 2) - pow(L2, 2) - pow(L3 + L4, 2))/(2 * L2 * (L3 + L4));


        theta[2] = atan2(pow(1 - pow(C2, 2), 0.5f), C2);

        M = L2 + (L3 + L4) * C2;
        N = (L3 + L4) * sin(theta[2]);

        theta[1] = atan2(M * A - N * B, N * A + M * B);

        C1 = cos(theta[0]);
        C23 = cos(theta[1] + theta[2]);
        S1 = sin(theta[0]);
        S23 = sin(theta[1] + theta[2]);
        
        asx = ax * C1 - az * S1; ;
        asy = ay * C23 + az * C1 * S23 + ax * S1 * S23;
        asz = az * C1 * C23 - ay * S23 + ax * C23 * S1;


        bsx = bx * C1 - bz * S1;
        bsy = by * C23 + bz * C1 * S23 + bx * S1 * S23;
        bsz = bz * C1 * C23 - by * S23 + bx * C23 * S1;


        theta[3]= atan2(asx, asz);
        theta[4] = atan2(sin(theta[3]) * asx + cos(theta[3]) * asz, asy);
        theta[5] = atan2(bsy , (bsx * cos(theta[3]) - bsz * sin(theta[3])) *sin(theta[4])); 
        bool NOK = false;
        for (int i=0;i<6;i++) //!!! L'idée ici c'est juste de vérifier que toutes les coords sont ok pas juste la dernière -> trouver un équivalent au |= du c#
      /*  {
            NOK =isnan(theta[i]);
  
        }
        
        	*/
        for (int i=0;i<6;i++)
        {
        	bool NOK_int= false;
        	NOK_int =isnan(theta[i]);
        	NOK=NOK_int||NOK;
        	
        }
   
        if (!NOK)
            for (int i = 0; i < 6; i++)
            {
                theta[i] *= 180.0f / M_PI;
                anc_theta[i] = theta[i];
	      //printf("\nModèle géométrique OK :\n");
	      //for (int i = 0; i < 6; i++)
	      //printf("theta[%d] : %f\n", i, theta[i]);
            }
        else
     	{
            for (int i = 0; i < 6; i++)
                theta[i] = anc_theta[i];
	  //printf("\nModèle géométrique PAS OK -> Valeurs anciennes conservées :\n");
	  for (int i = 0; i < 6; i++)
	  {
	  	//printf("theta[%d] : %f\n", i, theta[i]);
        }
	return NOK;
	}
} 
