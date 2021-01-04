//For The Gazebo Simulation//

//For the Time Sync ->> rosparam set use_sim_time true

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <eigen3/Eigen/Eigen>

#define TURTLE_NUM  3

static double dist = 2.0;
//Global Variables
double Turtlepos[TURTLE_NUM][2];
double* Turtle0pos = new double[2];
double* Turtle1pos = new double[2];
double* Turtle2pos = new double[2];
double* Turtle3pos = new double[2];
double* Turtle4pos = new double[2];

double* prelinearv = new double[TURTLE_NUM];
double* dot_linearv= new double[TURTLE_NUM];
double precontrol[TURTLE_NUM][4];
double dot_control[TURTLE_NUM][4];

double* preangular = new double[TURTLE_NUM];
double* preangle   = new double[TURTLE_NUM];

//double dix[TURTLE_NUM]      = {0,     -dist,  0,  dist,   0};
//double diy[TURTLE_NUM]      = {dist,  0,      0,  0,      -dist};

double dix[TURTLE_NUM]      = {-dist/2,   0,        dist/2};
double diy[TURTLE_NUM]      = {0,         dist*0.866,  0     };

double* dij        = new double[TURTLE_NUM];

double quaterion[TURTLE_NUM][4];
double sq_quater[TURTLE_NUM][4];

void positionCallback0(const nav_msgs::Odometry::ConstPtr& msg)
{  
  // Turtle Bot Position
  Turtlepos[0][0] = msg->pose.pose.position.x;
  Turtlepos[0][1] = msg->pose.pose.position.y;
  // Turtle Bot Quaterion
  quaterion[0][0] = msg->pose.pose.orientation.x;
  quaterion[0][1] = msg->pose.pose.orientation.y;
  quaterion[0][2] = msg->pose.pose.orientation.z;
  quaterion[0][3] = msg->pose.pose.orientation.w;
  // Quaterion 2 Eular Yaw(Z_Transform)
  for(int i=0; i<4; i++){
    sq_quater[0][i] = quaterion[0][i]*quaterion[0][i];
  }
  preangle[0] = (double)(atan2f(2.0*(quaterion[0][3]*quaterion[0][2] - quaterion[0][0]*quaterion[0][1]),
      -sq_quater[0][0]+sq_quater[0][1]-sq_quater[0][2]+sq_quater[0][3]));
}

void positionCallback1(const nav_msgs::Odometry::ConstPtr& msg)
{
  // Turtle Bot Position
  Turtlepos[1][0] = msg->pose.pose.position.x;
  Turtlepos[1][1] = msg->pose.pose.position.y;
  // Turtle Bot Quaterion
  quaterion[1][0] = msg->pose.pose.orientation.x;
  quaterion[1][1] = msg->pose.pose.orientation.y;
  quaterion[1][2] = msg->pose.pose.orientation.z;
  quaterion[1][3] = msg->pose.pose.orientation.w;
  // Quaterion 2 Eular Yaw(Z_Transform)
  for(int i=0; i<4; i++){
    sq_quater[1][i] = quaterion[1][i]*quaterion[1][i];
  }
  preangle[1] = (double)(atan2f(2.0*(quaterion[1][3]*quaterion[1][2] - quaterion[1][0]*quaterion[1][1]),
      -sq_quater[1][0]+sq_quater[1][1]-sq_quater[1][2]+sq_quater[1][3]));
}

void positionCallback2(const nav_msgs::Odometry::ConstPtr& msg)
{
  // Turtle Bot Position
  Turtlepos[2][0] = msg->pose.pose.position.x;
  Turtlepos[2][1] = msg->pose.pose.position.y;
  // Turtle Bot Quaterion
  quaterion[2][0] = msg->pose.pose.orientation.x;
  quaterion[2][1] = msg->pose.pose.orientation.y;
  quaterion[2][2] = msg->pose.pose.orientation.z;
  quaterion[2][3] = msg->pose.pose.orientation.w;
  // Quaterion 2 Eular Yaw(Z_Transform)
  for(int i=0; i<4; i++){
    sq_quater[2][i] = quaterion[2][i]*quaterion[2][i];
  }
  preangle[2] = (double)(atan2f(2.0*(quaterion[2][3]*quaterion[2][2] - quaterion[2][0]*quaterion[2][1]),
      -sq_quater[2][0]+sq_quater[2][1]-sq_quater[2][2]+sq_quater[2][3]));
}

//void positionCallback3(const nav_msgs::Odometry::ConstPtr& msg)
//{
//  // Turtle Bot Position
//  Turtlepos[3][0] = msg->pose.pose.position.x;
//  Turtlepos[3][1] = msg->pose.pose.position.y;
//  // Turtle Bot Quaterion
//  quaterion[3][0] = msg->pose.pose.orientation.x;
//  quaterion[3][1] = msg->pose.pose.orientation.y;
//  quaterion[3][2] = msg->pose.pose.orientation.z;
//  quaterion[3][3] = msg->pose.pose.orientation.w;
//  // Quaterion 2 Eular Yaw(Z_Transform)
//  for(int i=0; i<4; i++){
//    sq_quater[3][i] = quaterion[3][i]*quaterion[3][i];
//  }
//  preangle[3] = (double)(atan2f(2.0*(quaterion[3][3]*quaterion[3][2] - quaterion[3][0]*quaterion[3][1]),
//      -sq_quater[3][0]+sq_quater[3][1]-sq_quater[3][2]+sq_quater[3][3]));
//}

//void positionCallback4(const nav_msgs::Odometry::ConstPtr& msg)
//{
//  // Turtle Bot Position
//  Turtlepos[4][0] = msg->pose.pose.position.x;
//  Turtlepos[4][1] = msg->pose.pose.position.y;
//  // Turtle Bot Quaterion
//  quaterion[4][0] = msg->pose.pose.orientation.x;
//  quaterion[4][1] = msg->pose.pose.orientation.y;
//  quaterion[4][2] = msg->pose.pose.orientation.z;
//  quaterion[4][3] = msg->pose.pose.orientation.w;
//  // Quaterion 2 Eular Yaw(Z_Transform)
//  for(int i=0; i<4; i++){
//    sq_quater[4][i] = quaterion[4][i]*quaterion[4][i];
//  }
//  preangle[4] = (double)(atan2f(2.0*(quaterion[4][3]*quaterion[4][2] - quaterion[4][0]*quaterion[4][1]),
//      -sq_quater[4][0]+sq_quater[4][1]-sq_quater[4][2]+sq_quater[4][3]));
//}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "Main");
  ros::NodeHandle nh;

  ros::Subscriber sub0 = nh.subscribe("/tb3_0/odom", 6, positionCallback0);
  ros::Subscriber sub1 = nh.subscribe("/tb3_1/odom", 6, positionCallback1);
  ros::Subscriber sub2 = nh.subscribe("/tb3_2/odom", 6, positionCallback2);
//  ros::Subscriber sub3 = nh.subscribe("/tb3_3/odom", 6, positionCallback3);
//  ros::Subscriber sub4 = nh.subscribe("/tb3_4/odom", 6, positionCallback4);

  ros::Publisher pub0 = nh.advertise<geometry_msgs::Twist>("/tb3_0/cmd_vel",3);
  ros::Publisher pub1 = nh.advertise<geometry_msgs::Twist>("/tb3_1/cmd_vel",3);
  ros::Publisher pub2 = nh.advertise<geometry_msgs::Twist>("/tb3_2/cmd_vel",3);
//  ros::Publisher pub3 = nh.advertise<geometry_msgs::Twist>("/tb3_3/cmd_vel",3);
//  ros::Publisher pub4 = nh.advertise<geometry_msgs::Twist>("/tb3_4/cmd_vel",3);

  geometry_msgs::Twist msg0;
  geometry_msgs::Twist msg1;
  geometry_msgs::Twist msg2;
//  geometry_msgs::Twist msg3;
//  geometry_msgs::Twist msg4;

  prelinearv[0] = 0.01;
  prelinearv[1] = 0.01;
  prelinearv[2] = 0.01;
//  prelinearv[3] = 1;
//  prelinearv[4] = 1;

  ros::Rate rate(100); //10ms
  //Controller
  Eigen::MatrixXf Cont0(4,1);
  Eigen::MatrixXf Cont1(4,1);
  Eigen::MatrixXf Cont2(4,1);
//  Eigen::MatrixXf Cont3(4,1);
//  Eigen::MatrixXf Cont4(4,1);
  Cont0<<0,0,0,0;
  Cont1<<0,0,0,0;
  Cont2<<0,0,0,0;
//  Cont3<<0,0,0,0;
//  Cont4<<0,0,0,0;

  //Dot_Controller
  Eigen::MatrixXf Dot_Cont0(4,1);
  Eigen::MatrixXf Dot_Cont1(4,1);
  Eigen::MatrixXf Dot_Cont2(4,1);
//  Eigen::MatrixXf Dot_Cont3(4,1);
//  Eigen::MatrixXf Dot_Cont4(4,1);

  //Controller_A_Gain (A+KC-BB`P)
  Eigen::MatrixXf A2(4,4);
/* p = [-0.1 -0.3+-0.5i -0.5] L = [2 -1 -1; -1 2 -1; -1 -1 2] eps = 0.21 */
A2<<-0.6313,  1.0,        0.1026,   0,
      -0.4011,  -0.4964,  0.1580,   0,
      -0.0707,   0,        -0.5687,  1,
      0.1060,   0,        -0.5118,  -0.4964;
  //K_Gain
  Eigen::MatrixXf K(4,2);
/* p = [-0.1 -0.3+-0.5i -0.5] L = [2 -1 -1; -1 2 -1; -1 -1 2] eps = 0.21 */
K<< -0.6313,  0.1026,
    -0.1365,  0.1580,
    -0.0707,  -0.5687,
    0.1060,   -0.2472;
  //Signal From i robot & j's robot
  Eigen::MatrixXf Si(2,1);
  Eigen::MatrixXf Sj1(2,1);
  Eigen::MatrixXf Sj2(2,1);

  //Output(Dot v, Dot theta)
  Eigen::MatrixXf Out0(2,1);
  Eigen::MatrixXf Out1(2,1);
  Eigen::MatrixXf Out2(2,1);
//  Eigen::MatrixXf Out3(2,1);
//  Eigen::MatrixXf Out4(2,1);
  //InverseU_include vi
  Eigen::MatrixXf U_conv0(2,2);
  Eigen::MatrixXf U_conv1(2,2);
  Eigen::MatrixXf U_conv2(2,2);
//  Eigen::MatrixXf U_conv3(2,2);
//  Eigen::MatrixXf U_conv4(2,2);
  //Transpose of B
  Eigen::MatrixXf B_T(2,4);
  B_T<<0,1,0,0,
      0,0,0,1;
  //P_Gain
  Eigen::MatrixXf P(4,4);
/* p = [-0.1 -0.3+-0.5i -0.5] L = [2 -1 -1; -1 2 -1; -1 -1 2] eps = 0.21 */
P<<0.3940,  0.2646, 0,      0,
   0.2646,  0.4964, 0,      0,
   0,       0,      0.3940, 0.2646,
   0,       0,      0.2646, 0.4964;
while(ros::ok()){
    // Si : Signal From i robot
    Si<<(Turtlepos[0][0]-dix[0]),(Turtlepos[0][1]-diy[0]);
    // Sj1 : Signal From j1 robot
    Sj1<<(Turtlepos[1][0]-dix[1]),(Turtlepos[1][1]-diy[1]);
    // Sj2 : Signal From j2 robot
    Sj2<<(Turtlepos[2][0]-dix[2]),(Turtlepos[2][1]-diy[2]);
    Dot_Cont0 = A2*Cont0 +K*((Si-Sj1)+(Si-Sj2));
    Cont0 = Cont0 + Dot_Cont0 * 0.01;
    //ROS_INFO("00 Si1 - Sj1: %f %f ## Si1 - Sj2: %f %f",(Si-Sj1)(0),(Si-Sj1)(1),(Si-Sj2)(0),(Si-Sj2)(1));

    // Sj1 : Signal From j1 robot
    Sj1<<(Turtlepos[0][0]-dix[0]),(Turtlepos[0][1]-diy[0]);
    // Si : Signal From i robot
    Si<<(Turtlepos[1][0]-dix[1]),(Turtlepos[1][1]-diy[1]);
    // Sj2 : Signal From j2 robot
    Sj2<<(Turtlepos[2][0]-dix[2]),(Turtlepos[2][1]-diy[2]);
    Dot_Cont1 = A2*Cont1 +K*(1*(Si-Sj1)+1*(Si-Sj2));
    Cont1 = Cont1 + Dot_Cont1 * 0.01;
    //ROS_INFO("11 Si1 - Sj1: %f %f ## Si1 - Sj2: %f %f",(Si-Sj1)(0),(Si-Sj1)(1),(Si-Sj2)(0),(Si-Sj2)(1));

    // Sj1 : Signal From j1 robot
    Sj1<<(Turtlepos[0][0]-dix[0]),(Turtlepos[0][1]-diy[0]);
    // Sj2 : Signal From j2 robot
    Sj2<<(Turtlepos[1][0]-dix[1]),(Turtlepos[1][1]-diy[1]);
    // Si : Signal From i robot
    Si<<(Turtlepos[2][0]-dix[2]),(Turtlepos[2][1]-diy[2]);
    Dot_Cont2 = A2*Cont2 +K*(1*(Si-Sj1)+1*(Si-Sj2));
    Cont2 = Cont2 + Dot_Cont2*0.01;
    //ROS_INFO("11 Si1 - Sj1: %f %f ## Si1 - Sj2: %f %f",(Si-Sj1)(0),(Si-Sj1)(1),(Si-Sj2)(0),(Si-Sj2)(1));
    ROS_INFO("Angle 0:%f 1:%f 2:%f",preangle[0],preangle[1],preangle[2]);
    U_conv0<< cos(preangle[0]), sin(preangle[0]), -sin(preangle[0])/prelinearv[0], cos(preangle[0])/prelinearv[0];
    U_conv1<< cos(preangle[1]), sin(preangle[1]), -sin(preangle[1])/prelinearv[1], cos(preangle[1])/prelinearv[1];
    U_conv2<< cos(preangle[2]), sin(preangle[2]), -sin(preangle[2])/prelinearv[2], cos(preangle[2])/prelinearv[2];

    Out0=U_conv0*B_T*P*Cont0;
    Out1=U_conv1*B_T*P*Cont1;
    Out2=U_conv2*B_T*P*Cont2;

    prelinearv[0]+=Out0(0)*0.01;
    prelinearv[1]+=Out1(0)*0.01;
    prelinearv[2]+=Out2(0)*0.01;

    for (int i=0;i<TURTLE_NUM;i++) {
      if (prelinearv[i]>0 && prelinearv[i]<0.01) {
        prelinearv[i] = 0.01;
      }
      else if(prelinearv[i]<0 && prelinearv[i]>-0.01){
        prelinearv[i] = -0.01;
      }
     if (prelinearv[i]>0.1){
        prelinearv[i] = 0.1;
      }
      else if (prelinearv[i]<-0.1){
        prelinearv[i] = -0.1;
      }
      if (Out0(1)>5.0) Out0(1) =5.0;
      else if (Out0(1)<-5.0) Out0(1) =-5.0;
      if (Out1(1)>5.0) Out1(1) =5.0;
      else if (Out1(1)<-5.0) Out1(1) =-5.0;
      if (Out2(1)>5.0) Out2(1) =5.0;
      else if (Out2(1)<-5.0) Out2(1) =-5.0;
    }
    ROS_INFO("Angular 0 : %f 1: %f 2: %f",Out0(1),Out1(1),Out2(1));
    msg0.linear.x = prelinearv[0];
    msg1.linear.x = prelinearv[1];
    msg2.linear.x = prelinearv[2];
    msg0.angular.z = Out0(1);
    msg1.angular.z = Out1(1);
    msg2.angular.z = Out2(1);

    pub0.publish(msg0);
    pub1.publish(msg1);
    pub2.publish(msg2);
    // Print Distance BTW TurtleBot
//    ROS_INFO("0.1 : %f ", sqrt(pow(Turtlepos[0][0]-Turtlepos[1][0],2)+pow(Turtlepos[0][1]-Turtlepos[1][1],2)));
//    ROS_INFO("1.2 : %f ", sqrt(pow(Turtlepos[2][0]-Turtlepos[1][0],2)+pow(Turtlepos[2][1]-Turtlepos[1][1],2)));
//    ROS_INFO("0.2 : %f ", sqrt(pow(Turtlepos[0][0]-Turtlepos[2][0],2)+pow(Turtlepos[0][1]-Turtlepos[2][1],2)));

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}


