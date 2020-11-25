#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#define TURTLE_NUM  3

//Global Variables
double* Turtle0Pos =new double[2];
double* Turtle1Pos =new double[2];
double* Turtle2Pos =new double[2];
int flag = 0;
//double quaterion[TURTLE_NUM][4];
tf::Quaternion quarterion[TURTLE_NUM];
void cb(ar_track_alvar_msgs::AlvarMarkers req) {
       if (!req.markers.empty()) {

         tf::Transform tr_0; tf::Transform tr_1; tf::Transform tr_2; tf::Transform tr_3;
         tf::Quaternion q0, q1, q2, q3;
         flag = 0;
         for(int i = 0; i<4; i++){
           if(req.markers[i].id == 0){
              tr_0.setOrigin(tf::Vector3(req.markers[i].pose.pose.position.x,req.markers[i].pose.pose.position.y,req.markers[i].pose.pose.position.z));
              q0.setX(req.markers[i].pose.pose.orientation.x);
              q0.setY(req.markers[i].pose.pose.orientation.y);
              q0.setZ(req.markers[i].pose.pose.orientation.z);
              q0.setW(req.markers[i].pose.pose.orientation.w);
              tr_0.setRotation(q0);
              flag = 1;
           }
           if(req.markers[i].id == 1){
              tr_1.setOrigin(tf::Vector3(req.markers[i].pose.pose.position.x,req.markers[i].pose.pose.position.y,req.markers[i].pose.pose.position.z));
              q1.setX(req.markers[i].pose.pose.orientation.x);
              q1.setY(req.markers[i].pose.pose.orientation.y);
              q1.setZ(req.markers[i].pose.pose.orientation.z);
              q1.setW(req.markers[i].pose.pose.orientation.w);
              tr_1.setRotation(q1);
           }
           if(req.markers[i].id == 2){
              tr_2.setOrigin(tf::Vector3(req.markers[i].pose.pose.position.x,req.markers[i].pose.pose.position.y,req.markers[i].pose.pose.position.z));
              q2.setX(req.markers[i].pose.pose.orientation.x);
              q2.setY(req.markers[i].pose.pose.orientation.y);
              q2.setZ(req.markers[i].pose.pose.orientation.z);
              q2.setW(req.markers[i].pose.pose.orientation.w);
              tr_2.setRotation(q2);
           }
           if(req.markers[i].id == 3){
              tr_3.setOrigin(tf::Vector3(req.markers[i].pose.pose.position.x,req.markers[i].pose.pose.position.y,req.markers[i].pose.pose.position.z));
              q3.setX(req.markers[i].pose.pose.orientation.x);
              q3.setY(req.markers[i].pose.pose.orientation.y);
              q3.setZ(req.markers[i].pose.pose.orientation.z);
              q3.setW(req.markers[i].pose.pose.orientation.w);
              tr_3.setRotation(q3);
           }
         }
         if(flag == 1){
            for(int i =0; i<4; i++){
              if(req.markers[i].id == 1){
                 tr_1=tr_0.inverseTimes(tr_1);
              }
              if(req.markers[i].id == 2){
                 tr_2=tr_0.inverseTimes(tr_2);
              }
              if(req.markers[i].id == 3){
                 tr_3=tr_0.inverseTimes(tr_3);
              }
            }
         }
         flag = 0;
//         tr_0.setRotation(q0);
//         tr_1.setRotation(q1);
//         tr_2.setRotation(q2);
//         tr_3.setRotation(q3);

//         tr_1=tr_0.inverseTimes(tr_1);
//         tr_2=tr_0.inverseTimes(tr_2);
//         tr_3=tr_0.inverseTimes(tr_3);

         tf::Vector3 pos0,pos1,pos2;

         pos0 = tr_1.getOrigin();
         if(pos0.x()<10 &&pos0.x()>-10) {
           ROS_INFO("1x : %1.2f  1y : %1.2f  1z : %1.2f", pos0.x(), pos0.y(), pos0.z());
           Turtle0Pos[0]=pos0.x();
           Turtle0Pos[1]=pos0.y();
         }
         pos1 = tr_2.getOrigin();
         if(pos1.x()<10 &&pos1.x()>-10) {
           ROS_INFO("2x : %1.2f  2y : %1.2f  2z : %1.2f", pos1.x(), pos1.y(), pos1.z());
           Turtle1Pos[0]=pos1.x();
           Turtle1Pos[1]=pos1.y();
         }
         pos2 = tr_3.getOrigin();
         if(pos2.x()<10 &&pos2.x()>-10) {
           ROS_INFO("3x : %1.2f  3y : %1.2f  3z : %1.2f", pos2.x(), pos2.y(), pos2.z());
           Turtle2Pos[0]=pos2.x();
           Turtle2Pos[1]=pos2.y();
         }

         quarterion[0] = tr_0.getRotation();
         quarterion[1] = tr_1.getRotation();
         quarterion[2] = tr_2.getRotation();
         quarterion[3] = tr_3.getRotation();
//         tf::Matrix3x3 m0(q0);
//         tf::Matrix3x3 m1(q1);
//         tf::Matrix3x3 m2(q2);
//         tf::Matrix3x3 m3(q3);

         double roll, pitch, yaw;
//         m.getRPY(roll, pitch, yaw);
//         ROS_INFO("roll, pitch, yaw=%1.2f  %1.2f  %1.2f", roll, pitch, yaw);


       } // if
}

int main(int argc, char **argv) {
     ros::init(argc, argv, "TF_Transform");
     ros::NodeHandle nh;
     ros::Subscriber sub = nh.subscribe("ar_pose_marker", 1, cb);

     ros::Publisher pub0 =nh.advertise<nav_msgs::Odometry>("/tb3_0/odom",3);
     ros::Publisher pub1 =nh.advertise<nav_msgs::Odometry>("/tb3_1/odom",3);
     ros::Publisher pub2 =nh.advertise<nav_msgs::Odometry>("/tb3_2/odom",3);

     nav_msgs::Odometry Odom0;
     nav_msgs::Odometry Odom1;
     nav_msgs::Odometry Odom2;

     ros::Rate rate(500);
     while(ros::ok()){
       Odom0.pose.pose.position.x=Turtle0Pos[0];
       Odom0.pose.pose.position.y=Turtle0Pos[1];
       Odom1.pose.pose.position.x=Turtle1Pos[0];
       Odom1.pose.pose.position.y=Turtle1Pos[1];
       Odom2.pose.pose.position.x=Turtle2Pos[0];
       Odom2.pose.pose.position.y=Turtle2Pos[1];

       Odom0.pose.pose.orientation.x = quarterion[1].getX();
       Odom0.pose.pose.orientation.y = quarterion[1].getY();
       Odom0.pose.pose.orientation.z = quarterion[1].getZ();
       Odom0.pose.pose.orientation.w = quarterion[1].getW();

       Odom1.pose.pose.orientation.x = quarterion[2].getX();
       Odom1.pose.pose.orientation.y = quarterion[2].getY();
       Odom1.pose.pose.orientation.z = quarterion[2].getZ();
       Odom1.pose.pose.orientation.w = quarterion[2].getW();

       Odom2.pose.pose.orientation.x = quarterion[3].getX();
       Odom2.pose.pose.orientation.y = quarterion[3].getY();
       Odom2.pose.pose.orientation.z = quarterion[3].getZ();
       Odom2.pose.pose.orientation.w = quarterion[3].getW();
       pub0.publish(Odom0);
       pub1.publish(Odom1);
       pub2.publish(Odom2);

       ros::spinOnce();
       rate.sleep();
     }
//     ros::spin();
     return 0;

}
