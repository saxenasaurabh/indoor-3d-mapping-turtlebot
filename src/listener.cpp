#include "listener.h"
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Scalar.h>
#include <math.h>

void transformToKinectFrame(double arr[4])
{
//   double temp;
  
//   double angle = atan(arr[1]/arr[0]);
//   angle *= 3.14159/1.8;
//   double dist = sqrt(arr[1]*arr[1]+arr[0]*arr[0]);
//   double pi = 3.14159;
//   double angle = 0.9;
//   double tempy = arr[1]/sin(angle);
//   double tempx = arr[0]-arr[1]/tan(angle);
//   
//   arr[0] = tempx;
//   arr[1] = tempy;
  
//   double temp = arr[0];
//   arr[0] = arr[1]; //correct
//   arr[1] = -arr[2];
//   arr[2] = -temp;
}

bool get_current_orientation (double arr[4])
{
  char **argv=0;
  int argc=0;
  ros::init(argc, argv, "listener");
  tf::TransformListener listener;
  tf::StampedTransform transform;
  std::string source_frame = "/world";
  std::string target_frame = "/turtlebot";
  try{
    ros::Time now = ros::Time::now();
    listener.waitForTransform(source_frame, target_frame,
                              now, ros::Duration(2.0));
    listener.lookupTransform(source_frame, target_frame,  
			      ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
//     ROS_ERROR("%s",ex.what());
//     ros::Duration(0.2).sleep();
    return false;
  }
  tf::Quaternion qt = transform.getRotation();
  tf::Matrix3x3 rotationMat(qt), rotationMat1;
//   rotationMat1 = rotationMat.inverse();
  tf::Vector3 origin = transform.getOrigin();
  tfScalar roll, pitch, yaw;
  rotationMat.getRPY(roll, pitch, yaw);
  for(int i=0;i<3;i++)
  {
    arr[i] = origin[i];
  }
  arr[3] = yaw*3.14159/1.8;
  transformToKinectFrame(arr);
//   transform.getOpenGLMatrix(arr);
//   printf("Yaw: %f\n", o.yaw);
  return true;
}