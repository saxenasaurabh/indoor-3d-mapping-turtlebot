/**
 * Openni Grabber with added feature of using odometry feedback
 * Clouds are transformed into the frame of original cloud using odom feedback
 * Controls: Press 'c' in the grabber window for capturing current cloud
 * How to run: executable-name <prefix> <startIndex> [-useodom 0/1 default:0]
 */

#include "listener.h"
#include <pcl/common/transforms.h>
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloudType;

PointCloudType current_cloud;
pcl::visualization::CloudViewer viewer("Viewer");
int count = 0, start = 0;
bool locked = false;
char prefix[50];
// Eigen::Matrix3f rotation1, rotation2;
double yaw1, yaw2;
Eigen::Vector3f planar1, planar2;
bool useodom = false;
// rotation1 = Eigen::Matrix3f::Identity();
// rotation2 = Eigen::Matrix3f::Identity();

void getRotationMatrix (double yaw, Eigen::Matrix3f & mat)
{
  // Rotate by yaw about y-axis
  mat(1,1) = 1;
  mat(0,1) = mat(2,1) = mat(1,0) = mat(1,2) = 0;
  mat(0,0) = cos(yaw); mat(0,2) = sin(yaw);
  mat(2,0) = -sin(yaw); mat(2,2) = cos(yaw);
}

void getTransformation (Eigen::Matrix4f &tr)
{
  tr = Eigen::Matrix4f::Identity();
  Eigen::Matrix3f relative;
  getRotationMatrix(-yaw2, relative);
  for(int i=0;i<3;i++)
  {
    for(int j=0;j<3;j++)
    {
      tr(i,j) = relative(i,j);
    }
  }

  // planar
  double pi = 3.14159;
  Eigen::Vector3f temp = planar2;
  double len = temp.norm();
  double angle = atan(temp(1)/temp(0));
  
  // Handle cases for 2nd and 3rd quadrant
  
  angle = angle*pi/1.8;
  if(temp(0)<0)
  {
    if(temp(1)>=0)
    {
      printf("3rd quad\n");
      angle+=pi;
    }
    else
    {
      printf("2nd quad\n");
      angle-=pi;
    }
  }
  
  
  cout << "Temp: " << temp << endl;
  
//   angle+=yaw1;
  printf("Angle: %f\n", angle);
  
  Eigen::Matrix4f tr1 = Eigen::Matrix4f::Identity();
  Eigen::Vector3f vec;
  vec << -len*sin(angle), 0.0, len*cos(angle);
//   vec << 0.0, 0.0, len;
  
//   Eigen::Matrix3f rot = Eigen::Matrix3f::Identity();
//   getRotationMatrix(-yaw2, rot);
//   vec = rot*vec;
  
  tr(0,3) = vec(0);
  tr(1,3) = vec(1);
  tr(2,3) = vec(2);
  
//   tr = tr*tr1;
  
  printf("Translate: %f %f %f\n", tr(0,3), tr(1,3), tr(2,3));
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* p)
{
  boost::shared_ptr<pair<pcl::visualization::CloudViewer*, pcl::Grabber*> > p1 = *static_cast<boost::shared_ptr<pair<pcl::visualization::CloudViewer*, pcl::Grabber*> > *> (p);
  if (event.getKeySym () == "c" && event.keyDown ())
  {
    if(p1->second->isRunning())
    {
      p1->second->stop();
    }
    string fileName = prefix;
    char temp[10];
    if(useodom)
    {
      double arr[4];
      while(!get_current_orientation(arr))
      {
	
      }
      for(int i=0;i<4;i++)
      {
	printf("%lf ",arr[i]);
      }
      printf("\n");
      yaw1 = yaw2;
      yaw2 = arr[3];
      for(int i=0;i<3;i++)
      {
	planar1(i) = planar2(i);
	planar2(i) = arr[i];
      }
      fileName = prefix;
      Eigen::Matrix4f transformation;
      getTransformation(transformation);
      PointCloudType tempCloud;
      pcl::transformPointCloud (current_cloud, tempCloud, transformation);
      sprintf(temp,"%d_tr",::count);
      fileName+=temp;
      fileName+=".pcd";
      std::cout << "Capturing frame " << fileName << std::endl;
      pcl::io::savePCDFileASCII (fileName, tempCloud);
    }
    fileName = prefix;
    sprintf(temp,"%d",::count);
    fileName+=temp;
    fileName+=".pcd";
    std::cout << "Capturing frame " << fileName << std::endl;
    pcl::io::savePCDFileASCII (fileName, current_cloud);

    ::count++;
    
    p1->second->start();
  }
}

class SimpleOpenNIProcessor
{
public:
  void cloud_cb_ (const PointCloudType::ConstPtr &cloud)
  {
    current_cloud = *cloud;
    viewer.showCloud(cloud);
  }
  
  void run ()
  {
    // create a new grabber for OpenNI devices
    pcl::Grabber* interface = new pcl::OpenNIGrabber();
    boost::shared_ptr<pair<pcl::visualization::CloudViewer*, pcl::Grabber*> > p(new pair<pcl::visualization::CloudViewer*, pcl::Grabber*>(&viewer, interface));
    viewer.registerKeyboardCallback(keyboardEventOccurred, (void*)&p);
    
    // make callback function from member function
    boost::function<void (const PointCloudType::ConstPtr&)> f =
      boost::bind (&SimpleOpenNIProcessor::cloud_cb_, this, _1);

    // connect callback function for desired signal. In this case its a point cloud with color values
    boost::signals2::connection c = interface->registerCallback (f);

    // start receiving point clouds
    interface->start ();
    
//     viewer.spin();

    // wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
    while (!viewer.wasStopped())
      boost::this_thread::sleep (boost::posix_time::seconds (1));

    // stop the grabber
    interface->stop ();
  }
};

int main (int argc, char* argv[])
{
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  SimpleOpenNIProcessor v;
  if(argc<3)
  {
    printf("Usage: %s <prefix> <startIndex> [-useodom 0/1 default:0]\n", argv[0]);
    exit(0);
  }
  if(pcl::console::find_argument(argc, argv, "-useodom")!=-1)
  {
    int temp;
    pcl::console::parse_argument (argc, argv, "-useodom", temp);
    if(temp)
      useodom = true;
    else
      useodom = false;
  }
  sprintf(prefix,"%s",argv[1]);
  sscanf(argv[2], "%d", &::count);
  ::start = ::count;
  v.run ();
  return (0);
}
