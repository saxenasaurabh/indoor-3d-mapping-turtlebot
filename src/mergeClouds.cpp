#include "includes.h"

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::Ptr CloudPtr;

int main(int argc, char **argv)
{
  char filename[100];
  int numFiles = atoi(argv[1]);
  sensor_msgs::PointCloud2 cl;
  CloudPtr merged(new Cloud), cloud(new Cloud);
  for(int i=0;i<numFiles;i++)
  {
    if(loadPCDFile (argv[i+2], cl)<0)
    {
      printf("Unable to load %s\n", argv[i+2]);
    }
    fromROSMsg (cl, *cloud);
    *merged+=*cloud;
  }
  pcl::io::savePCDFileASCII (argv[numFiles+2], *merged);
  printf("Saving %s\n", argv[numFiles+2]);
}