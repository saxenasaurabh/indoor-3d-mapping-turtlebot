/**
 * How to run:
 * ./main -minps 150 -nordw 0.5 -disth 0.09 -iter 100 -nerad 0.11 -mincs 200 -ctol 0.07 -histcomp 3 -icpiter 0 -leaf 0.04 -lferad 0.1 -saciter 50 -sacmnsd 0.08 -prefix ./datasets/eelab1/sample -start 3 -end 10  -step 1 -mlsrad 0.1 -mlspol 0 -planeth 5 -zlimit 3.0 -coeffwt 0.5 -anchors 1
 * ./main -minps 100 -nordw 0.6 -disth 0.09 -iter 100 -nerad 0.11 -mincs 200 -ctol 0.07 -histcomp 3 -icpiter 50 -leaf 0.04 -lferad 0.1 -saciter 50 -sacmnsd 0.08 -prefix ./datasets/cselab1/sample -start 13 -end 14  -step 1 -mlsrad 0.1 -mlspol 0 -planeth 0.15 -zlimit 3.0 -anchors 1 -maxdist 5.0
 * ./main -minps 400 -nordw 0.6 -disth 0.09 -iter 100 -nerad 0.11 -mincs 200 -ctol 0.07 -histcomp 3 -icpiter 50 -leaf 0.04 -lferad 0.1 -saciter 50 -sacmnsd 0.08 -prefix ./datasets/cselab1/sample -start 13 -end 14  -step 1 -mlsrad 0.1 -mlspol 0 -zlimit 3.0 -anchors 1
 * ./main -minps 120 -nordw 0.6 -disth 0.07 -iter 100 -nerad 0.1 -mincs 200 -ctol 0.07 -histcomp 3 -icpiter 50 -leaf 0.04 -prefix ./datasets/3DObject/sample -start 5 -end 6  -step 1 -visualize
*/
#include "pairwiseRegistration.h"

char extension[] = ".pcd";
string prefix;
int startIndex, endIndex, psize, step = 1;
bool showLines = false;

void parseArguments(int argc, char* argv[], RegistrationUsingClusters &reg)
{ 
  int temp;
  float tempf;
  if(pcl::console::find_argument(argc, argv, "-prefix")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-prefix", prefix);
  }
  else
  {
    pcl::console::print_error ("No prefix specified\n");
    exit(0);
  }
  
  if(pcl::console::find_argument(argc, argv, "-start")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-start", startIndex);
  }
  else
  {
    pcl::console::print_error ("No start specified\n");
    exit(0);
  }
  
  if(pcl::console::find_argument(argc, argv, "-end")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-end", endIndex);
  }
  else
  {
    pcl::console::print_error ("No end specified\n");
    exit(0);
  }
  
  if(pcl::console::find_argument(argc, argv, "-step")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-step", step);
  }
  
  if(pcl::console::find_argument(argc, argv, "-minps")!=-1) // Min plane size
  {
    pcl::console::parse_argument (argc, argv, "-minps", temp);
    reg.setMinPlaneSize(temp);
  }
  
  if(pcl::console::find_argument(argc, argv, "-mincs")!=-1) // Min cluster size
  {
    pcl::console::parse_argument (argc, argv, "-mincs", temp);
    reg.setMinClusterSize(temp);
  }
  
  if(pcl::console::find_argument(argc, argv, "-nordw")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-nordw", tempf);
    reg.setNormalDistanceWeight(tempf);
  }
  
  if(pcl::console::find_argument(argc, argv, "-disth")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-disth", tempf);
    reg.setDistanceThreshold(tempf);
  }
  
  if(pcl::console::find_argument(argc, argv, "-iter")!=-1) 
  {
    pcl::console::parse_argument (argc, argv, "-iter", temp);
    reg.setMaxRANSACIterations(temp);
  }
  
  if(pcl::console::find_argument(argc, argv, "-nerad")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-nerad", tempf);
    reg.setNormalEstimationRadius(tempf);
  }
  
  if(pcl::console::find_argument(argc, argv, "-ctol")!=-1) 
  {
    pcl::console::parse_argument (argc, argv, "-ctol", tempf);
    reg.setClusterTolerance(tempf);
  }
  
  if(pcl::console::find_argument(argc, argv, "-maxcs")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-maxcs", temp);
    reg.setMaxClusterSize(temp);
  }
  
  if(pcl::console::find_argument(argc, argv, "-histcomp")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-histcomp", temp);
    if(!reg.isValidHistComparisonMethod(temp))
    {
      printf("Invalid value for -histcomp...Using default value\n");
    }
    else
    {
      reg.setHistComparisonMethod(temp);
    }
  }
  
  if(pcl::console::find_argument(argc, argv, "-icpiter")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-icpiter", temp);
    reg.setMaxICPIterations(temp);
  }
  
  if(pcl::console::find_argument(argc, argv, "-psize")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-psize", temp);
    reg.setPointSize(temp);
    psize = temp;
  }
  
  if(pcl::console::find_argument(argc, argv, "-leaf")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-leaf", tempf);
    reg.setLeafSize(tempf);
  }
  
  if(pcl::console::find_argument(argc, argv, "-visualize")!=-1)
  {
    reg.setVisualize(true);
  }
  
  if(pcl::console::find_argument(argc, argv, "-lferad")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-lferad", tempf);
    reg.setLocalFeatureRadius(tempf);
  }
  
  if(pcl::console::find_argument(argc, argv, "-saciter")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-saciter", temp);
    reg.setSACIAIterations(temp);
  }
  
  if(pcl::console::find_argument(argc, argv, "-sacmnsd")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-sacmnsd", tempf);
    reg.setSACIAMinSampleDistance(tempf);
  }
  
  if(pcl::console::find_argument(argc, argv, "-anchors")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-anchors", temp);
    reg.setNumAnchors(temp);
  }
  
  if(pcl::console::find_argument(argc, argv, "-zlimit")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-zlimit", tempf);
    reg.setZLimit(tempf);
  }
  
  if(pcl::console::find_argument(argc, argv, "-mlsrad")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-mlsrad", tempf);
    reg.setMLSSearchRadius(tempf);
  }
  
  if(pcl::console::find_argument(argc, argv, "-mlspol")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-mlspol", temp);
    reg.setMLSPolynomialFitFlag(tempf);
  }
  
  if(pcl::console::find_argument(argc, argv, "-planeth")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-planeth", tempf);
    reg.setPlaneMatchThreshold(tempf);
  }
  
  if(pcl::console::find_argument(argc, argv, "-clusterth")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-clusterth", tempf);
    reg.setClusterMatchThreshold(tempf);
  }
  
  if(pcl::console::find_argument(argc, argv, "-coeffwt")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-coeffwt", tempf);
    reg.setWeightOfCoeff(tempf);
  }
  
  if(pcl::console::find_argument(argc, argv, "-maxdist")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-maxdist", tempf);
    reg.setMaxDistForFitnessScore(tempf);
  }
  
  if(pcl::console::find_argument(argc, argv, "-saccycles")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-saccycles", temp);
    reg.setSACCycles(temp);
  }
  
  if(pcl::console::find_argument(argc, argv, "-saccw")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-saccw", tempf);
    reg.setSACWeightOfConstraints(tempf);
  }
  
  if(pcl::console::find_argument(argc, argv, "-ksearch")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-ksearch", temp);
    reg.setKSearch(temp);
    reg.setKSearchFlag(true);
  }
  
  if(pcl::console::find_argument(argc, argv, "-deltar")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-deltar", temp);
    reg.setDeltaRed(temp);
  }
  
  if(pcl::console::find_argument(argc, argv, "-deltag")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-deltag", temp);
    reg.setDeltaGreen(temp);
  }
  
  if(pcl::console::find_argument(argc, argv, "-deltab")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-deltab", temp);
    reg.setDeltaBlue(temp);
  }
  
  if(pcl::console::find_argument(argc, argv, "-kdrad")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-kdrad", tempf);
    reg.setKDTreeSearchRadius(tempf);
  }
  
  if(pcl::console::find_argument(argc, argv, "-showlines")!=-1)
  {
    showLines = true;
  }
  
  if(pcl::console::find_argument(argc, argv, "-useplanes")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-useplanes", temp);
    if(temp)
      reg.setUsePlanes(true);
    else
      reg.setUsePlanes(false);
  }
  
  if(pcl::console::find_argument(argc, argv, "-useclusters")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-useclusters", temp);
    if(temp)
      reg.setUseClusters(true);
    else
      reg.setUseClusters(false);
  }
  
  if(pcl::console::find_argument(argc, argv, "-fswad")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-fswad", tempf);
    reg.setWeightOfAvgDists(tempf);
  }
  
  if(pcl::console::find_argument(argc, argv, "-mls")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-mls", temp);
    if(temp)
      reg.setMLSFlag(true);
    else
      reg.setMLSFlag(false);
  }
}

int main(int argc, char* argv[])
{
  RegistrationUsingClusters reg;
  parseArguments(argc, argv, reg);
  char filename[100];
  sensor_msgs::PointCloud2 cl;
  CloudPtr sourceCloud (new Cloud);
  CloudPtr targetCloud (new Cloud);
  
  sprintf(filename, "%s%d%s", prefix.c_str(), startIndex, extension);
  if(loadPCDFile (filename, cl)<0)
  {
    printf("Unable to load %s\n", filename);
    return 0;
  }  
  
  fromROSMsg (cl, *sourceCloud);
  reg.setSeedCloud(sourceCloud);
  printf("Added %s\n",filename);
  int cur = startIndex+step;
  while(cur<=endIndex)
  {
//     CloudPtr registered = reg.getRegisteredCloud();
//     reg.setSeedCloud(registered);
    sprintf(filename, "%s%d%s", prefix.c_str(), cur, extension);
    if(loadPCDFile (filename, cl)<0)
    {
      printf("Unable to load %s\n", filename);
      cur+=step;
      continue;
    }  
    fromROSMsg (cl, *sourceCloud);
    reg.add(sourceCloud);
    printf("Added %s\n",filename);
    cur+=step;
  }
  CloudPtr registered = reg.getRegisteredCloud();
  pcl::visualization::PCLVisualizer viewer ("Registered");
  pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb(registered);
  viewer.addPointCloud<PointType> (registered, rgb, "Registered");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize, "Registered");
  sprintf(filename, "%s_%d-%d-%d%s",prefix.c_str(), startIndex, endIndex, step, extension);
  pcl::io::savePCDFileASCII (filename, *registered);
  
  if(showLines)
  {
    pcl::PointXYZ sourcePivot(reg.sourcePivot(0,0),reg.sourcePivot(1,0),reg.sourcePivot(2,0));
    pcl::PointXYZ targetPivot(reg.targetPivot[0],reg.targetPivot[1],reg.targetPivot[2]);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pivots(new pcl::PointCloud<pcl::PointXYZ>);
    pivots->push_back(sourcePivot);
    pivots->push_back(targetPivot);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(pivots, 255, 0, 0);
    viewer.addPointCloud<pcl::PointXYZ> (pivots, red, "Pivots");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "Pivots");
    
    float factor = 2;
    pcl::PointXYZ p1(reg.sourcePivot(0,0)-factor*reg.sourceLine1(3,0), reg.sourcePivot(1,0)-factor*reg.sourceLine1(4,0), reg.sourcePivot(2,0)-factor*reg.sourceLine1(5,0)), p2(reg.sourcePivot(0,0)+factor*reg.sourceLine1(3,0), reg.sourcePivot(1,0)+factor*reg.sourceLine1(4,0), reg.sourcePivot(2,0)+factor*reg.sourceLine1(5,0));
    viewer.addLine<pcl::PointXYZ> (p1, p2, "sourceLine1");
    pcl::PointXYZ p3(reg.sourcePivot(0,0)-factor*reg.sourceLine2(3,0), reg.sourcePivot(1,0)-factor*reg.sourceLine2(4,0), reg.sourcePivot(2,0)-factor*reg.sourceLine2(5,0)), p4(reg.sourcePivot(0,0)+factor*reg.sourceLine2(3,0), reg.sourcePivot(1,0)+factor*reg.sourceLine2(4,0), reg.sourcePivot(2,0)+factor*reg.sourceLine2(5,0));
    viewer.addLine<pcl::PointXYZ> (p3, p4, "sourceLine2");
    pcl::PointXYZ p5(reg.targetPivot(0,0)-factor*reg.targetLine1(3,0), reg.targetPivot(1,0)-factor*reg.targetLine1(4,0), reg.targetPivot(2,0)-factor*reg.targetLine1(5,0)), p6(reg.targetPivot(0,0)+factor*reg.targetLine1(3,0), reg.targetPivot(1,0)+factor*reg.targetLine1(4,0), reg.targetPivot(2,0)+factor*reg.targetLine1(5,0));
    viewer.addLine<pcl::PointXYZ> (p5, p6, "targetLine1");
    pcl::PointXYZ p7(reg.targetPivot(0,0)-factor*reg.targetLine2(3,0), reg.targetPivot(1,0)-factor*reg.targetLine2(4,0), reg.targetPivot(2,0)-factor*reg.targetLine2(5,0)), p8(reg.targetPivot(0,0)+factor*reg.targetLine2(3,0), reg.targetPivot(1,0)+factor*reg.targetLine2(4,0), reg.targetPivot(2,0)+factor*reg.targetLine2(5,0));
    viewer.addLine<pcl::PointXYZ> (p7, p8, "targetLine2");
    
    int line_width = 10;
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,line_width,"sourceLine1");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,255,0,"sourceLine1");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,line_width,"targetLine1");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,255,"targetLine1");
    
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,line_width,"sourceLine2");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,255,0,"sourceLine2");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,line_width,"targetLine2");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,255,"targetLine2");
  }
  
  viewer.spin();
}