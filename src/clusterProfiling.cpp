/**
 * ./clusterProfiling down_sample7.pcd down_sample8.pcd -minps 1000 -nordw 0.3 -disth 0.06 -iter 100 -nerad 0.1 -mincs 200 -ctol 0.04
 * ./clusterProfiling down_sample7.pcd down_sample8.pcd -minps 120 -nordw 0.4 -disth 0.07 -iter 100 -nerad 0.1 -mincs 200 -ctol 0.05
 * -minps 	: minimum points size
 * -nordw 	: NORMAL_DISTANCE_WEIGHT for segmentation
 * -disth 	: distance threshold for plane estimation using ransac 
 * -iter  	: no. of ransac iterations
 * -nerad 	: parameter for setRadiusSearch for normal estimation
 * -mincs 	: minimum cluster size
 * -maxcs 	: maximum cluster size
 * -ctol  	: cluster tolerance
 * -leaf  	: leaf size for scaleDown, if not specified the input cloud is not downsampled
 * -onlymerged	: show only merged histogram
 * -zlimit	: zlimit for passthrough filter
 * 
 * The program takes as input a pair of point clouds and
 * a) extracts planes from them of size >= "-minps" using RANSAC
 * b) extracts other clusters of size >= "-mincs" using euclidean clustering
 * c) provides a visualization of the various planes+clusters,
 * the original cloud and the residual cloud of both the
 * source and target frames in multiple viewports of a 
 * single window. Also are shown histograms for VFH and
 * RGB channels for the currently visible cloud in the 
 * window.
 * 
 * Window controls:
 * 'a': for advancing cloud in source(left) viewport
 * 's': for advancing cloud in target(target) viewport
 */

#include <stdio.h>
#include <pcl/io/pcd_io.h>
#include <vector>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>
#include <pcl/features/vfh.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/filters/voxel_grid.h>

#define HSIZE 256 // RGB histogram size
#define VFH_SIZE 180 // Only first 180 bytes of VFH are being considered as we dont want viewpoint dependence
#define RGB_NORMALIZE_SCALE 8
#define VFH_NORMALIZE_SCALE 2
#define DESCRIPTOR_HSIZE (3*(HSIZE/RGB_NORMALIZE_SCALE)+VFH_SIZE/VFH_NORMALIZE_SCALE)
#define planeText "Cloud-%d Plane-%d"
#define clusterText "Cloud-%d Cluster-%d"

using namespace std;
using namespace pcl;
using namespace pcl::io;

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef pcl::PointCloud<pcl::Normal> NormalsCloud;
typedef NormalsCloud::Ptr NormalsCloudPtr;
typedef pcl::PointCloud<pcl::VFHSignature308> VFHSig;
typedef VFHSig::Ptr VFHSigPtr;
typedef pcl::Histogram<HSIZE> HistPointType;
typedef pcl::PointCloud<HistPointType> HistCloud;
typedef HistCloud::Ptr HistCloudPtr;
typedef pcl::Histogram<DESCRIPTOR_HSIZE> DescriptorHistPointType;
typedef pcl::PointCloud<DescriptorHistPointType> DescriptorHistCloud;
typedef DescriptorHistCloud::Ptr DescriptorHistCloudPtr;

int MAX_ITERATIONS = 100, K_SEARCH = 50, MIN_PLANE_SIZE = 200, MIN_CLUSTER_SIZE = 200, MAX_CLUSTER_SIZE = 25000;
float DISTANCE_THRESHOLD = 0.06, NORMAL_DISTANCE_WEIGHT = 0.1, NE_RADIUS = 0.05, CLUSTER_TOLERANCE = 0.02;
float error=0.1, LEAF_SIZE=0.04, Z_LIMIT=4.0;
char name[40], text[50];
int totalLeft, totalRight, curLeft, curRight, planesLeft, planesRight;
vector<CloudPtr> sourceClusters, targetClusters;
vector<NormalsCloudPtr> sourceClusterNormals, targetClusterNormals;
vector<VFHSigPtr> sourceVFHSigs, targetVFHSigs;
CloudPtr source(new Cloud), target(new Cloud);
NormalsCloudPtr sourceNormals(new NormalsCloud), targetNormals(new NormalsCloud);
pcl::visualization::PCLHistogramVisualizer sourceVFHViewer, targetVFHViewer, sourceRHistViewer, sourceGHistViewer, sourceBHistViewer, targetRHistViewer, targetBHistViewer, targetGHistViewer, sourceMergedHistViewer, targetMergedHistViewer; 
vector<HistCloudPtr> sourceRHist, targetRHist, sourceGHist, targetGHist, sourceBHist, targetBHist;
vector<DescriptorHistCloudPtr> sourceDescriptors, targetDescriptors;
bool showOnlyMergedHist = false;

void filter(const CloudPtr& src, CloudPtr& tgt)
{
  boost::shared_ptr<std::vector<int> > indices(new std::vector<int>);
  pcl::removeNaNFromPointCloud(*src, *tgt, *indices);
  pcl::PassThrough<PointType> pass;
  pass.setInputCloud (tgt);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, Z_LIMIT);
  pass.filter (*tgt);
}

void scaleDown(const CloudPtr &source, CloudPtr &target)
{
  target->clear();
  pcl::VoxelGrid<PointType> sor;
  sor.setInputCloud (source);
  sor.setLeafSize (LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
  sor.filter (*target);
}

void computeNormals(CloudPtr cloud, NormalsCloudPtr normalsCloud)
{
  pcl::NormalEstimation<PointType, pcl::Normal> ne;
  pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud);
//   ne.setKSearch (K_SEARCH);
  ne.setRadiusSearch(NE_RADIUS);
  ne.compute (*normalsCloud);
}

void computeRGBHist(CloudPtr cloud, HistCloudPtr RHist, HistCloudPtr GHist, HistCloudPtr BHist)
{
  RHist->clear();
  GHist->clear();
  BHist->clear();
  HistPointType R, G, B;
  memset(R.histogram, 0, sizeof(R));
  memset(G.histogram, 0, sizeof(G));
  memset(B.histogram, 0, sizeof(B));
  
  for(int i=0;i<cloud->points.size();i++)
  {
    Eigen::Vector3i rgb = cloud->at(i).getRGBVector3i();
    R.histogram[rgb[0]]++;
    G.histogram[rgb[1]]++;
    B.histogram[rgb[2]]++;
  }
  for(int i=0;i<HSIZE;i++)
  {
    R.histogram[i] = R.histogram[i]/cloud->points.size();
    G.histogram[i] = G.histogram[i]/cloud->points.size();
    B.histogram[i] = B.histogram[i]/cloud->points.size();
  }
  RHist->push_back(R);
  GHist->push_back(G);
  BHist->push_back(B);
}

void computeVFHSig(CloudPtr cloud, NormalsCloudPtr normals, VFHSigPtr vfhs)
{
  pcl::VFHEstimation<PointType, pcl::Normal, pcl::VFHSignature308> vfh;
  pcl::search::KdTree<PointType>::Ptr tree1 (new pcl::search::KdTree<PointType> ());
  vfh.setSearchMethod (tree1);
  vfh.setInputCloud (cloud);
  vfh.setInputNormals (normals);
  vfh.compute (*vfhs);
}

void merge(const vector<VFHSigPtr> &VFHSigs, const vector<HistCloudPtr> &RHist, const vector<HistCloudPtr> &GHist, const vector<HistCloudPtr> &BHist, vector<DescriptorHistCloudPtr> &DescriptorHist)
{
  int size = VFHSigs.size();
  for(int i=0;i<size;i++)
  {
    DescriptorHistCloudPtr merged(new DescriptorHistCloud);
    DescriptorHistPointType point;
    memset(point.histogram, 0, sizeof(point.histogram));
    int offset = 0, bins = VFH_SIZE, factor = VFH_NORMALIZE_SCALE;
    for(int j=0;j<bins;j++)
    {
      point.histogram[offset+j/factor]+=VFHSigs[i]->at(0).histogram[j];
    }
    offset+=bins/factor;
    bins = HSIZE;
    factor = RGB_NORMALIZE_SCALE;
    for(int j=0;j<bins;j++)
    {
      point.histogram[offset+j/factor]+=RHist[i]->at(0).histogram[j];
    }
    offset+=bins/factor;
    for(int j=0;j<bins;j++)
    {
      point.histogram[offset+j/factor]+=GHist[i]->at(0).histogram[j];
    }
    offset+=bins/factor;
    for(int j=0;j<bins;j++)
    {
      point.histogram[offset+j/factor]+=BHist[i]->at(0).histogram[j];
    }
    merged->push_back(point);
    DescriptorHist.push_back(merged);
  }
}

void extractClustersEuclidean(CloudPtr cloud, vector<pcl::PointIndices> &clusterIndices)
{
  pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
  tree->setInputCloud (cloud);

  pcl::EuclideanClusterExtraction<PointType> ec;
  ec.setClusterTolerance (CLUSTER_TOLERANCE);
  ec.setMinClusterSize (MIN_CLUSTER_SIZE);
  ec.setMaxClusterSize (MAX_CLUSTER_SIZE);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (clusterIndices);
}

int extractClusters (CloudPtr cloud, NormalsCloudPtr normalsCloud, vector<CloudPtr> &clusters, vector<NormalsCloudPtr> &clusterNormals, vector<VFHSigPtr> &vfhsig, vector<HistCloudPtr> &RHistVect,  vector<HistCloudPtr> &GHistVect, vector<HistCloudPtr> &BHistVect)
{
  int planeCount = 0;
  // EXTRACTING PLANES
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ExtractIndices<PointType> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  
  // Create the segmentation object
  pcl::SACSegmentationFromNormals<PointType, pcl::Normal> seg;

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (NORMAL_DISTANCE_WEIGHT);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (MAX_ITERATIONS);
  seg.setDistanceThreshold (DISTANCE_THRESHOLD);
  
  do{
    seg.setInputCloud (cloud);
    seg.setInputNormals (normalsCloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
      break;
    }

    int coeffCount = coefficients->values.size();
//     cerr<< "Coefficients: "<< coeffCount<<endl;
//     for(int i=0;i<coeffCount;i++)
//     {
//       cerr << coefficients->values[i] << std::endl;
//     }
    cerr << "Model inliers: " << inliers->indices.size () << endl;

    CloudPtr newCloud(new Cloud);
    NormalsCloudPtr normalsCloud1 (new pcl::PointCloud<pcl::Normal>);
    CloudPtr plane(new Cloud);
    NormalsCloudPtr plane_normals (new pcl::PointCloud<pcl::Normal>);
    
    // Extract the planar inliers from the input cloud
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract_normals.setInputCloud (normalsCloud);
    extract_normals.setIndices (inliers);
    
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*newCloud);
    extract_normals.setNegative (true);
    extract_normals.filter (*normalsCloud1);
    
    if(inliers->indices.size()>MIN_PLANE_SIZE)
    {
      extract.setNegative (false);
      extract.filter(*plane);
      extract_normals.setNegative(false);
      extract_normals.filter(*plane_normals);
      clusters.push_back(plane);
      clusterNormals.push_back(plane_normals);
      
      // Compute VFH Signature for Plane
      VFHSigPtr vfhs (new VFHSig ());
      computeVFHSig(plane, plane_normals, vfhs);
      vfhsig.push_back(vfhs);
      
      // Compute RGB Signature for Plane
      HistCloudPtr R(new HistCloud), G(new HistCloud), B(new HistCloud);
      computeRGBHist(plane, R, G, B);
      RHistVect.push_back(R);
      GHistVect.push_back(G);
      BHistVect.push_back(B);
      
      cloud = newCloud;
      normalsCloud = normalsCloud1;
      planeCount++;
    }
    else
      break;
  }while(cloud->points.size()>MIN_PLANE_SIZE);
  
  // EXTRACTING OTHER CLUSTERS BY EUCLIDEAN CLUSTERING
  
  std::vector<pcl::PointIndices> clusterIndices;
  extractClustersEuclidean(cloud, clusterIndices);
  printf("%d clusters found\n", clusterIndices.size());
  
  for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it)
  {
    if((*it).indices.size()<MIN_CLUSTER_SIZE)
      continue;
    printf("%d points in cloud_cluster\n", (*it).indices.size());
    fflush(stdout);
    CloudPtr cloud_cluster (new Cloud);
    NormalsCloudPtr cloud_cluster_normals (new NormalsCloud);
    pcl::PointIndices::Ptr temp(new pcl::PointIndices);
//     for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
//       cloud_cluster->points.push_back (cloud->points[*pit]); //*
//     cloud_cluster->width = cloud_cluster->points.size ();
//     cloud_cluster->height = 1;
//     cloud_cluster->is_dense = true;
    temp->indices.insert(temp->indices.end(), (*it).indices.begin(), (*it).indices.end());
    extract.setInputCloud (cloud);
    extract.setIndices (temp);
    extract_normals.setInputCloud (normalsCloud);
    extract_normals.setIndices (temp);
    extract.setNegative (false);
    extract.filter(*cloud_cluster);
    extract_normals.setNegative(false);
    extract_normals.filter(*cloud_cluster_normals);
    clusters.push_back(cloud_cluster);
    clusterNormals.push_back(cloud_cluster_normals);
    
    // Compute VFH Signature for Cluster
    VFHSigPtr vfhs (new VFHSig ());
    computeVFHSig(cloud_cluster, cloud_cluster_normals, vfhs);
    vfhsig.push_back(vfhs);
    
    // Compute RGB Signature for Cluster
    HistCloudPtr R(new HistCloud), G(new HistCloud), B(new HistCloud);
    computeRGBHist(cloud_cluster, R, G, B);
    RHistVect.push_back(R);
    GHistVect.push_back(G);
    BHistVect.push_back(B);
  }
  
  pcl::PointIndices::Ptr allClusterIndices(new pcl::PointIndices);
  for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it)
  {
    allClusterIndices->indices.insert(allClusterIndices->indices.end(), (*it).indices.begin(), (*it).indices.end());
  }
  CloudPtr newCloud(new Cloud);
  NormalsCloudPtr normalsCloud1 (new pcl::PointCloud<pcl::Normal>);
  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud);
  extract.setIndices (allClusterIndices);
  extract_normals.setInputCloud (normalsCloud);
  extract_normals.setIndices (allClusterIndices);
  
  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*newCloud);
  extract_normals.setNegative (true);
  extract_normals.filter (*normalsCloud1);
  
  cloud = newCloud;
  normalsCloud = normalsCloud1;
  
  if(cloud->points.size()!=0)
  {
    clusters.push_back(cloud);
    clusterNormals.push_back(normalsCloud);
    VFHSigPtr vfhs (new VFHSig ());
    computeVFHSig(cloud, normalsCloud, vfhs);
    vfhsig.push_back(vfhs);
    
    HistCloudPtr R(new HistCloud), G(new HistCloud), B(new HistCloud);
    computeRGBHist(cloud, R, G, B);
    RHistVect.push_back(R);
    GHistVect.push_back(G);
    BHistVect.push_back(B);
  }
  return planeCount;
}

void keyboard_cb (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
  pcl::visualization::PCLVisualizer viewer = *static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  if (event.getKeySym () == "a" && event.keyDown ())
  {
    std::cout << "a was pressed" << std::endl;
    curLeft = (curLeft+1)%(totalLeft);
    pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb1(sourceClusters[curLeft]);
    viewer.updatePointCloud<PointType> (sourceClusters[curLeft], rgb1, "Source Cloud");
    sourceVFHViewer.updateFeatureHistogram<pcl::VFHSignature308>(*sourceVFHSigs[curLeft], HSIZE, "VFH Source");
    sourceRHistViewer.updateFeatureHistogram<HistPointType>(*sourceRHist[curLeft], HSIZE, "R Source");
    sourceGHistViewer.updateFeatureHistogram<HistPointType>(*sourceGHist[curLeft], HSIZE, "G Source");
    sourceBHistViewer.updateFeatureHistogram<HistPointType>(*sourceBHist[curLeft], HSIZE, "B Source");
    sourceMergedHistViewer.updateFeatureHistogram<DescriptorHistPointType>(*sourceDescriptors[curLeft], DESCRIPTOR_HSIZE, "EFPFHRGB Source");
    if(curLeft<totalLeft-2)
    {
      char text[50];
      if(curLeft<planesLeft)
      {
	sprintf(text, planeText, 1, curLeft);
      }
      else
      {
	sprintf(text, clusterText, 1, curLeft);
      }
      viewer.updateText(text, 10, 10, "v1 text");
    }
    else if(curLeft==totalLeft-2)
    {
      char text[50];
      sprintf(text, "Cloud-1 Residual");
      viewer.updateText(text, 10, 10, "v1 text");
    }
    else
    {
      char text[50];
      sprintf(text, "Cloud-1");
      viewer.updateText(text, 10, 10, "v1 text");
    }
  }
  else if(event.getKeySym () == "s" && event.keyDown ())
  {
    std::cout << "s was pressed" << std::endl;
    curRight = (curRight+1)%(totalRight);
    pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb2(targetClusters[curRight]);
    viewer.updatePointCloud<PointType> (targetClusters[curRight], rgb2, "Target Cloud");
    targetVFHViewer.updateFeatureHistogram<pcl::VFHSignature308>(*targetVFHSigs[curRight], HSIZE, "VFH Target");
    targetRHistViewer.updateFeatureHistogram<HistPointType>(*targetRHist[curRight], HSIZE, "R Target");
    targetGHistViewer.updateFeatureHistogram<HistPointType>(*targetGHist[curRight], HSIZE, "G Target");
    targetBHistViewer.updateFeatureHistogram<HistPointType>(*targetBHist[curRight], HSIZE, "B Target");
    targetMergedHistViewer.updateFeatureHistogram<DescriptorHistPointType>(*targetDescriptors[curRight], DESCRIPTOR_HSIZE, "EFPFHRGB Target");
    if(curRight<totalRight-2)
    {
      char text[50];
      if(curRight<planesRight)
      {
	sprintf(text, planeText, 2, curRight);
      }
      else
      {
	sprintf(text, clusterText, 2, curRight);
      }
      viewer.updateText(text, 10, 10, "v2 text");
    }
    else if(curRight==totalRight-2)
    {
      char text[50];
      sprintf(text, "Cloud-2 Residual", curRight);
      viewer.updateText(text, 10, 10, "v2 text");
    }    
    else
    {
      char text[50];
      sprintf(text, "Cloud-2", curRight);
      viewer.updateText(text, 10, 10, "v2 text");
    }
  }
}

int main(int argc, char* argv[])
{
  char filename[20];
  sensor_msgs::PointCloud2 cl;
  CloudPtr sourceCloud (new Cloud);
  CloudPtr targetCloud (new Cloud);
  pcl::visualization::PCLVisualizer viewer ("View Clusters");
  bool scaleDownFlag = false;
  
  if(argc<3)
  {
    printf("%s <sourceFilename> <targetFilename> \
    [-minps MIN_POINTS_IN_CLUSTER] \
    [-nordw NORMAL_DISTANCE_WEIGHT] \
    [-disth DISTANCE_THRESHOLD] \
    [-iter MAX_ITERATIONS]\n", argv[0]);
    return 0;
  }
  
  if(loadPCDFile (argv[1], cl)<0)
  {
    printf("Unable to load %s\n", argv[1]);
    return 0;
  }
  fromROSMsg (cl, *sourceCloud);
  if(loadPCDFile (argv[2], cl)<0)
  {
    printf("Unable to load %s\n", argv[2]);
    return 0;
  }
  fromROSMsg (cl, *targetCloud);
  
  if(pcl::console::find_argument(argc, argv, "-minps")!=-1) // Min cluster size
  {
    pcl::console::parse_argument (argc, argv, "-minps", MIN_PLANE_SIZE);
  }
  
  if(pcl::console::find_argument(argc, argv, "-mincs")!=-1) // Min cluster size
  {
    pcl::console::parse_argument (argc, argv, "-mincs", MIN_CLUSTER_SIZE);
  }
  else
  {
    MIN_CLUSTER_SIZE = MIN_PLANE_SIZE;
  }
  
  if(pcl::console::find_argument(argc, argv, "-nordw")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-nordw", NORMAL_DISTANCE_WEIGHT);
  }
  
  if(pcl::console::find_argument(argc, argv, "-disth")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-disth", DISTANCE_THRESHOLD);
  }
  
  if(pcl::console::find_argument(argc, argv, "-iter")!=-1) 
  {
    pcl::console::parse_argument (argc, argv, "-iter", MAX_ITERATIONS);
  }
  
  if(pcl::console::find_argument(argc, argv, "-nerad")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-nerad", NE_RADIUS);
  }
  
  if(pcl::console::find_argument(argc, argv, "-ctol")!=-1) 
  {
    pcl::console::parse_argument (argc, argv, "-ctol", CLUSTER_TOLERANCE);
  }
  
  if(pcl::console::find_argument(argc, argv, "-maxcs")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-maxcs", MAX_CLUSTER_SIZE);
  }
  
  if(pcl::console::find_argument(argc, argv, "-leaf")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-leaf", LEAF_SIZE);
    scaleDownFlag = true;
  }
  
  if(pcl::console::find_argument(argc, argv, "-onlymerged")!=-1)
  {
    showOnlyMergedHist = true;
  }
  
  if(pcl::console::find_argument(argc, argv, "-zlimit")!=-1)
  {
    pcl::console::parse_argument (argc, argv, "-zlimit", Z_LIMIT);
  }
  
  // Build a passthrough filter to remove spurious NaNs
//   pcl::PassThrough<PointType> pass;
//   pass.setInputCloud (sourceCloud);
//   pass.setFilterFieldName ("z");
//   pass.setFilterLimits (0, 1.5);
//   pass.filter (*source);
//   
//   pass.setInputCloud (targetCloud);
//   pass.setFilterFieldName ("z");
//   pass.setFilterLimits (0, 1.5);
//   pass.filter (*target);

  CloudPtr sourceTemp(new Cloud), targetTemp(new Cloud);
  filter(sourceCloud, sourceTemp);
  filter(targetCloud, targetTemp);
  if(scaleDownFlag)
  {
    scaleDown(sourceTemp, source);
    scaleDown(targetTemp, target);
  }
  else
  {
    source = sourceTemp->makeShared();
    target = targetTemp->makeShared();
  }
  computeNormals(source, sourceNormals);
  computeNormals(target, targetNormals);
  planesLeft = extractClusters(source->makeShared(), sourceNormals->makeShared(), sourceClusters, sourceClusterNormals, sourceVFHSigs, sourceRHist, sourceGHist, sourceBHist);
  planesRight = extractClusters(target->makeShared(), targetNormals->makeShared(), targetClusters, targetClusterNormals, targetVFHSigs, targetRHist, targetGHist, targetBHist);
  // VFH Hist
  VFHSigPtr vfhs(new VFHSig());
  computeVFHSig(source, sourceNormals, vfhs);
  sourceVFHSigs.push_back(vfhs);
  VFHSigPtr vfhs1(new VFHSig());
  computeVFHSig(target, targetNormals, vfhs1);
  targetVFHSigs.push_back(vfhs1);
  // RGB Hist
  HistCloudPtr R(new HistCloud), G(new HistCloud), B(new HistCloud);
  computeRGBHist(source, R, G, B);
  sourceRHist.push_back(R);
  sourceGHist.push_back(G);
  sourceBHist.push_back(B);
  
  HistCloudPtr R1(new HistCloud), G1(new HistCloud), B1(new HistCloud);
  computeRGBHist(target, R1, G1, B1);
  targetRHist.push_back(R1);
  targetGHist.push_back(G1);
  targetBHist.push_back(B1);
  
  // Merged Hist
  merge(sourceVFHSigs, sourceRHist, sourceGHist, sourceBHist, sourceDescriptors);
  merge(targetVFHSigs, targetRHist, targetGHist, targetBHist, targetDescriptors);
  // Clusters
  sourceClusters.push_back(source->makeShared());
  sourceClusterNormals.push_back(sourceNormals->makeShared());
  targetClusters.push_back(target->makeShared());
  targetClusterNormals.push_back(targetNormals->makeShared());
  
  curLeft = curRight = 0;
  totalLeft = sourceClusters.size();
  totalRight = targetClusters.size();
  
  int v1(0);
  viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer.setBackgroundColor (0, 0, 0, v1);
  if(curLeft<planesLeft)
  {
    sprintf(text, planeText, 1, curLeft);
  }
  else
  {
    sprintf(text, clusterText, 1, curLeft);
  }
  viewer.addText(text, 10, 10, "v1 text", v1);
  pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb1(sourceClusters[curLeft]);
  viewer.addPointCloud<PointType> (sourceClusters[curLeft], rgb1, "Source Cloud", v1);
  
  int v2(0);
  viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer.setBackgroundColor (0, 0, 0, v2);
  if(curRight<planesRight)
  {
    sprintf(text, planeText, 2, curRight);
  }
  else
  {
    sprintf(text, clusterText, 2, curRight);
  }
  viewer.addText(text, 10, 10, "v2 text", v2);
  pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb2(targetClusters[curRight]);
  viewer.addPointCloud<PointType> (targetClusters[curRight], rgb2, "Target Cloud", v2);

  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Source Cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Target Cloud");
//   viewer.addCoordinateSystem (1.0);
  
  viewer.registerKeyboardCallback(keyboard_cb, (void*)&viewer);
  
  if(!showOnlyMergedHist)
  {
    sourceVFHViewer.addFeatureHistogram<pcl::VFHSignature308>(*sourceVFHSigs[curLeft], HSIZE, "VFH Source");
    targetVFHViewer.addFeatureHistogram<pcl::VFHSignature308>(*targetVFHSigs[curRight], HSIZE, "VFH Target");

    sourceRHistViewer.addFeatureHistogram<HistPointType>(*sourceRHist[curLeft], HSIZE, "R Source");
    sourceGHistViewer.addFeatureHistogram<HistPointType>(*sourceGHist[curLeft], HSIZE, "G Source");
    sourceBHistViewer.addFeatureHistogram<HistPointType>(*sourceBHist[curLeft], HSIZE, "B Source");

    targetRHistViewer.addFeatureHistogram<HistPointType>(*targetRHist[curRight], HSIZE, "R Target");
    targetGHistViewer.addFeatureHistogram<HistPointType>(*targetGHist[curRight], HSIZE, "G Target");
    targetBHistViewer.addFeatureHistogram<HistPointType>(*targetBHist[curRight], HSIZE, "B Target");
  }
  sourceMergedHistViewer.setGlobalYRange(0.0,0.3);
  sourceMergedHistViewer.addFeatureHistogram<DescriptorHistPointType>(*sourceDescriptors[curLeft], DESCRIPTOR_HSIZE, "EFPFHRGB Source");
  targetMergedHistViewer.addFeatureHistogram<DescriptorHistPointType>(*targetDescriptors[curRight], DESCRIPTOR_HSIZE, "EFPFHRGB Target");
  viewer.spin();
}