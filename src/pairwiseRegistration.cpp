#include "pairwiseRegistration.h"

char RegistrationUsingClusters::HIST_COMPARISON_METHODS[4][50] = {"CORRELATION", "CHI SQUARE", "INTERSECTION", "BHATTACHARYA"};

// template<typename PointType>
void RegistrationUsingClusters::computeNormals(const CloudPtr &cloud, NormalsCloudPtr &normalsCloud)
{
  normalsCloud->clear();
  pcl::NormalEstimation<PointType, NormalPointType> ne;
  SearchTypePtr tree (new SearchType ());
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud);
//   ne.setKSearch (K_SEARCH);
  ne.setRadiusSearch(NE_RADIUS);
  ne.compute (*normalsCloud);
}

// template<typename PointType>
void RegistrationUsingClusters::computeRGBHist(const CloudPtr &cloud, HistCloudPtr &RHist, HistCloudPtr &GHist, HistCloudPtr &BHist)
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
  if(NORMALIZE_RGB)
  {
    for(int i=0;i<HSIZE;i++)
    {
      R.histogram[i] = R.histogram[i]/cloud->points.size();
      G.histogram[i] = G.histogram[i]/cloud->points.size();
      B.histogram[i] = B.histogram[i]/cloud->points.size();
    }
  }
  RHist->push_back(R);
  GHist->push_back(G);
  BHist->push_back(B);
}

// template<typename PointType>
void RegistrationUsingClusters::computeVFHSig(const CloudPtr &cloud, const NormalsCloudPtr &normals, VFHSigPtr &vfhs)
{
  vfhs->clear();
  pcl::VFHEstimation<PointType, NormalPointType, pcl::VFHSignature308> vfh;
  SearchTypePtr tree1 (new SearchType ());
  vfh.setSearchMethod (tree1);
  vfh.setInputCloud (cloud);
  vfh.setInputNormals (normals);
  vfh.compute (*vfhs);
  if(NORMALIZE_VFH)
  {
    float factor = cloud->size();
    for(int i=0;i<VFH_SIZE;i++)
    {
      vfhs->at(0).histogram[i]/=factor;
    }
  }
}

// template<typename PointType>
void RegistrationUsingClusters::extractClustersEuclidean(const CloudPtr &cloud, vector<pcl::PointIndices> &clusterIndices)
{
  clusterIndices.clear();
  SearchTypePtr tree (new SearchType);
  tree->setInputCloud (cloud);

  pcl::EuclideanClusterExtraction<PointType> ec;
  ec.setClusterTolerance (CLUSTER_TOLERANCE);
  ec.setMinClusterSize (MIN_CLUSTER_SIZE);
  ec.setMaxClusterSize (MAX_CLUSTER_SIZE);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (clusterIndices);
}

// template<typename PointType>
bool RegistrationUsingClusters::isValidHistComparisonMethod(int method)
{
  return method>=0 && method<4;
}

// template<typename PointType>
double RegistrationUsingClusters::getHistDistance(const DescriptorHistCloudPtr &hist1, const DescriptorHistCloudPtr &hist2)
{
  vector<float> v1(hist1->at(0).histogram, hist1->at(0).histogram+sizeof(hist1->at(0).histogram)/sizeof(float));
  vector<float> v2(hist2->at(0).histogram, hist2->at(0).histogram+sizeof(hist2->at(0).histogram)/sizeof(float));
  return cv::compareHist(v1, v2, HIST_COMPARISON_METHOD);
}

// template<typename PointType>
void RegistrationUsingClusters::process (const CloudPtr &cl, NormalsCloudPtr &normalsCloud, CloudPtr &residualCloud, NormalsCloudPtr &residualCloudNormals, vector<CloudPtr> &clusters, vector<NormalsCloudPtr> &clusterNormals, vector<VFHSigPtr> &vfhsig, vector<HistCloudPtr> &RHistVect,  vector<HistCloudPtr> &GHistVect, vector<HistCloudPtr> &BHistVect, vector<DescriptorHistCloudPtr> &descriptors, vector<PlaneCoeff, Eigen::aligned_allocator<PlaneCoeff> > &planeCoeffs, int &planeCount)
{
  CloudPtr cloud(cl);
  // Initialization
  normalsCloud->clear();
  residualCloud->clear();
  residualCloudNormals->clear();
  clusters.clear();
  clusterNormals.clear();
  vfhsig.clear();
  RHistVect.clear();
  GHistVect.clear();
  BHistVect.clear();
  descriptors.clear();
  planeCoeffs.clear();
  planeCount = 0;
  int tolerance = 0;
  
  // Generate Normals Cloud
  computeNormals(cloud, normalsCloud);
  // EXTRACTING PLANES
  
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ExtractIndices<PointType> extract;
  pcl::ExtractIndices<NormalPointType> extract_normals;
  
  // Create the segmentation object
  pcl::SACSegmentationFromNormals<PointType, NormalPointType> seg;

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (NORMAL_DISTANCE_WEIGHT);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (MAX_RANSAC_ITERATIONS);
  seg.setDistanceThreshold (DISTANCE_THRESHOLD);
    
  do{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    seg.setInputCloud (cloud);
    seg.setInputNormals (normalsCloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
      break;
    }

    int coeffCount = coefficients->values.size();
//     cerr << "Plane Model inliers: " << inliers->indices.size () << endl;

    CloudPtr newCloud(new Cloud);
    NormalsCloudPtr normalsCloud1 (new pcl::PointCloud<NormalPointType>);
    CloudPtr plane(new Cloud);
    NormalsCloudPtr plane_normals (new pcl::PointCloud<NormalPointType>);
   
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
      
//       Compute VFH Signature for Plane
      VFHSigPtr vfhs (new VFHSig ());
      computeVFHSig(plane, plane_normals, vfhs);
      vfhsig.push_back(vfhs);
      
//       Compute RGB Signature for Plane
      HistCloudPtr R(new HistCloud), G(new HistCloud), B(new HistCloud);
      computeRGBHist(plane, R, G, B);
      RHistVect.push_back(R);
      GHistVect.push_back(G);
      BHistVect.push_back(B);
      
//       cout << *coefficients << endl;
      Eigen::Vector4f temp(coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);
      planeCoeffs.push_back(temp);
      
      cloud = newCloud;
      normalsCloud = normalsCloud1;
      planeCount++;
      tolerance = 0;
    }
    else
    {
      if(tolerance<TOLERANCE)
      {
	tolerance++;
      }
      else
      {
	break;
      }
    }
  }while(cloud->points.size()>MIN_PLANE_SIZE);
  
  // EXTRACTING OTHER CLUSTERS BY EUCLIDEAN CLUSTERING
  
  std::vector<pcl::PointIndices> clusterIndices;
  extractClustersEuclidean(cloud, clusterIndices);
  
  for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it)
  {
    if((*it).indices.size()<MIN_CLUSTER_SIZE)
      continue;
    CloudPtr cloud_cluster (new Cloud);
    NormalsCloudPtr cloud_cluster_normals (new NormalsCloud);
    pcl::PointIndices::Ptr temp(new pcl::PointIndices);
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
  NormalsCloudPtr normalsCloud1 (new pcl::PointCloud<NormalPointType>);
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
  
  residualCloud = newCloud;
  residualCloudNormals = normalsCloud1;
  
  
//   if(cloud->points.size()!=0)
//   {
//     
//     clusters.push_back(cloud);
//     clusterNormals.push_back(normalsCloud);
//     VFHSigPtr vfhs (new VFHSig ());
//     computeVFHSig(cloud, normalsCloud, vfhs);
//     vfhsig.push_back(vfhs);
//     
//     HistCloudPtr R(new HistCloud), G(new HistCloud), B(new HistCloud);
//     computeRGBHist(cloud, R, G, B);
//     RHistVect.push_back(R);
//     GHistVect.push_back(G);
//     BHistVect.push_back(B);
//   }
  
  merge(vfhsig, RHistVect, GHistVect, BHistVect, descriptors);
}

/**
 * Generates merged descriptor by binning VFH and RGB descriptors
 */
// template<typename PointType>
void RegistrationUsingClusters::merge(const vector<VFHSigPtr> &VFHSigs, const vector<HistCloudPtr> &RHist, const vector<HistCloudPtr> &GHist, const vector<HistCloudPtr> &BHist, vector<DescriptorHistCloudPtr> &DescriptorHist)
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

// template<typename PointType>
bool RegistrationUsingClusters::getComparisonFlag()
{
  switch(HIST_COMPARISON_METHOD)
  {
    case 0: 
    case 2: return true;
    case 1:
    case 3: return false;
  }
}

double RegistrationUsingClusters::matchPlaneCoeff(const PlaneCoeff &c1, const PlaneCoeff& c2)
{
  double dist1 = 0.0, dist2 = 0.0;
  dist1 = (c1-c2).norm();
  dist2 = (c1+c2).norm();
  return min(dist1, dist2);
}

double RegistrationUsingClusters::matchPlanes(const PlaneCoeff &c1, const PlaneCoeff& c2, const DescriptorHistCloudPtr &hist1, const DescriptorHistCloudPtr &hist2)
{
  double distCoeff = matchPlaneCoeff(c1, c2);
  double distDesc = getHistDistance(hist1, hist2);
  double distance = WEIGHT_OF_COEFF*distCoeff + distDesc;
  return distance;
}

// template<typename PointType>
void RegistrationUsingClusters::generateCorrespondences()
{
  sourceCorrespondences.clear();
  targetCorrespondences.clear();
  vector<DescriptorHistCloudPtr>::iterator it1, it2;
  int i, j;
  CompareCorrespondence compObj(getComparisonFlag());
  CompareCorrespondence compPlane(compObj);
  for(it1 = sourceDescriptors.begin(), i = 0; it1!=sourceDescriptors.end(); ++it1, ++i)
  {
    vector<pcl::Correspondence> v;
    if(i<planesSource)
    {
      for(j=0;j<planesTarget;j++)
      {
	Correspondence c(i, j, matchPlanes(sourcePlaneCoeff[i], targetPlaneCoeff[j], *it1, targetDescriptors[j]));
	v.push_back(c);
      }
      sort(v.begin(), v.end(), compPlane);
    }
    else
    {
      for(it2 = targetDescriptors.begin(), j=0; it2!=targetDescriptors.end(); ++it2, ++j)
      {
	Correspondence c(i, j, getHistDistance(*it1, *it2));
	v.push_back(c);
      }
      sort(v.begin(), v.end(), compObj);
    }
    sourceCorrespondences.push_back(v);
  }
  for(it1 = targetDescriptors.begin(), i = 0; it1!=targetDescriptors.end(); ++it1, ++i)
  {
    vector<pcl::Correspondence> v;
    for(it2 = sourceDescriptors.begin(), j=0; it2!=sourceDescriptors.end(); ++it2, ++j)
    {
      Correspondence c(i, j, getHistDistance(*it1, *it2));
      v.push_back(c);
    }
    sort(v.begin(), v.end(), compObj);
    targetCorrespondences.push_back(v);
  }
}

void RegistrationUsingClusters::computeLocalFeatures(const CloudPtr &cloud, const NormalsCloudPtr &cloudNormals, FeatureCloudPtr &features)
{
  SearchTypePtr tree (new SearchType());
  FeatureEstimationMethodType feature_est;
  feature_est.setInputCloud (cloud);
  feature_est.setInputNormals (cloudNormals);
  feature_est.setSearchMethod (tree);
  feature_est.setRadiusSearch (LOCAL_FEATURE_RADIUS);
  feature_est.compute (*features);
}

Eigen::Matrix4f RegistrationUsingClusters::refineAlignmentICP(const CloudPtr& src, const CloudPtr& tgt, Eigen::Matrix4f& transformation)
{
  CloudPtr aligned1(new Cloud);
  pcl::IterativeClosestPoint<PointType, PointType> icp;
  icp.setInputCloud(src);
  icp.setInputTarget(tgt);
  icp.setMaximumIterations(MAX_ICP_ITERATIONS);
  icp.align(*aligned1);
  transformation = icp.getFinalTransformation();
}

// template<typename PointType>
void RegistrationUsingClusters::alignClusters(const CloudPtr &src, const NormalsCloudPtr &srcNormals, const CloudPtr &tgt, const NormalsCloudPtr &tgtNormals, Eigen::Matrix4f &transformation)
{
//   pcl::Registration<PointType, PointType>::Ptr registration (new pcl::IterativeClosestPoint<PointType, PointType>);
//   registration->setInputSource(source);
//   //registration->setInputCloud(source_segmented_);
//   registration->setInputTarget (target);
//   registration->setMaxCorrespondenceDistance(0.05);
//   registration->setRANSACOutlierRejectionThreshold (0.05);
//   registration->setTransformationEpsilon (0.000001);
//   registration->setMaximumIterations (1000);
//   registration->align(*aligned);
//   transformation = registration->getFinalTransformation();
  
  // TODO find optimal values of arguments for ICP
  
//   pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr srcWithNormals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
//   pcl::concatenateFields (*src, *srcNormals, *srcWithNormals);
//   pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr tgtWithNormals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
//   pcl::concatenateFields (*tgt, *tgtNormals, *tgtWithNormals);
//   
//   pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr aligned (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
//   pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
//   icp.setInputSource(srcWithNormals);
//   icp.setInputTarget(tgtWithNormals);
  
//   CloudPtr aligned(new Cloud);
//   pcl::IterativeClosestPoint<PointType, PointType> icp;
//   icp.setInputSource(src);
//   icp.setInputTarget(tgt);

//   NormalsCloudPtr aligned(new NormalsCloud);
//   pcl::IterativeClosestPoint<NormalPointType, NormalPointType> icp;
//   icp.setInputSource(srcNormals);
//   icp.setInputTarget(tgtNormals);

//   icp.setMaximumIterations(MAX_ICP_ITERATIONS);
//   icp.align(*aligned);
//   transformation = icp.getFinalTransformation();


// Using SAC-IA
  transformation = Eigen::Matrix4f::Identity();
  float score = getFitnessScore(transformation);
  FeatureCloudPtr srcFeatures(new FeatureCloud);
  FeatureCloudPtr tgtFeatures(new FeatureCloud);
  CloudPtr aligned(new Cloud);
  computeLocalFeatures(src, srcNormals, srcFeatures);
  computeLocalFeatures(tgt, tgtNormals, tgtFeatures);
  pcl::SampleConsensusInitialAlignment<PointType, PointType, FeaturePointType> sac_ia_;
  sac_ia_.setMinSampleDistance (SACIA_MIN_SAMPLE_DISTANCE);
  sac_ia_.setMaxCorrespondenceDistance (SACIA_MAX_CORRESPONDENCE_DISTANCE);
  sac_ia_.setMaximumIterations (SACIA_MAX_ITER);
  sac_ia_.setInputTarget (tgt);
  sac_ia_.setTargetFeatures (tgtFeatures);
  sac_ia_.setInputCloud (src);
  sac_ia_.setSourceFeatures (srcFeatures);
  for(int i=0;i<SAC_CYCLES;i++)
  {
    sac_ia_.align (*aligned);
    Eigen::Matrix4f tr = sac_ia_.getFinalTransformation ();
//     cout << "Align Clusters" << endl;
    float scoreTemp = getFitnessScore(tr);
//     cout << "Align Clusters" << endl;
    if(scoreTemp<score)
    {
      transformation = tr;
      score = scoreTemp;
    }
  }
  
  // ICP Refinement
  Eigen::Matrix4f tr;
  refineAlignmentICP(aligned, tgt, tr);
  transformation = tr*transformation;
}

bool RegistrationUsingClusters::intersect2Planes(const PlaneCoeff& plane_a, const PlaneCoeff& plane_b, Eigen::VectorXf& line)
{
  double angularTolerance = 0.1;
  double testCosine = plane_a.head<3>().dot(plane_b.head<3>());
  double upper_limit = 1 + angularTolerance;
  double lower_limit = 1 - angularTolerance;
  
  if ((testCosine < upper_limit) && (testCosine > lower_limit))
  {
    cout << "Planes are Parallel" << endl;
    return (false);
  }
  
  if ((testCosine > -upper_limit) && (testCosine < -lower_limit))
  {
    cout << "Planes are Parallel" << endl;
    return (false);
  }
  
  Eigen::Vector4f line_direction = plane_a.cross3(plane_b);
  line_direction.normalized();
  Eigen::MatrixXf langegrange_coefs(2,2);
  langegrange_coefs << plane_a[1],plane_a[2],plane_b[1],plane_b[2];  
   
  Eigen::VectorXf b;
  b.resize(2);
  b << -plane_a[3], -plane_b[3];
  
  //solve for the lagrange Multipliers
  Eigen::VectorXf x;
  x.resize(2);
  x = langegrange_coefs.colPivHouseholderQr().solve(b);
  
  line.resize(6);
  line[0] = 0;
  line[1] = x[0];
  line[2] = x[1];
  line[3] = line_direction[0];
  line[4] = line_direction[1];
  line[5] = line_direction[2];
  return true;
}

bool RegistrationUsingClusters::intersect3Planes(const PlaneCoeff& plane1, const PlaneCoeff& plane2, const PlaneCoeff& plane3, Eigen::Vector4f &intersect, Eigen::VectorXf& line1, Eigen::VectorXf& line2)
{
//   Eigen::VectorXf line1(6), line2(6);
  line1.resize(6);
  line2.resize(6);
  if(!intersect2Planes(plane1, plane2, line1))
    return false;
  if(!intersect2Planes(plane1, plane3, line2))
    return false;
//   Eigen::Vector4f plane_a, plane_b;
//   for(int i=0;i<4;i++)
//   {
//     plane_a(i,0) = plane1->values[i];
//     plane_b(i,0) = plane2->values[i];
//   }
//   bool res1 = planeWithPlaneIntersection(plane_a, plane_b, line1);
//   if(!res1)
//   {
//     printf("Planes 1,2 are parallel\n");
//   }
//   for(int i=0;i<4;i++)
//   {
//     plane_a(i,0) = plane1->values[i];
//     plane_b(i,0) = plane3->values[i];
//   }
//   bool res2 = planeWithPlaneIntersection(plane_a, plane_b, line2);
//   if(!res2)
//   {
//     printf("Planes 1,3 are parallel\n");
//   }
  bool res3 = lineWithLineIntersection(line1, line2, intersect);
  if(!res3)
  {
    cout << "The three planes do not intersect at a point" << endl;
  }
  return res3;
}

void RegistrationUsingClusters::transformSourcePlaneCoefficients(const Eigen::Matrix4f &transformation)
{
  for(vector<PlaneCoeff, Eigen::aligned_allocator<PlaneCoeff> >::iterator it = sourcePlaneCoeff.begin(); it!=sourcePlaneCoeff.end();++it)
  {
    (*it) = transformation*(*it);
  }
}

void RegistrationUsingClusters::recursiveAlignPlaneTriplets(int cur, const vector<pcl::Correspondence>& allCorrespondences, vector<int> &indices, Eigen::Matrix4f& transformation, double &score)
{
  if(indices.size()==3)
  {
    Eigen::Vector3f zsrc, ztgt, ysrc, ytgt;
  
    zsrc = sourcePlaneCoeff[allCorrespondences[indices[0]].index_query].topLeftCorner(3,1);
    ztgt = targetPlaneCoeff[allCorrespondences[indices[0]].index_match].topLeftCorner(3,1);
    ysrc = sourcePlaneCoeff[allCorrespondences[indices[1]].index_query].topLeftCorner(3,1);
    ytgt = targetPlaneCoeff[allCorrespondences[indices[1]].index_match].topLeftCorner(3,1);
    
    // Find points of intersection of three planes on source and target scenes
    Eigen::Vector4f srcIntersection, tgtIntersection;
    Eigen::VectorXf sl1, sl2, tl1, tl2;
    if( intersect3Planes(sourcePlaneCoeff[allCorrespondences[indices[0]].index_query], sourcePlaneCoeff[allCorrespondences[indices[1]].index_query], sourcePlaneCoeff[allCorrespondences[indices[2]].index_query], srcIntersection, sl1, sl2) &&
	intersect3Planes(targetPlaneCoeff[allCorrespondences[indices[0]].index_match], targetPlaneCoeff[allCorrespondences[indices[1]].index_match], targetPlaneCoeff[allCorrespondences[indices[2]].index_match], tgtIntersection, tl1, tl2))
    {
      // Find transformation of set of three planes
      Eigen::Vector3f srcPoint, tgtPoint;
      for(int i=0;i<3;i++)
      {
	srcPoint(i,0) = srcIntersection(i,0);
	tgtPoint(i,0) = tgtIntersection(i,0);
      }
      Eigen::Affine3f transformationSrc, transformationTgt;
      Eigen::Matrix4f tr;
      for(int i=0;i<4;i++)
      {
	Eigen::Vector3f zsrc1=zsrc, ztgt1=ztgt, ysrc1=ysrc, ytgt1=ytgt;
	int temp = i&1;
	if(temp)
	  zsrc1 = -zsrc1;
	temp = i>>1;
	temp = temp&1;
	if(temp)
	  ysrc1 = -ysrc1;
	getTransformationFromTwoUnitVectorsAndOrigin(zsrc1, ysrc1, srcPoint, transformationSrc);
	getTransformationFromTwoUnitVectorsAndOrigin(ztgt1, ytgt1, tgtPoint, transformationTgt);
	tr = (transformationTgt.inverse()*transformationSrc).matrix();
	double sc = getFitnessScore(tr);
// 	cout << tr << endl << "Score: " << sc << endl;
	if(sc<score)
	{
	  score = sc;
	  transformation = tr;
	  sourceLine1 = sl1;
	  sourceLine2 = sl2;
	  targetLine1 = tl1;
	  targetLine2 = tl2;
	  sourcePivot = srcPoint;
	  targetPivot = tgtPoint;
	}
      }
    }
    else
    {
//       cout << "A pair of planes is parallel" << endl;
    }
  }
  if(cur==allCorrespondences.size())
  {
    return;
  }
  recursiveAlignPlaneTriplets(cur+1, allCorrespondences, indices, transformation, score);
  indices.push_back(cur);
  recursiveAlignPlaneTriplets(cur+1, allCorrespondences, indices, transformation, score);
  indices.pop_back();
}

void RegistrationUsingClusters::recursiveAlignClusters(int anchors, int cur, const vector<pcl::Correspondence>& allCorrespondences, vector<int> &indices, Eigen::Matrix4f& transformation, double &score)
{
  if(anchors==indices.size())
  {
    Eigen::Matrix4f tr;
    CloudPtr sourceAnchors(new Cloud), targetAnchors(new Cloud);
    NormalsCloudPtr sourceAnchorNormals(new NormalsCloud), targetAnchorNormals(new NormalsCloud);
    for(int i=0;i<indices.size();i++)
    {
      int sourceIndex = allCorrespondences[indices[i]].index_query;
      int targetIndex = allCorrespondences[indices[i]].index_match;
      *sourceAnchors+=*sourceClusters[sourceIndex];
      *sourceAnchorNormals+=*sourceClusterNormals[sourceIndex];
      *targetAnchors+=*targetClusters[targetIndex];
      *targetAnchorNormals+=*targetClusterNormals[targetIndex];
    }
    alignClusters(sourceAnchors, sourceAnchorNormals, targetAnchors, targetAnchorNormals, tr);
//     cout << "Rec Align" << endl;
//     cout << tr << endl;
    double sc = getFitnessScore(tr);
//     cout << "Score: " << sc << endl;
//     cout << "Rec Align" << endl;
    if(sc<score)
    {
      score = sc;
      transformation = tr;
    }
    return;
  }
  if(cur==allCorrespondences.size())
    return;
  indices.push_back(cur);
  recursiveAlignClusters(anchors, cur+1, allCorrespondences, indices, transformation, score);
  indices.pop_back();
  recursiveAlignClusters(anchors, cur+1, allCorrespondences, indices, transformation, score);
}

// template<typename PointType>
void RegistrationUsingClusters::generateTransformation()
{
  // Find the best Correspondence index
  // TODO Implement a better approach for transformation estimation
  // Current approach greedily chooses the best matching correspondence
  // Cases where there is no "good enough" match will have to be considered
  
  // Using Planes
  vector<pcl::Correspondence> allCorrespondences;
  
  localTransformation = Eigen::Matrix4f::Identity();
  float fitnessScore = getFitnessScore(localTransformation);
  constraints.clear();
  
  if(usePlanes)
  {
    for(vector<vector<pcl::Correspondence> >::const_iterator it = sourceCorrespondences.begin(); it!=sourceCorrespondences.begin()+planesSource;++it)
    {
      if((*it)[0].distance<=PLANE_MATCH_THRESHOLD)
      {
	allCorrespondences.push_back((*it)[0]);
	// Populate Constraints
	constraints.add(sourcePlaneCoeff[(*it)[0].index_query], targetPlaneCoeff[(*it)[0].index_match]);
      }
    }
    
    if(allCorrespondences.size()>=3)
    {
      cout << "Aligning Using Planes" << endl;
      cout << "Candidate Plane Correspondences: " << allCorrespondences.size() << endl;
      CompareCorrespondence compObj(false);
      sort(allCorrespondences.begin(), allCorrespondences.end(), compObj);
      Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
      double score = getFitnessScore(transformation);
      vector<int> indices;
      recursiveAlignPlaneTriplets(0, allCorrespondences, indices, transformation, score);
      
      cout << transformation << endl;
      fitnessScore = score;
      localTransformation = transformation;
      printf("Score With Planes: %f\n", fitnessScore);    
    }
    else
    {
      printf("Insufficient plane correspondences\n");
    }
  }

  // NOTE Considering planes as clusters
  if(useClusters)
  {
    for(vector<vector<pcl::Correspondence> >::const_iterator it = sourceCorrespondences.begin(); it!=sourceCorrespondences.end();++it)
    {
	if((*it)[0].distance<=CLUSTER_MATCH_THRESHOLD)
	{
	  allCorrespondences.push_back((*it)[0]);
	}
    }

    cout << "Candidate Cluster Correspondences: " << allCorrespondences.size() << endl;

    if(allCorrespondences.size()==0)
    {
      cout << "No Matching Clusters" << endl;
    }
    CompareCorrespondence compObj(getComparisonFlag());
    sort(allCorrespondences.begin(), allCorrespondences.end(), compObj);

    if(allCorrespondences.size()==0)
    {
      SOURCE_CHOSEN_CLUSTER_INDEX = 0;
      TARGET_CHOSEN_CLUSTER_INDEX = 0;
    }
    else
    {
      SOURCE_CHOSEN_CLUSTER_INDEX = allCorrespondences[0].index_query;
      TARGET_CHOSEN_CLUSTER_INDEX = allCorrespondences[0].index_match;
    }

    // SACIA Alignment

    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    int anchors = ANCHORS;

    vector<int> selected;
    double score = getFitnessScore(transformation);
    if(allCorrespondences.size()<anchors)
      anchors = allCorrespondences.size();
    if(anchors==0)
    {
      cout << "No clusters to align" << endl;
    }
    else
    {
      for(int i=1;i<=anchors;i++)
      {
	recursiveAlignClusters(i, 0, allCorrespondences, selected, transformation, score);
      }
    }

    cout << transformation << endl;

    cout << "Score With Clusters: " << score << endl;
    if(score<fitnessScore)
    {
      fitnessScore = score;
      localTransformation = transformation;
    }
  }
  updateGlobalTransformation(localTransformation);
}

void RegistrationUsingClusters::visualize()
{
  pcl::visualization::PCLVisualizer viewer ("View Clusters");
    
  pair<RegistrationUsingClusters*,pcl::visualization::PCLVisualizer*> p(this,&viewer);
  viewer.registerKeyboardCallback(keyboard_cb, (void*)&p);
  
  curLeft = SOURCE_CHOSEN_CLUSTER_INDEX;
  totalLeft = sourceClusters.size();
  curRight = TARGET_CHOSEN_CLUSTER_INDEX;
  totalRight = targetClusters.size();
  
  int v1(0);
  viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer.setBackgroundColor (0, 0, 0, v1);
  pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb1(sourceClusters[curLeft]);
  viewer.addPointCloud<PointType> (sourceClusters[curLeft], rgb1, "Source Cluster", v1);
  
  int v2(0);
  viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer.setBackgroundColor (0, 0, 0, v2);
  pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb2(targetClusters[curRight]);
  viewer.addPointCloud<PointType> (targetClusters[curRight], rgb2, "Target Cluster", v2);

  char histDistText[100];
  double distance = getHistDistance(sourceDescriptors[curLeft], targetDescriptors[curRight]);
  sprintf(histDistText, "%s %lf", HIST_COMPARISON_METHODS[HIST_COMPARISON_METHOD], distance);
  viewer.addText(histDistText, 10, 20, "v1 distance", v1);
  
//   viewer.addCoordinateSystem (1.0);
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, POINT_SIZE, "Source Cluster");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, POINT_SIZE, "Target Cluster");
  
  sourceMergedHistViewer.addFeatureHistogram<DescriptorHistPointType>(*sourceDescriptors[curLeft], DESCRIPTOR_HSIZE, "EFPFHRGB Source");
  targetMergedHistViewer.addFeatureHistogram<DescriptorHistPointType>(*targetDescriptors[curRight], DESCRIPTOR_HSIZE, "EFPFHRGB Target");
  
  viewer.spin();
}

// template<typename PointType>
void RegistrationUsingClusters::copySourceToTarget()
{
  target->clear();
  targetNormals->clear();
  targetResidual->clear();
  targetResidualNormals->clear();
  targetClusters.clear();
  targetClusterNormals.clear();
  targetVFHSigs.clear();
  targetRHist.clear();
  targetGHist.clear();
  targetBHist.clear();
  targetDescriptors.clear();
  targetPlaneCoeff.clear();
//   target = sourceTransformed->makeShared();
// TODO use *source and find way of using transformation of previous source to find transformation of new cloud
  copyPointCloud<PointType>(*source, *target);
//   process(target, targetNormals, targetResidual, targetResidualNormals, targetClusters, targetClusterNormals, targetVFHSigs, targetRHist, targetGHist, targetBHist, targetDescriptors, planesTarget);
  copyPointCloud<NormalPointType>(*sourceNormals, *targetNormals);
  copyPointCloud<PointType>(*sourceResidual, *targetResidual);
  copyPointCloud<NormalPointType>(*sourceResidualNormals, *targetResidualNormals);
  targetClusters = sourceClusters;
  targetClusterNormals = sourceClusterNormals;
  targetVFHSigs = sourceVFHSigs;
  targetRHist = sourceRHist;
  targetGHist = sourceGHist;
  targetBHist = sourceBHist;
  targetDescriptors = sourceDescriptors;
  planesTarget = planesSource;
  targetPlaneCoeff = sourcePlaneCoeff;
  targetPivot = sourcePivot;
  targetLine1 = sourceLine1;
  targetLine2 = sourceLine2;
}

void RegistrationUsingClusters::updateGlobalTransformation(Eigen::Matrix4f transformation)
{
  globalTransformation = globalTransformation*transformation;
}

void RegistrationUsingClusters::filter(const CloudPtr& src, CloudPtr& tgt)
{
  boost::shared_ptr<std::vector<int> > indices(new std::vector<int>), indices_x(new std::vector<int>), indices_y(new std::vector<int>);
  pcl::removeNaNFromPointCloud(*src, *tgt, *indices);
  pcl::PassThrough<PointType> pass;
  pass.setInputCloud (tgt);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, Z_LIMIT);
  pass.filter (*tgt);
  
  pass.setInputCloud (tgt);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-Z_LIMIT, Z_LIMIT);
  pass.filter (*tgt);
  
  pass.setInputCloud (tgt);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-Z_LIMIT, Z_LIMIT);
  pass.filter (*tgt);
}

void RegistrationUsingClusters::smooth(const CloudPtr& src, CloudPtr& tgt)
{
  pcl::MovingLeastSquares<PointType, PointType> mls;
  SearchTypePtr tree (new SearchType);
//   mls.setComputeNormals (false);

  // Set parameters
  mls.setInputCloud (src);
  mls.setPolynomialFit (MLS_POLYNOMIAL_FIT_FLAG);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (MLS_SEARCH_RADIUS);

  // Reconstruct
  mls.process (*tgt);
}

// template<typename PointType>
void RegistrationUsingClusters::scaleDown(const CloudPtr &source, CloudPtr &target)
{
  target->clear();
  pcl::VoxelGrid<PointType> sor;
  sor.setInputCloud (source);
  sor.setLeafSize (LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
  sor.filter (*target);
}

void RegistrationUsingClusters::scaleDownAndSmooth(const CloudPtr &source, CloudPtr& target)
{
  if(!MLS_FLAG)
  {
    scaleDown(source, target);
  }
  else
  {
    CloudPtr temp(new Cloud);
    scaleDown(source, temp);
    smooth(temp, target);
  }
  // NOTE PCL 1.6 does not allow MLS over RGB
}

void RegistrationUsingClusters::transformSourcePivotAndLines(const Eigen::Matrix4f& transformation)
{
  Eigen::Affine3f tr(transformation);
  pcl::PointXYZ p(sourcePivot(0,0), sourcePivot(1,0), sourcePivot(2,0));
  p = transformPoint(p, tr);
  sourcePivot(0,0) = p.x;
  sourcePivot(1,0) = p.y;
  sourcePivot(2,0) = p.z;
  
  // Line 1
  p.x = sourceLine1(0,0);
  p.y = sourceLine1(1,0);
  p.z = sourceLine1(2,0);
  p = transformPoint(p, tr);
  sourceLine1(0,0) = p.x;
  sourceLine1(1,0) = p.y;
  sourceLine1(2,0) = p.z;
  p.x = sourceLine1(3,0);
  p.y = sourceLine1(4,0);
  p.z = sourceLine1(5,0);
  p = transformPoint(p, tr);
  sourceLine1(3,0) = p.x-tr(0,3);
  sourceLine1(4,0) = p.y-tr(1,3);
  sourceLine1(5,0) = p.z-tr(2,3);
  
  // Line 2
  p.x = sourceLine2(0,0);
  p.y = sourceLine2(1,0);
  p.z = sourceLine2(2,0);
  p = transformPoint(p, tr);
  sourceLine2(0,0) = p.x;
  sourceLine2(1,0) = p.y;
  sourceLine2(2,0) = p.z;
  p.x = sourceLine2(3,0);
  p.y = sourceLine2(4,0);
  p.z = sourceLine2(5,0);
  p = transformPoint(p, tr);
  sourceLine2(3,0) = p.x-tr(0,3);
  sourceLine2(4,0) = p.y-tr(1,3);
  sourceLine2(5,0) = p.z-tr(2,3);
}

/**
 * Returns fitness score defined the number of overlapping points
 */
double RegistrationUsingClusters::getFitnessScore (const Eigen::Matrix4f &transformation)
{
  double fitness_score = 0.0;

  // Transform the input dataset using the final transformation
  CloudPtr sourceTransformed (new Cloud);
  transformPointCloud (*source, *sourceTransformed, transformation);

  std::vector<int> nn_indices (1);
  std::vector<float> nn_dists (1);

  SearchTypePtr tree (new SearchType);
  tree->setInputCloud(target);
  
  // For each point in the source dataset
  int matches = 0;
  double sum_dists = 0.0;
  for (size_t i = 0; i < sourceTransformed->points.size (); ++i)
  {
//     Eigen::Vector4f p1 = Eigen::Vector4f (sourceTransformed->points[i].x,
// 					  sourceTransformed->points[i].y,
// 					  sourceTransformed->points[i].z, 0);
    PointType sourcePoint = sourceTransformed->points[i];
    
    // Find its nearest neighbor in the target
    
    int found;
    if(KSEARCH_FLAG)
    {
      found = tree->nearestKSearch (sourceTransformed->points[i], K_SEARCH, nn_indices, nn_dists);
    }
    else
    {
      found = tree->radiusSearch (sourceTransformed->points[i], KDTREE_SEARCH_RADIUS, nn_indices, nn_dists);
    }
//     cout << "Neighbours found: " << found << endl;
    for(int i=0;i<found;i++)
    {
      if (nn_dists[i] > FITNESS_SCORE_MAX_DIST)
	continue;
      PointType targetPoint = target->points[nn_indices[i]];
      if (abs(sourcePoint.r-targetPoint.r)<DELTA_RED && abs(sourcePoint.g-targetPoint.g)<DELTA_GREEN && abs(sourcePoint.b-targetPoint.b)<DELTA_BLUE)
      {
	matches++;
	sum_dists += nn_dists[i];
	break;
      }
    }
    
//     Eigen::Vector4f p2 = Eigen::Vector4f (target->points[nn_indices[0]].x,
// 					  target->points[nn_indices[0]].y,
// 					  target->points[nn_indices[0]].z, 0);
//     // Calculate the fitness score
//     fitness_score += fabs ((p1-p2).squaredNorm ());
  }
  double avg_dists = 0.0;
  if(matches)
    avg_dists = sum_dists/matches;
  double matchScore = 1.0-double(matches)/(sourceTransformed->points.size());
  double constraintScore = constraints.getFitnessScore(transformation);
  fitness_score = (1.0-FS_WEIGHT_OF_AVG_DISTS)*((1.0-SAC_WEIGHT_OF_CONSTRAINTS)*matchScore + SAC_WEIGHT_OF_CONSTRAINTS*constraintScore) + FS_WEIGHT_OF_AVG_DISTS*avg_dists;
  return fitness_score;
}

void RegistrationUsingClusters::getRotationMatrixAboutVerticalAxis (double yaw, Eigen::Matrix3f & mat)
{
  mat(1,1) = 1;
  mat(0,1) = mat(2,1) = mat(1,0) = mat(1,2) = 0;
  mat(0,0) = cos(yaw); mat(0,2) = sin(yaw);
  mat(2,0) = -sin(yaw); mat(2,2) = cos(yaw);
}

void RegistrationUsingClusters::getTransformationFromOdom(Eigen::Matrix4f& tr)
{
  tr = Eigen::Matrix4f::Identity();
  Eigen::Matrix3f relative;
  getRotationMatrixAboutVerticalAxis(-yaw1, relative);
  for(int i=0;i<3;i++)
  {
    for(int j=0;j<3;j++)
    {
      tr(i,j) = relative(i,j);
    }
  }

  // planar
  double pi = 3.14159;
  Eigen::Vector3f temp = translation1;
  double len = temp.norm();
  double angle = atan(temp(1)/temp(0));
  
  angle = angle*pi/1.8;
  
  // Handle cases for 2nd and 3rd quadrant
  if(temp(0)<0)
  {
    if(temp(1)>=0)
    {
      angle+=pi;
    }
    else
    {
      angle-=pi;
    }
  }
  
  Eigen::Vector3f vec;
  vec << -len*sin(angle), 0.0, len*cos(angle);
  
  tr(0,3) = vec(0);
  tr(1,3) = vec(1);
  tr(2,3) = vec(2);
  
}

void RegistrationUsingClusters::add(const CloudPtr &in)
{
  Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
  if(useTurtlebotOdometry)
  {
    double arr[4];
    while(!get_current_orientation(arr))
    {
      
    }
    printf("Odometry feedback: ");
    for(int i=0;i<4;i++)
    {
      printf("%lf ",arr[i]);
    }
    printf("\n");
    yaw2 = yaw1;
    yaw1 = arr[3];
    for(int i=0;i<3;i++)
    {
      translation2(i) = translation1(i);
      translation1(i) = arr[i];
    }
    getTransformationFromOdom(guess);
  }
  add(in, guess);
}

// template<typename PointType>
void RegistrationUsingClusters::add(const CloudPtr &in, const Eigen::Matrix4f& guess)
{
  CloudPtr input(new Cloud), cloud(new Cloud);
  filter(in, input);
  transformPointCloud(*input, *cloud, guess);
  if(clouds.empty())
  {
    setSeedCloud(cloud);
    return;
  }
  if(!undoUsed)
    copySourceToTarget();
  source->clear();
  sourceTransformed->clear();
  copyPointCloud<PointType>(*cloud, *sourceOriginal);
  scaleDownAndSmooth(cloud, source);
  printf("Smoothed\n");
  process(source, sourceNormals, sourceResidual, sourceResidualNormals, sourceClusters, sourceClusterNormals, sourceVFHSigs, sourceRHist, sourceGHist, sourceBHist, sourceDescriptors, sourcePlaneCoeff, planesSource);
  printf("Processed\n");
  generateCorrespondences();
  printf("Generated Corrs\n");
  generateTransformation();
  printf("Generated Tr\n");
  transformPointCloud(*sourceOriginal, *sourceTransformed, globalTransformation);
  transformSourcePivotAndLines(globalTransformation);
//   printf("Fitness Score:%lf\n", getFitnessScore(localTransformation));
//   std::cout << transformation << std::endl;
//   (*registered)+=*sourceTransformed;
  localTransformations.push_back (localTransformation);
  globalTransformations.push_back (globalTransformation);
  CloudPtr temp (new Cloud);
  copyPointCloud<PointType> (*sourceTransformed, *temp);
  clouds.push_back (temp);
  undoUsed = false;
  if(VISUALIZE)
  {
    visualize();
  }
}

// template<typename PointType>
void RegistrationUsingClusters::setSeedCloud(const CloudPtr &input)
{
//   CloudPtr cloud(new Cloud);
//   cloud = input;
//   filter(input, cloud);
  copyPointCloud<PointType>(*input, *sourceOriginal);
//   copyPointCloud<PointType>(*cloud, *registered);
  copyPointCloud<PointType>(*input, *sourceTransformed);  
  scaleDownAndSmooth(input, source);
  process(source, sourceNormals, sourceResidual, sourceResidualNormals, sourceClusters, sourceClusterNormals, sourceVFHSigs, sourceRHist, sourceGHist, sourceBHist, sourceDescriptors, sourcePlaneCoeff, planesSource); 
  localTransformation = Eigen::Matrix4f::Identity();
  updateGlobalTransformation (localTransformation);
  localTransformations.push_back (localTransformation);
  globalTransformations.push_back (globalTransformation);
  CloudPtr temp (new Cloud);
  copyPointCloud<PointType> (*sourceTransformed, *temp);
  clouds.push_back (temp);
  undoUsed = false;
}

// template<typename PointType>
CloudPtr RegistrationUsingClusters::getRegisteredCloud()
{
  CloudPtr registered(new Cloud);
  vector<CloudPtr>::const_iterator it;
  for(it = clouds.begin();it!=clouds.end();++it)
  {
    *registered+=**it;
  }
  return registered->makeShared();
}

void RegistrationUsingClusters::loadPreviousState ()
{
  clouds.pop_back();
  globalTransformations.pop_back();
  localTransformations.pop_back();
  yaw1 = yaw2;
  translation1 = translation2;
  if(globalTransformations.empty())
  {
    localTransformation = Eigen::Matrix4f::Identity();
    globalTransformation = Eigen::Matrix4f::Identity();
  }
  else
  {
    localTransformation = localTransformations.back();
    globalTransformation = globalTransformations.back();
  }
}

void RegistrationUsingClusters::dropPreviousCloud ()
{
  if(!undoUsed)
  {
    loadPreviousState ();
    undoUsed = true;
  }
  else
  {
    pcl::console::print_error ("Undo Not Allowed\n");
  }
}

void RegistrationUsingClusters::setMaxRANSACIterations(int n)
{
  MAX_RANSAC_ITERATIONS = n;
}

void RegistrationUsingClusters::setMaxICPIterations(int n)
{
  MAX_ICP_ITERATIONS = n;
}

void RegistrationUsingClusters::setKSearch(int n)
{
  K_SEARCH = n;
}

void RegistrationUsingClusters::setMinPlaneSize(int n)
{
  MIN_PLANE_SIZE = n;
}

void RegistrationUsingClusters::setMinClusterSize(int n)
{
  MIN_CLUSTER_SIZE = n;
}

void RegistrationUsingClusters::setMaxClusterSize(int n)
{
  MAX_CLUSTER_SIZE = n;
}

void RegistrationUsingClusters::setHistComparisonMethod(int n)
{
  HIST_COMPARISON_METHOD = n;
  if(getComparisonFlag())
  {
    WEIGHT_OF_COEFF = -WEIGHT_OF_COEFF;
  }
}

void RegistrationUsingClusters::setPointSize(int n)
{
  POINT_SIZE = n;
}

void RegistrationUsingClusters::setDistanceThreshold(float f)
{
  DISTANCE_THRESHOLD = f;
}

void RegistrationUsingClusters::setNormalDistanceWeight(float f)
{
  NORMAL_DISTANCE_WEIGHT = f;
}

void RegistrationUsingClusters::setNormalEstimationRadius(float f)
{
  NE_RADIUS = f;
}

void RegistrationUsingClusters::setClusterTolerance(float f)
{
  CLUSTER_TOLERANCE = f;
}

void RegistrationUsingClusters::setLeafSize(float f)
{
  LEAF_SIZE = f;
}

void RegistrationUsingClusters::setVFHNormalizeFlag(bool b)
{
  NORMALIZE_VFH = b;
}

void RegistrationUsingClusters::setRGBNormalizeFlag(bool b)
{
  NORMALIZE_RGB = b;
}

void RegistrationUsingClusters::setVisualize(bool b)
{
  VISUALIZE = b;
}

void RegistrationUsingClusters::setLocalFeatureRadius(float f)
{
  LOCAL_FEATURE_RADIUS = f;
}

void RegistrationUsingClusters::setSACIAIterations(int i)
{
  SACIA_MAX_ITER = i;
}

void RegistrationUsingClusters::setSACIAMinSampleDistance(float f)
{
  SACIA_MIN_SAMPLE_DISTANCE = f;
}

void RegistrationUsingClusters::setNumAnchors(int i)
{
  ANCHORS = i;
}

void RegistrationUsingClusters::setZLimit(float f)
{
  Z_LIMIT = f;
}

void RegistrationUsingClusters::setMLSSearchRadius(float f)
{
  MLS_SEARCH_RADIUS = f;
}

void RegistrationUsingClusters::setMLSPolynomialFitFlag(bool b)
{
  MLS_POLYNOMIAL_FIT_FLAG = b;
}

void RegistrationUsingClusters::setPlaneMatchThreshold(float f)
{
  PLANE_MATCH_THRESHOLD = f;
}

void RegistrationUsingClusters::setClusterMatchThreshold(float f)
{
  CLUSTER_MATCH_THRESHOLD = f;
}

void RegistrationUsingClusters::setWeightOfCoeff(float f)
{
  WEIGHT_OF_COEFF = f;
}

void RegistrationUsingClusters::setMaxDistForFitnessScore(float f)
{
  FITNESS_SCORE_MAX_DIST = f;
}

void RegistrationUsingClusters::setSACCycles(int i)
{
  SAC_CYCLES = i;
}

void RegistrationUsingClusters::setSACWeightOfConstraints(float f)
{
  SAC_WEIGHT_OF_CONSTRAINTS = f;
}

void RegistrationUsingClusters::setDeltaRed(int i)
{
  DELTA_RED = i;
}

void RegistrationUsingClusters::setDeltaGreen(int i)
{
  DELTA_GREEN = i;
}

void RegistrationUsingClusters::setDeltaBlue(int i)
{
  DELTA_BLUE = i;
}

void RegistrationUsingClusters::setKDTreeSearchRadius(float f)
{
  KDTREE_SEARCH_RADIUS = f;
}

void RegistrationUsingClusters::setKSearchFlag(bool b)
{
  KSEARCH_FLAG = b;
}

int RegistrationUsingClusters::getPointSize()
{
  return POINT_SIZE;
}

bool RegistrationUsingClusters::isBetter(float f1, float f2)
{
  if(getComparisonFlag())
    return f1>f2;
  else
    return f1<f2;
}

void RegistrationUsingClusters::setUseTurtlebotOdometry(bool b)
{
  useTurtlebotOdometry = b;
}

void RegistrationUsingClusters::setUsePlanes(bool b)
{
  usePlanes = b;
}

void RegistrationUsingClusters::setUseClusters(bool b)
{
  useClusters = b;
}

void RegistrationUsingClusters::setWeightOfAvgDists(float f)
{
  FS_WEIGHT_OF_AVG_DISTS = f;
}

void RegistrationUsingClusters::setMLSFlag(bool b)
{
  MLS_FLAG = b;
}

void RegistrationUsingClusters::keyboard_cb (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
  pair<RegistrationUsingClusters*,pcl::visualization::PCLVisualizer*> p = *static_cast<pair<RegistrationUsingClusters*,pcl::visualization::PCLVisualizer*> *> (viewer_void);
  pcl::visualization::PCLVisualizer viewer = *(p.second);
  RegistrationUsingClusters* r = p.first;
  char histDistText[100];
  if ((event.getKeySym () == "a" || event.getKeySym () == "A") && event.keyDown ())
  {
//     std::cout << event.getKeySym () << " was pressed" << std::endl;
    r->curLeft = (r->curLeft+1)%(r->totalLeft);
    pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb1(r->sourceClusters[r->curLeft]);
    viewer.updatePointCloud<PointType> (r->sourceClusters[r->curLeft], rgb1, "Source Cluster");
//     double distance = r->getHistDistance(r->sourceDescriptors[r->curLeft], r->targetDescriptors[r->curRight]);
//     sprintf(histDistText, "%s %lf", r->HIST_COMPARISON_METHODS[r->HIST_COMPARISON_METHOD], distance);
//     viewer.updateText(histDistText, 10, 20, "v1 distance");
    
  }
  else if((event.getKeySym () == "s" || event.getKeySym () == "S") && event.keyDown ())
  {
//     std::cout << event.getKeySym () << " was pressed" << std::endl;
    r->curRight = (r->curRight+1)%(r->totalRight);
    pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb2(r->targetClusters[r->curRight]);
    viewer.updatePointCloud<PointType> (r->targetClusters[r->curRight], rgb2, "Target Cluster");  
//     double distance = r->getHistDistance(r->sourceDescriptors[r->curLeft], r->targetDescriptors[r->curRight]);
//     sprintf(histDistText, "%s %lf", r->HIST_COMPARISON_METHODS[r->HIST_COMPARISON_METHOD], distance);
//     viewer.updateText(histDistText, 10, 20, "v1 distance");
  }
  else if((event.getKeySym () == "d" || event.getKeySym () == "D") && event.keyDown ())
  {
    // Update right viewport with the cluster best matching the left viewport cluster 
//     std::cout << event.getKeySym () << " was pressed" << std::endl;
    r->curRight = r->sourceCorrespondences[r->curLeft][0].index_match;
    pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb2(r->targetClusters[r->curRight]);
    viewer.updatePointCloud<PointType> (r->targetClusters[r->curRight], rgb2, "Target Cluster");  
//     double distance = r->getHistDistance(r->sourceDescriptors[r->curLeft], r->targetDescriptors[r->curRight]);
//     sprintf(histDistText, "%s %lf", r->HIST_COMPARISON_METHODS[r->HIST_COMPARISON_METHOD], distance);
//     viewer.updateText(histDistText, 10, 20, "v1 distance");
  }
  if(r->curLeft<r->planesSource && r->curRight<r->planesTarget)
  {
    double distance = r->matchPlanes(r->sourcePlaneCoeff[r->curLeft], r->targetPlaneCoeff[r->curRight], r->sourceDescriptors[r->curLeft], r->targetDescriptors[r->curRight]);
    sprintf(histDistText, "%s %lf", "Planes Distance : ", distance);
    viewer.updateText(histDistText, 10, 20, "v1 distance");
//     cout << *(r->sourcePlaneCoeff[r->curLeft]) <<  endl << *(r->targetPlaneCoeff[r->curRight]) << endl;
  }
  else
  {
    double distance = r->getHistDistance(r->sourceDescriptors[r->curLeft], r->targetDescriptors[r->curRight]);
    sprintf(histDistText, "%s %lf", r->HIST_COMPARISON_METHODS[r->HIST_COMPARISON_METHOD], distance);
    viewer.updateText(histDistText, 10, 20, "v1 distance");
  }
  
  r->sourceMergedHistViewer.updateFeatureHistogram<DescriptorHistPointType>(*(r->sourceDescriptors[r->curLeft]), DESCRIPTOR_HSIZE, "EFPFHRGB Source");
  r->targetMergedHistViewer.updateFeatureHistogram<DescriptorHistPointType>(*(r->targetDescriptors[r->curRight]), DESCRIPTOR_HSIZE, "EFPFHRGB Target");
  r->sourceMergedHistViewer.setGlobalYRange(0.0, 0.3);
  r->targetMergedHistViewer.setGlobalYRange(0.0, 0.3);
}
