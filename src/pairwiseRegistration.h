#include "includes.h"
#include "defines.h"
#include "typedefs.h"
#include "listener.h"

// template<typename PointType>
class RegistrationUsingClusters
{
  
  static char HIST_COMPARISON_METHODS[4][50];

  int MAX_RANSAC_ITERATIONS, K_SEARCH, MIN_PLANE_SIZE, MIN_CLUSTER_SIZE, MAX_CLUSTER_SIZE, MAX_ICP_ITERATIONS, HIST_COMPARISON_METHOD, TOLERANCE; // ONLY 0-3 ALLOWED
  int POINT_SIZE, SOURCE_CHOSEN_CLUSTER_INDEX, TARGET_CHOSEN_CLUSTER_INDEX, ANCHORS, SAC_CYCLES, DELTA_RED, DELTA_GREEN, DELTA_BLUE;
  float DISTANCE_THRESHOLD, NORMAL_DISTANCE_WEIGHT, NE_RADIUS, CLUSTER_TOLERANCE, LEAF_SIZE, MLS_SEARCH_RADIUS;
  float LOCAL_FEATURE_RADIUS, SACIA_MIN_SAMPLE_DISTANCE, SACIA_MAX_ITER, SACIA_MAX_CORRESPONDENCE_DISTANCE, Z_LIMIT, PLANE_MATCH_THRESHOLD;
  bool NORMALIZE_VFH, NORMALIZE_RGB, VISUALIZE, MLS_POLYNOMIAL_FIT_FLAG, KSEARCH_FLAG;
  float WEIGHT_OF_COEFF, FITNESS_SCORE_MAX_DIST, SAC_WEIGHT_OF_CONSTRAINTS, KDTREE_SEARCH_RADIUS, CLUSTER_MATCH_THRESHOLD, FS_WEIGHT_OF_AVG_DISTS;
  
  int totalLeft, totalRight, curLeft, curRight, planesSource, planesTarget;
  bool undoUsed, useTurtlebotOdometry, usePlanes, useClusters, MLS_FLAG;

  double yaw1, yaw2;
  Eigen::Vector3f translation1, translation2;
  
  CloudPtr source, target, sourceOriginal, sourceTransformed, sourceResidual, targetResidual;
//   CloudPtr registered;
  NormalsCloudPtr sourceNormals, targetNormals, sourceResidualNormals, targetResidualNormals;

  vector<CloudPtr> clouds;
  
  vector<CloudPtr> sourceClusters, targetClusters;
  vector<NormalsCloudPtr> sourceClusterNormals, targetClusterNormals;

  vector<PlaneCoeff, Eigen::aligned_allocator<PlaneCoeff> > sourcePlaneCoeff, targetPlaneCoeff;
  
  vector<HistCloudPtr> sourceRHist, targetRHist, sourceGHist, targetGHist, sourceBHist, targetBHist; 
  vector<VFHSigPtr> sourceVFHSigs, targetVFHSigs;
  vector<DescriptorHistCloudPtr> sourceDescriptors, targetDescriptors;

  vector<vector<pcl::Correspondence> > sourceCorrespondences, targetCorrespondences;
  Eigen::Matrix4f globalTransformation, localTransformation;
  vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > localTransformations, globalTransformations;
  
  pcl::visualization::PCLHistogramVisualizer sourceMergedHistViewer, targetMergedHistViewer;
  
//   pcl::visualization::PCLHistogramVisualizer sourceVFHViewer, targetVFHViewer, sourceRHistViewer, sourceGHistViewer, sourceBHistViewer, targetRHistViewer, targetBHistViewer, targetGHistViewer, sourceDescriptorHistViewer, targetDescriptorHistViewer; 
//   pcl::visualization::PCLVisualizer viewer;

//   char name[40], text[50], histDistText[100], prefix[100], extension[] = ".pcd";
//   int startIndex, endIndex;
  
  void computeNormals(const CloudPtr &cloud, NormalsCloudPtr &normalsCloud);
  void computeRGBHist(const CloudPtr &cloud, HistCloudPtr &RHist, HistCloudPtr &GHist, HistCloudPtr &BHist);
  void computeVFHSig(const CloudPtr &cloud, const NormalsCloudPtr &normals, VFHSigPtr &vfhs);
  void extractClustersEuclidean(const CloudPtr &cloud, vector<pcl::PointIndices> &clusterIndices);
  double getHistDistance(const DescriptorHistCloudPtr &hist1, const DescriptorHistCloudPtr &hist2);
  void process (const CloudPtr &, 
		NormalsCloudPtr &, 
		CloudPtr &, NormalsCloudPtr &, 
		vector<CloudPtr> &, 
		vector<NormalsCloudPtr> &, 
		vector<VFHSigPtr> &, 
		vector<HistCloudPtr> &, 
		vector<HistCloudPtr> &, 
		vector<HistCloudPtr> &, 
		vector<DescriptorHistCloudPtr> &, 
		vector<PlaneCoeff, 
		Eigen::aligned_allocator<PlaneCoeff> > &, 
		int &);
  void merge(const vector<VFHSigPtr> &VFHSigs, 
	     const vector<HistCloudPtr> &RHist, 
	     const vector<HistCloudPtr> &GHist, 
	     const vector<HistCloudPtr> &BHist, 
	     vector<DescriptorHistCloudPtr> &DescriptorHist);
  bool getComparisonFlag();
  double matchPlanes(const PlaneCoeff &, const PlaneCoeff& , const DescriptorHistCloudPtr &, const DescriptorHistCloudPtr &);
  double matchPlaneCoeff(const PlaneCoeff &c1, const PlaneCoeff& c2);
  void generateCorrespondences();
  void alignClusters(const CloudPtr &source, 
		     const NormalsCloudPtr&, 
		     const CloudPtr &target, 
		     const NormalsCloudPtr&, 
		     Eigen::Matrix4f &transformation);
  void generateTransformation();
  void scaleDown(const CloudPtr &source, CloudPtr &target);
  void copySourceToTarget();
  void visualize();
  static void keyboard_cb (const pcl::visualization::KeyboardEvent &event, void* viewer_void);
  bool isBetter(float, float);
  void updateGlobalTransformation(Eigen::Matrix4f);
  void computeLocalFeatures(const CloudPtr &cloud, const NormalsCloudPtr &cloudNormals, FeatureCloudPtr &features);
  void filter(const CloudPtr&, CloudPtr&);
  void smooth(const CloudPtr&, CloudPtr&);
  void scaleDownAndSmooth(const CloudPtr&, CloudPtr&);
  Eigen::Matrix4f refineAlignmentICP(const CloudPtr& , const CloudPtr& , Eigen::Matrix4f& );
  bool intersect2Planes(const PlaneCoeff& plane1, const PlaneCoeff& plane2, Eigen::VectorXf& line);
  bool intersect3Planes(const PlaneCoeff& plane1, 
			const PlaneCoeff& plane2, 
			const PlaneCoeff& plane3, 
			Eigen::Vector4f &intersect, 
			Eigen::VectorXf&, 
			Eigen::VectorXf&);
  void transformSourcePlaneCoefficients(const Eigen::Matrix4f &transformation);
  void transformSourcePivotAndLines(const Eigen::Matrix4f& transformation);
  void recursiveAlignClusters(int , int , const vector<pcl::Correspondence>& , vector<int> &, Eigen::Matrix4f& , double &);
  void recursiveAlignPlaneTriplets(int , const vector<pcl::Correspondence>& , vector<int> &, Eigen::Matrix4f& , double &);
  void getTransformationFromOdom(Eigen::Matrix4f&);
  void getRotationMatrixAboutVerticalAxis (double , Eigen::Matrix3f &);
  
  class CompareCorrespondence
  {
    bool reverse;
  public:
    CompareCorrespondence(const bool revparam=false)
    {
      reverse=revparam;
    }
    bool operator() (const pcl::Correspondence &c1, const pcl::Correspondence &c2) const
    {
      if (reverse) return (c1.distance>c2.distance);
      else return (c1.distance<c2.distance);
    }
  };
  
  class Constraint
  {
    Eigen::Vector4f source, target;
  public:
    Constraint(const Eigen::Vector4f &s, const Eigen::Vector4f &t) : source(s), target(t) {}
    double getFitnessScore(Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity()) const
    {
      Eigen::Vector4f temp = transformation*source;
      return (target-temp).norm();
    }
  };
  
  class Constraints
  {
    vector<Constraint, Eigen::aligned_allocator<Constraint> > constraints;
  public:
    void add(const Eigen::Vector4f &s, const Eigen::Vector4f &t)
    {
      Constraint c(s, t);
      constraints.push_back(c);
    }
    double getFitnessScore(Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity())
    {
      if(constraints.size()==0)
	return 0.0;
      vector<Constraint, Eigen::aligned_allocator<Constraint> >::iterator it;
      Eigen::Vector4f temp;
      double score = 0.0;
      for(it = constraints.begin(); it!=constraints.end(); ++it)
      {
	score+=(*it).getFitnessScore(transformation);
      }
      return score/constraints.size();
    }
    void clear()
    {
      constraints.clear();
    }
  }constraints;
  
public:
  RegistrationUsingClusters() : source(new Cloud), 
				sourceOriginal(new Cloud),
				target(new Cloud),
				sourceResidual(new Cloud), 
				targetResidual(new Cloud),
				sourceResidualNormals(new NormalsCloud), 
				targetResidualNormals(new NormalsCloud),
// 				registered(new Cloud),
				sourceTransformed(new Cloud), 
				sourceNormals(new NormalsCloud), 
				targetNormals(new NormalsCloud),
				globalTransformation(Eigen::Matrix4f::Identity()),
				MAX_RANSAC_ITERATIONS(100), 
				K_SEARCH(50),
				MIN_PLANE_SIZE(200), 
				MIN_CLUSTER_SIZE(200), 
				MAX_CLUSTER_SIZE(25000), 
				MAX_ICP_ITERATIONS(100),
				HIST_COMPARISON_METHOD(3),
				POINT_SIZE(3),
				SOURCE_CHOSEN_CLUSTER_INDEX(0),
				TARGET_CHOSEN_CLUSTER_INDEX(0),
				DISTANCE_THRESHOLD(0.06), 
				NORMAL_DISTANCE_WEIGHT(0.1), 
				NE_RADIUS(0.05), 
				CLUSTER_TOLERANCE(0.02), 
				LEAF_SIZE(0.04),
				NORMALIZE_VFH(true), 
				NORMALIZE_RGB(true),
				VISUALIZE(false),
				TOLERANCE(5),
				LOCAL_FEATURE_RADIUS(0.05),
				SACIA_MIN_SAMPLE_DISTANCE(0.05),
				SACIA_MAX_CORRESPONDENCE_DISTANCE(50.0),
				SACIA_MAX_ITER(50),
				ANCHORS(1),
				Z_LIMIT(FLT_MAX),
				MLS_SEARCH_RADIUS(0.1),
				PLANE_MATCH_THRESHOLD(0.02),
				CLUSTER_MATCH_THRESHOLD(0.1),
				WEIGHT_OF_COEFF(1.0),
				FITNESS_SCORE_MAX_DIST(5.0),
				sourceLine1(6),
				sourceLine2(6),
				targetLine1(6),
				targetLine2(6),
				SAC_CYCLES(10),
				SAC_WEIGHT_OF_CONSTRAINTS(0.5),
				DELTA_RED(5),
				DELTA_GREEN(5),
				DELTA_BLUE(5),
				KDTREE_SEARCH_RADIUS(0.2),
				KSEARCH_FLAG(false),
				undoUsed(true),
				useTurtlebotOdometry(false),
				usePlanes(true),
				useClusters(true),
				FS_WEIGHT_OF_AVG_DISTS(0.0),
				MLS_FLAG(false){}
				
				
  
  void setSeedCloud(const CloudPtr &cloud);
  void add(const CloudPtr &cloud);
  void add(const CloudPtr &cloud, const Eigen::Matrix4f &guess);
  void setMaxRANSACIterations(int);
  void setMaxICPIterations(int);
  void setKSearch(int);
  void setMinPlaneSize(int);
  void setMinClusterSize(int);
  void setMaxClusterSize(int);
  void setHistComparisonMethod(int);
  void setPointSize(int);
  void setDistanceThreshold(float);
  void setNormalDistanceWeight(float);
  void setNormalEstimationRadius(float);
  void setClusterTolerance(float);
  void setLeafSize(float);
  void setVFHNormalizeFlag(bool);
  void setRGBNormalizeFlag(bool);
  void setVisualize(bool);
  void setLocalFeatureRadius(float);
  void setSACIAIterations(int);
  void setSACIAMinSampleDistance(float);
  void setNumAnchors(int);
  void setZLimit(float);
  void setMLSSearchRadius(float);
  void setMLSPolynomialFitFlag(bool);
  void setPlaneMatchThreshold(float);
  void setClusterMatchThreshold(float);
  void setWeightOfCoeff(float);
  void setMaxDistForFitnessScore(float);
  void setSACCycles(int);
  void setSACWeightOfConstraints(float);
  void setDeltaRed(int);
  void setDeltaGreen(int);
  void setDeltaBlue(int);
  void setKDTreeSearchRadius(float);
  void setKSearchFlag(bool);
  double getFitnessScore (const Eigen::Matrix4f&);
  int getPointSize();
  // TODO add methods for setSACIA properties
  CloudPtr getRegisteredCloud();
  void dropPreviousCloud();
  void loadPreviousState ();
  bool isValidHistComparisonMethod(int method);
  void setUseTurtlebotOdometry (bool);
  void setUsePlanes (bool);
  void setUseClusters (bool);
  void setWeightOfAvgDists(float);
  void setMLSFlag(bool);
  
  Eigen::Vector3f sourcePivot, targetPivot;
  Eigen::VectorXf sourceLine1, sourceLine2, targetLine1, targetLine2;
};