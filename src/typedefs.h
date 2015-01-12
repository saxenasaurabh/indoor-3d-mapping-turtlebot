typedef pcl::PointXYZRGB PointType;
typedef pcl::PointNormal NormalPointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef pcl::PointCloud<NormalPointType> NormalsCloud;
typedef NormalsCloud::Ptr NormalsCloudPtr;
typedef pcl::PointCloud<pcl::VFHSignature308> VFHSig;
typedef VFHSig::Ptr VFHSigPtr;
typedef pcl::Histogram<HSIZE> HistPointType;
typedef pcl::PointCloud<HistPointType> HistCloud;
typedef HistCloud::Ptr HistCloudPtr;
typedef pcl::search::KdTree<PointType> SearchType;
typedef SearchType::Ptr SearchTypePtr;

typedef Eigen::Vector4f PlaneCoeff;

// typedef pcl::FPFHSignature33 FeaturePointType;
typedef pcl::PFHRGBSignature250 FeaturePointType;
// typedef pcl::FPFHEstimation<PointType, NormalPointType, FeaturePointType> FeatureEstimationMethodType;
typedef pcl::PFHRGBEstimation<PointType, NormalPointType, FeaturePointType> FeatureEstimationMethodType;
typedef pcl::PointCloud<FeaturePointType> FeatureCloud;
typedef FeatureCloud::Ptr FeatureCloudPtr;

typedef pcl::Histogram<DESCRIPTOR_HSIZE> DescriptorHistPointType;
typedef pcl::PointCloud<DescriptorHistPointType> DescriptorHistCloud;
typedef DescriptorHistCloud::Ptr DescriptorHistCloudPtr;