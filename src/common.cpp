#include "common.h"

void filter(const CloudPtr& src, CloudPtr& tgt, float Z_LIMIT=4.0f)
{
  boost::shared_ptr<std::vector<int> > indices(new std::vector<int>);
  pcl::removeNaNFromPointCloud(*src, *tgt, *indices);
  pcl::PassThrough<PointType> pass;
  pass.setInputCloud (tgt);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, Z_LIMIT);
  pass.filter (*tgt);
}

void scaleDown(const CloudPtr &source, CloudPtr &target, float LEAF_SIZE=0.04)
{
  target->clear();
  pcl::VoxelGrid<PointType> sor;
  sor.setInputCloud (source);
  sor.setLeafSize (LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
  sor.filter (*target);
}

Eigen::Matrix4f ICPAlign(const CloudPtr& src, const CloudPtr& tgt, Eigen::Matrix4f& transformation, int MAX_ICP_ITERATIONS = 50)
{
  CloudPtr aligned1(new Cloud);
  pcl::IterativeClosestPoint<PointType, PointType> icp;
  icp.setInputCloud(src);
  icp.setInputTarget(tgt);
  icp.setMaximumIterations(MAX_ICP_ITERATIONS);
  icp.align(*aligned1);
  transformation = icp.getFinalTransformation();
}