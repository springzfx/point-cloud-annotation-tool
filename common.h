#ifndef COMMON_H
#define COMMON_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl/visualization/pcl_visualizer.h"

typedef pcl::PointXYZI  PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef PointCloudT::Ptr PointCloudTPtr;
typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> PCLViewer;

#endif // COMMON_H
