#ifndef COMMON_H
#define COMMON_H

#include "pcl/visualization/pcl_visualizer_extented.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// we only use XYZI
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef boost::shared_ptr<PointCloudT> PointCloudTPtr;
typedef PCLVisualizerExtented PCLViewer;
typedef boost::shared_ptr<PCLVisualizerExtented> PCLViewerPtr;

#endif // COMMON_H
