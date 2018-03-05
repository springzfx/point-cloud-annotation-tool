#include "objectAnnotaion.h"
#include <Eigen/Core>
#include <iostream>

void Annotaions::computeOBB(const  PointCloudT::Ptr  cloud, Eigen::Vector3f &p1, Eigen::Vector3f &p2)
{
    p1(0)=std::numeric_limits <float>::max ();
    p1(1)=std::numeric_limits <float>::max ();
    p1(2)=std::numeric_limits <float>::max ();

    p2(0)=-std::numeric_limits <float>::max ();
    p2(1)=-std::numeric_limits <float>::max ();
    p2(2)=-std::numeric_limits <float>::max ();


    for (int i=0;i<cloud->size();i++){
        p1(0)=std::min(p1(0),cloud->points[i].x);
        p1(1)=std::min(p1(1),cloud->points[i].y);
        p1(2)=std::min(p1(2),cloud->points[i].z);

        p2(0)=std::max(p2(0),cloud->points[i].x);
        p2(1)=std::max(p2(1),cloud->points[i].y);
        p2(2)=std::max(p2(2),cloud->points[i].z);

    }
}

