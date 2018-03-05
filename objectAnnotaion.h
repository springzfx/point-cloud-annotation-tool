#ifndef AnnotaionOBB_H
#define AnnotaionOBB_H
#include "common.h"
#include <Eigen/Core>
#include <vector>
using namespace std;

struct BoxLabel
{
    BoxLabel(){
        type="unknown";
        this->detail.center_x=this->detail.center_y=this->detail.center_z=this->detail.yaw=0;
        this->detail.length=this->detail.width=this->detail.height=1;
    }
    BoxLabel(const Eigen::Vector3f &p1,const Eigen::Vector3f &p2){
        type="unknown";
        this->detail.center_x=(p1[0]+p2[0])/2;
        this->detail.center_y=(p1[1]+p2[1])/2;
        this->detail.center_z=(p1[2]+p2[2])/2;
        this->detail.yaw=0;
        this->detail.length=p1[0]-p2[0];
        this->detail.width=p1[1]-p2[1];
        this->detail.height=p1[2]-p2[2];

    }
    string type;
    union{
        float data[7];
        struct{
            float center_x;
            float center_y;
            float center_z;
            float length;
            float width;
            float height;
            float yaw;
        } detail;
    };
};


struct Annotation
{
    Annotation() {}
    vtkSmartPointer<vtkActor> actor;
    vtkSmartPointer<vtkTransform> t;
};

class Annotaions
{
public:
    static void computeOBB(const PointCloudT::Ptr cloud,Eigen::Vector3f &p1,Eigen::Vector3f &p2);

protected:
    vector<Annotation> annotations;

    // model coefficients (Tx, Ty, Tz, Qx, Qy, Qz, Qw, width, height, depth)
//    pcl::ModelCoefficients obbCoefficients;
};

#endif //AnnotaionOBB_H
