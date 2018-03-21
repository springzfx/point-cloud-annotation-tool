#include "pcl/visualization/pcl_visualizer.h"
using namespace std;
using namespace pcl::visualization;

typedef pcl::PointCloud<pcl::PointXYZI> PointCloudT;

bool loadBinFile(string filename_,PointCloudT& cloud_)
{
    std::ifstream input(filename_.c_str(), std::ios_base::binary);
    if(!input.good()){
        std::cerr<<"Cannot open file : "<<filename_<<std::endl;
        return false;
    }

    cloud_.clear();
    cloud_.height = 1;

    for (int i=0; input.good() && !input.eof(); i++) {
        pcl::PointXYZI point;
        input.read((char *) &point.x, 3*sizeof(double));
        input.read((char *) &point.intensity, sizeof(double));
        cloud_.push_back(point);
    }
    input.close();
    return true;
}

int main(int argc, char *argv[]){
    if (argc<=1){
        std::cerr<<"usage:: visiualizer pointCloudFilename"<<std::endl;
        return 0;
    }

    string filename=argv[1];
    PointCloudT::Ptr cloud(new PointCloudT);
    if (!loadBinFile(filename,*cloud)){
        return 0;
    }

    PCLVisualizer viewer;
    viewer.addPointCloud<pcl::PointXYZI>(cloud);
    viewer.spin(); //loop
    return 0;
}
