#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "common.h"
#include <QMainWindow>
#include <pcl/visualization/PointCloudColorHandlerLUT.h>
#include <Annotaion.h>

namespace Ui {
class MainWindow;
}

using namespace std;

class Visualizer:public QMainWindow
{
public:
    explicit Visualizer(QWidget *parent = 0);
    ~Visualizer();
    void highlightPoint(vector<int>& slice);
    void defaultColorPoint(vector<int>& slice);
    void groundColorPoint(vector<int>& slice);
    void inboxColorPoint(vector<int>& slice);
    void pickAnnotation(double x,double y);
    void createAnnotationFromSelectPoints(string type="unknown");
    void typeButtonClickedProcess(string type);
    void updateCloud();
	double* getBoxPose(void);
	void pointsInBox(void);
	void pointsOutBox(void);
protected:
    Ui::MainWindow* ui;
    PCLViewerPtr viewer;
    string pointcloudFileName;
    string annotationFileName;

	bool fullscreen;
	double width;
    double epi;
	double currentx;
	double currenty;

    /**
     * @brief the loaded cloud
     */
    PointCloudTPtr cloud;
    /**
     * @brief state of each point to identity color or selectable
     */
    int* cloudLabel;
    PointCloudColorHandlerLUT<PointT> colorHandler;
    vector<int> last_selected_slice;
    // manage annotations
    boost::shared_ptr<Annotaions> annoManager;

    /**
     * @brief show loaded cloud and annotations
     */
    void refresh();

    /**
     * @brief initialize UI, slot connection and something else
     */
    void initialize();
    /**
     * @brief open cloud file
     */
    void openFile();
    /**
     * @brief clear state before load new cloud and annotation
     */
    void clear();
    /**
     * @brief save annotations
     */
    void save();

    /**
     * @brief load APPLI or KITTI  cloud bin file
     * @param filename_
     * @param cloud_
     */
    void loadBinFile(string filename_, PointCloudT &cloud_);
    void planeDetect();
    void threshold();


private:
    void AreaPickingEventProcess(const pcl::visualization::AreaPickingEvent& event);
    void MouseEventProcess (const pcl::visualization::MouseEvent& event);
    void ConfigureEventProcess(void);
    void KeyboardEventProcess(const pcl::visualization::KeyboardEvent& event);

    // visibility
    void showAnnotation();
    void showAnnotation(const Annotation* anno);
    void removeAnnotation();
    void removeAnnotation(Annotation* anno);
	void toggleRenderers(void);
    std::clock_t start;
    double duration;

    // axes
    vtkSmartPointer<vtkOrientationMarkerWidget> axesWidget;

	vtkSmartPointer<vtkCamera> camera1;
	vtkSmartPointer<vtkCamera> camera2;
	vtkSmartPointer<vtkCamera> camera3;
    // for pick
    Annotation *currPickedAnnotation;
};

#endif // VISUALIZER_H
