#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "common.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "objectAnnotaion.h"
#include <vtkBoxWidget.h>

struct LabelActor
{
    LabelActor(vtkActor *a,vtkActor *b) {
        cubeActor=a;
        arrowActor=b;
    }
    vtkSmartPointer<vtkActor> cubeActor;
    vtkSmartPointer<vtkActor> arrowActor;
};

class Visualizer:public MainWindow
{
public:
    Visualizer();
    ~Visualizer(){
        boxWidget->Delete();
    };
    void highlightSelectedPoint();
    void drawOBB();
protected:
    std::string fileName;
    PointCloudT::Ptr cloud;
    PointCloudT::Ptr selectedCloud;
    pcl::GlasbeyLUT colors;
    std::vector<int> selected_slice;
    Annotaions objectAnnotation;


    void planeDetect();
    void openFile();
    void showPlaneDetectionDialog();

    //box annotation
    vtkSmartPointer<vtkBoxWidget> boxWidget;

private:
    static void AreaPickingEventOccurred(const pcl::visualization::AreaPickingEvent& event, void* visualizer);
    LabelActor showAnnotation(const BoxLabel &label);
    void showAxes();
    void test();
};

#endif // VISUALIZER_H
