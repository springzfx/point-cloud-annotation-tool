#include "mainwindow.h"
#include "visualizer.h"
#include "objectAnnotaion.h"
#include "pcl/visualization/pcl_visualizer.h"

#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/common/actor_map.h>
#include <pcl/filters/extract_indices.h>
#include <boost/unordered/unordered_map_fwd.hpp>
#include <QFileDialog>
#include <QVTKWidget.h>
#include <vtkArrowSource.h>
#include <vtkCubeSource.h>
#include <vtkConeSource.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>
#include <vtkTransform.h>
#include <vtkBoxWidget.h>

using namespace pcl::visualization;

class vtkMyCallback : public vtkCommand
{
public:
  static vtkMyCallback *New()
  {
    return new vtkMyCallback;
  }
  virtual void Execute( vtkObject *caller, unsigned long, void* )
  {
    vtkSmartPointer<vtkTransform> t =
      vtkSmartPointer<vtkTransform>::New();
    vtkBoxWidget *widget = reinterpret_cast<vtkBoxWidget*>(caller);
    widget->GetTransform( t );
    widget->GetProp3D()->SetUserTransform( t );
  }
};

Visualizer::Visualizer():
    MainWindow(), boxWidget(vtkSmartPointer<vtkBoxWidget>::New())
{  
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer",false));
    ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->qvtkWidget->GetInteractor(),ui->qvtkWidget->GetRenderWindow());
    ui->qvtkWidget->update();

    // register selection http://www.pcl-users.org/Select-set-of-points-using-mouse-td3424113.html
    viewer->registerAreaPickingCallback (Visualizer::AreaPickingEventOccurred,(void*)this);

//    viewer->addCoordinateSystem();
    connect(ui->action_Open,&QAction::triggered,this,&Visualizer::openFile);
    connect(ui->action_BoundBox,&QAction::triggered,this,&Visualizer::drawOBB);
    connect(ui->action_detect_plane,&QAction::triggered,this,&Visualizer::showPlaneDetectionDialog);

    ui->CLoudOperation_dockWidget->hide();
    ui->distanceThreshold_lineEdit->setText("0.05");
    ui->distanceThreshold_lineEdit->setValidator( new QDoubleValidator(-5, 5, 2, this) );
    connect(ui->applyPlaneDetection_pushButton ,&QPushButton::clicked,this,&Visualizer::planeDetect);

    cloud.reset(new PointCloudT);
    selectedCloud.reset(new PointCloudT);

    boxWidget->SetInteractor( viewer->getRenderWindowInteractor());
}



void Visualizer::highlightSelectedPoint()
{
    if (selected_slice.size()<1) return;

    selectedCloud->clear();
    for (std::vector<int>::iterator it = selected_slice.begin(); it != selected_slice.end(); it++)    {
        selectedCloud->push_back(cloud->points[*it]);
    }
    pcl::visualization::PointCloudColorHandlerCustom<PointT> colorHandler(cloud, 255, 0, 0);
    viewer->removePointCloud("selectedCloud");
    viewer->addPointCloud<PointT>(selectedCloud,colorHandler,"selectedCloud");
    ui->qvtkWidget->update();
}


void Visualizer::drawOBB()
{
    if (selectedCloud->size()>1){
        Eigen::Vector3f p1;
        Eigen::Vector3f p2;
        Annotaions::computeOBB(selectedCloud,p1,p2);
        BoxLabel label(p1,p2);
        showAnnotation(label);

//        viewer->removeShape("cube");
//        viewer->addCube(p1[0],p2[0],p1[1],p2[1],p1[2],p2[2],1.0,0.0,0.0,"cube");
////        pcl::visualization::ShapeActorMapPtr shape_actor_map_=viewer->getShapeActorMap();
////        pcl::visualization::ShapeActorMap::iterator am_it = shape_actor_map_->find("cube");
////        if (am_it!= shape_actor_map_->end ()){
////              std::printf("%s","actor found");
////              vtkSmartPointer<vtkLODActor> actor=vtkLODActor::SafeDownCast(am_it->second);
////              actor->GetProperty()->SetRepresentationToWireframe();
////        }
//        /// draw arrow
//        pcl::PointXYZ s(p1[0],p1[1],p1[2]),t(p2[0],p2[1],p2[2]);
//        viewer->addArrow(t,s,1.0,1.0,1.0,false);
        ui->qvtkWidget->update();

    }else{
        std::printf("%s","no points selected");
    }
}

void Visualizer::AreaPickingEventOccurred (const pcl::visualization::AreaPickingEvent& event, void* visualizer)
{
    // TODO whatif multi-cloud
    event.getPointsIndices(((Visualizer*)visualizer)->selected_slice);
    ((Visualizer*)visualizer)->highlightSelectedPoint();
}


void Visualizer::planeDetect(){
    if (ui->distanceThreshold_lineEdit->text().isEmpty()) return;
    double  distanceThreshold=ui->distanceThreshold_lineEdit->text().toDouble();

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (distanceThreshold);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    PointCloudT::Ptr groundPlaneCloud;
    PointCloudT::Ptr remainCloud;
    groundPlaneCloud.reset(new PointCloudT);
    remainCloud.reset(new PointCloudT);

    // http://docs.pointclouds.org/trunk/classpcl_1_1_extract_indices.html
    pcl::ExtractIndices<PointT> eifilter (true); // Initializing with true will allow us to extract the removed indices
    eifilter.setInputCloud (cloud);
    eifilter.setIndices (inliers);
    eifilter.filter (*groundPlaneCloud);
    eifilter.setNegative (true);
    eifilter.filter (*remainCloud);

    //    PointCloudT::Ptr groundPlaneCloud;
    //    groundPlaneCloud.reset(new PointCloudT);
    //    for (size_t i = 0; i < inliers->indices.size (); ++i){
    //        groundPlaneCloud->push_back(cloud->points[inliers->indices[i]]);
    //    }

    viewer->removeAllPointClouds();
    ui->qvtkWidget->update();
    if (ui->removePlane_checkBox->isChecked()){
        pcl::visualization::PointCloudColorHandlerCustom<PointT> colorHandler1(remainCloud, colors.at(1).r,colors.at(1).g,colors.at(1).b);
        viewer->addPointCloud<PointT>(remainCloud,colorHandler1,"remainCloud");
        ui->qvtkWidget->update();
    }else{
        pcl::visualization::PointCloudColorHandlerCustom<PointT> colorHandler1(remainCloud, colors.at(1).r,colors.at(1).g,colors.at(1).b);
        pcl::visualization::PointCloudColorHandlerCustom<PointT> colorHandler2(groundPlaneCloud, colors.at(2).r,colors.at(2).g,colors.at(2).b);
        viewer->addPointCloud<PointT>(remainCloud,colorHandler1,"remainCloud");
        viewer->addPointCloud<PointT>(groundPlaneCloud,colorHandler2,"groundPlaneCloud");
        ui->qvtkWidget->update();
    }


    std::cout<<"plane detection: "<<groundPlaneCloud->size()<<endl;
    std::cout<<"remain cloud: "<<remainCloud->size()<<endl;
}

void Visualizer::openFile()
{
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    fileName = QFileDialog::getOpenFileName(this, tr("Open PCD file"), "/home/fancy", tr("PCD Files (*.pcd *.bin)")).toStdString();
    if (fileName.empty()) return;

    pcl::io::loadPCDFile(fileName,*cloud);
    std::cout<<"cloud point number: "<<cloud->width*cloud->height<<endl;

    //http://www.pcl-users.org/how-to-add-color-to-a-xyz-cloud-td4040030.html
    pcl::visualization::PointCloudColorHandlerCustom<PointT> colorHandler(cloud, 255, 255, 255);
    //    pcl::visualization::PointCloudColorHandlerGenericField<PointT> colorHandler(cloud, "intensity");
    viewer->addPointCloud<PointT>(cloud,colorHandler,"cloud",0);
    viewer->resetCamera();
    ui->qvtkWidget->update();

}

void Visualizer::showPlaneDetectionDialog(){
    ui->CLoudOperation_dockWidget->show();
}

LabelActor Visualizer::showAnnotation(const BoxLabel &label){
    ///////////////////////////////////////////////////
    /// Create an cube
    ///////////////////////////////////////////////////
    vtkSmartPointer<vtkCubeSource> cubeSource =
    vtkSmartPointer<vtkCubeSource>::New();

    vtkSmartPointer<vtkPolyDataMapper> cubeMapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
    cubeMapper->SetInputConnection(cubeSource->GetOutputPort());

    vtkSmartPointer<vtkActor> cubeActor =
    vtkSmartPointer<vtkActor>::New();
    cubeActor->SetMapper(cubeMapper);
    cubeActor->GetProperty()->SetRepresentationToWireframe();
    cubeActor->GetProperty ()->SetLighting (false);

    vtkSmartPointer<vtkTransform> cubeTransform =
    vtkSmartPointer<vtkTransform>::New();
    cubeTransform->PostMultiply();
    cubeTransform->Scale(label.detail.length,label.detail.width,label.detail.height);
    cubeTransform->RotateZ(label.detail.yaw);
    cubeTransform->Translate(label.detail.center_x,label.detail.center_y,label.detail.center_z);
    cubeActor->SetUserTransform(cubeTransform);
    viewer->addActorToRenderer(cubeActor);


    ///////////////////////////////////////////////////
    /// Create an arrow.
    ///////////////////////////////////////////////////
     vtkSmartPointer<vtkArrowSource> arrowSource =
       vtkSmartPointer<vtkArrowSource>::New();
     //arrowSource->SetShaftRadius(1.0);
     //arrowSource->SetTipLength(1.0);
//     arrowSource->SetShaftRadius(label.detail.length);
     arrowSource->Update();

     // Create a mapper and actor
     vtkSmartPointer<vtkPolyDataMapper> arrowMapper =
       vtkSmartPointer<vtkPolyDataMapper>::New();
     arrowMapper->SetInputConnection(arrowSource->GetOutputPort());
     vtkSmartPointer<vtkActor> arrowActor =
       vtkSmartPointer<vtkActor>::New();
     arrowActor->SetMapper(arrowMapper);
     arrowActor->GetProperty ()->SetLighting (false);
//     arrowActor->SetUserTransform(cubeTransform);
     viewer->addActorToRenderer(arrowActor);


//     setup boxwidget
    boxWidget->Off();
    boxWidget->SetProp3D(cubeActor);
    boxWidget->SetPlaceFactor(1.25); // Make the box 1.25x larger than the actor
    boxWidget->PlaceWidget();

    vtkSmartPointer<vtkMyCallback> callback =
    vtkSmartPointer<vtkMyCallback>::New();
    boxWidget->AddObserver( vtkCommand::InteractionEvent, callback );

    boxWidget->On();

    LabelActor labelActor(cubeActor,arrowActor);
    return labelActor;
}

void Visualizer::showAxes(){
    vtkSmartPointer<vtkAxesActor> axes =
    vtkSmartPointer<vtkAxesActor>::New();
    viewer->addActorToRenderer(axes);

//    vtkSmartPointer<vtkOrientationMarkerWidget> widget =
//    vtkSmartPointer<vtkOrientationMarkerWidget>::New();
//    widget->SetOutlineColor( 0.9300, 0.5700, 0.1300 );
//    widget->SetOrientationMarker( axes );
//    widget->SetInteractor( viewer->getInteractorStyle()->GetInteractor() );
//    widget->SetViewport( 0.0, 0.0, 0.4, 0.4 );
//    widget->SetEnabled( 1 );
//    widget->InteractiveOn();
}

void Visualizer::test(){

}
