#include "visualizer.h"
#include "ui_mainwindow.h"
#include "Annotaion.h"
#include "pcl/visualization/pcl_visualizer_extented.h"

#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/common/actor_map.h>
#include <boost/unordered/unordered_map_fwd.hpp>
#include <QFileDialog>
#include <QVTKWidget.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>
#include <vtkTransform.h>
#include <pcl/visualization/PointCloudColorHandlerLUT.h>
#include <pcl/visualization/mouse_event.h>
#include <vtkPropPicker.h>
#include <QBoxLayout>
#include <QMessageBox>
#include <vtkRenderWindow.h>
#include <view/flowlayout.h>

using namespace pcl::visualization;
using namespace std;
//#define connect(...) QObject::connect(__VA_ARGS__);

#define DEFAULT_POINT 0
#define SELECTED_POINT 1
#define GROUND_POINT 2

Visualizer::Visualizer(QWidget *parent)
    :QMainWindow(parent),
    ui(new Ui::MainWindow){

    ui->setupUi(this);
    this->setWindowTitle("3D annotation tool");

    initialize();

    // register selection http://www.pcl-users.org/Select-set-of-points-using-mouse-td3424113.html

    viewer->registerKeyboardCallback(boost::bind(&Visualizer::KeyboardEventProcess,this,_1));
    viewer->registerAreaPickingCallback(boost::bind(&Visualizer::AreaPickingEventProcess,this,_1));
    viewer->registerMouseCallback (boost::bind(&Visualizer::MouseEventProcess,this,_1));

    // UI
    connect(ui->action_Open,&QAction::triggered,this,&Visualizer::openFile);
    connect(ui->action_Save,&QAction::triggered,this,&Visualizer::save);
    connect(ui->action_detect_plane,&QAction::triggered,ui->PlaneDetect_dockWidget,&QDockWidget::show);
    connect(ui->action_Threshold,&QAction::triggered,ui->threshold_dockWidget,&QDockWidget::show);

    ui->PlaneDetect_dockWidget->hide();
    ui->distanceThreshold_lineEdit->setText("0.1");
    ui->distanceThreshold_lineEdit->setValidator( new QDoubleValidator(-5, 5, 2) );
    connect(ui->applyPlaneDetection_pushButton ,&QPushButton::clicked,this,&Visualizer::planeDetect);

    ui->threshold_dockWidget->hide();
    ui->threshold_lineEdit->setText("-1.5");
    ui->threshold_lineEdit->setValidator( new QDoubleValidator(-100, 100, 2) );
    connect(ui->threshold_pushButton ,&QPushButton::clicked,this,&Visualizer::threshold);
}

Visualizer::~Visualizer(){
    delete ui;
}

void Visualizer::initialize()
{
    // init viewer
    viewer.reset(new PCLViewer("",false));
    // BUG under windows, this cause execution exception
    ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->qvtkWidget->GetInteractor(),ui->qvtkWidget->GetRenderWindow());
    ui->qvtkWidget->update();

    // init label type
    Annotation::getTypes()->clear();
    Annotation::getTypes()->push_back("dontCare");
    Annotation::getTypes()->push_back("cyclist");
    Annotation::getTypes()->push_back("pedestrian");
    Annotation::getTypes()->push_back("vehicle");
    Annotation::getTypes()->push_back("unknown");
    FlowLayout *layout = new FlowLayout();
    for(auto type : *(Annotation::getTypes())){
        QPushButton* button= new QPushButton(QString::fromStdString(type));
        connect(button,&QPushButton::clicked,this,[=]() { this->typeButtonClickedProcess(button->text().toStdString()); });

        pcl::RGB c=Annotation::getColor(type);
        QPalette pal = button->palette();
        pal.setColor(QPalette::Button, QColor(c.r,c.g,c.b,c.a));
        button->setAutoFillBackground(true);
        button->setPalette(pal);
        button->update();
        layout->addWidget(button);
    }
    ui->groupBox_Types->setLayout(layout);
    ui->groupBox_Types->setMaximumWidth(200);
    ui->groupBox_Types->setMaximumHeight(200);


    //init annotation
    annoManager.reset(new Annotaions());
    currPickedAnnotation=NULL;

    //axes
    vtkSmartPointer<vtkAxesActor> axes =
            vtkSmartPointer<vtkAxesActor>::New();
    axesWidget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    axesWidget->SetOrientationMarker( axes );
//    axesWidget->SetInteractor( viewer->getInteractorStyle()->GetInteractor() );
    axesWidget->SetInteractor( viewer->getRenderWindowInteractor() );
    axesWidget->SetViewport( 0.0, 0.0, 0.2, 0.2 );
    axesWidget->SetEnabled( 1 );
    axesWidget->InteractiveOff();

    // init cloud
    cloud.reset(new PointCloudT);
    cloudLabel=NULL;
}

void Visualizer::refresh()
{
    cloudLabel=new int[cloud->size()];
    memset(cloudLabel, 0, cloud->size()*sizeof(int));

    ui->label_filename->setText(QString::fromStdString(pointcloudFileName));
    colorHandler.setInputCloud(cloud);
    colorHandler.setLabel(cloudLabel);
    viewer->addPointCloud<PointT>(cloud,colorHandler,"cloud",0);

//    // show cloud
//    //http://www.pcl-users.org/how-to-add-color-to-a-xyz-cloud-td4040030.html
//    pcl::visualization::PointCloudColorHandlerCustom<PointT> colorHandler(cloud, 255, 255, 255);
//    //    pcl::visualization::PointCloudColorHandlerGenericField<PointT> colorHandler(cloud, "intensity");


//    PointCloudColorHandlerLabelField<PointT> colorHandler;
//    colorHandler.

    viewer->resetCamera();
    ui->qvtkWidget->update();

    // show annotation if exists
    showAnnotation();
}

void Visualizer::pickAnnotation(double x,double y){
    vtkSmartPointer<vtkPropPicker>  picker =
            vtkSmartPointer<vtkPropPicker>::New();
    picker->Pick(x, y, 0, viewer->getRendererCollection()->GetFirstRenderer());
    vtkActor* pickedActor = picker->GetActor();

    if (currPickedAnnotation){
        currPickedAnnotation->unpicked();
        currPickedAnnotation=NULL;
    }
    // get the correspond annotation
    currPickedAnnotation = annoManager->getAnnotation(pickedActor);
    if (currPickedAnnotation){
        currPickedAnnotation->picked(viewer->getRenderWindowInteractor());
    }

}

void Visualizer::highlightPoint(std::vector<int>& slice)
{
    if (slice.size()<1) return;

    for (std::vector<int>::iterator it = slice.begin(); it != slice.end(); it++)    {
        cloudLabel[*it]=SELECTED_POINT;
    }
}


void Visualizer::defaultColorPoint(std::vector<int>& slice)
{
    if (slice.size()==0){
       memset(cloudLabel, DEFAULT_POINT, cloud->size()*sizeof(int));
    }

    for (std::vector<int>::iterator it = slice.begin(); it != slice.end(); it++)    {
        cloudLabel[*it]=DEFAULT_POINT;
    }
}

void Visualizer::groundColorPoint(std::vector<int>& slice)
{
    if (slice.size()<1) return;

    for (std::vector<int>::iterator it = slice.begin(); it != slice.end(); it++)    {
        cloudLabel[*it]=GROUND_POINT;
    }
}



void Visualizer::createAnnotationFromSelectPoints(string type)
{
    if (last_selected_slice.size()>3){
        Annotation* anno=new Annotation(cloud,last_selected_slice,type);
        annoManager->push_back(anno);
        showAnnotation(anno);
        ui->qvtkWidget->update();
    }else{
        std::printf("%s","no points selected");
    }
}

void Visualizer::typeButtonClickedProcess(string type)
{
    // QMessageBox::information(this, QString::fromStdString("information"),QString::fromStdString(type));
    if (currPickedAnnotation){
        currPickedAnnotation->setType(type);
        ui->qvtkWidget->update();
        return;
    }

    if (last_selected_slice.size()>3){
        createAnnotationFromSelectPoints(type);
    }
}

void Visualizer::updateCloud()
{
    viewer->updatePointCloud<PointT>(cloud,colorHandler,"cloud");
    ui->qvtkWidget->update();
}

vector<int> intersectionVector(vector<int> &a,vector<int> &b){
    vector<int> c;
    sort(a.begin(), a.end());
    sort(b.begin(), b.end());
    set_intersection(a.begin(),a.end(),b.begin(),b.end(),back_inserter(c));
    return c;
}

vector<int> unionVector(vector<int> &a,vector<int> &b){
    vector<int> c;
    set_union(a.begin(),a.end(),b.begin(),b.end(),back_inserter(c));
    return c;
}

vector<int> diffVector(vector<int> a,vector<int> b){
    vector<int> c;
    sort(a.begin(), a.end());
    sort(b.begin(), b.end());
    set_difference(a.begin(),a.end(),b.begin(),b.end(),back_inserter(c));
    return c;
}


void Visualizer::AreaPickingEventProcess (const pcl::visualization::AreaPickingEvent& event)
{
    vector<int> new_selected_slice;
    event.getPointsIndices(new_selected_slice);

    if (new_selected_slice.empty()) return;

    int s=viewer->getRenderWindowInteractor()->GetShiftKey();
    int a=viewer->getRenderWindowInteractor()->GetControlKey();

    // remove ground points
    vector<int> r;
    for (auto x:new_selected_slice){
        if (cloudLabel[x]!=GROUND_POINT){
            r.push_back(x);
        }
    }
    new_selected_slice=r;

    if (!last_selected_slice.empty()){
        defaultColorPoint(last_selected_slice);
    }

    if (s && a ){ // intersection
        last_selected_slice=intersectionVector(last_selected_slice,new_selected_slice);
    }else if (s){ // union
        last_selected_slice=unionVector(last_selected_slice,new_selected_slice);
    }else if (a){ // remove
        last_selected_slice=diffVector(last_selected_slice,new_selected_slice);
    }else{ // new
        last_selected_slice=new_selected_slice;
    }


    highlightPoint(last_selected_slice);
    updateCloud();
}


void Visualizer::MouseEventProcess (const pcl::visualization::MouseEvent& event)
{
    if (event.getButton()==pcl::visualization::MouseEvent::LeftButton
            && event.getType()==pcl::visualization::MouseEvent::MouseButtonPress){
        pickAnnotation(event.getX(), event.getY());
    }
}

void Visualizer::KeyboardEventProcess(const KeyboardEvent& event)
{
    std::cout<<event.getKeySym()<<std::endl;

    // delete annotation
    if (event.getKeySym()=="Delete" && currPickedAnnotation){
        removeAnnotation(currPickedAnnotation);
    }
}

void Visualizer::showAnnotation()
{
    for (auto anno:annoManager->getAnnotations()){
        showAnnotation(anno);
    }
}

void Visualizer::threshold(){
    double threhold_;
    if (ui->threshold_lineEdit->text().isEmpty()) return;
    threhold_=ui->threshold_lineEdit->text().toDouble();

    vector<int> slice;
    for (int i=0;i<cloud->size();i++){
        if (cloud->points[i].z<threhold_){
            slice.push_back(i);
        }
    }
    memset(cloudLabel, DEFAULT_POINT, cloud->size()*sizeof(int));
    groundColorPoint(slice);
    updateCloud();
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

    std::cout<<"plane detection: "<<inliers->indices.size()<<std::endl;

    memset(cloudLabel, DEFAULT_POINT, cloud->size()*sizeof(int));
    groundColorPoint(inliers->indices);
    updateCloud();




//    PointCloudT::Ptr groundPlaneCloud;
//    groundPlaneCloud.reset(new PointCloudT);

//    // http://docs.pointclouds.org/trunk/classpcl_1_1_extract_indices.html
//    pcl::ExtractIndices<PointT> eifilter (true); // Initializing with true will allow us to extract the removed indices
//    eifilter.setInputCloud (cloud);
//    eifilter.setIndices (inliers);
//    eifilter.filter (*groundPlaneCloud);

//    //    PointCloudT::Ptr groundPlaneCloud;
//    //    groundPlaneCloud.reset(new PointCloudT);
//    //    for (size_t i = 0; i < inliers->indices.size (); ++i){
//    //        groundPlaneCloud->push_back(cloud->points[inliers->indices[i]]);
//    //    }

//    viewer->removeAllPointClouds();
//    ui->qvtkWidget->update();
//    if (ui->removePlane_checkBox->isChecked()){
//        pcl::visualization::PointCloudColorHandlerCustom<PointT> colorHandler1(remainCloud, colors.at(1).r,colors.at(1).g,colors.at(1).b);
//        viewer->addPointCloud<PointT>(remainCloud,colorHandler1,"remainCloud");
//        ui->qvtkWidget->update();
//    }else{
//        pcl::visualization::PointCloudColorHandlerCustom<PointT> colorHandler1(remainCloud, colors.at(1).r,colors.at(1).g,colors.at(1).b);
//        pcl::visualization::PointCloudColorHandlerCustom<PointT> colorHandler2(groundPlaneCloud, colors.at(2).r,colors.at(2).g,colors.at(2).b);
//        viewer->addPointCloud<PointT>(remainCloud,colorHandler1,"remainCloud");
//        viewer->addPointCloud<PointT>(groundPlaneCloud,colorHandler2,"groundPlaneCloud");
//        ui->qvtkWidget->update();
//    }
}

void Visualizer::openFile()
{
    pointcloudFileName = QFileDialog::getOpenFileName(this, tr("Open PCD file"), "/home/fancy", tr("PCD Files (*.pcd *.bin)")).toStdString();
    if (pointcloudFileName.empty()) return;

    clear();
    QFileInfo file(QString::fromStdString(pointcloudFileName));
    QString ext = file.completeSuffix();  // ext = "bin" ,"pcd"

    if (ext=="pcd"){
        pcl::io::loadPCDFile(pointcloudFileName,*cloud);
    }else{
        loadBinFile(pointcloudFileName,*cloud);
    }

    std::cout<<pointcloudFileName<<"cloud point loaded"<<endl;
    std::cout<<"cloud point number: "<<cloud->width*cloud->height<<endl;

    annotationFileName=pointcloudFileName+".txt";
    if (QFile::exists(QString::fromStdString(annotationFileName))){
        annoManager->loadAnnotations(annotationFileName);
    }

    refresh();

}

void Visualizer::clear()
{
    removeAnnotation();
    viewer->removeAllPointClouds();
    annoManager->clear();

    currPickedAnnotation=NULL;
    last_selected_slice.clear();

    if (cloudLabel){
        delete [] cloudLabel;
        cloudLabel=NULL;
    }
}

void Visualizer::save()
{
    annoManager->saveAnnotations(annotationFileName);
}

void Visualizer::loadBinFile(string filename_,PointCloudT& cloud_)
{
    std::ifstream input(filename_.c_str(), std::ios_base::binary);
    if(!input.good()){
        std::cerr<<"Cannot open file : "<<filename_<<std::endl;
        return;
    }

    cloud_.clear();
    cloud_.height = 1;

    for (int i=0; input.good() && !input.eof(); i++) {
        PointT point;
        input.read((char *) &point.x, 3*sizeof(double));
        input.read((char *) &point.intensity, sizeof(double));
        cloud_.push_back(point);
    }
    input.close();
}

void Visualizer::showAnnotation(const Annotation* anno){
    viewer->addActorToRenderer(anno->getActor());
}

void Visualizer::removeAnnotation()
{
    for (auto anno:annoManager->getAnnotations()){
        removeAnnotation(anno);
    }
}

void Visualizer::removeAnnotation(Annotation *anno)
{
    if (currPickedAnnotation){
        currPickedAnnotation->unpicked();
        currPickedAnnotation=NULL;
    }
    viewer->removeActorFromRenderer(anno->getActor());
}

