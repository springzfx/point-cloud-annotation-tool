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
#include <vtkCamera.h>
#include <vtkTextActor.h>
#include <vtkTextWidget.h>
#include <vtkTextRepresentation.h>
#include <vtkRenderer.h>
#include <pcl/visualization/PointCloudColorHandlerLUT.h>
#include <pcl/visualization/mouse_event.h>
#include <vtkPropPicker.h>
#include <QBoxLayout>
#include <QMessageBox>
#include <vtkRenderWindow.h>
#include <view/flowlayout.h>

#include <ctime>

using namespace pcl::visualization;
using namespace std;

#define DEFAULT_POINT 0
#define SELECTED_POINT 1
#define GROUND_POINT 2
#define INBOX_POINT 3

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
		
		//Link ConfigureEventProcess to interactor ConfigureEvent to update message box with the current selected boxwidget
		ui->qvtkWidget->GetInteractor()->AddObserver(vtkCommand::ConfigureEvent, this, &Visualizer::ConfigureEventProcess);
		
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

		//Flags to toggle between fullscreen and normal mode
		fullscreen = false;
		//Mouse click points
		currentx = 0.0;
		currenty = 0.0;
		//Timer to prevent multiple button press for full screen
		start = std::clock();
}

Visualizer::~Visualizer(){
	delete ui;
}

/* @brief: Helper initialize function. Creates the UI layout, render windows & axes widget
 * */
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

	//Creates 4 windows to view point clouds
	width = 0.6; // Width of main view
	epi = 0.001; // create boarders for views

	viewer->getRendererCollection()->InitTraversal();
	vtkRenderer* temp = NULL;
	temp = viewer->getRendererCollection()->GetNextItem();	
	temp->SetViewport(0.0, 0.0, width - epi, 1);	
	
	int v1(1);
	viewer->createViewPort (width + epi, 0.671, 1.0, 1.00, v1);
	viewer->setBackgroundColor (.2, .2, .2, v1);

	int v2(2);
	viewer->createViewPort (width + epi, 0.331, 1.0, 0.669, v2);
	viewer->setBackgroundColor (.2, .2, .2, v2);

	int v3(3);
	viewer->createViewPort (width + epi, 0.00, 1.0, 0.329, v3);
	viewer->setBackgroundColor (.2, .2, .2, v3);	

	//Create a camera pointer for each render, set when selecting an actor
	camera1 =  vtkSmartPointer<vtkCamera>::New();
	camera2 =  vtkSmartPointer<vtkCamera>::New();
	camera3 =  vtkSmartPointer<vtkCamera>::New();

	//axes
	vtkSmartPointer<vtkAxesActor> axes =
		vtkSmartPointer<vtkAxesActor>::New();
	axesWidget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
	axesWidget->SetOrientationMarker( axes );
	axesWidget->SetInteractor( viewer->getRenderWindowInteractor() );
	axesWidget->SetViewport( 0.0, 0.0, 0.2, 0.2 );
	axesWidget->SetEnabled( 1 );
	axesWidget->InteractiveOff();

	//init annotation
	annoManager.reset(new Annotaions());
	currPickedAnnotation=NULL;

	// init cloud
	cloud.reset(new PointCloudT);
	cloudLabel=NULL;
}


/*---------------------------------- Annotation ---------------------------------------------*/
/* @brief: Pick the annotation within screen. If an actor already exist, unselect it (this will only work in another render), 
 * 		   else select another actor, else let the user pan the camera. 
 * 		   If an actor is select, configure the top, front and side view to zero in on the actor
 * @param x: x position of mouse click
 * @param y: y position of mouse click
 * */
void Visualizer::pickAnnotation(double x,double y){
	vtkSmartPointer<vtkPropPicker>  picker =
		vtkSmartPointer<vtkPropPicker>::New();

	//Change renderer based on where the mouse is clicked
	vtkRenderer* renderer = viewer->getRenderWindow()->GetInteractor()->FindPokedRenderer(x, y);
	picker->Pick(x, y, 0, renderer);
	vtkActor* pickedActor = picker->GetActor();
	
	// get the correspond annotation
	Annotation *clicked = annoManager->getAnnotation(pickedActor);
	if(clicked){
		pointsOutBox();
		if(currPickedAnnotation != NULL){
			currPickedAnnotation->unpicked();
		}
		
		if(currPickedAnnotation == clicked){
			currPickedAnnotation = NULL;
			return;
		}

		currPickedAnnotation = clicked;	
		currPickedAnnotation->picked(viewer->getRenderWindowInteractor());
		viewer->getRenderWindow()->GetInteractor()->ConfigureEvent();	
		
		//Find the actor position, orientation
		double *act_ori;
		act_ori = currPickedAnnotation->getBoxOri();
		double x_ori, y_ori, z_ori;
		x_ori = *act_ori;
		y_ori = *(act_ori + 1);
		z_ori = *(act_ori + 2);

		double *act_pos;
		act_pos = currPickedAnnotation->getBoxPose();
		double x, y ,z;
		x = *act_pos;
		y = *(act_pos + 1);
		z = *(act_pos + 2);

		//For each render, set the camera to face the actor
		viewer->getRendererCollection()->InitTraversal();
		vtkRenderer* temp = NULL;
		viewer->getRendererCollection()->GetNextItem();	

		//top view	
		temp = viewer->getRendererCollection()->GetNextItem();
		camera1->SetPosition(x,y,z+15);
		camera1->SetFocalPoint(x,y,z);
		camera1->SetRoll(-z_ori);
		temp->SetActiveCamera(camera1);	
		//Front_view
		double front_x = cos((vtkMath::Pi()*z_ori)/180.0)*(10.0) + x;
		double front_y = sin((vtkMath::Pi()*z_ori)/180.0)*(10.0) + y;
		temp = viewer->getRendererCollection()->GetNextItem();
		camera2->SetPosition(front_x,front_y,z);
		camera2->SetFocalPoint(x,y,z);
		camera2->SetViewUp(0,0,1);
		temp->SetActiveCamera(camera2);	
	
		//Side view
		double side_x = cos((vtkMath::Pi()*(z_ori - 90.0))/180.0)*(10.0) + x;
		double side_y = sin((vtkMath::Pi()*(z_ori - 90.0))/180.0)*(10.0) + y;
		temp = viewer->getRendererCollection()->GetNextItem();
		camera3->SetPosition(side_x,side_y,z);
		camera3->SetFocalPoint(x,y,z);
		camera3->SetViewUp(0,0,1);
		temp->SetActiveCamera(camera3);	
	
		pointsInBox();

	}

}

/* @brief: Show all annotation to render
 * */
void Visualizer::showAnnotation()
{
	for (auto anno:annoManager->getAnnotations()){
		showAnnotation(anno);
	}
}

/* @brief: show specific annotation to renderer
 * @param anno: annotation to show
 * */
void Visualizer::showAnnotation(const Annotation* anno){
	viewer->addActorToRenderer(anno->getActor());
}

/* @brief: remove all annotation to render
 * */
void Visualizer::removeAnnotation()
{
	for (auto anno:annoManager->getAnnotations()){
		removeAnnotation(anno);
	}
}

/* @brief: remove specific annotation to renderer
 * @param anno: annotation to remove
 * */
void Visualizer::removeAnnotation(Annotation *anno)
{
	if (currPickedAnnotation){
		currPickedAnnotation->unpicked();
		currPickedAnnotation=NULL;
	}
	viewer->removeActorFromRenderer(anno->getActor());
}


/*---------------------- Creating Annotation -----------------------------*/
/* @brief: create an annotation if button is pressed
 * @param type: what type of labels
 * */
void Visualizer::typeButtonClickedProcess(string type)
{
	if (currPickedAnnotation){
		currPickedAnnotation->setType(type);
		ui->qvtkWidget->update();
		return;
	}

	if (last_selected_slice.size()>3){
		createAnnotationFromSelectPoints(type);
	}
}

/* @brief: From the list of points, create an annotation
 * @param type: type of labels to create
 * */
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

/*---------------------Cloud Manupilation-----------------------------------------*/
/* @brief: updates the clour the all the cloud in the various view ports
 * */
void Visualizer::updateCloud()
{
	viewer->updatePointCloud<PointT>(cloud,colorHandler,"cloud");
	viewer->updatePointCloud<PointT>(cloud,colorHandler,"cloud1");
	viewer->updatePointCloud<PointT>(cloud,colorHandler,"cloud2");
	viewer->updatePointCloud<PointT>(cloud,colorHandler,"cloud3");
	ui->qvtkWidget->update();
}

/* @brief: helper to find the intersection of point cloud
 * @param a: vector of points
 * @param b: vector of points 
 * @return intersection point cloud
 * */
vector<int> intersectionVector(vector<int> &a,vector<int> &b){
	vector<int> c;
	sort(a.begin(), a.end());
	sort(b.begin(), b.end());
	set_intersection(a.begin(),a.end(),b.begin(),b.end(),back_inserter(c));
	return c;
}

/* @brief: helper to find the union of point cloud
 * @param a: vector of points
 * @param b: vector of points 
 * @return union point cloud
 * */
vector<int> unionVector(vector<int> &a,vector<int> &b){
	vector<int> c;
	set_union(a.begin(),a.end(),b.begin(),b.end(),back_inserter(c));
	return c;
}

/* @brief: helper to find the difference between  point cloud
 * @param a: vector of points
 * @param b: vector of points 
 * @return diff point cloud
 * */
vector<int> diffVector(vector<int> a,vector<int> b){
	vector<int> c;
	sort(a.begin(), a.end());
	sort(b.begin(), b.end());
	set_difference(a.begin(),a.end(),b.begin(),b.end(),back_inserter(c));
	return c;
}

/* @brief: Select the are and highlight points. 
 * 		   If shfit key is pressed, union the points
 * 		   If ctrl key is press, remove the points
 * @param event: event triggered by pcl visualizer
 * */
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

/* @brief: find the points in the box, using the right side method.
 * 		   Find the vector of line from p1 p2 of bounding box and 
 * 		   compute the dot product. If it is <= 0, then it is on the line 
 * 		   and on the right side of the line. Repeate for all 4 side of box
 * */
void Visualizer::pointsInBox(void){

	BoxLabel label;
	label = currPickedAnnotation->getBoxLabel();
	double min_box_z = label.data[2] - label.data[5]/2.0;
	double max_box_z = label.data[2] + label.data[5]/2.0;

	double sd = sin(label.data[6]);
	double cd = cos(label.data[6]);

	double x1 = label.data[0] + (cd*(-label.data[3]/2.0) - sd*(-label.data[4]/2.0));
	double y1 = label.data[1] + (sd*(-label.data[3]/2.0) + cd*(-label.data[4]/2.0));

	double x2 = label.data[0] + (cd*(-label.data[3]/2.0) - sd*(label.data[4]/2.0));
	double y2 = label.data[1] + (sd*(-label.data[3]/2.0) + cd*(label.data[4]/2.0));

	double x3 = label.data[0] + (cd*(label.data[3]/2.0) - sd*(label.data[4]/2.0));
	double y3 = label.data[1] + (sd*(label.data[3]/2.0) + cd*(label.data[4]/2.0));

	double x4 = label.data[0] + (cd*(label.data[3]/2.0) - sd*(-label.data[4]/2.0));
	double y4 = label.data[1] + (sd*(label.data[3]/2.0) + cd*(-label.data[4]/2.0));

	vector<int> slice;
	for (int i=0;i<cloud->size();i++){
		//If Point in current selected bounding box
		double s1 = (x2 - x1)*(cloud->points[i].y - y1) - (cloud->points[i].x - x1)*(y2 - y1);
		double s2 = (x3 - x2)*(cloud->points[i].y - y2) - (cloud->points[i].x - x2)*(y3 - y2);
		double s3 = (x4 - x3)*(cloud->points[i].y - y3) - (cloud->points[i].x - x3)*(y4 - y3);
		double s4 = (x1 - x4)*(cloud->points[i].y - y4) - (cloud->points[i].x - x4)*(y1 - y4);

		if( s1 <= 0.0 && s2 <=0.0 && s3<=0.0 && s4<=0.0	&&
	 	   cloud->points[i].z > min_box_z && cloud->points[i].z < max_box_z){
			slice.push_back(i);
		}
	}
	memset(cloudLabel, DEFAULT_POINT, cloud->size()*sizeof(int));
	inboxColorPoint(slice);
	updateCloud();
}


void Visualizer::pointsOutBox(void){
	vector<int> slice;
	for (int i=0;i<cloud->size();i++){
			slice.push_back(i);
	}
	memset(cloudLabel, DEFAULT_POINT, cloud->size()*sizeof(int));
	defaultColorPoint(slice);
	updateCloud();

}
/*---------------------------------------------Color points----------------------------------------------*/
/* @brief: highlights points that are selected
 * @param: indexes of points selected 
 * */
void Visualizer::highlightPoint(std::vector<int>& slice)
{
	if (slice.size()<1) return;

	for (std::vector<int>::iterator it = slice.begin(); it != slice.end(); it++)    {
		cloudLabel[*it]=SELECTED_POINT;
	}
}

/* @brief: make all the points the default color
 * @param: indexes of points selected 
 * */
void Visualizer::defaultColorPoint(std::vector<int>& slice)
{
	if (slice.size()==0){
		memset(cloudLabel, DEFAULT_POINT, cloud->size()*sizeof(int));
	}

	for (std::vector<int>::iterator it = slice.begin(); it != slice.end(); it++)    {
		cloudLabel[*it]=DEFAULT_POINT;
	}
}

/* @brief: make all the points selected ground color
 * @param: indexes of points selected 
 * */
void Visualizer::groundColorPoint(std::vector<int>& slice)
{
	if (slice.size()<1) return;

	for (std::vector<int>::iterator it = slice.begin(); it != slice.end(); it++)    {
		cloudLabel[*it]=GROUND_POINT;
	}
}

/* @brief: make all the points within a bounding box green color
 * @param: indexes of points selected 
 * */
void Visualizer::inboxColorPoint(std::vector<int>& slice)
{
	if (slice.size()<1) return;

	for (std::vector<int>::iterator it = slice.begin(); it != slice.end(); it++)    {
		cloudLabel[*it]=INBOX_POINT;
	}
}

/*-------------------------------- interactor  events------------------------------------------*/
void Visualizer::MouseEventProcess (const pcl::visualization::MouseEvent& event)
{
	if (event.getButton()==pcl::visualization::MouseEvent::LeftButton
			&& event.getType()==pcl::visualization::MouseEvent::MouseButtonPress){

		pickAnnotation(event.getX(), event.getY());

		currentx = event.getX();
		currenty = event.getY();
	}
}

/* @brief: events that triggers when actor is selected and moving the handles
 * 		   triggers to update label message box
 * */
void Visualizer::ConfigureEventProcess(){
	
	if(currPickedAnnotation != NULL){
			
			BoxLabel label;
			label = currPickedAnnotation->getBoxLabel();
			
			std::string temp[7];
			for(int i=0; i <7; i++){
				temp[i] = std::to_string(label.data[i]);	
				temp[i].resize(7);
			}
			std::string str = "X:"       + temp[0]  + " Lenght:"  + temp[3]+
			                  "\r\nY:"   + temp[1]  + " Width :"  + temp[4]+
							  "\r\nZ:"   + temp[2]  + " Height:" + temp[5]+ 
							  "\r\nYaw:" + temp[6]  + " deg";

			ui->label_message->setText(QString::fromStdString(str));
	}	
}

void Visualizer::KeyboardEventProcess(const KeyboardEvent& event)
{

	// delete annotation
	if (event.getKeySym()=="Delete" && currPickedAnnotation){
		removeAnnotation(currPickedAnnotation);
	}
	
	//Toggle fullscreen between renders
	int c=viewer->getRenderWindowInteractor()->GetControlKey();
	char* z=viewer->getRenderWindowInteractor()->GetKeySym();

	duration = ((std::clock() - start)*1000.0) / (double) CLOCKS_PER_SEC;
	if (*z=='z' && c && duration > 35.0){
		start = std::clock();	
		toggleRenderers();
	}

}

/*------------------------------------------File related functions--------------------------------------------------*/
/* @brief: Open files. Only accpts pcd and bin fils for point cloud and txt file for annotation.
 * 		   Prints out instruction for user
 * */
void Visualizer::openFile()
{
	//change location of open file to user desktop
	string path(getenv("HOME"));
	path += "/Desktop/";
	QString qstr = QString::fromStdString(path);

	pointcloudFileName = QFileDialog::getOpenFileName(this, tr("Open PCD file"), qstr, tr("PCD Files (*.pcd *.bin)")).toStdString();
	if (pointcloudFileName.empty()) return;

	clear();
	QFileInfo file(QString::fromStdString(pointcloudFileName));
	QString ext = file.completeSuffix();  // ext = "bin" ,"pcd"

	if (ext=="pcd"){
		pcl::io::loadPCDFile(pointcloudFileName,*cloud);
	}else{
		loadBinFile(pointcloudFileName,*cloud);
	}

	std::cout<<pointcloudFileName<<" cloud point loaded"<<endl;
	std::cout<<" cloud point number: "<<cloud->width*cloud->height<<endl;
		
	
	annotationFileName=pointcloudFileName+".txt";
	if (QFile::exists(QString::fromStdString(annotationFileName))){
		annoManager->loadAnnotations(annotationFileName);
	}

	refresh();
	std::cout<<"Instructions: "<<std::endl;
	std::cout<<"    Mouse Center : Pan camera view"<<std::endl;
	std::cout<<"    Shift + mouse: Pan camera view"<<std::endl;
	std::cout<<"    Ctrl  + mouse: Rotate camera view"<<std::endl;
	std::cout<<"    Ctrl + z     : toggle between normal view and fullscreen of current render window"<<std::endl;
	std::cout<<"    x            : toggle between select mode and normal mode"<<std::endl;	
	std::cout<<"In Select mode"<<std::endl;	
	std::cout<<"    Shift + mouse: Union selected points"<<std::endl;	
	std::cout<<"    Ctrl  + mouse: Remove selected points"<<std::endl;
}

/* @breif: clears the annotation and point cloud
 * */
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

/* @brief: when a new file is loaded, it reinit all the point cloud and annotations
 * */
void Visualizer::refresh()
{
	cloudLabel=new int[cloud->size()];
	memset(cloudLabel, 0, cloud->size()*sizeof(int));

	ui->label_filename->setText(QString::fromStdString(pointcloudFileName));
	colorHandler.setInputCloud(cloud);
	colorHandler.setLabel(cloudLabel);

	viewer->addPointCloud<PointT>(cloud,colorHandler,"cloud",0);
	viewer->addPointCloud<PointT>(cloud,colorHandler,"cloud1",1);
	viewer->addPointCloud<PointT>(cloud,colorHandler,"cloud2",2);
	viewer->addPointCloud<PointT>(cloud,colorHandler,"cloud3",3);

	viewer->resetCamera();
	ui->qvtkWidget->update();

	// show annotation if exists
	showAnnotation();
}

/* @brief: save annotation to current point cloud location
 * */
void Visualizer::save()
{
	annoManager->saveAnnotations(annotationFileName);
}

/* @brief: load binary point cloud files 
 * */
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



/*----------------------------------------------------------- Extended Functions ------------------------------------------------*/
/*@brief: Gets the threshold value set from the UI*/
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

/* @brief: when button to detect plane is click, it trys to find the ground plane*/
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

	memset(cloudLabel, DEFAULT_POINT, cloud->size()*sizeof(int));
	groundColorPoint(inliers->indices);
	updateCloud();

}

/*--------------------------------View port Manipulation--------------------------------------------*/
/* @brief: Toggles the viewport from full screen to zoomed in view of the selected viewport
 * */
void Visualizer::toggleRenderers(void){
	
	if(fullscreen){
		fullscreen = false;
	
		viewer->getRendererCollection()->InitTraversal();
		vtkRenderer* temp = NULL;
		temp = viewer->getRendererCollection()->GetNextItem();	
		temp->SetViewport(0.0, 0.0, width - epi, 1);	
		temp->SetBackground (.0, .0, .0);

		temp = viewer->getRendererCollection()->GetNextItem();
		temp->SetViewport (width + epi, 0.67, 1.0, 1.00);
		temp->SetBackground (.2, .2, .2);

		temp = viewer->getRendererCollection()->GetNextItem();
		temp->SetViewport(width + epi, 0.330, 1.0, 0.67);
		temp->SetBackground (.25, .25, .25);

		temp = viewer->getRendererCollection()->GetNextItem();
		temp->SetViewport (width + epi, 0.00, 1.0, 0.33);
		temp->SetBackground (.2, .2, .2);


	}else{
		//Find the current renderer
		vtkRenderer* renderer = viewer->getRenderWindow()->GetInteractor()->FindPokedRenderer(currentx, currenty);
	
		viewer->getRendererCollection()->InitTraversal();
		vtkRenderer* temp = NULL;
		
		for(int i =0; i < 4; i ++){
		
			temp = viewer->getRendererCollection()->GetNextItem();	
			if(temp == renderer){
				temp->SetViewport (0.0, 0.0, 1., 1.);
			}else{
				temp->SetViewport (0.0, 0.0, 0.001, 0.001);
			}
		}
		
		fullscreen = true;
	}

}


