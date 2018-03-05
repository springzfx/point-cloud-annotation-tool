#include "pcl/visualization/pcl_visualizer.h"
#include <vtkSmartPointer.h>
// For the rendering pipeline setup:
#include <vtkConeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
// For vtkBoxWidget:
#include <vtkBoxWidget.h>
#include <vtkCommand.h>
#include <vtkTransform.h>
 
class vtkMyCallback : public vtkCommand
{
public:
  static vtkMyCallback *New()
  {
    return new vtkMyCallback;
  }
  virtual void Execute( vtkObject *caller, unsigned long, void* )
  {
    // Here we use the vtkBoxWidget to transform the underlying coneActor
    // (by manipulating its transformation matrix).
    vtkSmartPointer<vtkTransform> t =
      vtkSmartPointer<vtkTransform>::New();
    vtkBoxWidget *widget = reinterpret_cast<vtkBoxWidget*>(caller);
    widget->GetTransform( t );
    widget->GetProp3D()->SetUserTransform( t );
  }
};
 
int main( int vtkNotUsed( argc ), char* vtkNotUsed( argv )[] )
{
  vtkSmartPointer<vtkConeSource> cone =
    vtkSmartPointer<vtkConeSource>::New();
 
  vtkSmartPointer<vtkPolyDataMapper> coneMapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  coneMapper->SetInputConnection( cone->GetOutputPort() );
 
  vtkSmartPointer<vtkActor> coneActor =
    vtkSmartPointer<vtkActor>::New();
  coneActor->SetMapper( coneMapper );


  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer.reset(new pcl::visualization::PCLVisualizer("viewer",true));
  viewer->addActorToRenderer(coneActor);


 
//  vtkSmartPointer<vtkRenderer> renderer =
//    vtkSmartPointer<vtkRenderer>::New();
//  renderer->AddActor( coneActor );
//  renderer->SetBackground( 0.1, 0.2, 0.4 );
 
//  vtkSmartPointer<vtkRenderWindow> window =
//    vtkSmartPointer<vtkRenderWindow>::New();
//  window->AddRenderer( renderer );
//  window->SetSize( 300, 300 );
 
//  vtkSmartPointer<vtkRenderWindowInteractor> interactor =
//    vtkSmartPointer<vtkRenderWindowInteractor>::New();
//  interactor->SetRenderWindow( window );

//  vtkSmartPointer<vtkInteractorStyleTrackballCamera> style =
//    vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
//  interactor->SetInteractorStyle( style );
 
  vtkSmartPointer<vtkBoxWidget> boxWidget =
    vtkSmartPointer<vtkBoxWidget>::New();
  boxWidget->SetInteractor( viewer->getRenderWindowInteractor() );
 
  boxWidget->SetProp3D( coneActor );
  boxWidget->SetPlaceFactor( 1.25 ); // Make the box 1.25x larger than the actor
  boxWidget->PlaceWidget();
 
  vtkSmartPointer<vtkMyCallback> callback =
    vtkSmartPointer<vtkMyCallback>::New();
  boxWidget->AddObserver( vtkCommand::InteractionEvent, callback );
 
  boxWidget->On();

  viewer->getRenderWindowInteractor()->Start();

 
  return EXIT_SUCCESS;
}
