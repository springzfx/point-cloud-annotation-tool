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
#include <vtkBoxWidgetRestricted.h>
#include <vtkCommand.h>
#include <vtkTransform.h>
 
class BoxWidgetCallback : public vtkCommand
{
public:
  static BoxWidgetCallback *New()
  {
    return new BoxWidgetCallback;
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
 
  vtkSmartPointer<vtkRenderer> renderer =
    vtkSmartPointer<vtkRenderer>::New();
  renderer->AddActor( coneActor );
  renderer->SetBackground( 0.1, 0.2, 0.4 );
 
  vtkSmartPointer<vtkRenderWindow> window =
    vtkSmartPointer<vtkRenderWindow>::New();
  window->AddRenderer( renderer );
  window->SetSize( 300, 300 );
 
  vtkSmartPointer<vtkRenderWindowInteractor> interactor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  interactor->SetRenderWindow( window );

  vtkSmartPointer<vtkInteractorStyleTrackballCamera> style =
    vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
  interactor->SetInteractorStyle( style );
 
  vtkSmartPointer<vtkBoxWidgetRestricted> boxWidget =
    vtkSmartPointer<vtkBoxWidgetRestricted>::New();
  boxWidget->SetInteractor( interactor );
  boxWidget->SetProp3D( coneActor );
  boxWidget->SetPlaceFactor( 1.25 ); // Make the box 1.25x larger than the actor
  boxWidget->PlaceWidget();
 
  vtkSmartPointer<BoxWidgetCallback> callback =
    vtkSmartPointer<BoxWidgetCallback>::New();
  boxWidget->AddObserver( vtkCommand::InteractionEvent, callback );
 
  boxWidget->On();

 interactor->Start();

 
  return EXIT_SUCCESS;
}
