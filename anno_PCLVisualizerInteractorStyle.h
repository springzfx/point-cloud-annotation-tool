#ifndef AnnoPCLVisualizerInteractorStyle_H
#define AnnoPCLVisualizerInteractorStyle_H
#include <vtkPropPicker.h>
#include <vtkProperty.h>
#include <vtkRenderWindowInteractor.h>
#include <pcl/visualization/interactor_style.h>
#include <vtkPropPicker.h>


namespace pcl
{
namespace visualization
{
class AnnoPCLVisualizerInteractorStyle: public PCLVisualizerInteractorStyle
{

private:
    vtkActor    *LastPickedActor;
    vtkProperty *LastPickedProperty;

public:
    static AnnoPCLVisualizerInteractorStyle *New ();
    // this macro defines Superclass, the isA functionality and the safe downcast method
    vtkTypeMacro (AnnoPCLVisualizerInteractorStyle, PCLVisualizerInteractorStyle);

    AnnoPCLVisualizerInteractorStyle():PCLVisualizerInteractorStyle (){
        LastPickedActor = NULL;
        LastPickedProperty = vtkProperty::New();
    }

    virtual ~AnnoPCLVisualizerInteractorStyle()
    {
        LastPickedProperty->Delete();
    }

    virtual void OnLeftButtonDown()
    {
        int* clickPos = this->GetInteractor()->GetEventPosition();
        // Pick from this location.
//        vtkSmartPointer<vtkPropPicker>  picker =
//                vtkSmartPointer<vtkPropPicker>::New();
//        picker->PickProp(clickPos[0], clickPos[1], this->GetCurrentRenderer());

//        // If we picked something before, reset its property
//        if (this->LastPickedActor)
//        {
//            this->LastPickedActor->GetProperty()->DeepCopy(this->LastPickedProperty);
//        }

//        this->LastPickedActor = picker->GetActor();
//        if (this->LastPickedActor)
//        {
//            // Save the property of the picked actor so that we can
//            // restore it next time
//            this->LastPickedProperty->DeepCopy(this->LastPickedActor->GetProperty());
//            // Highlight the picked actor by changing its properties
//            this->LastPickedActor->GetProperty()->SetColor(1.0, 0.0, 0.0);
//            this->LastPickedActor->GetProperty()->SetDiffuse(1.0);
//            this->LastPickedActor->GetProperty()->SetSpecular(0.0);
//        }

        // Forward events
        PCLVisualizerInteractorStyle::OnLeftButtonDown();
    }

};

//// Standard VTK macro for *New ()
//vtkStandardNewMacro (AnnoPCLVisualizerInteractorStyle);

}
}

#endif //AnnoPCLVisualizerInteractorStyle_H
