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
#include <vtkAnnotationBoxSource.h>
//#include <vtkCubeSource.h>
#include <vtkCellData.h>
#include <vtkFloatArray.h>
#include <vtkLookupTable.h>
#include <vtkProperty.h>

int main( int vtkNotUsed( argc ), char* vtkNotUsed( argv )[] )
{
    vtkSmartPointer<vtkAnnotationBoxSource> cone =
            vtkSmartPointer<vtkAnnotationBoxSource>::New();

    cone->Update();

    vtkSmartPointer<vtkLookupTable> lut =
            vtkSmartPointer<vtkLookupTable>::New();
    lut->SetNumberOfTableValues(2);
    lut->Build();

    // Fill in a few known colors, the rest will be generated if needed
    lut->SetTableValue(0, 1, 1, 1, 0);  //
    lut->SetTableValue(1, 0, 1, 0, 1);


    // color map
    vtkSmartPointer<vtkFloatArray> cellData =
            vtkSmartPointer<vtkFloatArray>::New();

    //line color
    for (int i = 0; i < 12; i++)
    {
        cellData->InsertNextValue(1);
    }

    //face color
    for (int i = 0; i < 6; i++)
    {
        cellData->InsertNextValue(0);
    }
    cellData->InsertValue(12,1);

    cone->GetOutput()->GetCellData()->SetScalars(cellData);

    vtkSmartPointer<vtkPolyDataMapper> coneMapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    coneMapper->SetInputConnection( cone->GetOutputPort() );
    coneMapper->SetColorModeToMapScalars();
    coneMapper->SetScalarModeToUseCellData();
    coneMapper->SetLookupTable(lut);


    vtkSmartPointer<vtkActor> coneActor =
            vtkSmartPointer<vtkActor>::New();
    coneActor->SetMapper( coneMapper );
    coneActor->GetProperty()->SetLighting(0);


    vtkSmartPointer<vtkRenderer> renderer =
            vtkSmartPointer<vtkRenderer>::New();
    renderer->AddActor( coneActor );

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

    interactor->Start();


    return EXIT_SUCCESS;
}
