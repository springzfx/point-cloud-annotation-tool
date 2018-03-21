#include "vtkAnnotationBoxSource.h"
#include "vtkCellArray.h"
#include "vtkFloatArray.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkPointData.h"
#include "vtkPoints.h"
#include "vtkPolyData.h"
#include "vtkCellData.h"

#include <cmath>

vtkStandardNewMacro(vtkAnnotationBoxSource);

vtkAnnotationBoxSource::vtkAnnotationBoxSource()
{
    this->SetNumberOfInputPorts(0);
}

int vtkAnnotationBoxSource::RequestData(
        vtkInformation *vtkNotUsed(request),
        vtkInformationVector **vtkNotUsed(inputVector),
        vtkInformationVector *outputVector)
{
    // get the info object
    vtkInformation *outInfo = outputVector->GetInformationObject(0);

    // get the ouptut
    vtkPolyData *output = vtkPolyData::SafeDownCast(
                outInfo->Get(vtkDataObject::DATA_OBJECT()));


    int numPolys=6, numPts=8;
    vtkIdType pts[4],pts_[2];
    vtkPoints *newPoints;
    vtkCellArray *newPolys;
    vtkCellArray *newLines;

    //
    // Set things up; allocate memory
    //
    newPoints = vtkPoints::New(VTK_DOUBLE);
    newPoints->SetNumberOfPoints(numPts);
    double bounds[6]={-0.5,0.5,-0.5,0.5,-0.5,0.5};
    newPoints->SetPoint(0, bounds[0], bounds[2], bounds[4]);
    newPoints->SetPoint(1, bounds[1], bounds[2], bounds[4]);
    newPoints->SetPoint(2, bounds[1], bounds[3], bounds[4]);
    newPoints->SetPoint(3, bounds[0], bounds[3], bounds[4]);
    newPoints->SetPoint(4, bounds[0], bounds[2], bounds[5]);
    newPoints->SetPoint(5, bounds[1], bounds[2], bounds[5]);
    newPoints->SetPoint(6, bounds[1], bounds[3], bounds[5]);
    newPoints->SetPoint(7, bounds[0], bounds[3], bounds[5]);

    // faces
    newPolys = vtkCellArray::New();
    newPolys->Allocate(newPolys->EstimateSize(numPolys,4));
    pts[0] = 1; pts[1] = 2; pts[2] = 6; pts[3] = 5;  // +x
    newPolys->InsertNextCell(4,pts);
    pts[0] = 3; pts[1] = 0; pts[2] = 4; pts[3] = 7;  // -x
    newPolys->InsertNextCell(4,pts);
    pts[0] = 0; pts[1] = 1; pts[2] = 5; pts[3] = 4;  // +z
    newPolys->InsertNextCell(4,pts);
    pts[0] = 2; pts[1] = 3; pts[2] = 7; pts[3] = 6;  // -z
    newPolys->InsertNextCell(4,pts);
    pts[0] = 0; pts[1] = 3; pts[2] = 2; pts[3] = 1;  // -y
    newPolys->InsertNextCell(4,pts);
    pts[0] = 4; pts[1] = 5; pts[2] = 6; pts[3] = 7;  // +y
    newPolys->InsertNextCell(4,pts);

    // lines
    newLines =  vtkCellArray::New();
    newLines->Allocate(newLines->EstimateSize(12,2));
    pts_[0] = 0; pts_[1] = 4;       //the -x face
    newLines->InsertNextCell(2,pts_);
    pts_[0] = 4; pts_[1] = 7;
    newLines->InsertNextCell(2,pts_);
    pts_[0] = 7; pts_[1] = 3;       //the +x face
    newLines->InsertNextCell(2,pts_);
    pts_[0] = 3; pts_[1] = 0;
    newLines->InsertNextCell(2,pts_);
    pts_[0] = 2; pts_[1] = 6;       //the -y face
    newLines->InsertNextCell(2,pts_);
    pts_[0] = 6; pts_[1] = 5;
    newLines->InsertNextCell(2,pts_);
    pts_[0] = 5; pts_[1] = 1;       //the +y face
    newLines->InsertNextCell(2,pts_);
    pts_[0] = 1; pts_[1] = 2;
    newLines->InsertNextCell(2,pts_);
    pts_[0] = 0; pts_[1] = 1;       //the -z face
    newLines->InsertNextCell(2,pts_);
    pts_[0] = 4; pts_[1] = 5;
    newLines->InsertNextCell(2,pts_);
    pts_[0] = 7; pts_[1] = 6;       //the +Z face
    newLines->InsertNextCell(2,pts_);
    pts_[0] = 3; pts_[1] = 2;
    newLines->InsertNextCell(2,pts_);

    //
    // Update ourselves and release memory
    //
    output->SetPoints(newPoints);
    newPoints->Delete();
    output->SetLines(newLines);
    newLines->Delete();
    newPolys->Squeeze();
    output->SetPolys(newPolys);
    newPolys->Delete();

    return 1;
}


