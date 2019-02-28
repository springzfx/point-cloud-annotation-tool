#include <vtkBoxWidgetRestricted.h>

vtkStandardNewMacro(vtkBoxWidgetRestricted);

void vtkBoxWidgetRestricted::Rotate(int X, int Y, double *p1, double *p2, double *vpn){
	double *pts =
		static_cast<vtkDoubleArray *>(this->Points->GetData())->GetPointer(0);
	double *center =
		static_cast<vtkDoubleArray *>(this->Points->GetData())->GetPointer(3*14);
	double v[3]; //vector of motion
	//        double axis[3]; //axis of rotation
	double theta; //rotation angle
	int i;

	v[0] = p2[0] - p1[0];
	v[1] = p2[1] - p1[1];
	v[2] = p2[2] - p1[2];

	// Create axis of rotation and angle of rotation
	// compute the vector which perpendicular to Z axis in current view
	double z[3]={0,0,1}; //restrict to Z rotation
	double m[3];
	vtkMath::Cross(z,vpn,m);
	if ( vtkMath::Normalize(m) == 0.0 ||  vtkMath::Normalize(v) == 0.0)
	{
		return;
	}
	// alpha is the angle between the motion vector and vector m
	double cos_alpha=vtkMath::Dot(m,v);
	//    std::cout<<cos_alpha<<endl;
	int *size = this->CurrentRenderer->GetSize();
	double l2 = (X-this->Interactor->GetLastEventPosition()[0])*(X-this->Interactor->GetLastEventPosition()[0]) + (Y-this->Interactor->GetLastEventPosition()[1])*(Y-this->Interactor->GetLastEventPosition()[1]);
	theta = 360.0 * cos_alpha *sqrt(l2/(size[0]*size[0]+size[1]*size[1]));

	//Manipulate the transform to reflect the rotation
	this->Transform->Identity();
	this->Transform->Translate(center[0],center[1],center[2]);
	this->Transform->RotateZ(theta);
	this->Transform->Translate(-center[0],-center[1],-center[2]);

	//Set the corners
	vtkPoints *newPts = vtkPoints::New(VTK_DOUBLE);
	this->Transform->TransformPoints(this->Points,newPts);

	for (i=0; i<8; i++, pts+=3)
	{
		this->Points->SetPoint(i, newPts->GetPoint(i));
	}

	newPts->Delete();
	this->PositionHandles();
}

void vtkBoxWidgetRestricted::SizeHandles(void){

	double radius = this->vtk3DWidget::SizeHandles(0.6);

	for(int i=0; i<7; i++)
	{
		this->HandleGeometry[i]->SetRadius(radius);
	} 

}


void vtkBoxWidgetRestricted::OnMouseMove(void){

	// See whether we're active
	if ( this->State == vtkBoxWidget::Outside ||
			this->State == vtkBoxWidget::Start )
	{
		return;
	}

	int X = this->Interactor->GetEventPosition()[0];
	int Y = this->Interactor->GetEventPosition()[1];

	// Do different things depending on state
	// Calculations everybody does
	double focalPoint[4], pickPoint[4], prevPickPoint[4];
	double z, vpn[3];

	vtkCamera *camera = this->CurrentRenderer->GetActiveCamera();
	if ( !camera )
	{
		return;
	}


	// Compute the two points defining the motion vector
	this->ComputeWorldToDisplay(this->LastPickPosition[0], this->LastPickPosition[1],
			this->LastPickPosition[2], focalPoint);
	z = focalPoint[2];
	this->ComputeDisplayToWorld(double(this->Interactor->GetLastEventPosition()[0]),
			double(this->Interactor->GetLastEventPosition()[1]),
			z, prevPickPoint);
	this->ComputeDisplayToWorld(double(X), double(Y), z, pickPoint);

	// Process the motion
	if ( this->State == vtkBoxWidget::Moving )
	{
		// Okay to process
		if ( this->CurrentHandle )
		{
			if ( this->RotationEnabled && this->CurrentHandle == this->HexFace )
			{
				camera->GetViewPlaneNormal(vpn);
				this->Rotate(X, Y, prevPickPoint, pickPoint, vpn);
			}
			else if ( this->TranslationEnabled && this->CurrentHandle == this->Handle[6] )
			{
				this->Translate(prevPickPoint, pickPoint);
			}
			else if ( this->TranslationEnabled && this->ScalingEnabled )
			{
				if ( this->CurrentHandle == this->Handle[0] )
				{
					this->MoveMinusXFace(prevPickPoint, pickPoint);
				}
				else if ( this->CurrentHandle == this->Handle[1] )
				{
					this->MovePlusXFace(prevPickPoint, pickPoint);
				}
				else if ( this->CurrentHandle == this->Handle[2] )
				{
					this->MoveMinusYFace(prevPickPoint, pickPoint);
				}
				else if ( this->CurrentHandle == this->Handle[3] )
				{
					this->MovePlusYFace(prevPickPoint, pickPoint);
				}
				else if ( this->CurrentHandle == this->Handle[4] )
				{
					this->MoveMinusZFace(prevPickPoint, pickPoint);
				}
				else if ( this->CurrentHandle == this->Handle[5] )
				{
					this->MovePlusZFace(prevPickPoint, pickPoint);
				}
			}
		}


	}
	else if ( this->ScalingEnabled && this->State == vtkBoxWidget::Scaling )
	{
		this->Scale(prevPickPoint, pickPoint, X, Y);
	}

	// Interact, if desired
	this->EventCallbackCommand->SetAbortFlag(1);
	this->InvokeEvent(vtkCommand::InteractionEvent, nullptr);
	this->Interactor->ConfigureEvent();
	this->Interactor->Render();

}
