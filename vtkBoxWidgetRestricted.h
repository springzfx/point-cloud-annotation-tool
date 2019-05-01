#ifndef vtkBoxWidgetRestricted_h
#define vtkBoxWidgetRestricted_h

#include <vtkBoxWidget.h>
#include <vtkMath.h>
#include <vtkTransform.h>
#include <vtkDoubleArray.h>
#include <vtkPoints.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include "vtkSphereSource.h"
#include "vtkCamera.h"
#include "vtkCallbackCommand.h"

/**
 * @brief The vtkBoxWidgetRestricted class
 * vtkBoxWidgetRestricted restricts the rotation with Z axis
 *
 *
 */

class vtkBoxWidgetRestricted: public vtkBoxWidget{
public:
    static vtkBoxWidgetRestricted *New();

    vtkTypeMacro(vtkBoxWidgetRestricted,vtkBoxWidget);

    virtual void Rotate(int X, int Y, double *p1, double *p2, double *vpn);
    virtual void SizeHandles(void);
    virtual void OnMouseMove(void);
};
#endif
