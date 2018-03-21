#ifndef vtkAnnotationBoxSource_h
#define vtkAnnotationBoxSource_h

#include "vtkFiltersSourcesModule.h" // For export macro
#include "vtkPolyDataAlgorithm.h"

class  vtkAnnotationBoxSource : public vtkPolyDataAlgorithm
{
public:
  static vtkAnnotationBoxSource *New();
  vtkTypeMacro(vtkAnnotationBoxSource,vtkPolyDataAlgorithm);

protected:
  vtkAnnotationBoxSource();
  ~vtkAnnotationBoxSource() VTK_OVERRIDE {}
  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *) VTK_OVERRIDE;
private:
  vtkAnnotationBoxSource(const vtkAnnotationBoxSource&) VTK_DELETE_FUNCTION;
  void operator=(const vtkAnnotationBoxSource&) VTK_DELETE_FUNCTION;
};

#endif
