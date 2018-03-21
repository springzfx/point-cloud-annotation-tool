#ifndef PCLVisualizerExtented_H
#define PCLVisualizerExtented_H

#include <pcl/visualization/pcl_visualizer.h>

class PCLVisualizerExtented : public pcl::visualization::PCLVisualizer{

public:
    PCLVisualizerExtented (const std::string &name = "", const bool create_interactor = true);

    bool removeActorFromRenderer (const vtkSmartPointer<vtkActor> &actor,
                                  int viewport = 0);

    void addActorToRenderer (const vtkSmartPointer<vtkProp> &actor,
                             int viewport = 0);

    /**
     * @brief setupInteractor  overide to init interactor_
     * @param iren
     * @param win
     */
    void setupInteractor (
      vtkRenderWindowInteractor *iren,
      vtkRenderWindow *win);


    /** \brief Get a pointer to the current interactor style used. */
    inline vtkSmartPointer<vtkRenderWindowInteractor>
    getRenderWindowInteractor (){
        return (interactor_);
    }
};

#endif
