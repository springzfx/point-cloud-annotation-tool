#include <pcl/visualization/pcl_visualizer_extented.h>
#include <vtkRenderWindowInteractor.h>

PCLVisualizerExtented::PCLVisualizerExtented(const std::string &name, const bool create_interactor)
    :PCLVisualizer(name,create_interactor) {}

bool PCLVisualizerExtented::removeActorFromRenderer(const vtkSmartPointer<vtkActor> &actor, int viewport){
    vtkProp* actor_to_remove = vtkProp::SafeDownCast (actor);

    // Initialize traversal
    getRendererCollection()->InitTraversal ();
    vtkRenderer* renderer = NULL;
    int i = 0;
    while ((renderer = getRendererCollection()->GetNextItem ()) != NULL)
    {
        // Should we remove the actor from all renderers?
        if (viewport == 0)
        {
            renderer->RemoveActor (actor);
        }
        else if (viewport == i)               // add the actor only to the specified viewport
        {
            // Iterate over all actors in this renderer
            vtkPropCollection* actors = renderer->GetViewProps ();
            actors->InitTraversal ();
            vtkProp* current_actor = NULL;
            while ((current_actor = actors->GetNextProp ()) != NULL)
            {
                if (current_actor != actor_to_remove)
                    continue;
                renderer->RemoveActor (actor);
                // Found the correct viewport and removed the actor
                return (true);
            }
        }
        ++i;
    }
    if (viewport == 0) return (true);
    return (false);
}

void PCLVisualizerExtented::addActorToRenderer(const vtkSmartPointer<vtkProp> &actor, int viewport){
    // Add it to all renderers
    getRendererCollection()->InitTraversal ();
    vtkRenderer* renderer = NULL;
    int i = 0;
    while ((renderer = getRendererCollection()->GetNextItem ()) != NULL)
    {
        // Should we add the actor to all renderers?
        if (viewport == 0)
        {
            renderer->AddActor (actor);
        }
        else if (viewport == i)               // add the actor only to the specified viewport
        {
            renderer->AddActor (actor);
        }
        ++i;
    }
}

void PCLVisualizerExtented::setupInteractor(vtkRenderWindowInteractor *iren, vtkRenderWindow *win)
{
    interactor_=iren;
    PCLVisualizer::setupInteractor(iren,win);
}

