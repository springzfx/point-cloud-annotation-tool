#include <pcl/visualization/pcl_visualizer.h>

class SelectRect
{
public:
    static void AreaPickingEventOccurred (const pcl::visualization::AreaPickingEvent& event_,
                              std::vector<int>& selected_slice_);

}

