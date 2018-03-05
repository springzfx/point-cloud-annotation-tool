#include "select_rect.h"

void SelectRect:: AreaPickingEventOccurred (const pcl::visualization::AreaPickingEvent& event_,
                                            std::vector<int>& selected_slice_)
{
    event_.getPointsIndices(selected_slice_);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> colorHandler(cloud, 0, 0, 255);


}
