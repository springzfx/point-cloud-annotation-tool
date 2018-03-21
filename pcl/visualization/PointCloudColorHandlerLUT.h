#ifndef PointCloudColorHandlerLUT_H
#define PointCloudColorHandlerLUT_H


#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/MyCloudLUT.h>
#include <vtkDataArray.h>
#include <vtkSmartPointer.h>
#include <vtkUnsignedCharArray.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using pcl::visualization::PointCloudColorHandler;

template <typename PointT>
class PointCloudColorHandlerLUT : public PointCloudColorHandler<PointT> {

public:
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    typedef boost::shared_ptr<PointCloudColorHandlerLUT<PointT> > Ptr;
    typedef boost::shared_ptr<const PointCloudColorHandlerLUT<PointT> > ConstPtr;

    /** \brief Constructor. */
    PointCloudColorHandlerLUT () :
        PointCloudColorHandler<PointT>()
    {
         capable_ = false;
    }

    /** \brief Constructor. */
    PointCloudColorHandlerLUT (const PointCloudConstPtr &cloud):
        PointCloudColorHandler<PointT>(cloud)
    {
         setInputCloud (cloud);
    }

    /** \brief Destructor. */
    virtual ~PointCloudColorHandlerLUT () {}

    /** \brief Check if this handler is capable of handling the input data or not. */
    inline bool
    isCapable () const {
        return (capable_);
    }

    /** \brief Abstract getName method. */
    virtual std::string
    getName () const {
        return "";
    };

    /** \brief Abstract getFieldName method. */
    virtual std::string
    getFieldName () const{
        return "";
    };

    /** \brief Obtain the actual color for the input dataset as vtk scalars.
    * \param[out] scalars the output scalars containing the color for the dataset
    * \return true if the operation was successful (the handler is capable and
    * the input cloud was given as a valid pointer), false otherwise
    */
    virtual bool
    getColor (vtkSmartPointer<vtkDataArray> &scalars) const {
        if (!capable_ || !cloud_)
            return (false);

        if (!scalars)
            scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
        scalars->SetNumberOfComponents (3);

        vtkIdType nr_points = cloud_->points.size ();
        reinterpret_cast<vtkUnsignedCharArray*> (&(*scalars))->SetNumberOfTuples (nr_points);
        unsigned char* colors = reinterpret_cast<vtkUnsignedCharArray*> (&(*scalars))->GetPointer (0);

        int j = 0;
        for (vtkIdType cp = 0; cp < nr_points; ++cp)
        {
            if (pcl::isFinite (cloud_->points[cp]))
            {
                const pcl::RGB& color = MyCloudLUT::at (label[cp] % MyCloudLUT::size ());
                colors[j    ] = color.r;
                colors[j + 1] = color.g;
                colors[j + 2] = color.b;
                j += 3;
            }
        }

        return (true);

    };

    /** \brief Set the input cloud to be used.
    * \param[in] cloud the input cloud to be used by the handler
    */
    virtual void
    setInputCloud (const PointCloudConstPtr &cloud)
    {
        cloud_ = cloud;
    }

    void setLabel(int* value){
        label = value;
        capable_=true;
    };


private:
    /**
     * @brief array of cloud label
     */
    int* label;

    // Members derived from the base class
    using PointCloudColorHandler<PointT>::cloud_;
    using PointCloudColorHandler<PointT>::capable_;
    using PointCloudColorHandler<PointT>::field_idx_;
    using PointCloudColorHandler<PointT>::fields_;
};
#endif
