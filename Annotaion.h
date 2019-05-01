#ifndef Annotaions_H
#define Annotaions_H

#include <common.h>
#include <vtkSmartPointer.h>
using namespace std;

struct BoxLabel
{
    BoxLabel(){
        type="unknown";
        this->detail.center_x=this->detail.center_y=this->detail.center_z=0;
        this->detail.yaw=2;
        this->detail.length=this->detail.width=this->detail.height=1;
    }
    BoxLabel(const double p1[3],const double p2[3],string type_="unknown"){
        type=type_;
        this->detail.center_x=(p1[0]+p2[0])/2;
        this->detail.center_y=(p1[1]+p2[1])/2;
        this->detail.center_z=(p1[2]+p2[2])/2;
        this->detail.yaw=0;
        this->detail.length=p2[0]-p1[0];
        this->detail.width=p2[1]-p1[1];
        this->detail.height=p2[2]-p1[2];
    }
    string type;
    union{
        double data[7];
        struct{
            double center_x;
            double center_y;
            double center_z;
            double length;
            double width;
            double height;
            double yaw;
        } detail;
    };

    string toString(){
        char buffer [200];
        sprintf(buffer,"%s %f %f %f %f %f %f %f",
                type.c_str(),data[0],data[1],data[2],data[3],data[4],data[5],data[6]);
        return std::string(buffer);;
    }
};


class vtkBoxWidgetCallback0;
class vtkBoxWidgetCallback1;
class vtkAnnotationBoxSource;
class vtkBoxWidgetRestricted;

class vtkTransform;
class vtkRenderWindowInteractor;
class vtkActor;
class vtkPolyDataMapper;

class Annotation
{
public:
    /**
     * @brief Annotation  construct from boxlabel which load from label file
     * @param label
     * @param visible_
     * @param lock_
     */
    Annotation(const BoxLabel &label,bool visible_=true,bool lock_=false);

    /**
     * @brief Annotation construct from part of cloud points
     * @param cloud
     * @param slice
     * @param type_
     */
    Annotation(const PointCloudTPtr cloud, vector<int> & slice,string type_);

    ~Annotation();

    /**
     * @brief getBoxLabel get boxLabel from annotaion tranformation
     * @return
     */
    BoxLabel getBoxLabel();

    /**
     * @brief apply transform to annotation
     * @param t
     */
    void applyTransform(vtkSmartPointer<vtkTransform> t);

    /**
     * @brief enter picked state, show boxwidget which allow to adjust annotation
     * @param interactor
     */
    void picked(vtkRenderWindowInteractor* interactor);

    /* @brief: Get the box pose 
	 * @return ptr that contains box position
	 */
	double* getBoxPose(void);
	
	/* @brief: Get the box orientation(You never know, it may be gay)
	 * @return: ptr that contains orientation
	 */
	double* getBoxOri(void);

    /**
     * @brief disable boxWidget
     */
    void unpicked();

    /**
     * @brief keep current orientation, re-compute the center and scale
     * to make annotation fit to selected point well enough
     */
    void adjustToAnchor();

    /**
     * @brief change the type of annotation, and color too
     * @param value
     */
    void setType(const string value);
    vtkSmartPointer<vtkActor> getActor() const;
    string getType() const;


protected:
    void initial();

    /**
     * @brief color the annotation with given color
     * @param color_index
     * if color_index>=0,refer to @ref pcl::GlasbeyLUT
     * otherwise use color already mapped by type
     */
    void colorAnnotation(int color_index=-1);

    /**
     * @brief copy selected points as anchor to current annotation
     * @param cloud
     * @param slice
     */
    void setAnchorPoint(const PointCloudTPtr cloud, const vector<int> &slice);

    /**
     * @brief computeScaleAndCenterShift
     * @param o direction
     * @param scs ["scale", "center shift"]
     * @return scale
     */
    double computeScaleAndCenterShift(double o[3],double scs[2]);




private:
    string type;
    vtkSmartPointer<vtkAnnotationBoxSource> source;
    vtkSmartPointer<vtkActor> actor;
    vtkSmartPointer<vtkPolyDataMapper> mapper;
    vtkSmartPointer<vtkTransform> transform;

    vtkSmartPointer<vtkBoxWidgetRestricted> boxWidget;
    vtkSmartPointer<vtkBoxWidgetCallback0> boxWidgetCallback0;
    vtkSmartPointer<vtkBoxWidgetCallback1> boxWidgetCallback1;

    vector<double*> anchorPoints;
    double center[3];

    // NOTE not used
    bool visible;
    bool lock;



public:
    /**
     * @brief get types vector pointer
     * @return
     */
    static vector<string>* getTypes();

    /**
     * @brief getTypeIndex  auto add to vector map if has not
     * @param type_
     * @return
     */
    static int getTypeIndex(string type_);

    /**
     * @brief getColor map type to color in pcl::GlasbeyLUT
     * @param type_
     * @return
     */
    static pcl::RGB getColor(string type_);

    /**
     * @brief computeOBB compute max,min [x,y,z] aligned to xyz axis
     * @param cloud
     * @param slice
     * @param p1 min [x,y,z]
     * @param p2 max [x,y,z]
     */
    static void computeOBB(const  PointCloudTPtr cloud, vector<int>& slice, double p1[3], double p2[3]);

private:
    /**
     * @brief types all annotation type here
     */
    static vector<string>* types;

};



class Annotaions
{
public:
    /**
     * @brief from annotatin box actor to find annotation itself
     * @param actor
     * @return
     */
    Annotation* getAnnotation(vtkActor* actor);

    void push_back(Annotation* anno);
    void remove(Annotation* anno);
    void clear();
    int getSize();

    /**
     * @brief load annotations from file
     * @param filename
     */
    void loadAnnotations(string filename);

    /**
     * @brief save annotations to file
     * @param filename
     */
    void saveAnnotations(string filename);

    vector<Annotation *>& getAnnotations();

protected:
    /**
     * @brief keep all annotation from current cloud
     */
    vector<Annotation*> annotations;

};



#endif //Annotaions_H
