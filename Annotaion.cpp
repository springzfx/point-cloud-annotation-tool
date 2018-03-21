#include <Annotaion.h>
#include <vtkAnnotationBoxSource.h>
#include <vtkBoxWidgetRestricted.h>
#include <vtkBoxWidgetCallback.h>

using namespace std;

Annotation::Annotation(const BoxLabel &label, bool visible_, bool lock_)
    :visible(visible_),lock(lock_){
    //type
    type=label.type;

    // init variable
    initial();

    // apply transform
    vtkSmartPointer<vtkTransform> cubeTransform = vtkSmartPointer<vtkTransform>::New();
    cubeTransform->PostMultiply();
    cubeTransform->Scale(label.detail.length,label.detail.width,label.detail.height);
    //        cubeTransform->RotateZ(label.detail.yaw);
    cubeTransform->RotateZ(label.detail.yaw*180/vtkMath::Pi());
    cubeTransform->Translate(label.detail.center_x,label.detail.center_y,label.detail.center_z);

    applyTransform(cubeTransform);
}

Annotation::Annotation(const PointCloudTPtr cloud, vector<int> &slice, string type_)  {
    double p1[3];
    double p2[3];
    computeOBB(cloud,slice,p1,p2);
    BoxLabel label(p1,p2,type_);

    setAnchorPoint(cloud,slice);

    this->type=type_;

    // init variable
    initial();

    // apply transform
    vtkSmartPointer<vtkTransform> cubeTransform = vtkSmartPointer<vtkTransform>::New();
    cubeTransform->PostMultiply();
    cubeTransform->Scale(label.detail.length,label.detail.width,label.detail.height);
    //        cubeTransform->RotateZ(label.detail.yaw);
    cubeTransform->RotateZ(label.detail.yaw*180/vtkMath::Pi());
    cubeTransform->Translate(label.detail.center_x,label.detail.center_y,label.detail.center_z);

    applyTransform(cubeTransform);
}

Annotation::~Annotation(){
    // release anchorPoints
    for (auto p:anchorPoints){
        delete[] p;
    }
    anchorPoints.clear();
}

BoxLabel Annotation::getBoxLabel(){
    BoxLabel label;
    label.type=type;

    double p[3];
    double s[3];
    double o[3];
    transform->GetPosition(p);
    transform->GetScale(s);
    transform->GetOrientation(o);
    memcpy(label.data, p, 3*sizeof(double));
    memcpy(label.data+3, s,3*sizeof(double));
    label.detail.yaw=o[2]/180*vtkMath::Pi();
    return label;
}

void Annotation::applyTransform(vtkSmartPointer<vtkTransform> t){
    if (t==transform) return;

    transform=t;
    actor->SetUserTransform(t);

}

void Annotation::picked(vtkRenderWindowInteractor *interactor){
    // enable box widget
    boxWidget = vtkSmartPointer<vtkBoxWidgetRestricted>::New();
    boxWidgetCallback0 = vtkSmartPointer<vtkBoxWidgetCallback0>::New();
    boxWidgetCallback0->setAnno(this);
    boxWidgetCallback1 = vtkSmartPointer<vtkBoxWidgetCallback1>::New();
    boxWidgetCallback1->setAnno(this);

    boxWidget->SetInteractor(interactor);
    // boxWidget->SetProp3D( cubeActor );
    // boxWidget->SetPlaceFactor( 1.25 ); // Make the box 1.25x larger than the actor
    // boxWidget->PlaceWidget();

    // default is [-0.5, 0.5], NOTE
    // [-1,1] makes boxwidget fit to annotion,
    // but [-0.5, 0.5] should be in the correct way, may be some bug
    double bounds[6]={-1,1,-1,1,-1,1};
    boxWidget->PlaceWidget(bounds);

    vtkSmartPointer<vtkTransform> t=vtkSmartPointer<vtkTransform>::New();
    t->DeepCopy(actor->GetUserTransform());
    boxWidget->SetTransform(t);
    boxWidget->SetHandleSize(0.01);
    boxWidget->GetOutlineProperty()->SetAmbientColor(1.0,0.0,0.0);

    boxWidget->AddObserver( vtkCommand::InteractionEvent, boxWidgetCallback0 );
    boxWidget->AddObserver( vtkCommand::EndInteractionEvent, boxWidgetCallback1 );
    boxWidget->On();
}

void Annotation::unpicked(){
    boxWidget->Off();
}

void Annotation::adjustToAnchor(){
    if (anchorPoints.size()==0) return;

    transform->GetPosition(center);

    double r[3],x[3],y[3],z[3]={0,0,1};
    double s[3]; // scale

    transform->GetOrientation(r);
    x[0]=cos(vtkMath::RadiansFromDegrees(r[2])); x[1]=sin(vtkMath::RadiansFromDegrees(r[2])); x[2]=0; // direction
    vtkMath::Cross(z,x,y);

    double scs[2];
    s[0]=computeScaleAndCenterShift(x,scs);
    vtkMath::MultiplyScalar(x,scs[1]);
    s[1]=computeScaleAndCenterShift(y,scs);
    vtkMath::MultiplyScalar(y,scs[1]);
    s[2]=computeScaleAndCenterShift(z,scs);
    vtkMath::MultiplyScalar(z,scs[1]);

    // apply center shift
    vtkMath::Add(center,x,center);
    vtkMath::Add(center,y,center);
    vtkMath::Add(center,z,center);

    vtkSmartPointer<vtkTransform> t=vtkSmartPointer<vtkTransform>::New();
    t->Translate(center);
    t->RotateZ(r[2]);
    t->Scale(s);

    //        transform->Scale(s);
    boxWidget->SetTransform(t);
    applyTransform(t);
}


string Annotation::getType() const
{
    return type;
}

void Annotation::setType(const string value)
{
    if (value!=type){
        type = value;
        colorAnnotation();
    }
}

vtkSmartPointer<vtkActor> Annotation::getActor() const
{
    return actor;
}

void Annotation::initial(){
    // Cube
    source = vtkSmartPointer<vtkAnnotationBoxSource>::New();
    mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(source->GetOutputPort());
    actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    transform = vtkSmartPointer<vtkTransform>::New();
    colorAnnotation();
}

void Annotation::colorAnnotation(int color_index){
    vtkSmartPointer<vtkLookupTable> lut =
            vtkSmartPointer<vtkLookupTable>::New();
    lut->SetNumberOfTableValues(2);
    lut->Build();
    if (color_index<0){
        pcl::RGB c=getColor(type);
        lut->SetTableValue(0, c.r/255.0, c.g/255.0, c.b/255.0, 0);
        lut->SetTableValue(1, c.r/255.0, c.g/255.0, c.b/255.0, 1);
    }else{
        pcl::RGB c=pcl::GlasbeyLUT::at(color_index);
        lut->SetTableValue(0, c.r/255.0, c.g/255.0, c.b/255.0, 0);
        lut->SetTableValue(1, c.r/255.0, c.g/255.0, c.b/255.0, 1);
    }

    vtkSmartPointer<vtkFloatArray> cellData =
            vtkSmartPointer<vtkFloatArray>::New();

    //line color
    for (int i = 0; i < 12; i++)
    {
        cellData->InsertNextValue(1);
    }

    //face color
    for (int i = 0; i < 6; i++)
    {
        cellData->InsertNextValue(0);
    }

    // plusX face
    cellData->InsertValue(12,1);

    source->Update();
    source->GetOutput()->GetCellData()->SetScalars(cellData);

    actor->GetProperty()->SetLineWidth(2);
    actor->GetProperty ()->SetLighting (false);

    mapper->SetLookupTable(lut);
}

void Annotation::setAnchorPoint(const PointCloudTPtr cloud, const vector<int> &slice){
    anchorPoints.clear();
    for (auto i:slice){
        double* p=new double[3];
        p[0]=cloud->points[i].x;
        p[1]=cloud->points[i].y;
        p[2]=cloud->points[i].z;
        anchorPoints.push_back(p);
    }
}

double Annotation::computeScaleAndCenterShift(double o[], double scs[]){
    vtkMath::Normalize(o);

    double a,b;
    a=-std::numeric_limits <double>::max ();
    b=std::numeric_limits <double>::max ();

    double t[3];
    for (auto x:anchorPoints){
        vtkMath::Subtract(x,this->center,t);
        double s=vtkMath::Dot(t,o);
        a=std::max(a,s);
        b=std::min(b,s);
    }
    scs[0]=a-b;scs[1]=(a+b)/2;
    return a-b;
}

vector<string>* Annotation::getTypes()
{
    return types;
}

int Annotation::getTypeIndex(string type_){
    for (int i=0;i<types->size();i++){
            if (types->at(i)==type_) return i;
        }
        types->push_back(type_);
        return types->size()-1;
}

pcl::RGB Annotation::getColor(string type_){
    return pcl::GlasbeyLUT::at(getTypeIndex(type_));
}

void Annotation::computeOBB(const  PointCloudTPtr cloud, vector<int>& slice,double p1[3], double p2[3])
{
    p1[0]=std::numeric_limits <double>::max ();
    p1[1]=std::numeric_limits <double>::max ();
    p1[2]=std::numeric_limits <double>::max ();

    //std::numeric_limits <double>::min (); is a number close enough to 0
    p2[0]=-std::numeric_limits <double>::max ();
    p2[1]=-std::numeric_limits <double>::max ();
    p2[2]=-std::numeric_limits <double>::max ();

    for (auto i:slice){
        p1[0]=std::min(p1[0],(double)cloud->points[i].x);
        p1[1]=std::min(p1[1],(double)cloud->points[i].y);
        p1[2]=std::min(p1[2],(double)cloud->points[i].z);

        p2[0]=std::max(p2[0],(double)cloud->points[i].x);
        p2[1]=std::max(p2[1],(double)cloud->points[i].y);
        p2[2]=std::max(p2[2],(double)cloud->points[i].z);

    }
}

// static variable
vector<string>* Annotation::types=new vector<string>();


Annotation *Annotaions::getAnnotation(vtkActor *actor){
    for (auto* anno : annotations)
    {
        if (anno->getActor()==actor){
            return anno;
        }
    }
    return 0;
}

void Annotaions::push_back(Annotation *anno){
    annotations.push_back(anno);
}

void Annotaions::remove(Annotation *anno){
    annotations.erase(std::remove(annotations.begin(), annotations.end(), anno), annotations.end());
}

void Annotaions::clear(){
    for (auto p:annotations){
        delete p;
    }
    annotations.clear();
}

int Annotaions::getSize(){
    return annotations.size();
}

void Annotaions::loadAnnotations(string filename)
{
    annotations.clear();

    std::ifstream input(filename.c_str(), std::ios_base::in);
    if(!input.good()){
        std::cerr<<"Cannot open file : "<<filename<<std::endl;
        return;
    }
    std::string type;
    while (input>>type){
        BoxLabel label;
        label.type=type;
        for (int j=0;j<7;j++){
            input>>label.data[j];
        }
        annotations.push_back(new Annotation(label));
    }
    std::cout<<filename<<": load "<<annotations.size()<<" boxes"<<std::endl;
    input.close();
}

void Annotaions::saveAnnotations(string filename)
{
    if (annotations.empty()) return;
    std::ofstream output(filename.c_str(), std::ios_base::out);
    for (auto anno:annotations){
        output<<anno->getBoxLabel().toString()<<std::endl;
    }
    output.close();
}

vector<Annotation *>& Annotaions::getAnnotations()
{
    return annotations;
}


