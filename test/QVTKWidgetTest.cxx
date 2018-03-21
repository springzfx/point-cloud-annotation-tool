#include <QApplication>
#include <QMainWindow>
#include <ui_QVTKWidgetTest.h>
#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkConeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <pcl/visualization/pcl_visualizer.h>


using namespace std;
using namespace pcl::visualization;
class MainWindow:public QMainWindow{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent=0): QMainWindow(parent), ui(new Ui::MainWindow){
        ui->setupUi(this);
        this->setWindowTitle("QVTKWidget Test");

        vtkSmartPointer<vtkConeSource> cone =
                vtkSmartPointer<vtkConeSource>::New();

        vtkSmartPointer<vtkPolyDataMapper> coneMapper =
                vtkSmartPointer<vtkPolyDataMapper>::New();
        coneMapper->SetInputConnection( cone->GetOutputPort() );

        vtkSmartPointer<vtkActor> coneActor =
                vtkSmartPointer<vtkActor>::New();
        coneActor->SetMapper( coneMapper );

        vtkSmartPointer<vtkRenderer> renderer =
                vtkSmartPointer<vtkRenderer>::New();
        renderer->AddActor( coneActor );

        vtkSmartPointer<vtkRenderWindow> window =
                vtkSmartPointer<vtkRenderWindow>::New();
        window->AddRenderer( renderer );

        v=new PCLVisualizer("",false);
        ui->qvtkWidget->SetRenderWindow(v->getRenderWindow());
        v->setupInteractor(ui->qvtkWidget->GetInteractor(),ui->qvtkWidget->GetRenderWindow());
//        ui->qvtkWidget->SetRenderWindow(window);
        ui->qvtkWidget->update();
            }

     Ui::MainWindow* ui;
     PCLVisualizer* v;
};

#include "QVTKWidgetTest.moc"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
