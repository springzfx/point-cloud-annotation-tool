#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QMainWindow>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>

typedef pcl::PointXYZI  PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
