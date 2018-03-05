#include "mainwindow.h"
#include "ui_mainwindow.h"
//#include "visualizer.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("3D annotation tool");

//    connect(ui->plane_pushButton,  &QPushButton::clicked, this, &Visualizer::planeDetect);

//     connect(ui->action_Selection,&QAction::triggered,this,& )
}

MainWindow::~MainWindow()
{
    delete ui;
}

