#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent): QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    connect(this, SIGNAL(showAxis(bool)), ui->pointCloudWidget, SLOT(showAxis(bool)));
    connect(this, SIGNAL(showMesh(bool)), ui->pointCloudWidget, SLOT(showMesh(bool)));
    //    //取消标题栏
    //    this->setWindowFlags(Qt::FramelessWindowHint);
    //    // 去掉标题栏,去掉工具栏，窗口置顶
    //    setWindowFlags(Qt::FramelessWindowHint | Qt::Tool | Qt::WindowStaysOnTopHint);
    //    //设置窗体透明度
    //    setWindowOpacity(0.7);


}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_showMesh_clicked()
{
    if (ui->showMesh->isChecked())
    {
        emit showMesh(true);
    }
    else
    {
        emit showMesh(false);
    }
}


void MainWindow::on_showAxis_clicked()
{
    if (ui->showAxis->isChecked())
    {
        emit showAxis(true);
    }
    else
    {
        emit showAxis(false);
    }
}

