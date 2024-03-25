#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent): QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    SignalsSlotsRegister();

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

bool MainWindow::SignalsSlotsRegister()
{
    //工具栏按钮绑定
    connect(this, SIGNAL(modeSelect(int)), ui->pointCloudWidget, SLOT(modeSelect(int)));
    connect(this, SIGNAL(resetView()), ui->pointCloudWidget, SLOT(resetView()));
    connect(this, SIGNAL(showAxis(bool)), ui->pointCloudWidget, SLOT(showAxis(bool)));
    connect(this, SIGNAL(showMesh(bool)), ui->pointCloudWidget, SLOT(showMesh(bool)));
    connect(this, SIGNAL(clearPointCloud()), ui->pointCloudWidget, SLOT(clearPointCloud()));

    //ros节点收发链路
    connect(&this->dc, SIGNAL(sendPointsSignals(message::msg::LidarData::SharedPtr)),
            ui->pointCloudWidget,  SLOT(recvPointsData(message::msg::LidarData::SharedPtr)));
    connect(&this->dc, SIGNAL(sendImuDataSignals(message::msg::ImuData::SharedPtr)),
            ui->pointCloudWidget, SLOT(recvImuData(message::msg::ImuData::SharedPtr)));

    //点云界面
    connect(&ui->pointCloudWidget->pointCloudDataManager, SIGNAL(updateGraph()), ui->pointCloudWidget, SLOT(update()));
    connect(&ui->pointCloudWidget->camera, SIGNAL(updateGraph()), ui->pointCloudWidget, SLOT(update()));

    //树形数据接收链路
    connect(ui->pointCloudWidget, SIGNAL(infoTreeUpdate(CameraController)), ui->infoTree, SLOT(update(CameraController)));

    return true;
}




void MainWindow::on_comboBox_currentIndexChanged(int index)
{
    emit modeSelect(index);
}

void MainWindow::on_resetView_clicked()
{
    emit resetView();
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

void MainWindow::on_clearPointCloud_clicked()
{
    emit clearPointCloud();
}



