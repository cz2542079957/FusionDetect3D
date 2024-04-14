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
    connect(this, SIGNAL(modeSelectSignal(int)), ui->pointCloudWidget, SLOT(modeSelectSlot(int)));
    connect(this, SIGNAL(resetViewSignal()), ui->pointCloudWidget, SLOT(resetViewSlot()));
    connect(this, SIGNAL(showAxisSignal(bool)), ui->pointCloudWidget, SLOT(showAxisSlot(bool)));
    connect(this, SIGNAL(showMeshSignal(bool)), ui->pointCloudWidget, SLOT(showMeshSlot(bool)));
    connect(this, SIGNAL(clearPointCloudSignal()), ui->pointCloudWidget, SLOT(clearPointCloudSlot()));

    //ros节点收发链路
    connect(&this->dc, SIGNAL(sendPointsSignal(message::msg::LidarData::SharedPtr)),
            ui->pointCloudWidget,  SLOT(recvPointsDataSlot(message::msg::LidarData::SharedPtr)));
    connect(&this->dc, SIGNAL(sendImuDataSignal(message::msg::ImuData::SharedPtr)),
            ui->pointCloudWidget, SLOT(recvImuDataSlot(message::msg::ImuData::SharedPtr)));

    //点云界面
    connect(&ui->pointCloudWidget->pointCloudDataManager, SIGNAL(updateGraph()), ui->pointCloudWidget, SLOT(update()));
    connect(&ui->pointCloudWidget->camera, SIGNAL(updateGraph()), ui->pointCloudWidget, SLOT(update()));
    connect(&ui->pointCloudWidget->camera, SIGNAL(fovChangedSignal(int)), this, SLOT(fovChangedSlot(int)));
    connect(this, SIGNAL(fovChangedSignal(int)), &ui->pointCloudWidget->camera, SLOT(fovChangedSlot(int)));
    connect(this, SIGNAL(baseSpeedSignal(float)), &ui->pointCloudWidget->camera, SLOT(baseSpeedSlot(float)));


    //树形数据接收链路
    connect(ui->pointCloudWidget, SIGNAL(infoTreeUpdateSignal(CameraController)), ui->infoTree, SLOT(update(CameraController)));

    return true;
}


void MainWindow::on_comboBox_currentIndexChanged(int index)
{
    emit modeSelectSignal(index);
}

void MainWindow::on_resetView_clicked()
{
    emit resetViewSignal();
}

void MainWindow::on_showMesh_clicked()
{
    if (ui->showMesh->isChecked())
    {
        emit showMeshSignal(true);
    }
    else
    {
        emit showMeshSignal(false);
    }
}

void MainWindow::on_showAxis_clicked()
{
    if (ui->showAxis->isChecked())
    {
        emit showAxisSignal(true);
    }
    else
    {
        emit showAxisSignal(false);
    }
}

void MainWindow::on_clearPointCloud_clicked()
{
    emit clearPointCloudSignal();
}

void MainWindow::on_fovController_valueChanged(int value)
{
    ui->fovValue->setText(QString::number(value));
    // qDebug() << value;
    emit fovChangedSignal(value);
}

void MainWindow::on_baseSpeedController_valueChanged(int value)
{
    ui->baseSpeedValue->setText(QString::number(value / 100.0f, 'f', 2));
    emit baseSpeedSignal(value / 100.0f);
}

void MainWindow::fovChangedSlot(int value)
{
    ui->fovController->setValue(value);
    ui->fovValue->setText(QString::number(value));
}

