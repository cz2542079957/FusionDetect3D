#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    dc = new DeviceController();
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
    delete dc;
}

bool MainWindow::SignalsSlotsRegister()
{
    // 工具栏按钮绑定
    connect(this, SIGNAL(resetViewSignal()), ui->pointCloudWidget, SLOT(resetViewSlot()));
    connect(this, SIGNAL(showAxisSignal(bool)), ui->pointCloudWidget, SLOT(showAxisSlot(bool)));
    connect(this, SIGNAL(showMeshSignal(bool)), ui->pointCloudWidget, SLOT(showMeshSlot(bool)));
    connect(this, SIGNAL(clearPointCloudSignal()), ui->pointCloudWidget, SLOT(clearPointCloudSlot()));
    connect(&ui->pointCloudWidget->camera, SIGNAL(fovChangedSignal(int)), this, SLOT(fovChangedSlot(int)));
    connect(this, SIGNAL(fovChangedSignal(int)), &ui->pointCloudWidget->camera, SLOT(fovChangedSlot(int)));
    connect(this, SIGNAL(baseSpeedSignal(float)), &ui->pointCloudWidget->camera, SLOT(baseSpeedSlot(float)));
    connect(this, SIGNAL(carSpeedSignal(int)), &ui->pointCloudWidget->car, SLOT(carSpeedSlot(int)));

    // ros节点收发链路
    connect(this->dc, &DeviceController::sendPointsSignal, ui->pointCloudWidget, &PointCloudWidget::recvPointsDataSlot);
    connect(this->dc, &DeviceController::sendServoDataSignal, ui->pointCloudWidget, &PointCloudWidget::recvServoDataSlot);
    connect(this->dc, &DeviceController::sendLidarImuDataSignal, ui->pointCloudWidget, &PointCloudWidget::recvLidarImuDataSlot);
    connect(this->dc, &DeviceController::sendEncoderDataSignal, ui->pointCloudWidget, &PointCloudWidget::recvEncoderDataSlot);
    connect(&ui->pointCloudWidget->car, &CarController::sendControlSignal, this->dc, &DeviceController::sendControlSlot);
    connect(this->dc, &DeviceController::sendVoltageDataSignal, this, &MainWindow::sendVoltageDataSlot);

    // 点云界面
    connect(&ui->pointCloudWidget->pointCloudDataManager, SIGNAL(updateGraph()), ui->pointCloudWidget, SLOT(update()));
    connect(&ui->pointCloudWidget->camera, SIGNAL(updateGraph()), ui->pointCloudWidget, SLOT(update()));

    // 树形数据接收链路
    connect(ui->pointCloudWidget, SIGNAL(infoTreeUpdateSignal(CameraController)), ui->infoTree, SLOT(update(CameraController)));

    return true;
}

void MainWindow::on_comboBox_currentIndexChanged(int index)
{
    ui->pointCloudWidget->camera.modeSelect(index);
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

void MainWindow::on_carModeSelecter_currentIndexChanged(int index)
{
    ui->pointCloudWidget->car.setMode(index);
    dc->carMaterNode->publishModeControl(index);
}

void MainWindow::on_enableKeyboardControl_clicked()
{
    if (ui->enableKeyboardControl->isChecked())
    {
        ui->pointCloudWidget->setEnableCarControl(true);
    }
    else
    {
        ui->pointCloudWidget->setEnableCarControl(false);
    }
}

void MainWindow::on_carSpeedController_valueChanged(int value)
{
    ui->carSpeedValue->setText(QString::number(value));
    emit carSpeedSignal(value);
}

void MainWindow::sendVoltageDataSlot(const message::msg::CarVotageData::SharedPtr msg)
{
    float percentage =  (msg->voltage - 9.6) / 3.0;
    if (percentage > 1)
    {
        percentage = 1;
    }
    if (percentage < 0)
    {
        percentage = 0;
    }
    // qDebug() << msg->voltage << " "  << percentage;
    ui->voltageBar->setValue(percentage * 100);
    if (percentage > 0.8)
    {
        ui->voltageBar->setStyleSheet("QProgressBar{background:white; color: white; text-align:center; } QProgressBar::chunk{background:#37b24d}");
    }
    else if (percentage > 0.4)
    {
        ui->voltageBar->setStyleSheet("QProgressBar{background:white; text-align:center;} QProgressBar::chunk{background:#ffb90f}");
    }
    else
    {
        ui->voltageBar->setStyleSheet("QProgressBar{background:white; text-align:center;} QProgressBar::chunk{background:#ff3300}");
    }
}
