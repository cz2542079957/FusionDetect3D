#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "QFileDialog"
#include "QMessageBox"

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
    connect(this, &MainWindow::resetViewSignal, ui->pointCloudWidget, &PointCloudWidget::resetViewSlot);
    connect(this, &MainWindow::showAxisSignal, ui->pointCloudWidget, &PointCloudWidget::showAxisSlot);
    connect(this, &MainWindow::showMeshSignal, ui->pointCloudWidget, &PointCloudWidget::showMeshSlot);
    connect(this, &MainWindow::clearPointCloudSignal, ui->pointCloudWidget, &PointCloudWidget::clearPointCloudSlot);
    connect(this, &MainWindow::clearPositionPointSignal, ui->pointCloudWidget, &PointCloudWidget::clearPositionPointSlot);
    connect(this, &MainWindow::syncIMURollSignal, ui->pointCloudWidget, &PointCloudWidget::syncIMURollSlot);

    connect(&ui->pointCloudWidget->camera, &CameraController::fovChangedSignal, this, &MainWindow::fovChangedSlot);
    connect(this, &MainWindow::fovChangedSignal, &ui->pointCloudWidget->camera, &CameraController::fovChangedSlot);
    connect(this, &MainWindow::baseSpeedSignal, &ui->pointCloudWidget->camera, &CameraController::baseSpeedSlot);
    connect(this, &MainWindow::carSpeedSignal, &ui->pointCloudWidget->car, &CarController::carSpeedSlot);

    // ros节点收发链路
    connect(this->dc, &DeviceController::sendPointsSignal, ui->pointCloudWidget, &PointCloudWidget::recvPointsDataSlot);
    connect(this->dc, &DeviceController::sendServoDataSignal, ui->pointCloudWidget, &PointCloudWidget::recvServoDataSlot);
    connect(this->dc, &DeviceController::sendLidarImuDataSignal, ui->pointCloudWidget, &PointCloudWidget::recvLidarImuDataSlot);
    connect(this->dc, &DeviceController::sendEncoderDataSignal, ui->pointCloudWidget, &PointCloudWidget::recvEncoderDataSlot);
    connect(this->dc, &DeviceController::sendCarImuDataSignal, ui->pointCloudWidget, &PointCloudWidget::recvCarImuDataSlot);
    connect(&ui->pointCloudWidget->car, &CarController::sendControlSignal, this->dc, &DeviceController::sendControlSlot);
    connect(this->dc, &DeviceController::sendVoltageDataSignal, this, &MainWindow::sendVoltageDataSlot);
    connect(ui->imageWidget, &ImageWidget::sendCameraControlSignal, this->dc, &DeviceController::sendCameraControlSlot);
    connect(this->dc, &DeviceController::sendCameraDataSignal, ui->imageWidget, &ImageWidget::recvCameraDataSlot);

    // 点云界面
    connect(&ui->pointCloudWidget->pointCloudDataManager, &PointCloudDataManager::updateGraph, ui->pointCloudWidget,
            static_cast<void (PointCloudWidget::*)()>(&PointCloudWidget::update));
    connect(&ui->pointCloudWidget->camera, &CameraController::updateGraph, ui->pointCloudWidget,
            static_cast<void (PointCloudWidget::*)()>(&PointCloudWidget::update));

    // 树形数据接收链路
    connect(ui->pointCloudWidget, &PointCloudWidget::infoTreeUpdateSignal, ui->infoTree, &InfoTree::update);

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


void MainWindow::on_clearPositionPoint_clicked()
{
    emit clearPositionPointSignal();
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

void MainWindow::on_minRenderDistanceController_valueChanged(int value)
{
    ui->minRenderDistanceValue->setText(QString::number(value / 100.0f, 'f', 2));
    ui->pointCloudWidget->setMinRenderDistanceSlot(value / 100.0f);
}


void MainWindow::on_maxRenderDistanceController_valueChanged(int value)
{
    ui->maxRenderDistanceValue->setText(QString::number(value));
    ui->pointCloudWidget->setMaxRenderDistanceSlot(value);
}


void MainWindow::on_pointSizeController_valueChanged(int value)
{
    ui->pointSizeValue->setText(QString::number(value / 10.0f, 'f', 2));
    ui->pointCloudWidget->setPointSizeSlot(value / 10.0f);
}


void MainWindow::on_positionPointSizeController_valueChanged(int value)
{
    ui->positionPointSizeValue->setText(QString::number(value / 10.0f, 'f', 2));
    ui->pointCloudWidget->setPositionPointSizeSlot(value / 10.0f);
}


void MainWindow::on_syncIMURoll_clicked()
{
    emit syncIMURollSignal();
}

void MainWindow::on_savePointCloud_clicked()
{
    QString dirPath;
    QString saveName = "points"; // 文本文件的默认名称
    QDateTime time = QDateTime::currentDateTime();
    QString str = time.toString("yyyyMMdd_hhmmss"); // 当前时间的字符串表示
    QString saveNameWithTime = QString("%1_%2.bin").arg(saveName).arg(str); // 结合时间和文件名

    dirPath = QFileDialog::getExistingDirectory(this, "选择保存文件夹", ""); // 弹出文件夹选择对话框

    if (dirPath.isEmpty())
    {
        QMessageBox::information(this, "信息", "保存失败，未选择文件夹");
    }
    else
    {
        QString filePath = QString("%1/%2").arg(dirPath).arg(saveNameWithTime); // 完整的文件路径
        QFile file(filePath);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
        {
            QMessageBox::information(this, "信息", "保存失败，无法打开文件");
        }
        else
        {
            QDataStream out(&file);
            out.setVersion(QDataStream::Qt_6_0); // 设置数据流的版本，确保兼容性
            for (const PointCloudVertex &vertex : ui->pointCloudWidget->pointCloudDataManager.getData())
            {
                out << vertex.x << vertex.y << vertex.z << vertex.red << vertex.green << vertex.blue;
            }
            file.close();
            QMessageBox::information(this, "信息", "保存成功");
        }
    }
}


void MainWindow::on_loadPointCloud_clicked()
{
    QString dirPath = QFileDialog::getOpenFileName(this, "选择要读取的文件", "", "Binary Files (*.bin);;All Files (*)"); // 弹出文件选择对话框

    if (dirPath.isEmpty())
    {
        QMessageBox::information(this, "信息", "读取失败，未选择文件");
        return;
    }

    QFile file(dirPath);
    if (!file.open(QIODevice::ReadOnly))
    {
        QMessageBox::information(this, "信息", "读取失败，无法打开文件");
    }
    else
    {

        QDataStream in(&file);
        in.setVersion(QDataStream::Qt_6_0);
        std::vector<PointCloudVertex> list;
        while (!in.atEnd())
        {
            PointCloudVertex vertex;
            in >> vertex.x >> vertex.y >> vertex.z >> vertex.red >> vertex.green >> vertex.blue;
            list.push_back(vertex);
        }
        ui->pointCloudWidget->pointCloudDataManager.loadPointCloud(list);
        file.close();
        QMessageBox::information(this, "信息", "读取成功");
    }

}

