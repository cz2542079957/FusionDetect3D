#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "deviceController.h"

QT_BEGIN_NAMESPACE
namespace Ui
{
    class MainWindow;
}
QT_END_NAMESPACE


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    //页面组件
    Ui::MainWindow *ui;
    //设备控制器
    DeviceController *dc;

    //信号槽注册中心
    bool SignalsSlotsRegister();


signals:
    void modeSelectSignal(int mode);
    void resetViewSignal();
    void showAxisSignal(bool val);
    void showMeshSignal(bool val);
    void clearPointCloudSignal();
    void fovChangedSignal(int value);
    void baseSpeedSignal(float value);

private slots:
    void on_showMesh_clicked();
    void on_showAxis_clicked();
    void on_clearPointCloud_clicked();
    void on_comboBox_currentIndexChanged(int index);
    void on_resetView_clicked();

    void on_fovController_valueChanged(int value);
    void fovChangedSlot(int value);
    void on_baseSpeedController_valueChanged(int value);

    void on_carModeSelecter_currentIndexChanged(int index);
    void on_enableKeyboardControl_clicked();

};
#endif // MAINWINDOW_H
