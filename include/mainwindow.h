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
    DeviceController dc;

    //信号槽注册中心
    bool SignalsSlotsRegister();


signals:
    void modeSelect(int mode);
    void resetView();
    void showAxis(bool val);
    void showMesh(bool val);
    void clearPointCloud();

private slots:
    void on_showMesh_clicked();
    void on_showAxis_clicked();

    void on_clearPointCloud_clicked();
    void on_comboBox_currentIndexChanged(int index);
    void on_resetView_clicked();
};
#endif // MAINWINDOW_H
