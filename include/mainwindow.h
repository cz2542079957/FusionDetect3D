#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

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
    Ui::MainWindow *ui;
    //设备控制器
//    deviceController dc;


signals:
    void showAxis(bool val);
    void showMesh(bool val);

private slots:
    void on_showMesh_clicked();
    void on_showAxis_clicked();
};
#endif // MAINWINDOW_H
