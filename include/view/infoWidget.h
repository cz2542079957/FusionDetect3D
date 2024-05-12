#ifndef INFOWIDGET_H
#define INFOWIDGET_H

#include "QWidget"

namespace Ui
{
    class InfoWidget;
}

class CameraController;
class PointCloudDataManager;

class InfoWidget: public QWidget
{
    Q_OBJECT
public:
    InfoWidget(QWidget *parent = nullptr);
    ~InfoWidget();

private:
    Ui::InfoWidget *ui;

signals:
    void setCarPosSignal(float x, float y);

public slots:
    void refresh(const CameraController &camera, PointCloudDataManager &pointCloudDataManager);


private slots:
    void on_confirm_clicked();
};


#endif // INFOWIDGET_H
