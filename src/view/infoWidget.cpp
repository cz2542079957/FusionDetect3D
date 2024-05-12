#include "infoWidget.h"
#include "ui_infowidget.h"
#include "cameraController.h"
#include "pointCloudDataManager.h"


InfoWidget::InfoWidget(QWidget *parent): QWidget(parent), ui(new Ui::InfoWidget)
{
    ui->setupUi(this);
}

InfoWidget::~InfoWidget()
{
    delete ui;
}

void InfoWidget::refresh(const CameraController &camera, PointCloudDataManager &pointCloudDataManager)
{

    ui->cameraPosX->setText(QString::number(camera.getBasePos().x()));
    ui->cameraPosY->setText(QString::number(camera.getBasePos().y()));
    ui->cameraPosZ->setText(QString::number(camera.getBasePos().z()));

    ui->cameraFront->setText(QString::number(camera.getBaseVector().x(), 'g', 2) + "，"
                             + QString::number(camera.getBaseVector().y(), 'g', 2) + "，"
                             + QString::number(camera.getBaseVector().z(), 'g', 2) );
    ui->cameraUp->setText(QString::number(camera.getCameraUp().x(), 'g', 2) + "，"
                          + QString::number(camera.getCameraUp().y(), 'g', 2) + "，"
                          + QString::number(camera.getCameraUp().z(), 'g', 2));
    ui->cameraRight->setText(QString::number(camera.getCameraRight().x(), 'g', 2) + "，"
                             + QString::number(camera.getCameraRight().y(), 'g', 2) + "，"
                             + QString::number(camera.getCameraRight().z(), 'g', 2));

    CarPosition cp = pointCloudDataManager.getCarPosition();

    ui->carPosX->setText(QString::number(cp.x));
    ui->carPosY->setText(QString::number(cp.y));
}



void InfoWidget::on_confirm_clicked()
{
    emit setCarPosSignal(ui->setCarPosX->value(),  ui->setCarPosY->value());
}

