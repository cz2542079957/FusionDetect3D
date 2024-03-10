#include "infoTree/infoTreeModel.h"


InfoTreeModel::InfoTreeModel(QObject *parent) : QStandardItemModel(parent)
{
    //方位信息
    position = new QStandardItem("世界坐标");
    position->appendRow(new QStandardItem("X: "));
    position->appendRow(new QStandardItem("Y: "));
    position->appendRow(new QStandardItem("Z: "));
    position->appendRow(new QStandardItem("Roll: "));
    position->appendRow(new QStandardItem("Pitch: "));
    position->appendRow(new QStandardItem("Yaw: "));
    appendRow(position);
}

void InfoTreeModel::updateData(const  CameraController &camera)
{
    position->child(0, 0)->setText("X: " + QString::number(camera.getBasePos().x()));
    position->child(1, 0)->setText("Y: " + QString::number(camera.getBasePos().y()));
    position->child(2, 0)->setText("Z: " + QString::number(camera.getBasePos().z()));

    position->child(3, 0)->setText("前向量: (" + QString::number(camera.getBaseVector().x()) + "，"
                                   + QString::number(camera.getBaseVector().y()) + "，"
                                   + QString::number(camera.getBaseVector().z()) + ")");
    position->child(4, 0)->setText("上向量: (" + QString::number(camera.getCameraUp().x()) + "，"
                                   + QString::number(camera.getCameraUp().y()) + "，"
                                   + QString::number(camera.getCameraUp().z()) + ")");
    position->child(5, 0)->setText("右向量: (" + QString::number(camera.getCameraRight().x()) + "，"
                                   + QString::number(camera.getCameraRight().y()) + "，"
                                   + QString::number(camera.getCameraRight().z()) + ")");

}


