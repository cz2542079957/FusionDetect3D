#ifndef INFOTREEMODEL_H
#define INFOTREEMODEL_H

#include "QStandardItemModel"
#include <cameraController.h>

class InfoTreeModel : public QStandardItemModel
{
    Q_OBJECT
public:
    InfoTreeModel(QObject *parent = nullptr);

    void updateData(const   CameraController &camera);

private:
    QStandardItem *position;

};

#endif // INFOTREEMODEL_H
