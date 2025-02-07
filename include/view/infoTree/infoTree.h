#ifndef INFOTREE_H
#define INFOTREE_H

#include "QTreeView"
#include "infoTreeModel.h"

class InfoTree : public QTreeView
{
    Q_OBJECT
public:
    InfoTree(QWidget *parent = nullptr);
    //完成信息状态 显示

private:
    InfoTreeModel *model;

public slots:
    void update(const CameraController &camera);

};

#endif // INFOTREE_H
