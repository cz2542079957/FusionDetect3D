#include "infoTree/infoTree.h"

InfoTree::InfoTree(QWidget *parent): QTreeView(parent)
{
    model = new InfoTreeModel();
    setModel(model);
    // setSelectionMode(QAbstractItemView::ExtendedSelection);
    setEditTriggers(QAbstractItemView::NoEditTriggers);
}

void InfoTree::update(const CameraController &camera)
{
    model->updateData(camera);
}
