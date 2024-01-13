#include "infoTree/infoTree.h"

using namespace  NSInfoTree;

InfoTree::InfoTree(QWidget *parent): QTreeView(parent)
{
    model = new QStandardItemModel();
    for (int row = 0; row < 4; ++row)
    {
        QStandardItem *item = new QStandardItem(QString::number(row));
        for (int i = 0; i < 3; ++i)
        {
            QStandardItem *childItem = new QStandardItem("Child " + QString::number(i));
            item->appendRow(childItem);
        }
        model->appendRow(item);
    }
    setModel(model);
    // setSelectionMode(QAbstractItemView::ExtendedSelection);
    setEditTriggers(QAbstractItemView::NoEditTriggers);
}
