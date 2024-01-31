#include "infoTree/infoTreeModel.h"

using namespace NSInfoTree;

InfoTreeModel::InfoTreeModel(QObject *parent) : QStandardItemModel(parent)
{
    for (int row = 0; row < 4; ++row)
    {
        QStandardItem *item = new QStandardItem(QString::number(row));
        for (int i = 0; i < 3; ++i)
        {
            QStandardItem *childItem = new QStandardItem("Child " + QString::number(i));
            item->appendRow(childItem);
        }
        appendRow(item);
    }
}
