#ifndef INFOTREEMODEL_H
#define INFOTREEMODEL_H

#include "QStandardItemModel"

namespace NSInfoTree
{
    class InfoTreeModel : public QStandardItemModel
    {
        Q_OBJECT
    public:
        InfoTreeModel(QObject *parent = nullptr);
    };
}

#endif // INFOTREEMODEL_H
