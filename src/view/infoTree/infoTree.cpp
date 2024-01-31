#include "infoTree/infoTree.h"

using namespace NSInfoTree;

InfoTree::InfoTree(QWidget *parent): QTreeView(parent)
{

    model = new InfoTreeModel();
    setModel(model);
    // setSelectionMode(QAbstractItemView::ExtendedSelection);
    setEditTriggers(QAbstractItemView::NoEditTriggers);
}
