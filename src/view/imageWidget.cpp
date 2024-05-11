#include "imageWidget.h"
#include "ui_imagewidget.h"
#include "QStandardPaths"
#include "QDir"


ImageWidget::ImageWidget(QWidget *parent): QWidget(parent),
    ui(new Ui::imageWidget)
{
    ui->setupUi(this);

    refresh();
}

ImageWidget::~ImageWidget()
{
    delete ui;
}

void ImageWidget::refresh()
{
    // 渲染rawPhotos
    ui->needDetectImageWidget->clear();
    for (Photo p : rawPhotos)
    {
        QPixmap objPixmap(QString::fromStdString(p.path));
        if (!objPixmap.isNull())
        {
            std::filesystem::path path(p.path);
            std::string fileName = path.filename().string();
            QListWidgetItem *pItem = new QListWidgetItem(QIcon(objPixmap.scaled(QSize(IMAGE_SIZE, IMAGE_SIZE))), QString::fromStdString(fileName));
            //设置单元项的宽度和高度
            pItem->setSizeHint(QSize(IMAGE_SIZE, IMAGE_SIZE));
            ui->needDetectImageWidget->addItem(pItem);
        }
    }
}

void ImageWidget::recvCameraDataSlot(std::string path)
{
    long long time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    // CarPosition cp = pointCloudDataManager.getCarPosition();
    // Photo p = {fileName, time, cp.x, cp.y, cp.roll};
    Photo p = {path, time, 0, 0, 0};
    rawPhotos.push_back(p);
    refresh();
}


void ImageWidget::on_cameraTakePhoto_clicked()
{
    emit sendCameraControlSignal();
}


void ImageWidget::on_detectPhoto_clicked()
{

}

