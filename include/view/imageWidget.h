#ifndef IMAGEWIDGET_H
#define IMAGEWIDGET_H

#include "QWidget"
#include "QListWidget"
#include "QVBoxLayout"
#include "QProgressDialog"
#include "QLabel"
#include "QFileInfo"
#include "yolo.h"

namespace Ui
{
    class ImageWidget;
}

#define IMAGE_SIZE 120

struct Photo
{
    //保存的文件名
    std::string path;
    //时间戳
    long long timestamp;
    //方位
    float x, y;
    double roll;
};


class ImageWidget : public QWidget
{
    Q_OBJECT
public:
    ImageWidget(QWidget *parent = nullptr);
    ~ImageWidget();

    void refresh();

private:
    Ui::ImageWidget *ui;
    //YOLO
    YOLO *yolo;

    //拍摄的图片
    std::vector<Photo> rawPhotos;
    std::vector<Photo> handledPhotos;

    QProgressDialog *progressDialog;

signals:
    void sendCameraControlSignal();


public slots:
    //拿到camera拍摄的图片名
    void recvCameraDataSlot(std::string fileName);

private slots:
    void on_cameraTakePhoto_clicked();
    void on_detectPhoto_clicked();
};

class ImageBrowser : public QDialog
{
public:
    ImageBrowser(const QString &imagePath, QWidget *parent = nullptr) : QDialog(parent)
    {
        QVBoxLayout *layout = new QVBoxLayout(this);
        QLabel *imageLabel = new QLabel(this);
        QImage image(imagePath);
        imageLabel->setPixmap(QPixmap::fromImage(image));
        layout->addWidget(imageLabel);

        // 设置对话框的标题为图片的文件名
        QFileInfo fileInfo(imagePath);
        setWindowTitle(fileInfo.fileName());
    }
};

#endif // IMAGEWIDGET_H
