#ifndef IMAGEWIDGET_H
#define IMAGEWIDGET_H

#include "QWidget"
#include "QListWidget"
#include "QVBoxLayout"

namespace Ui
{
    class imageWidget;
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
    Ui::imageWidget *ui;
    QListWidget needDetectImageWidget;
    QListWidget detectedImageWidget;

    //拍摄的图片
    std::vector<Photo> rawPhotos;
    std::vector<Photo> handledPhotos;

signals:
    void sendCameraControlSignal();


public slots:
    //拿到camera拍摄的图片名
    void recvCameraDataSlot(std::string fileName);

private slots:
    void on_cameraTakePhoto_clicked();
    void on_detectPhoto_clicked();
};

#endif // IMAGEWIDGET_H
