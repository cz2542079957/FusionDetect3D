#include "imageWidget.h"
#include "ui_imagewidget.h"
#include "QStandardPaths"
#include "QThread"
#include "QMessageBox"
#include "QDir"


ImageWidget::ImageWidget(QWidget *parent): QWidget(parent),
    ui(new Ui::ImageWidget)
{
    ui->setupUi(this);

    QObject::connect(ui->needDetectImageWidget, &QListWidget::itemDoubleClicked, [&](QListWidgetItem * item)
    {
        QString picturesFolderPath = QStandardPaths::writableLocation(QStandardPaths::PicturesLocation);
        QString imagePath = picturesFolderPath + QLatin1String("/FD3_RawImages/") + item->text();
        ImageBrowser browser(imagePath);
        browser.exec();
    });

    QObject::connect(ui->detectedImageWidget, &QListWidget::itemDoubleClicked, [&](QListWidgetItem * item)
    {
        QString picturesFolderPath = QStandardPaths::writableLocation(QStandardPaths::PicturesLocation);
        QString imagePath = picturesFolderPath + QLatin1String("/FD3_DetectedImages/") + item->text();
        ImageBrowser browser(imagePath);
        browser.exec();
    });

    //初始化YOLO
    QFile classesFile("://yolo-fastest/coco.names");
    if (!classesFile.open(QIODevice::ReadOnly))
    {
        qDebug() << "Failed to open classes file.";
    }
    QByteArray classesData = classesFile.readAll();
    classesFile.close();
    QFile configFile("://yolo-fastest/yolo-fastest-xl.cfg");
    if (!configFile.open(QIODevice::ReadOnly))
    {
        qDebug() << "Failed to open configuration file.";
    }
    QByteArray cfgData = configFile.readAll();
    configFile.close();
    QFile weightsFile("://yolo-fastest/yolo-fastest-xl.weights");
    if (!weightsFile.open(QIODevice::ReadOnly))
    {
        qDebug() << "Failed to open weights file.";
    }
    QByteArray weightsData = weightsFile.readAll();
    weightsFile.close();

    QString classesString = QString(classesData);
    QStringList stringList = classesString.split('\n', Qt::SkipEmptyParts);
    std::vector<std::string> classLabels;
    for (const QString &className : stringList)
    {
        classLabels.push_back(className.toStdString());
    }
    std::vector<uchar> cfgBuffer(cfgData.begin(), cfgData.end());
    std::vector<uchar> weightsBuffer(weightsData.begin(), weightsData.end());

    yolo =  new YOLO(classLabels, cfgBuffer, weightsBuffer);

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

    //渲染handledPhotos
    ui->detectedImageWidget->clear();
    for (Photo p : handledPhotos)
    {
        QPixmap objPixmap(QString::fromStdString(p.path));
        if (!objPixmap.isNull())
        {
            std::filesystem::path path(p.path);
            std::string fileName = path.filename().string();
            QListWidgetItem *pItem = new QListWidgetItem(QIcon(objPixmap.scaled(QSize(IMAGE_SIZE, IMAGE_SIZE))), QString::fromStdString(fileName));
            //设置单元项的宽度和高度
            pItem->setSizeHint(QSize(IMAGE_SIZE, IMAGE_SIZE));
            ui->detectedImageWidget->addItem(pItem);
        }
    }
}

void ImageWidget::recvCameraDataSlot(std::string path)
{
    long long time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
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
    QList<QListWidgetItem *> list =  ui->needDetectImageWidget->selectedItems();
    if (list.size() == 0)
    {
        QMessageBox::warning(this, "提示", "请先选择可用的原始图片");
        return;
    }
    QString picturesFolderPath = QStandardPaths::writableLocation(QStandardPaths::PicturesLocation);
    QString directory = picturesFolderPath + QLatin1String("/FD3_RawImages/");
    QString detectedDirectory = picturesFolderPath + QLatin1String("/FD3_DetectedImages/");

    // 创建进度条弹窗
    progressDialog = new QProgressDialog( "等待...", "取消", 0, 100, this);
    progressDialog->setWindowTitle("目标识别中...");
    progressDialog->setWindowModality(Qt::WindowModal);
    progressDialog->show();

    auto task = [this, list, directory, detectedDirectory]()
    {
        /*
        对progressDialog的函数的触发使用QMetaObject::invokeMethod的方式的原因是：
            task函数运行在另一个线程，直接调用UI主线程中创建的progressDialog会导致线程相关错误
        */
        int size = list.size();
        int handledNumber = 0;
        for (QListWidgetItem *item : list)
        {
            try
            {
                QString path = directory + item->text();
                QMetaObject::invokeMethod(progressDialog, [this, path]()
                {
                    progressDialog->setLabelText("正在识别:" + path);
                }, Qt::QueuedConnection);
                for (int i = rawPhotos.size() - 1; i >= 0; --i)
                {
                    const Photo &p = rawPhotos[i];
                    if (path == QString::fromStdString(p.path))
                    {
                        rawPhotos.erase(rawPhotos.begin() + i);
                        Mat image = imread(path.toStdString());
                        yolo->detect(image);
                        QString fileName = QString("image_%1.jpg").arg(QDateTime::currentDateTime().toString("yyyy-MM-dd_HH-mm-ss_zzz"));
                        std::string path = detectedDirectory.toStdString() + fileName.toStdString();
                        // 检查目录是否存在，如果不存在则创建
                        if (!std::filesystem::exists(detectedDirectory.toStdString()))
                        {
                            std::filesystem::create_directories(detectedDirectory.toStdString());
                        }
                        //保存图片到输出目录
                        if (imwrite(path, image))
                        {
                            long long time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                            Photo p = {path, time, 0, 0, 0};
                            handledPhotos.push_back(p);
                        }
                        break;
                    }
                }
                handledNumber++;
                QMetaObject::invokeMethod(progressDialog, [this, handledNumber, size]()
                {
                    progressDialog->setValue(((handledNumber  + 0.0) / size) * 100.0);
                }, Qt::QueuedConnection);
            }
            catch (cv::Exception e)
            {
                qDebug() << e.what();
            }
        }
        QMetaObject::invokeMethod(this, &ImageWidget::refresh, Qt::QueuedConnection);
    };
    QThread *workerThread = new QThread(this);
    QObject::connect(workerThread, &QThread::started, task);
    connect(progressDialog, &QProgressDialog::canceled, workerThread, &QThread::requestInterruption);
    connect(workerThread, &QThread::finished, this, [this, workerThread]()
    {
        workerThread->deleteLater();
        progressDialog->deleteLater();
    }, Qt::QueuedConnection);
    workerThread->start();
}

