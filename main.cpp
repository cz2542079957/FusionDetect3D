#include "mainwindow.h"
#include <QApplication>
#include <QFile>
#include <QSurfaceFormat>

int main(int argc, char *argv[])
{

    // 调用rclcpp的初始化函数
    //    rclcpp::init(0, NULL);
    // 调用rclcpp的循环运行我们创建的first_node节点
    //    rclcpp::spin(std::make_shared<rclcpp::Node>("first_node"));/

    // QSurfaceFormat format;
    // format.setVersion(4, 3);    // 关键是这一句，设置opengl版本号
    // QSurfaceFormat::setDefaultFormat(format);

    QApplication a(argc, argv);
    MainWindow w;

    QFile styleFile(":style/main.qss");
    if (styleFile.open(QFile::ReadOnly))
    {
        QString styleSheet = QLatin1String(styleFile.readAll());
        a.setStyleSheet(styleSheet);
    }


    w.show();
    return a.exec();
}
