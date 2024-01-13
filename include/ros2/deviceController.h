#ifndef DEVICECONTROLLER_H
#define DEVICECONTROLLER_H
#include <QObject>
#include "lidar/lidarNode.h"

class DeviceController : public QObject
{
    Q_OBJECT
public:
    DeviceController();

private:
    //雷达扫描数据回调函数，用于对接ui线程
    void lidarScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

signals:
    //发送点云数据
    void sendPointsSignals(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};



#endif // DEVICECONTROLLER_H
