#ifndef DEVICECONTROLLER_H
#define DEVICECONTROLLER_H
#include <QObject>
#include <lidarNode.h>
#include <imuNode.h>

class DeviceController : public QObject
{
    Q_OBJECT
public:
    DeviceController();

private:
    //雷达扫描数据回调函数，用于对接ui线程
    void lidarScanCallback(const message::msg::LidarData::SharedPtr msg);
    //惯导模块数据回调函数
    void imuDataCallback(const message::msg::ImuData::SharedPtr msg);

signals:
    //发送点云数据
    void sendPointsSignals(const message::msg::LidarData::SharedPtr msg);
    //发送点云数据
    void sendImuDataSignals(const message::msg::ImuData::SharedPtr msg);
};



#endif // DEVICECONTROLLER_H
