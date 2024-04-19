#ifndef DEVICECONTROLLER_H
#define DEVICECONTROLLER_H
#include <QObject>
#include <carMasterNode.h>
#include <lidarNode.h>
#include <imuNode.h>

class DeviceController : public QObject
{
    Q_OBJECT
public:
    DeviceController();

    std::shared_ptr<CarMasterNode> carMaterNode;

private:
    //雷达扫描数据回调函数，用于对接ui线程
    void lidarScanCallback(const message::msg::LidarData::SharedPtr msg);
    //惯导模块数据回调函数
    void imuDataCallback(const message::msg::ImuData::SharedPtr msg);
    //编码器数据回调函数
    void encoderDataCallback(const message::msg::CarEncoderData::SharedPtr msg);
    //舵机数据回调函数
    void servoDataCallback(const message::msg::CarServoData::SharedPtr msg);

signals:
    //发送点云数据
    void sendPointsSignal(const message::msg::LidarData::SharedPtr msg);
    //发送点云数据
    void sendImuDataSignal(const message::msg::ImuData::SharedPtr msg);


public slots:
    // void  sendModeSlot(int mode);
    void  sendControlSlot(int state, int speed);
};



#endif // DEVICECONTROLLER_H
