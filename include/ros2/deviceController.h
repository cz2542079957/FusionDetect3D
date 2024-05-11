#ifndef DEVICECONTROLLER_H
#define DEVICECONTROLLER_H
#include <QObject>
#include <carMasterNode.h>
#include "carImuNode.h"
#include "cameraNode.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <lidarNode.h>
#include <lidarImuNode.h>

class DeviceController : public QObject
{
    Q_OBJECT
public:
    DeviceController();

    std::shared_ptr<CarMasterNode> carMaterNode;
    std::shared_ptr<CameraNode> cameraNode;

private:
    //雷达扫描数据回调函数，用于对接ui线程
    void lidarScanCallback(const message::msg::LidarData::SharedPtr msg);
    //惯导模块数据回调函数
    void lidarImuDataCallback(const message::msg::ImuData::SharedPtr msg);
    //编码器数据回调函数
    void encoderDataCallback(const message::msg::CarEncoderData::SharedPtr msg);
    //惯导模块数据回调函数
    void carImuDataCallback(const message::msg::ImuData::SharedPtr msg);
    //舵机数据回调函数
    void servoDataCallback(const message::msg::CarServoData::SharedPtr msg);
    //电池电压数据回调函数
    void voltageDataCallback(const message::msg::CarVotageData::SharedPtr msg);
    //相机数据回调函数
    void cameraDataCallback(const sensor_msgs::msg::Image::SharedPtr msg);

signals:
    //发送点云数据
    void sendPointsSignal(const message::msg::LidarData::SharedPtr msg);
    //发送舵机数据r
    void sendServoDataSignal(const message::msg::CarServoData::SharedPtr msg);
    //发送lidarIMU数据
    void sendLidarImuDataSignal(const message::msg::ImuData::SharedPtr msg);
    //发送编码器数据
    void sendEncoderDataSignal(const message::msg::CarEncoderData::SharedPtr msg);
    //发送carIMU数据
    void sendCarImuDataSignal(const message::msg::ImuData::SharedPtr msg);
    //发送电池电压数据
    void sendVoltageDataSignal(const message::msg::CarVotageData::SharedPtr msg);
    //发送camera拍摄保存的图片
    void sendCameraDataSignal(const std::string fileName);


public slots:
    // void  sendModeSlot(int mode);
    void  sendControlSlot(int state, int speed);

    void sendCameraControlSlot();
};



#endif // DEVICECONTROLLER_H
