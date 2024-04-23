#ifndef POINTCLOUDWIDGET_H
#define POINTCLOUDWIDGET_H

#include "cameraController.h"
#include "carController.h"
#include "pointCloudDataManager.h"

#include "QOpenGLWidget"
#include "QOpenGLFunctions_4_3_Core"
#include "QOpenGLShaderProgram"


class PointCloudWidget : public QOpenGLWidget, QOpenGLFunctions_4_3_Core
{
    Q_OBJECT
public:
    explicit PointCloudWidget(QWidget *parent = nullptr);
    ~PointCloudWidget();
    // 点云数据管理器
    PointCloudDataManager pointCloudDataManager;
    //相机控制器
    CameraController camera;
    //小车控制器
    CarController car;

    void setEnableCarControl(bool val);

private :
    //变换矩阵(模型、视野、投影)
    QMatrix4x4 model, view, projection;
    //渲染距离
    float renderMinDistance =  0.05f;
    float renderMaxDistance = 100.0f;

    QTimer timer;

    //坐标轴
    bool enableAxis  = true;
    unsigned int axisVAO, axisVBO;
    QOpenGLShaderProgram shaderProgramAxis;
    //坐标轴长度
    float axisLength = 1;
    //坐标轴宽度
    float axisWidth = 80;
    //坐标轴渲染数据
    float axisData[36] =
    {
        0.0f, 0.0f, 0.0f, 0.4f, 0.0f, 0.0f,
        axisLength, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.4f, 0.0f,
        0.0f, axisLength, 0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.4f,
        0.0f, 0.0f, axisLength, 0.0f, 0.0f, 1.0f
    };


    //地平线网格
    bool enableMesh = true;
    unsigned meshVAO, meshVBO ;
    QOpenGLShaderProgram shaderProgramMesh;
    //网格单位长度(m)
    float meshLength =  0.2;
    //网格线宽(m)
    float meshWidth = 2.0;
    //网格线数量
    unsigned int meshLinesCount = 0;
    //网格渲染数据
    std::vector<float> meshData;


    //点云
    bool enablePoints = true;
    unsigned pointsVAO, pointsVBO;
    QOpenGLShaderProgram shaderProgramPoints;
    //点大小
    float pointSize = 2;

    //是否允许小车控制
    bool enableCarControl = false;
    //按键监听
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event)override;
    //鼠标监听
    void mousePressEvent(QMouseEvent *event)override;
    void mouseMoveEvent(QMouseEvent *event)override;
    void mouseReleaseEvent(QMouseEvent *event)override;
    //鼠标滚轮监听
    void wheelEvent(QWheelEvent *event) override;


protected:
    virtual void initializeGL() override;
    virtual void resizeGL(int w, int h) override;
    virtual void paintGL() override;

    virtual void focusOutEvent(QFocusEvent *event) override;

signals:
    void infoTreeUpdateSignal(const CameraController &camera);

public slots:
    void onTimeout();
    //重置视角
    void resetViewSlot();
    //显示坐标
    void showAxisSlot(bool val);
    //显示网格
    void showMeshSlot(bool val);
    //绘制地平线网格
    void drawMeshSlot(int rowBegin, int  rows, int  columnBegin, int  columns);
    //清空点云
    void clearPointCloudSlot();

    //拿到点云数据
    void recvPointsDataSlot(message::msg::LidarData::SharedPtr msg);
    //拿到舵机数据
    void recvServoDataSlot(message::msg::CarServoData::SharedPtr msg);
    //拿到惯导数据
    void recvLidarImuDataSlot(message::msg::ImuData::SharedPtr msg);
    //拿到编码器数据
    void recvEncoderDataSlot(message::msg::CarEncoderData::SharedPtr msg);

};
#endif // POINTCLOUDWIDGET_H
