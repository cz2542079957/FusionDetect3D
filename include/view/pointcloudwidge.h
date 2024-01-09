#ifndef POINTCLOUDWIDGE_H
#define POINTCLOUDWIDGE_H

#include "QOpenGLWidget"
#include "QOpenGLFunctions_4_3_Core"
#include "QOpenGLShaderProgram"
#include "QKeyEvent"
#include "QTimer"
#include "QTime"
#include "cameracontroller.h"


class pointCloudWidge : public QOpenGLWidget, QOpenGLFunctions_4_3_Core
{
    Q_OBJECT
public:
    explicit pointCloudWidge(QWidget *parent = nullptr);
    ~pointCloudWidge();

private :
    //相机控制器
    cameraController camera;
    //变换矩阵(模型、视野、投影)
    QMatrix4x4 model, view, projection;
    //渲染距离
    float renderMinDistance =  0.05f;
    float renderMaxDistance = 100.0f;

    QTimer timer;


    std::vector<float> linesDataBuffer;

    QOpenGLShaderProgram shaderProgram;
    unsigned int  VAO, VBO;
    float vertices[18] =
    {
        0.5f, 0.5f, 0.0f,
        0.5f, -0.5f, 0.0f,
        -0.5f, 0.5f, 0.0f,
        0.5f, -0.5f, 0.0f,
        -0.5f, -0.5f, 0.0f,
        -0.5f, 0.5f, 0.0f,
    };


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
    //网格单位长度
    float meshLength =  0.2;
    //网格线宽
    float meshWidth = 2.0;
    //网格线数量
    unsigned int meshLinesCount = 0;
    //网格渲染数据
    std::vector<float> meshData;


    //按键监听
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event)override;
    //鼠标监听
    void mousePressEvent(QMouseEvent *event)override;
    void mouseMoveEvent(QMouseEvent *event)override;
    void mouseReleaseEvent(QMouseEvent *event)override;
    //鼠标滚轮监听
    void wheelEvent(QWheelEvent *event) override;

    void sendPoints(QVector<float> data);
    void drawPointsHandle();


protected:
    virtual void initializeGL() override;
    virtual void resizeGL(int w, int h) override;
    virtual void paintGL() override;

signals:

public slots:
    void onTimeout();
    //显示坐标轴
    void showAxis(bool val);
    //显示网格
    void showMesh(bool val);
    //绘制地平线网格
    void drawMesh(int rowBegin, int  rows, int  columnBegin, int  columns);

    //拿到点云数据
    //    void recvPointsData();


};


#endif // POINTCLOUDWIDGE_H
