﻿#include "pointCloudWidget.h"

PointCloudWidget::PointCloudWidget(QWidget *parent): QOpenGLWidget(parent)
{
    this->setGeometry(parent->rect().x(), parent->rect().y(), parent->rect().width(), parent->rect().height());
    setFocusPolicy(Qt::ClickFocus);
}

PointCloudWidget::~PointCloudWidget()
{
    if (!isValid())
    {
        return;
    }
    makeCurrent();
    glDeleteBuffers(1, &axisVBO);
    glDeleteVertexArrays(1, &axisVAO);
    glDeleteBuffers(1, &meshVBO);
    glDeleteVertexArrays(1, &meshVAO);
    glDeleteBuffers(1, &pointsVBO);
    glDeleteVertexArrays(1, &pointsVAO);
    glDeleteBuffers(1, &positionVBO);
    glDeleteVertexArrays(1, &positionVAO);
    doneCurrent();
}

void PointCloudWidget::setEnableCarControl(bool val)
{
    enableCarControl = val;
    if (val)
    {
        camera.clearKeys();
    }
    else
    {
        car.clearKeys();
    }
}

void PointCloudWidget::keyPressEvent(QKeyEvent *event)
{
    if (enableCarControl)
    {
        car.keypressActionHandler(event);
    }
    else
    {
        camera.keypressActionHandler(event);
    }
}

void PointCloudWidget::keyReleaseEvent(QKeyEvent *event)
{
    if (enableCarControl)
    {
        car.keyreleaseActionHandler(event);
    }
    else
    {
        camera.keyreleaseActionHandler(event);
    }
}

void PointCloudWidget::mousePressEvent(QMouseEvent *event)
{
    camera.mousepressActionHandler(event);
}

void PointCloudWidget::mouseMoveEvent(QMouseEvent *event)
{
    camera.mousemoveActionHandler(event);
}

void PointCloudWidget::mouseReleaseEvent(QMouseEvent *event)
{
    camera.mousereleaseActionHandler(event);
}

void PointCloudWidget::wheelEvent(QWheelEvent *event)
{
    camera.wheelActionHandler(event);
}

void PointCloudWidget::initializeGL()
{
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);

    shaderProgramAxis.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shader/axis_430.vert");
    shaderProgramAxis.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shader/axis_430.frag");
    bool success = shaderProgramAxis.link();
    if (!success)
    {
        qDebug() << "ERR:" << shaderProgramAxis.log();
    }
    shaderProgramMesh.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shader/mesh_430.vert");
    shaderProgramMesh.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shader/mesh_430.frag");
    success = shaderProgramMesh.link();
    if (!success)
    {
        qDebug() << "ERR:" << shaderProgramMesh.log();
    }
    shaderProgramPoints.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shader/point_430.vert");
    shaderProgramPoints.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shader/point_430.frag");
    success = shaderProgramPoints.link();
    if (!success)
    {
        qDebug() << "ERR:" << shaderProgramPoints.log();
    }
    shaderProgramPosition.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shader/position_430.vert");
    shaderProgramPosition.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shader/position_430.frag");
    if (!success)
    {
        qDebug() << "ERR:" << shaderProgramPosition.log();
    }

    //---坐标轴数据---
    glGenVertexArrays(1, &axisVAO);
    glGenBuffers(1, &axisVBO);
    glBindVertexArray(axisVAO);
    glBindBuffer(GL_ARRAY_BUFFER, axisVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(axisData), axisData, GL_STATIC_DRAW);
    shaderProgramAxis.bind();
    GLint pos1 = shaderProgramAxis.attributeLocation("aPos");
    glVertexAttribPointer(pos1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(pos1);
    GLint color1 = shaderProgramAxis.attributeLocation("aColor");
    glVertexAttribPointer(color1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(color1);

    //---网格数据---
    drawMeshSlot(-5, 10, -5, 10);
    glGenVertexArrays(1, &meshVAO);
    glGenBuffers(1, &meshVBO);
    glBindVertexArray(meshVAO);
    glBindBuffer(GL_ARRAY_BUFFER, meshVBO);
    glBufferData(GL_ARRAY_BUFFER, meshData.size() * sizeof(float), &meshData[0],  GL_DYNAMIC_DRAW);
    shaderProgramMesh.bind();
    GLint pos2 = shaderProgramMesh.attributeLocation("aPos");
    glVertexAttribPointer(pos2, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)(0 * sizeof(float)));
    glEnableVertexAttribArray(pos2);

    //---点云---
    glGenVertexArrays(1, &pointsVAO);
    glGenBuffers(1, &pointsVBO);
    glBindVertexArray(pointsVAO);
    glBindBuffer(GL_ARRAY_BUFFER, pointsVBO);
    glBufferData(GL_ARRAY_BUFFER,  pointCloudDataManager.getMaxCacheSize() * sizeof(PointCloudVertex), nullptr,  GL_DYNAMIC_DRAW);
    shaderProgramPoints.bind();
    GLint pos3 = shaderProgramPoints.attributeLocation("aPos");
    glVertexAttribPointer(pos3, 3, GL_FLOAT, GL_FALSE, sizeof(PointCloudVertex),  (void *)0);
    glEnableVertexAttribArray(pos3);
    GLint gPointColor1 = shaderProgramPoints.attributeLocation("aColor");
    glVertexAttribPointer(gPointColor1, 3, GL_FLOAT, GL_FALSE, sizeof(PointCloudVertex),   (void *)offsetof(PointCloudVertex, red));
    glEnableVertexAttribArray(gPointColor1);

    //---位置点---
    glGenVertexArrays(1, &positionVAO);
    glGenBuffers(1, &positionVBO);
    glBindVertexArray(positionVAO);
    glBindBuffer(GL_ARRAY_BUFFER, positionVBO);
    glBufferData(GL_ARRAY_BUFFER,  pointCloudDataManager.getMaxCacheSize() * sizeof(PointCloudVertex), nullptr,  GL_DYNAMIC_DRAW);
    shaderProgramPosition.bind();
    GLint pos4 = shaderProgramPosition.attributeLocation("aPos");
    glVertexAttribPointer(pos4, 3, GL_FLOAT, GL_FALSE, sizeof(PointCloudVertex),  (void *)0);
    glEnableVertexAttribArray(pos4);
    GLint gPointColor2 = shaderProgramPoints.attributeLocation("aColor");
    glVertexAttribPointer(gPointColor2, 3, GL_FLOAT, GL_FALSE, sizeof(PointCloudVertex),   (void *)offsetof(PointCloudVertex, red));
    glEnableVertexAttribArray(gPointColor2);

    //释放
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void PointCloudWidget::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);

}

void PointCloudWidget::paintGL()
{
    glClearColor(RGBNormalized(73), RGBNormalized(80), RGBNormalized(87), 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    //变换矩阵设置
    view.setToIdentity();
    view.lookAt(camera.getBasePos(), camera.getBasePos() + camera.getBaseVector(), camera.getCameraUp());
    projection.setToIdentity();
    projection.perspective(camera.getFov(), (float)width() / height(), minRenderDistance, maxRenderDistance);

    //渲染坐标轴
    if (enableAxis)
    {
        shaderProgramAxis.bind();
        shaderProgramAxis.setUniformValue("model", model);
        shaderProgramAxis.setUniformValue("view", view);
        shaderProgramAxis.setUniformValue("projection", projection);
        glLineWidth(axisWidth);
        glBindVertexArray(axisVAO);
        glDrawArrays(GL_LINES, 0, 6);
    }
    //渲染网格
    if (enableMesh)
    {
        shaderProgramMesh.bind();
        shaderProgramMesh.setUniformValue("model", model);
        shaderProgramMesh.setUniformValue("view", view);
        shaderProgramMesh.setUniformValue("projection", projection);
        glLineWidth(meshWidth);
        glBindVertexArray(meshVAO);
        glDrawArrays(GL_LINES, 0, meshLinesCount * 2);
    }
    //渲染点云
    if (enablePoints)
    {
        glBindVertexArray(pointsVAO);
        glBindBuffer(GL_ARRAY_BUFFER, pointsVBO);
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("1"),  pointCloudDataManager->getPointNeedDrawNumber());
        // RCLCPP_INFO_STREAM( rclcpp::get_logger("lidarNodeSubscriber"),
        //                     pointCloudDataManager -> getCurrentCacheSize() * 2  << " ," << needPaintNumber);
        unsigned long needPaintNumber = pointCloudDataManager.getPointNeedDrawNumber();
        unsigned long offset =  pointCloudDataManager.getCurrentCacheSize() - needPaintNumber;
        glBufferSubData(GL_ARRAY_BUFFER, offset * sizeof(PointCloudVertex),
                        needPaintNumber * sizeof(PointCloudVertex),
                        &pointCloudDataManager.getData()[offset]);
        shaderProgramPoints.bind();
        shaderProgramPoints.setUniformValue("model", model);
        shaderProgramPoints.setUniformValue("view", view);
        shaderProgramPoints.setUniformValue("projection", projection);
        glPointSize(pointSize);
        glDrawArrays(GL_POINTS, 0, pointCloudDataManager.getCurrentCacheSize());
    }

    if (enablePositionPoints)
    {
        glBindVertexArray(positionVAO);
        glBindBuffer(GL_ARRAY_BUFFER, positionVBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, pointCloudDataManager.getPositionNeedDrawNumber() * sizeof(PointCloudVertex), &pointCloudDataManager.getPositionData()[0]);
        shaderProgramPosition.bind();
        shaderProgramPoints.setUniformValue("model", model);
        shaderProgramPoints.setUniformValue("view", view);
        shaderProgramPoints.setUniformValue("projection", projection);
        glPointSize(positionPointSize);
        glDrawArrays(GL_POINTS, 0, pointCloudDataManager.getCurrentPositionCacheSize());
    }

    //同步更新信息
    emit infoWidgetUpdateSignal(camera, pointCloudDataManager);
}

void PointCloudWidget::focusOutEvent(QFocusEvent *event)
{
    Q_UNUSED(event);
    //失去焦点，相机控制器按键清空
    camera.clearKeys();
    car.clearKeys();
}

void PointCloudWidget::onTimeout()
{
    makeCurrent();
    doneCurrent();
    update();
}


void PointCloudWidget::resetViewSlot()
{
    camera.resetView();
}

void PointCloudWidget::showAxisSlot(bool val)
{
    enableAxis  = val;
    update();
}

void PointCloudWidget::showMeshSlot(bool val)
{
    enableMesh = val;
    update();
}

void PointCloudWidget::showPositionPointSlot(bool val)
{
    enablePositionPoints = val;
    update();
}

void PointCloudWidget::drawMeshSlot(int rowBegin, int  rows, int  columnBegin, int  columns)
{
    meshData.clear();
    meshLinesCount = rows  + columns + 2;
    //生成横线
    for (int i = 0; i <= rows; i ++)
    {
        meshData.push_back((rowBegin + i)* meshLength);
        meshData.push_back(columnBegin * meshLength);
        meshData.push_back(0);
        meshData.push_back((rowBegin + i)* meshLength);
        meshData.push_back((columnBegin + columns) * meshLength);
        meshData.push_back(0);
    }
    //生成竖线
    for (int i = 0; i <= columns; i ++)
    {
        meshData.push_back(rowBegin * meshLength);
        meshData.push_back((columnBegin + i)* meshLength);
        meshData.push_back(0);
        meshData.push_back((rowBegin + rows) * meshLength);
        meshData.push_back((columnBegin + i)* meshLength);
        meshData.push_back(0);
    }
    //    qDebug() << meshData;
}

void PointCloudWidget::clearPointCloudSlot()
{
    pointCloudDataManager.clearPointCloud();
    update();
}

void PointCloudWidget::clearPositionPointSlot()
{
    pointCloudDataManager.clearPositionPoint();
    update();
}

void PointCloudWidget::syncIMURollSlot()
{
    pointCloudDataManager.syncIMURoll();
}

void PointCloudWidget::setMinRenderDistanceSlot(float val)
{
    minRenderDistance = val;
    update();
}

void PointCloudWidget::setMaxRenderDistanceSlot(float val)
{
    maxRenderDistance = val;
    update();
}

void PointCloudWidget::setPointSizeSlot(float val)
{
    pointSize = val;
    update();
}

void PointCloudWidget::setPositionPointSizeSlot(float val)
{
    positionPointSize = val;
    update();
}


void PointCloudWidget::recvPointsDataSlot(message::msg::LidarData::SharedPtr msg)
{
    pointCloudDataManager.addPoint(msg);
}

void PointCloudWidget::recvServoDataSlot(message::msg::CarServoData::SharedPtr msg)
{
    pointCloudDataManager.addServoData(msg);
}

void PointCloudWidget::recvLidarImuDataSlot(message::msg::ImuData::SharedPtr msg)
{
    pointCloudDataManager.addLidarImuData(msg);
}

void PointCloudWidget::recvEncoderDataSlot(message::msg::CarEncoderData::SharedPtr msg)
{
    pointCloudDataManager.addEncoderData(msg);
}

void PointCloudWidget::recvCarImuDataSlot(message::msg::ImuData::SharedPtr msg)
{
    pointCloudDataManager.addCarImuData(msg);
}

