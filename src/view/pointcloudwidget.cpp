#include "pointcloudwidget.h"


PointCloudWidget::PointCloudWidget(QWidget *parent): QOpenGLWidget(parent)
{
    this->setGeometry(parent->rect().x(), parent->rect().y(), parent->rect().width(), parent->rect().height());
    setFocusPolicy(Qt::ClickFocus);
    //    timer.start(1);
    connect(&timer, SIGNAL(timeout()), this, SLOT(onTimeout()));
    connect(&camera, SIGNAL(updateGraph()), this, SLOT(update()));
    //    connect(&camera, SIGNAL())
    //    qDebug() << camera.getBasePos();
    //    qDebug() << camera.getBaseDirection();
    //    qDebug() << camera.getBaseVector();
    //    qDebug() << camera.getBaseUp();
    //    qDebug() << camera.getCameraRight();
    //    qDebug() << camera.getCameraUp();
}

PointCloudWidget::~PointCloudWidget()
{
    if (!isValid())
    {
        return;
    }
    makeCurrent();
    glDeleteBuffers(1, &axisVAO);
    glDeleteVertexArrays(1, &axisVBO);
    glDeleteBuffers(1, &meshVAO);
    glDeleteVertexArrays(1, &meshVBO);
    doneCurrent();
}

void PointCloudWidget::keyPressEvent(QKeyEvent *event)
{
    camera.keypressActionHandler(event);
}

void PointCloudWidget::keyReleaseEvent(QKeyEvent *event)
{
    camera.keyreleaseActionHandler(event);
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

void PointCloudWidget::sendPoints(QVector<float> data)
{

}

void PointCloudWidget::drawPointsHandle()
{
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
        qDebug() << "ERR:" << shaderProgramMesh.log();
    }

    //坐标轴数据
    glGenVertexArrays(1, &axisVAO);
    glGenBuffers(1, &axisVBO);
    glBindVertexArray(axisVAO);
    glBindBuffer(GL_ARRAY_BUFFER, axisVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(axisData), axisData, GL_STATIC_DRAW);
    shaderProgramAxis.bind();
    GLint pos1 = shaderProgramAxis.attributeLocation("aPos");
    glVertexAttribPointer(pos1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(pos1);
    GLint color2 = shaderProgramAxis.attributeLocation("aColor");
    glVertexAttribPointer(color2, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(color2);

    //网格数据
    drawMesh(-5, 10, -5, 10);
    glGenVertexArrays(1, &meshVAO);
    glGenBuffers(1, &meshVBO);
    glBindVertexArray(meshVAO);
    glBindBuffer(GL_ARRAY_BUFFER, meshVBO);
    glBufferData(GL_ARRAY_BUFFER, meshData.size() * sizeof(float), &meshData[0],  GL_DYNAMIC_DRAW);
    shaderProgramMesh.bind();
    GLint pos2 = shaderProgramMesh.attributeLocation("aPos");
    glVertexAttribPointer(pos2, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)(0 * sizeof(float)));
    glEnableVertexAttribArray(pos2);

    //点云
    glGenVertexArrays(1, &pointsVAO);
    glGenBuffers(1, &pointsVBO);
    glBindVertexArray(pointsVAO);
    glBindBuffer(GL_ARRAY_BUFFER, pointsVBO);
    glBufferData(GL_ARRAY_BUFFER, pointsData.size() * sizeof(float), &pointsData[0],  GL_DYNAMIC_DRAW);
    shaderProgramPoints.bind();
    GLint pos3 = shaderProgramMesh.attributeLocation("aPos");
    glVertexAttribPointer(pos3, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)(0 * sizeof(float)));
    glEnableVertexAttribArray(pos3);

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
    glClearColor(0.3255, 0.3569, 0.3882, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    //变换矩阵设置
    view.setToIdentity();
    view.lookAt(camera.getBasePos(), camera.getBasePos() + camera.getBaseVector(), camera.getCameraUp());
    projection.setToIdentity();
    projection.perspective(camera.getFov(), (float)width() / height(), renderMinDistance, renderMaxDistance);

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
        RCLCPP_INFO_STREAM( rclcpp::get_logger("lidarNodeSubscriber"), pointsData.size());
        glGenVertexArrays(1, &pointsVAO);
        glGenBuffers(1, &pointsVBO);
        glBindVertexArray(pointsVAO);
        glBindBuffer(GL_ARRAY_BUFFER, pointsVBO);
        glBufferData(GL_ARRAY_BUFFER, pointsData.size() * sizeof(float), &pointsData[0],  GL_DYNAMIC_DRAW);
        shaderProgramPoints.bind();
        shaderProgramPoints.setUniformValue("model", model);
        shaderProgramPoints.setUniformValue("view", view);
        shaderProgramPoints.setUniformValue("projection", projection);
        GLint pos3 = shaderProgramMesh.attributeLocation("aPos");
        glVertexAttribPointer(pos3, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)(0 * sizeof(float)));
        glEnableVertexAttribArray(pos3);
        glBindVertexArray(pointsVAO);
        glDrawArrays(GL_POINTS, 0, pointsData.size() / 3);
    }

}

void PointCloudWidget::onTimeout()
{
    makeCurrent();
    doneCurrent();
    update();
}

void PointCloudWidget::showAxis(bool val)
{
    enableAxis  = val;
    update();
}

void PointCloudWidget::showMesh(bool val)
{
    enableMesh = val;
    update();
}

void PointCloudWidget::drawMesh(int rowBegin, int  rows, int  columnBegin, int  columns)
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

void PointCloudWidget::recvPointsData(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    pointsData.clear();
    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
        //将角度距离数据解析为空间坐标
        float x, y, z;
        float rad = i * M_PI / 180.0f  ;
        z = 0;
        x = std::cos(rad) * msg->ranges[i];
        y = std::sin(rad) * msg->ranges[i];
        pointsData.push_back(x);
        pointsData.push_back(y);
        pointsData.push_back(z);
        // RCLCPP_INFO_STREAM( rclcpp::get_logger("lidarNodeSubscriber"),
        //                     "Index: " << i << ", Range: " << msg->ranges[i] << ", Intensity: " << msg->intensities[i] << ",");
        // RCLCPP_INFO( rclcpp::get_logger("lidarNodeSubscriber"),
        //              "positon: %8f, %8f, %8f;", x, y, z);

    }
    update();
}


