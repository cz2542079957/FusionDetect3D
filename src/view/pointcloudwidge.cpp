#include "pointcloudwidge.h"


pointCloudWidge::pointCloudWidge(QWidget *parent): QOpenGLWidget(parent)
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


pointCloudWidge::~pointCloudWidge()
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

void pointCloudWidge::keyPressEvent(QKeyEvent *event)
{
    camera.keypressActionHandler(event);
}

void pointCloudWidge::keyReleaseEvent(QKeyEvent *event)
{
    camera.keyreleaseActionHandler(event);
}

void pointCloudWidge::mousePressEvent(QMouseEvent *event)
{
    camera.mousepressActionHandler(event);
}

void pointCloudWidge::mouseMoveEvent(QMouseEvent *event)
{
    camera.mousemoveActionHandler(event);
}

void pointCloudWidge::mouseReleaseEvent(QMouseEvent *event)
{
    camera.mousereleaseActionHandler(event);
}

void pointCloudWidge::wheelEvent(QWheelEvent *event)
{
    camera.wheelActionHandler(event);
}

void pointCloudWidge::sendPoints(QVector<float> data)
{
}

void pointCloudWidge::drawPointsHandle()
{
}

void pointCloudWidge::initializeGL()
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

    //坐标轴数据
    glGenVertexArrays(1, &axisVAO);
    glGenBuffers(1, &axisVBO);
    glBindVertexArray(axisVAO);
    glBindBuffer(GL_ARRAY_BUFFER, axisVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(axisData), axisData, GL_STATIC_DRAW);
    shaderProgramAxis.bind();
    GLint pos2 = shaderProgramAxis.attributeLocation("aPos");
    glVertexAttribPointer(pos2, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(pos2);
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
    GLint pos3 = shaderProgramMesh.attributeLocation("aPos");
    glVertexAttribPointer(pos3, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)(0 * sizeof(float)));
    glEnableVertexAttribArray(pos3);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);


}

void pointCloudWidge::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
}

void pointCloudWidge::paintGL()
{
    glClearColor(0.4255, 0.4569, 0.4882, 1.0f);
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

}

void pointCloudWidge::onTimeout()
{
    makeCurrent();
    doneCurrent();
    update();
}

void pointCloudWidge::showAxis(bool val)
{
    enableAxis  = val;
    update();
}

void pointCloudWidge::showMesh(bool val)
{
    enableMesh = val;
    update();
}

void pointCloudWidge::drawMesh(int rowBegin, int  rows, int  columnBegin, int  columns)
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


