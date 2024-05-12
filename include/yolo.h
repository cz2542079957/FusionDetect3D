#ifndef YOLO_H
#define YOLO_H
#include <fstream>
#include <sstream>
#include <iostream>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
using namespace std;
using namespace cv;
using namespace dnn;

struct Net_config
{
    float confThreshold; // Confidence threshold
    float nmsThreshold;  // Non-maximum suppression threshold
    int inpWidth;        // Width of network's input image
    int inpHeight;       // Height of network's input image
    string classesFile;
    string modelConfiguration;
    string modelWeights;
    string netname;
};

class YOLO
{
public:
    YOLO(vector<string> &classes,  vector<uchar> &bufferCfg, vector<uchar> &bufferModel);
    void detect(Mat &frame);

private:
    float confThreshold;
    float nmsThreshold;
    int inpWidth;
    int inpHeight;
    char netname[20];
    vector<std::string> classes;
    Net net;
    void postprocess(Mat &frame, const  vector<Mat> &outs);
    void drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat &frame);
};


#endif // YOLO_H
