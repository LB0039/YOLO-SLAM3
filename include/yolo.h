#ifndef YOLOV5DETECTED_H
#define YOLOV5DETECTED_H

#include <opencv2/opencv.hpp>
#include <torch/script.h>
#include <algorithm>
#include <iostream>
#include <time.h>
#include <thread>
#include <unistd.h>
#include <mutex>

#include "Tracking.h"

namespace ORB_SLAM2
{

class Tracking;

class YOLOv5Detector
{
public:
    cv::Mat mImage;
    vector<string> mvDynamicNames;
    vector<cv::Rect2i> mvDynamicArea;
    ///vector<cv::Rect2i> mvPersonArea = {};
    map<string, vector<cv::Rect2i>> mmDetectMap;
    Tracking *mpTracker;

public:
    YOLOv5Detector();

    void Run();

    std::vector<torch::Tensor> non_max_suppression(torch::Tensor preds, float score_thresh=0.5, float iou_thresh=0.5);

    bool isNewimgArrived();

    bool CheckFinshed();

    void RequestFinshed();

    void ProductYolo();

    void SetFinish();

    void RequestFinish();

    bool isFinished();

    void SetTracking(Tracking *pTracker);

public:
    std::vector<std::string> classnames;

    torch::jit::script::Module module;

    bool mbFinshedRequested;
    bool mbImagFlag;
    bool mbFinished;

    std::mutex mMutexImgYolo;
    std::mutex mMutexGetNewImg;
    std::mutex mMutexFinish;

};


}
#endif
