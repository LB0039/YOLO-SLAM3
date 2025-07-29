//
// Created by yuwenlu on 2022/3/14.
//
#ifndef YOLO_DETECT_H
#define YOLO_DETECT_H

#include <opencv2/opencv.hpp>
#include <torch/script.h>
#include <algorithm>
#include <iostream>
#include <utility>
#include <thread>
#include <unistd.h>
#include <time.h>
#include <mutex>

#include "Tracking.h"
#include <torch/csrc/jit/passes/tensorexpr_fuser.h>

namespace ORB_SLAM3
{

class Tracking;

class YoloDetection
{

public:
    YoloDetection();
    ~YoloDetection();
    bool Run();
    void ClearArea();
    bool isNewimgArrived();
    bool CheckFinshed();
    void RequestFinshed();
    void ProductYolo();
    void SetFinish();
    void RequestFinish();
    bool isFinished();
    void SetTracking(Tracking *pTracker);
    vector<torch::Tensor> non_max_suppression(torch::Tensor preds, float score_thresh=0.5, float iou_thresh=0.5);

public:
    cv::Mat mImage;
    Tracking *mpTracker;
    torch::jit::script::Module Module;
    std::vector<std::string> mClassnames;

    // 结果参数
    vector<cv::Rect2i> mvPersonArea;
    vector<string> mvDynamicNames;
    vector<string> mvPersonNames;
    vector<cv::Rect2i> mvDynamicArea;
    map<string, vector<cv::Rect2i>> mmDetectMap;

public:
    std::vector<std::string> classnames;

    bool mbFinshedRequested;
    bool mbImagFlag;
    bool mbFinished;

    std::mutex mMutexImgYolo;
    std::mutex mMutexGetNewImg;
    std::mutex mMutexFinish;

};

}
#endif //YOLO_DETECT_H
