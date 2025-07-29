/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 5)  // 检查  命令行  参数  的数量是否正确   argc: 整数,用来统计你运行程序时送给main函数的  命令行参数  的个数
    {
        // 如果参数数量不正确，输出错误信息并返回1表示程序异常退出
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association" << endl;
        return 1;
    }

    // Retrieve paths to images  检索图像路径
    vector<string> vstrImageFilenamesRGB;  // 创建一个字符串向量，用于存储RGB图像的文件名
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    // 从命令行参数中获取关联文件名，并将其存储在字符串变量strAssociationFilename中
    string strAssociationFilename = string(argv[4]);
    // 调用LoadImages函数，加载关联文件中的图像文件名、深度图像文件名和时间戳
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps  检查图像数量和深度图的一致性
    int nImages = vstrImageFilenamesRGB.size();  //获取图像数量
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // 创建 SLAM 系统，它初始化所有系统线程并准备处理帧
    ORB_SLAM3::System SLAM(// 创建一个ORB_SLAM3::System对象，用于实时定位和建图（SLAM）
    argv[1],  // argv[1] - 配置文件路径
    argv[2],  // argv[2] - 数据集路径
    ORB_SLAM3::System::RGBD,  // ORB_SLAM3::System::RGBD - 表示使用RGB-D相机输入
    true);  // ：true - 启用可视化功能
    float imageScale = SLAM.GetImageScale();  // 获取图像的缩放比例

    // Vector for tracking time statistics  用于跟踪时间统计的向量
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages); // 调整 vTimesTrack 的大小为 nImages，以便存储每个图像的时间戳信息

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop  
    cv::Mat imRGB, imD;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

        if(imageScale != 1.f)
        {
            int width = imRGB.cols * imageScale;
            int height = imRGB.rows * imageScale;
            cv::resize(imRGB, imRGB, cv::Size(width, height));
            cv::resize(imD, imD, cv::Size(width, height));
        }

#ifdef COMPILEDWITHC14
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system  将图像传递给 SLAM 系统
        SLAM.TrackRGBD(imRGB,imD,tframe);

#ifdef COMPILEDWITHC14
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)  //ni图像数量     nImages图像数
            T = vTimestamps[ni+1]-tframe;  //第ni帧图像的时间戳tframe  如果满足条件，计算当前图像与下一图像的时间戳差值，并赋值给T
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];  // 如果不满足上述条件且ni大于0，计算当前图像与上一图像的时间戳差值，并赋值给T

        if(ttrack<T)  // 判断ttrack是否小于T
            usleep((T-ttrack)*10);    // 如果满足条件，暂停执行一段时间，使得ttrack接近于T
    }

    // Stop all threads
    SLAM.Shutdown();  // 调用SLAM对象的Shutdown方法，用于关闭SLAM系统

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");   

    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)  // 定义一个函数LoadImages，用于从关联文件中加载图像文件名和时间戳
{
    ifstream fAssociation;  // 打开关联文件   ifstream 打开文件
    fAssociation.open(strAssociationFilename.c_str());  
    while(!fAssociation.eof())  // 逐行读取关联文件内容  .eof()用于检查文件是否结束
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;  // 将字符串s放入字符串流ss中
            double t;   
            string sRGB, sD;
            ss >> t;  // 从字符串流中读取时间戳t
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}
