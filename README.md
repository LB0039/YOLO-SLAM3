# YOLO_ORB_SLAM3

**This is an improved version of [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) that adds an object detection module implemented with [YOLOv5](https://github.com/ultralytics/yolov5) to achieve SLAM in dynamic environments.**
- Object Detection
- Dynamic SLAM


The provided `requirements.txt` corresponds to the Python environment
used for training and exporting the YOLOv5 detector.
The SLAM system itself is implemented in C++ and uses the same C++ dependencies
as ORB-SLAM3 (OpenCV, Pangolin, Eigen, libtorch, etc.).

pip install -r yolov5_requirements.txt  # for YOLOv5 training/export


<p align="center">
  <img src="Fig.png"/>
  <br>
  <em>Fig 1 : Test with TUM dataset</em>
</p>

## Getting Started
### 0. Prerequisites

We have tested on:

>
> OS = Ubuntu 20.04
> 
> OpenCV = 4.2
> 
> [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page) = 3.3.9
>
> [Pangolin](https://github.com/stevenlovegrove/Pangolin) = 0.5
>
> [ROS](http://wiki.ros.org/ROS/Installation) = Noetic


### 1. Install libtorch

#### Recommended way
You can download the compatible version of libtorch from [Baidu Netdisk](https://pan.baidu.com/s/1DQGM3rt3KTPWtpRK0lu8Fg?pwd=8y4k) 
code: 8y4k,  then
```bash
unzip libtorch.zip
mv libtorch/ PATH/YOLO_ORB_SLAM3/Thirdparty/
```
#### Or you can

```bash
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.11.0%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-1.11.0%2Bcpu.zip
mv libtorch/ PATH/YOLO_ORB_SLAM3/Thirdparty/
```

### 2. Build
```bash
cd YOLO_ORB_SLAM3
chmod +x build.sh
./build.sh
```

Only the rgbd_tum target will be build.

### 3. Build ROS Examples
Add the path including *Examples/ROS/YOLO_ORB_SLAM3* to the ROS_PACKAGE_PATH environment variable. Open .bashrc file:
```bash
gedit ~/.bashrc
```
and add at the end the following line. Replace PATH by the folder where you cloned YOLO_ORB_SLAM3:
```bash
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/YOLO_ORB_SLAM3/Examples/ROS
```
Then build
```bash
chmod +x build_ros.sh
./build_ros.sh
```

Only the RGBD target has been improved.

The frequency of camera topic must be lower than 15 Hz.

You can run this command to change the frequency of topic which published by the camera driver. 
```bash
roslaunch YOLO_ORB_SLAM3 camera_topic_remap.launch
```

############################################################################################################################################################

####KITTI数据集运行单目


./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt ./Examples/Monocular/KITTI00-02.yaml /home/z/LIU/data/KITTI/00

./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt ./Examples/Monocular/KITTI00-02.yaml /home/z/LIU/data/KITTI/01

./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt ./Examples/Monocular/KITTI04-12.yaml /home/z/LIU/data/KITTI/05

./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt ./Examples/Monocular/KITTI04-12.yaml /home/z/LIU/data/KITTI/05



./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt ./Examples/Monocular/KITTI04-12.yaml /home/z/LIU/data/KITTI/07

./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt ./Examples/Monocular/KITTI04-12.yaml /home/z/LIU/data/KITTI/10

####KITTI数据集运行双目

./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt ./Examples/Stereo/KITTI04-12.yaml /home/z/LIU/data/KITTI/05

./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt ./Examples/Stereo/KITTI04-12.yaml /home/z/LIU/data/KITTI/07

./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt ./Examples/Stereo/KITTI04-12.yaml /home/z/LIU/data/KITTI/10
###########################################################################################################################################################

#### TUM Dataset
##生成associate.txt
python associate.py rgb.txt depth.txt > associate.txt

#RGB-D
```bash
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM3.yaml /home/z/LIU/data/RGB-D/rgbd_dataset_freiburg3_walking_xyz /home/z/LIU/data/RGB-D/rgbd_dataset_freiburg3_walking_xyz/associate.txt
```

./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM3.yaml /home/z/LIU/data/RGB-D/rgbd_dataset_freiburg3_walking_halfsphere /home/z/LIU/data/RGB-D/rgbd_dataset_freiburg3_walking_halfsphere/associate.txt



./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM3.yaml /home/z/LIU/data/RGB-D/rgbd_dataset_freiburg3_walking_rpy /home/z/LIU/data/RGB-D/rgbd_dataset_freiburg3_walking_rpy/associate.txt


./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM3.yaml /home/z/LIU/data/RGB-D/rgbd_dataset_freiburg3_walking_static /home/z/LIU/data/RGB-D/rgbd_dataset_freiburg3_walking_static/associate.txt


./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM3.yaml /home/z/LIU/data/RGB-D/rgbd_dataset_freiburg3_sitting_static /home/z/LIU/data/RGB-D/rgbd_dataset_freiburg3_sitting_static/associate.txt
###########################################################################################################################################################
# ATE RPE 产生轨迹误差图（红色标出）
#单目
python2 evaluate_ate.py --save alignedTrajectory_ate.txt --plot ate.png groundtruth.txt KeyFrameTrajectory.txt

python2 evaluate_rpe.py --fixed_delta --delta_unit s --save alignedTrajectory_rpe.txt --plot rpe.png groundtruth.txt KeyFrameTrajectory.txt

python2 evaluate_rpe.py --fixed_delta --delta_unit m --save alignedTrajectory_rpe.txt --plot rpe.png groundtruth.txt KeyFrameTrajectory.txt


#RGB-D

python2 evaluate_ate.py --save alignedTrajectory_ate.txt --plot ate.png groundtruth.txt CameraTrajectory.txt

evo_ape tum name1.tum name2.tum -va --plot

python2 evaluate_rpe.py --fixed_delta --delta_unit m --save alignedTrajectory_rpe.txt --plot rpe.png groundtruth.txt CameraTrajectory.txt

###########################################################################################################################################################
#Bonn数据集运行

./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/Bonn.yaml /home/z/LIU/data/Bonn/rgbd_bonn_balloon /home/z/LIU/data/Bonn/rgbd_bonn_balloon/associations.txt

./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/Bonn.yaml /home/z/LIU/data/Bonn/rgbd_bonn_crowd /home/z/LIU/data/Bonn/rgbd_bonn_crowd/associate.txt

./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/Bonn.yaml /home/z/LIU/data/Bonn/rgbd_bonn_crowd2 /home/z/LIU/data/Bonn/rgbd_bonn_crowd2/associate.txt

./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/Bonn.yaml /home/z/LIU/data/Bonn/rgbd_bonn_crowd3 /home/z/LIU/data/Bonn/rgbd_bonn_crowd3/associate.txt

##./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/Bonn.yaml /home/z/LIU/data/Bonn/rgbd_bonn_kidnapping_box2 /home/z/LIU/data/Bonn/rgbd_bonn_kidnapping_box2/associate.txt

./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/Bonn.yaml /home/z/LIU/data/Bonn/rgbd_bonn_moving_nonobstructing_box /home/z/LIU/data/Bonn/rgbd_bonn_moving_nonobstructing_box/associate.txt

./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/Bonn.yaml /home/z/LIU/data/Bonn/rgbd_bonn_moving_nonobstructing_box2 /home/z/LIU/data/Bonn/rgbd_bonn_moving_nonobstructing_box2/associate.txt

##./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/Bonn.yaml /home/z/LIU/data/Bonn/rgbd_bonn_moving_obstructing_box2 /home/z/LIU/data/Bonn/rgbd_bonn_moving_obstructing_box2/associate.txt

./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/Bonn.yaml /home/z/LIU/data/Bonn/rgbd_bonn_person_tracking /home/z/LIU/data/Bonn/rgbd_bonn_person_tracking/associate.txt

./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/Bonn.yaml /home/z/LIU/data/Bonn/rgbd_bonn_person_tracking2 /home/z/LIU/data/Bonn/rgbd_bonn_person_tracking2/associate.txt

./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/Bonn.yaml /home/z/LIU/data/Bonn/rgbd_bonn_synchronous /home/z/LIU/data/Bonn/rgbd_bonn_synchronous/associate.txt

./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/Bonn.yaml /home/z/LIU/data/Bonn/rgbd_bonn_synchronous2 /home/z/LIU/data/Bonn/rgbd_bonn_synchronous2/associate.txt



#纯单目

cd Examples
./Monocular/mono_tum_vi Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml ${dir}/dataset-room1_512_16/mav0/cam0/data Monocular/TUM_TimeStamps/dataset-room1_512.txt dataset-room1_512_mono

#### ROS

```bash
roslaunch YOLO_ORB_SLAM3 camera_topic_remap.launch
rosrun YOLO_ORB_SLAM3 RGBD PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
```


#Evo评估 TUM 数据集    （轨迹绘制）
##绘制单个轨迹，groundtruth.txt是数据集的真实轨迹，-p表示绘制图像

evo_traj tum groundtruth.txt -p -p --plot_mode xyz

evo_traj tum CameraTrajectory.txt -p -p --plot_mode xyz

#绘制多个轨迹：真实轨迹和刚刚运行得到的相机轨迹

evo_traj tum groundtruth.txt CameraTrajectory.txt -p --plot_mode xz 

evo_ape tum groundtruth.txt CameraTrajectory.txt -p --plot_mode xyz --plot_trajectory_connectors --plot_error_type rmse --plot_error_color red

#上面的轨迹在旋转和平移上不对齐，可以通过--ref参数指定参考轨迹，并且添加参数-a来对齐轨迹

evo_traj tum --ref=groundtruth.txt CameraTrajectory.txt -p -a --plot_mode xyz 


#如果是单目程序，还需要添加一个参数-s进行尺度上面的对齐

evo_traj tum --ref=groundtruth.txt CameraTrajectory.txt -p -a -s

#或者直接使用参数-as同时使用旋转和平移以及尺度上面的对齐

evo_traj tum --ref=groundtruth.txt CameraTrajectory.txt -p -as


#通过观察，上面的轨迹在x和y轴上面变化不大，可以添加参数--plot_mode=xz将轨迹压缩在xz平面上

evo_traj tum --ref=groundtruth.txt CameraTrajectory.txt -p -as --plot_mode=xz


evo_ape tum groundtruth.txt CameraTrajectory.txt -va --plot --plot_mode xyz --save_results ./

evo_rpe kitti groundtruth.txt CameraTrajectory.txt -r full -va --plot --plot_mode xyz --save_plot ./tra1plot --save_results ./tra1.zip



#EVO使用方法   （数据）
https://blog.csdn.net/weixin_44034287/article/details/119618386

evo中把ATE称为APE    evo中默认计算平移部分，需要通过输入-r full来改为计算平移+旋转

#将ATE结果直接输出为压缩文件，进行曲线绘制
evo_ape tum groundtruth.txt CameraTrajectory.txt -va --plot --plot_mode xyz --save_results ATEresules.zip


#2.绝对轨迹误差
evo_ape tum groundtruth.txt CameraTrajectory.txt -p -as 

#max：表示最大误差；mean：平均误差；median：误差中位数；min：最小误差；rmse：均方根误差；sse：和方差/误差平方和；std：标准差

#3.相对位姿误差
#和绝对轨迹误差参数相同，使用相机轨迹的原因在于它包含了关键帧轨迹，反映的是全局一致的地图

#平移漂移
evo_rpe tum groundtruth.txt CameraTrajectory.txt -p -as


#旋转漂移
evo_rpe tum -r rot_part groundtruth.txt CameraTrajectory.txt -p -as




############################################################################################################################################################

# ATE RPE 产生轨迹误差图（红色标出）
#单目
python2 evaluate_ate.py --save alignedTrajectory_ate.txt --plot ate.png groundtruth.txt KeyFrameTrajectory.txt

python2 evaluate_rpe.py --fixed_delta --delta_unit s --save alignedTrajectory_rpe.txt --plot rpe.png groundtruth.txt KeyFrameTrajectory.txt

python2 evaluate_rpe.py --fixed_delta --delta_unit m --save alignedTrajectory_rpe.txt --plot rpe.png groundtruth.txt KeyFrameTrajectory.txt


#RGB-D

python2 evaluate_ate.py --save alignedTrajectory_ate.txt --plot ate.png groundtruth.txt CameraTrajectory.txt

python2 evaluate_rpe.py --fixed_delta --delta_unit m --save alignedTrajectory_rpe.txt --plot rpe.png groundtruth.txt CameraTrajectory.txt



