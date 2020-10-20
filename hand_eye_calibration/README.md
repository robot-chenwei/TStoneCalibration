# hand_eye_calibration
机器人和相机的手眼机标定程序

基于Qt和OpenCV开发，标定算法来源"A New Technique for Fully Autonomous and Efficient 3D Robotics Hand-Eye Calibration, Tsai, R.Y. and Lenz, R.K"

## 功能列表：
* 支持手眼在眼标定（Eye-in-Hand）
* 支持手眼到眼标定（Eye-to-Hand）

## 文件结构
|文件名/文件夹名|功能|
|:--|:--|
|HandEyeCalibration.cpp|程序主界面和主要逻辑的实现|
|TSAIleastSquareCalibration.cpp|标定算法的实现|
|res|图标资源文件夹|

## 软件运行截图：

![screenshot](screenshot.jpg)

## 说明
* 暂时只支持导入标定，需要导入图片和机器人位姿文件
* 先导入照片，然后选择与列表中对应编号的机器人位姿文件（有些图片会检测角点失败，不在列表中）

## 使用
* 方法1 （暂时只支持此方法）
++ 1.将标定板固定在机器人的末端
++ 2.新建一个data文件夹，在文件夹中运行\data_collection_tool\read_calibration_data.py,  usb相机驱动使用可作为参数传入（默认为1），在命令行输入不同字符存储图片和机械臂位姿（输入'q'退出，每次输入不同字母存储一组数据）
++ 3.利用calibration软件加载图片和位姿文件进行标定，导出数据即可
