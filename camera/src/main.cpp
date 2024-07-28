#include "CameraApi.h" //相机SDK的API头文件
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <cstdio>

using namespace cv;

unsigned char *g_pRgbBuffer;     //处理后数据缓存区

int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher Pub = it.advertise("Image_Handle", 1);

    int iCameraCounts = 1;
    int iStatus = -1;
    tSdkCameraDevInfo tCameraEnumList;
    int hCamera;
    tSdkCameraCapbility tCapability;      //设备描述信息
    tSdkFrameHead sFrameInfo;
    BYTE *pbyBuffer;
    int iDisplayFrames = 10000;
    Mat *iplImage = NULL;
    int channel = 3;

    CameraSdkInit(1);

    //枚举设备，并建立设备列表
    iStatus = CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
    printf("state = %d\n", iStatus);

    printf("count = %d\n", iCameraCounts);
    //没有连接设备
    if (iCameraCounts == 0) {
        return -1;
    }

    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&tCameraEnumList, -1, -1, &hCamera);

    //初始化失败
    printf("state = %d\n", iStatus);
    if (iStatus != CAMERA_STATUS_SUCCESS) {
        printf("ERR");
        return -1;
    }

    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(hCamera, &tCapability);

    //
    g_pRgbBuffer = (unsigned char *) malloc(
            tCapability.sResolutionRange.iHeightMax * tCapability.sResolutionRange.iWidthMax * 3);
    //g_readBuf = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);

    /*让SDK进入工作模式，开始接收来自相机发送的图像
    数据。如果当前相机是触发模式，则需要接收到
    触发帧以后才会更新图像。    */
    CameraPlay(hCamera);

    /*其他的相机参数设置
    例如 CameraSetExposureTime   CameraGetExposureTime  设置/读取曝光时间
         CameraSetImageResolution  CameraGetImageResolution 设置/读取分辨率
         CameraSetGamma、CameraSetConrast、CameraSetGain等设置图像伽马、对比度、RGB数字增益等等。
         本例程只是为了演示如何将SDK中获取的图像，转成OpenCV的图像格式,以便调用OpenCV的图像处理函数进行后续开发
    */
    //CameraSetWhiteLevel(hCamera,100);
    int a;
    CameraGetWhiteLevel(hCamera, &a);
    ROS_WARN_STREAM("WC" << a);

    CameraSetExposureTime(hCamera, 10000);
    double ep;
    CameraGetExposureTime(hCamera, &ep);
    ROS_WARN_STREAM(ep);
    tSdkImageResolution resolution;
    CameraGetImageResolution(hCamera, &resolution);
    ROS_INFO_STREAM("1:" << resolution.iIndex);//1索引号
    ROS_INFO_STREAM("2:" << resolution.iHOffsetFOV);//2采集视场相对于Sensor最大视场左上角的垂直偏移
    ROS_INFO_STREAM("3:" << resolution.iVOffsetFOV);//3采集视场相对于Sensor最大视场左上角的水平偏移
    ROS_INFO_STREAM("4:" << resolution.iWidthFOV);//4采集视场的宽度
    ROS_INFO_STREAM("5:" << resolution.iHeightFOV);//5采集视场的高度
    ROS_INFO_STREAM("6:" << resolution.iWidth);//6相机最终输出的图像的宽度
    ROS_INFO_STREAM("7:" << resolution.iHeight);//7相机最终输出的图像的高度
    /**
     * INT     iIndex;             // 索引号，[0,N]表示预设的分辨率(N 为预设分辨率的最大个数，一般不超过20),OXFF 表示自定义分辨率(ROI)
  char    acDescription[32];  // 该分辨率的描述信息。仅预设分辨率时该信息有效。自定义分辨率可忽略该信息
  UINT    uBinSumMode;        // BIN(求和)的模式,范围不能超过tSdkResolutionRange中uBinSumModeMask
  UINT    uBinAverageMode;    // BIN(求均值)的模式,范围不能超过tSdkResolutionRange中uBinAverageModeMask
  UINT    uSkipMode;          // 是否SKIP的尺寸，为0表示禁止SKIP模式，范围不能超过tSdkResolutionRange中uSkipModeMask
  UINT    uResampleMask;      // 硬件重采样的掩码
  INT     iHOffsetFOV;        // 采集视场相对于Sensor最大视场左上角的垂直偏移
  INT     iVOffsetFOV;        // 采集视场相对于Sensor最大视场左上角的水平偏移
  INT     iWidthFOV;          // 采集视场的宽度
  INT     iHeightFOV;         // 采集视场的高度
  INT     iWidth;             // 相机最终输出的图像的宽度
  INT     iHeight;            // 相机最终输出的图像的高度
  INT     iWidthZoomHd;       // 硬件缩放的宽度,不需要进行此操作的分辨率，此变量设置为0.
  INT     iHeightZoomHd;      // 硬件缩放的高度,不需要进行此操作的分辨率，此变量设置为0.
  INT     iWidthZoomSw;       // 软件缩放的宽度,不需要进行此操作的分辨率，此变量设置为0.
  INT     iHeightZoomSw;      // 软件缩放的高度,不需要进行此操作的分辨率，此变量设置为0.*/
    resolution.iIndex = 10;
    CameraSetImageResolution(hCamera, &resolution);
    ROS_INFO_STREAM("1:" << resolution.iIndex);//1索引号
    ROS_INFO_STREAM("2:" << resolution.iHOffsetFOV);//2采集视场相对于Sensor最大视场左上角的垂直偏移
    ROS_INFO_STREAM("3:" << resolution.iVOffsetFOV);//3采集视场相对于Sensor最大视场左上角的水平偏移
    ROS_INFO_STREAM("4:" << resolution.iWidthFOV);//4采集视场的宽度
    ROS_INFO_STREAM("5:" << resolution.iHeightFOV);//5采集视场的高度
    ROS_INFO_STREAM("6:" << resolution.iWidth);//6相机最终输出的图像的宽度
    ROS_INFO_STREAM("7:" << resolution.iHeight);//7相机最终输出的图像的高度
    if (tCapability.sIspCapacity.bMonoSensor) {
        channel = 1;
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_MONO8);
    } else {
        channel = 3;
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);
    }


    //循环显示1000帧图像
    while (ros::ok()) {
        if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS) {
            CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);

            cv::Mat matImage(
                    Size(sFrameInfo.iWidth, sFrameInfo.iHeight),
                    sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
                    g_pRgbBuffer
            );
            //imshow("Opencv Demo", matImage);
            sensor_msgs::ImagePtr msg;
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", matImage).toImageMsg();
            Pub.publish(msg);
            //ROS_INFO_STREAM("Image have sent");
            ros::spinOnce();

            waitKey(5);

            //在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
            //否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
            CameraReleaseImageBuffer(hCamera, pbyBuffer);

        }
    }

    CameraUnInit(hCamera);
    //注意，现反初始化后再free
    free(g_pRgbBuffer);

    return 0;
}

