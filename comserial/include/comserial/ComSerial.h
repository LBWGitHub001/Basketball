#ifndef YOLOV5_COMSERIAL_H
#define YOLOV5_COMSERIAL_H

#define Focal_Length 17
#define Sensor_Coefficient 0.09 //相机参数，这个数通过图片大小推算相机的像素密度
#define Basketboll_Width 100 //m
#define Direction_Cofficient 2



#include <iostream>
#include <string>
#include <serial/serial.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <vector>
#include <memory>
#include <cmath>
#include <map>
#include <deque>
#include <ctime>

#include <types.h>
#include <postprocess.h>
/*
typedef struct FixFrame{
    std::deque<int> rev;
    std::deque<int> dist;
    std::deque<int> id;
    std::deque<double> duration;
    int count;
}Frame;
*/

class ComSerial {
public:
    ComSerial() = delete;
    ~ComSerial();
    ComSerial(ComSerial&) = delete;
    ComSerial(std::string com_path,int baudRate,int outTime);

    bool sendPort(cv::Mat img, std::vector<Detection>& res);   //传入接口
private:
    std::string com_path;
    int baudRate;
    int outTime;
    serial::Serial sp;//串口对象
    std::map<int,std::string> id_obj = {
            {0,"Volleyball"},{1,"Basketball"},{3,"colunm"}
    };
    int idgot;//串口收到的id
    int id;         //模型处理的id，两者不一样所以需要进行一次转换
    //Frame frame;

    //滤波器
    int pre_x;
    int pre_id;
    const double wr = 0.5;
    //计时器
    clock_t start;
    clock_t stop;

    void calDistance(Property& pro);
    void calDirection(Property& pro);
    void readRes(cv::Mat img, Detection& res,Property& pro);
    void sendUart(Property& pro);
    void readUart();

};


#endif //YOLOV5_COMSERIAL_H
