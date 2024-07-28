#include <ros/rate.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Int16.h>
#include <serial/serial.h>
#include <map>


serial::Serial sp;

std::map<int,std::string> id_obj = {
        {0,"Volleyball"},{1,"Basketball"},{3,"colunm"}
};
void SendU(int x,int distance,int isfind,int id);

/**********上一帧目标记录***********/
int pre_id;
int pre_d;
int pre_x;
int lo_dis;
int vis_mod;
const double wd=0.1;
double delta_d;
/********************************/
//判断当前模式：检测到1 OR 未检测0
void Getmode(int& mod,int isfind,int dis){
    if(dis <= 25 && isfind == 180){
        mod = 0;
        lo_dis = 20;
        delta_d = 1;
    }
    else if(mod == 0 && isfind == 180)
    {
        mod = 1;
        lo_dis = 20;
        delta_d = 1;
    }

}

void imageCallback(const std_msgs::Int16 &msg){
    //解码：
    int data = msg.data;
    int isfind = data%1000;
    data/1000;
    int distance = data%10000;
    data = data/10000;
    int x = data%1000;
    data = data/10;
    int id = data;
    Getmode(vis_mod,isfind,distance);
    if(vis_mod==1){
        SendU(x,distance,isfind,id);
        pre_id = id;
    }
    else{
        SendU(642,lo_dis,180,pre_id);
        lo_dis = lo_dis - delta_d;
        delta_d += wd;
    }


}

void SendU(int x,int distance,int isfind,int id){
    uint8_t send[8] = {1, 2, 3, 4, 5, 6, 7 , 8};
    send[0] = (uint8_t) distance;
    send[1] = (uint8_t) ((distance) >> 8);
    send[2] = (uint8_t) x;
    send[3] = (uint8_t) ((x) >> 8);
    send[4] = (uint8_t) 0;
    send[5] = (uint8_t) ((0) >> 8);
    send[6] = (uint8_t) (isfind);
    send[7] = (uint8_t) ((isfind) >> 8);

    sp.write(send, 8);  //把数据发送回去
    ROS_INFO_STREAM("Object:"<<id_obj[id]);
    ROS_INFO_STREAM("distance:"<<distance);
    ROS_INFO_STREAM("X:"<<x);
    ROS_INFO_STREAM("direction:"<<0);
    ROS_INFO_STREAM("isfind:"<<isfind);
    ROS_WARN("send=======================");
}


int main(int argc,char** argv){
    vis_mod=1;
    ros::init(argc,argv,"Handle_node");
    ros::NodeHandle nh;
    ros::Subscriber Sub = nh.subscribe("ComSerial", 10, imageCallback);
    //串口
    sp.setPort("/dev/ttyUSB0");
    sp.setBaudrate(115200);
    //串口设置timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    sp.setTimeout(to);
    while(ros::ok()){


        ros::spin();

    }


    return 0;
}









