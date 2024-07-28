#include "ComSerial.h"

ComSerial::ComSerial(std::string com_path, int baudRate, int outTime) :
    com_path(com_path),baudRate(baudRate),outTime(outTime){
    serial::Timeout to = serial::Timeout::simpleTimeout(outTime);
    //设置要打开的串口名称
    sp.setPort(com_path);
    //设置串口通信的波特率
    sp.setBaudrate(baudRate);
    //串口设置timeout
    sp.setTimeout(to);
    try
    {
        //打开串口
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");

        return;
    }

    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO_STREAM(com_path << " is opened.");
        std::cout<<"Serial Register!" << std::endl;
        Pub= nh_cl.advertise<std_msgs::Int32>("ComSerial", 1);
        pre_x = -1;
    }
}

ComSerial::~ComSerial() {
    sp.close();
    ROS_INFO_STREAM(com_path<<" is closed.");
}

bool ComSerial::sendPort(cv::Mat img, std::vector<Detection>& res) {
    readUart();

    switch (idgot) {
        case 1://排球
            id = 0;
            break;
        case 2://篮球
            id = 1;
            break;
        case 3://定位柱
            id = 3;
            break;
        default:
            id = -1;
    }
    Property detect;    //这是处理结果
    detect.isfind=179;  //此id号表示没有找到目标
    detect.distance=8000;   //距离初始值
    detect.x=640;
    for(auto& iter : res)
    {
        Property temp;

        readRes(img, iter, temp);//读取视觉识别结果
        if(temp.class_id == id){    //当检测到电控要求的目标时
            calDistance(temp);      //计算距离
            calDirection(temp);     //计算角度
            if(detect.distance > temp.distance){ //查找距离最近的目标
                detect = temp;
                detect.isfind = 180;    //已经找到目标
                //std::cout << "temp.distance:" << temp.distance << std::endl;
            }
        }

    }
/*
    if(detect.isfind == 179 && frame.count==2){
        //假如没有找到目标，进入插帧模式
        if(frame.duration[1] < 0.01 && frame.id[0] == frame.id[1]){
            //时间跨度太长，计算结果将会及其粗糙，
            detect.x = 2 * frame.rev[1]-frame.rev[0];
            detect.distance = 2 * frame.dist[1]-frame.dist[0];
            detect.isfind = 180;
        }
        frame.rev.clear();
        frame.dist.clear();
        frame.id.clear();
        frame.count = 0;
        frame.duration.clear();

    }
*/
    if(detect.distance < 8000){ //当找到
        if(sp.isOpen()){//串口

            // 发包
            sendUart(detect);
            stop = clock();
            //frame.rev.push_back(detect.x);
            //frame.dist.push_back(detect.distance);
            //frame.id.push_back(detect.class_id);
            //frame.count++;
            //frame.duration.push_back( double (stop-start)/CLOCKS_PER_SEC);
            start=clock();
            return true;
        }
        else{
            ROS_FATAL_STREAM("URAT NOT OPEN!");
            return false;
        }
    }
    else{//没找到
        ROS_FATAL_STREAM("No Objects!");
        sendUart(detect);
    }
    return true;


}

void ComSerial::readRes(cv::Mat img, Detection& res,Property& pro) {
    cv::Rect r = get_rect(img, res.bbox);
    pro.x = r.x+r.width/2;
    if(r.width>1000)
        r.width=0;
    //ROS_FATAL_STREAM("x:"<<r.x);
    //ROS_FATAL_STREAM("width:"<<r.width);
    pro.y = r.y+r.height/2;
    pro.width = r.width;
    pro.height = r.height;
    pro.imgH = img.rows;
    pro.imgW = img.cols;
    pro.class_id = res.class_id;
    //std::cout<<"#X&Y:"<<std::endl;
    //std::cout << pro.x<<" "<<pro.y<<std::endl;
}

void ComSerial::calDistance(Property &pro) {
    int width = pro.width;
    int height = pro.height;
    //修改
    float d1,d2;
    if(id==1){
        d1 = Basketboll_Width*Focal_Length/(Sensor_Coefficient*width);
        d2 = Basketboll_Width*Focal_Length/(Sensor_Coefficient*height);
    }
    if(id==0){
        d1 = Basketboll_Width*Focal_Length/(Sensor_Coefficient*width);
        d2 = Basketboll_Width*Focal_Length/(Sensor_Coefficient*height);
    }
    if(id==3){
        d1 = Basketboll_Width*Focal_Length/(Sensor_Coefficient*width);
        d2 = Basketboll_Width*Focal_Length/(Sensor_Coefficient*height);
    }
    //std::cout << d1 << " "<< d2 << std::endl;
    pro.distance = round((d1+d2)/2);
}

void ComSerial::calDirection(Property &pro) {
    int x = pro.x + pro.width/2;
    int deltaX = abs(x-pro.imgW/2);
    float deltaL = deltaX*Direction_Cofficient/(Sensor_Coefficient*pro.width);
    //std::cout <<"Angle:"<<deltaL << " " << deltaL/pro.distance << std::endl;
    pro.direction = deltaL/pro.distance;
    //std::cout<<round(atan(deltaL/pro.distance)*180/M_PI)<<std::endl;
    pro.direction = round(atan(deltaL/pro.distance)*180/M_PI);

}

void ComSerial::sendUart(Property &pro) {
    std_msgs::Int32 sendInt;
    sendInt.data = pro.class_id*100000000 + pro.x*10000000+pro.distance*1000+pro.isfind;
    Pub.publish(sendInt);
    /*
    uint8_t send[8] = {1, 2, 3, 4, 5, 6, 7 , 8};
    int now_x = pro.x;
    ROS_FATAL_STREAM("wr" << wr*(now_x - pre_x));
    if(pre_x!=-1 && pre_id == id){
        now_x = now_x - wr*(now_x - pre_x);
        //pre_x = now_x;
    }
    pre_id = id;
    pre_x = now_x;
    //距离修正
    now_x = 640 + pro.distance*(now_x-640)/(pro.distance + 20);
    /*
    union TEXT{
        uint8_t send[8];
        float CH[2];
    }text;

    text.send[4] = 0x00;text.send[5] = 0x00; text.send[6] = 0x80; text.send[7] = 0x7f;
    text.CH[0] = float(now_x);
    for(int i = 0;i<8;i++)
    {
        send[i] = text.send[i];
    }

    send[0] = (uint8_t) pro.distance;
    send[1] = (uint8_t) ((pro.distance) >> 8);
    send[2] = (uint8_t) now_x;
    send[3] = (uint8_t) ((now_x) >> 8);
    send[4] = (uint8_t) pro.direction;
    send[5] = (uint8_t) ((pro.direction) >> 8);
    send[6] = (uint8_t) (pro.isfind);
    send[7] = (uint8_t) ((pro.isfind) >> 8);

    sp.write(send, 8);  //把数据发送回去
    ROS_INFO_STREAM("Object:"<<id_obj[id]);
    ROS_INFO_STREAM("distance:"<<pro.distance);
    ROS_INFO_STREAM("X:"<<now_x);
    ROS_INFO_STREAM("direction:"<<pro.direction);
    ROS_INFO_STREAM("isfind:"<<pro.isfind);
    ROS_WARN("send=======================");
*/
    //std::cout << std::hex << (send[0] & 0xff) << std::endl;
}

void ComSerial::readUart() {
    if(sp.isOpen()){
        size_t n = sp.available();
        if(n!=0) {
            uint8_t buffer[1024];
            //读出数据
            n = sp.read(buffer, n);

            for (int i = 0; i < n; i++) {
                //16进制的方式打印到屏幕
                //std::cout << (buffer[i] & 0xff) << " ";
                idgot = buffer[i];

            }
            std::cout << "idgot:" << idgot << std::endl;
            std::cout << std::endl;
        }

    }

}


