#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <string>

void imageCallback(const sensor_msgs::ImageConstPtr &msg){
    try
    {
        // 将ROS图像消息转换为OpenCV图像
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

        // 显示图像
        cv::imshow("Image", image);

        cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Failed to convert ROS image to OpenCV image: %s", e.what());
    }
}



int main(int argc,char** argv){
    ros::init(argc,argv,"Handle_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);
    image_transport::Subscriber Sub = it.subscribe("Result", 10, imageCallback);

    while(ros::ok()){


        ros::spin();

    }


    cv::destroyAllWindows();

    return 0;
}




