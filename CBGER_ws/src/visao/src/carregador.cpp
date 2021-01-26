#include <iostream>

#include <cstdlib>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/Bool.h"

using namespace std;

bool verifica = false;

void stopCallback(const std_msgs::Bool& bottomStop)
{
    verifica = bottomStop.data;
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{   
    try
    {
        cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

}

int main(int argc, char **argv)
{   
    if (argc != 3)
    {
        cout << "agrv must have image topic name" << endl;
        exit(1);
    }

    string imageTopic = argv[1];
    string msgTopic = argv[2];
    cout << "Nome dos tópicos: " << imageTopic << " " << msgTopic << endl;
    
    string nodeName("CBGER_carregador");
    int _argc=0;
    char **_argv = NULL;
    ros::init(_argc, _argv, nodeName.c_str());
    ros::NodeHandle nh;

    cv::namedWindow("view");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(imageTopic.c_str(), 1, imageCallback);
    ros::Subscriber sub2 = nh.subscribe(msgTopic, 1, stopCallback);

    while(ros::ok())
    {
        if(verifica == true)
        {
            cv::destroyWindow("view");
            ros::shutdown();
        }
        ros::spinOnce();
    }
}
