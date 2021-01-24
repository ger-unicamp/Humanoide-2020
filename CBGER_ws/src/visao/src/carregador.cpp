#include <iostream>

#include <cstdlib>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;

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
    if (argc != 2)
    {
        cout << "agrv must have image topic name" << endl;
        exit(1);
    }

    cout << argv[1] << endl;
    string topicName(argv[1]);
    string nodeName("CBGER_carregador");
    int _argc=0;
    char **_argv = NULL;
    ros::init(_argc, _argv, nodeName.c_str());
    ros::NodeHandle nh;

    cv::namedWindow("view");
    cv::startWindowThread();

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(topicName.c_str(), 1, imageCallback);

    ros::spin();
    cv::destroyWindow("view");
}