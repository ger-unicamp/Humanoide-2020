#include <string>

#include <cstdlib>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"

bool sensorTrigger = false;
struct timeval tv;
float timeUpdatedByTopicSim = 0;
float simTime = 0;

void sensNoseCallback(const std_msgs::Bool& sensTrigger);
void simTimeCallback(const std_msgs::Float32& simulationTime);

int main(int argc, char **argv)
{
    int _argc = 0;
    char **_argv = NULL;
    std::string nodeName("bubbleRobControllerNode");
    ros::init(_argc, _argv, nodeName);
    
    std::string leftMotorTopic;
    std::string rightMotorTopic;
    std::string sensingNoseTopic;
    std::string simulationTimeTopic;

    if (argc >= 5)
    {
        leftMotorTopic = argv[1];
        rightMotorTopic = argv[2];
        sensingNoseTopic = argv[3];
        simulationTimeTopic = argv[4];
	std::cout << "Topic names: " << leftMotorTopic << " " << rightMotorTopic << " "  << sensingNoseTopic << " "  << simulationTimeTopic << std::endl;
    }
    else
    {
        ROS_INFO("[ERROR] Usage %s %s %s %s", leftMotorTopic.c_str(), rightMotorTopic.c_str(),
         sensingNoseTopic.c_str(), simulationTimeTopic.c_str());
        return 1;
    }

    ros::NodeHandle nodeHandle;

    ros::Subscriber sensNoseSubs = nodeHandle.subscribe(sensingNoseTopic, 1, sensNoseCallback);
    ros::Subscriber simTimeSubs = nodeHandle.subscribe(simulationTimeTopic, 1, simTimeCallback);

    ros::Publisher leftMotorPub = nodeHandle.advertise<std_msgs::Float32>(leftMotorTopic.c_str(), 1);
    ros::Publisher rightMotorPub = nodeHandle.advertise<std_msgs::Float32>(rightMotorTopic.c_str(), 1);

    ros::Rate loop_rate(10);
    float driveBackStartTime = -99;
    unsigned int currentTime;
    if (gettimeofday(&tv, NULL) == 0)
    {
        timeUpdatedByTopicSim = tv.tv_sec;
        currentTime = timeUpdatedByTopicSim;
    }
    while (ros::ok())
    {
        if (gettimeofday(&tv, NULL) == 0)
        {
            currentTime = tv.tv_sec;
            if (currentTime - timeUpdatedByTopicSim > 9)
            {
                break;
            }
        }

        float disaredLeftMotorSpeed;
        float disaredRightMotorSpeed;

        if (simTime - driveBackStartTime < 3.0)
        {
            disaredLeftMotorSpeed = 0;
            disaredRightMotorSpeed = 3.1514 * 0.8;
        }
        else
        {
            disaredLeftMotorSpeed = 3.1415;
            disaredRightMotorSpeed = 3.1415;

            if (sensorTrigger)
            {
                driveBackStartTime = simTime;
            }
            sensorTrigger = false;
        }
        
        std_msgs::Float32 speed;
        speed.data = disaredLeftMotorSpeed;
        leftMotorPub.publish(speed);
        speed.data = disaredRightMotorSpeed;
        rightMotorPub.publish(speed);

        ros::spinOnce();

        loop_rate.sleep();
    }

    ros::shutdown();
    return 0;
    
}

void sensNoseCallback(const std_msgs::Bool& sensTrigger)
{
    sensorTrigger = sensTrigger.data;
}

void simTimeCallback(const std_msgs::Float32& simulationTime)
{
    simTime = simulationTime.data;
}
