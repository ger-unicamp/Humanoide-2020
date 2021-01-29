#include <iostream>
#include <numeric>

#include <cstdlib>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"

using namespace std;

bool verifica = false;

// flag referente a cor identificada
std_msgs::Int32 colorFlag;

// funcao que envia um 'int' com base na cor detectada pela camera 
// 0 - vermelho, 1 - verde, 2- nenhuma das duas
int colorIdentify(const cv::Mat image)
{
    // convertendo a imagem para o padrao 'hsv'
    cv::Mat imageHSV;
    cv::cvtColor(image, imageHSV, cv::COLOR_BGR2HSV);

    // criando mascaras para identificar a cor vermelha
    cv::Mat maskRed1, maskRed2;
    cv::inRange(imageHSV, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), maskRed1);
    cv::inRange(imageHSV, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), maskRed2);
    const cv::Mat maskRed = maskRed1 + maskRed2;

    // criando a mascara para identificar a cor verde
    cv::Mat maskGreen;
    cv::inRange(imageHSV, cv::Scalar(36, 0, 0), cv::Scalar(86, 255, 255), maskGreen);

    // identificando os maiores e menores valores para cada mascara
    double minvalRed, maxvalRed, minvalGreen, maxvalGreen;
    cv::minMaxLoc(maskRed, &minvalRed, &maxvalRed);
    cv::minMaxLoc(maskGreen, &minvalGreen, &maxvalGreen);

    // caso a cor seja vermelha
    if ((minvalRed == 255) && (maxvalRed == 255)) return 0;

    // caso a cor seja verde
    else if ((minvalGreen == 255) && (maxvalGreen == 255)) return 1;

    // caso nao seja nenhuma das duas
    else return 2;
}

// funcao para atualizar a flag enviada pelo ros para a garra com base na cor identificada 
void colorUpdate(const cv::Mat image)
{
    // identifica a cor, sendo '0' para o vermelho, '1' para o verde, e '2' nenhuma das duas
    colorFlag.data = colorIdentify(image);

    return;
}

void stopCallback(const std_msgs::Bool& bottomStop)
{
    verifica = bottomStop.data;
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{   
    try
    {
        // pegamos a imagem do open cv atraves da mensagem enviada pelo ros
        // ela eh codificada inicialmente no padrao 'bgr8'
        const cv::Mat imageBGR = cv_bridge::toCvShare(msg, "bgr8")->image;

        // agora, executamos a funcao que atualiza a flag que eh enviada para a garra com base na cor presente na imagem
        colorUpdate(imageBGR);

        // exibimos a imagem da camera na janela aberta
        cv::imshow("view", imageBGR);
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
    string colorTopic = argv[3];
    cout << "Nome dos tópicos: " << imageTopic << " " << msgTopic << " " << colorTopic << endl;
    
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
    ros::Publisher colorPub = nh.advertise<std_msgs::Int32>(colorTopic, 1);
    colorPub.publish(colorFlag);

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
