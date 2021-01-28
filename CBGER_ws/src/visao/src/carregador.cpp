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

using namespace std;

bool verifica = false;

// funcao que envia um 'int' com base na cor detectada pela camera 
// 0 - vermelho, 1 - verde, 2- nenhuma das duas
int colorIdentify(const cv::Mat image)
{
    // convertendo a imagem para o padrao 'hsv'
    cv::Mat image_hsv;
    cv::cvtColor(image, image_hsv, cv::COLOR_BGR2HSV);

    // criando mascaras para identificar a cor vermelha
    cv::Mat mask_red_1, mask_red_2;
    cv::inRange(image_hsv, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), mask_red_1);
    cv::inRange(image_hsv, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask_red_2);
    const cv::Mat mask_red = mask_red_1 + mask_red_2;

    // criando a mascara para identificar a cor verde
    cv::Mat mask_green;
    cv::inRange(image_hsv, cv::Scalar(36, 0, 0), cv::Scalar(86, 255, 255), mask_green);

    // identificando os maiores e menores valores para cada mascara
    double min_val_red, max_val_red, min_val_green, max_val_green;
    cv::minMaxLoc(mask_red, &min_val_red, &max_val_red);
    cv::minMaxLoc(mask_green, &min_val_green, &max_val_green);

    // caso a cor seja vermelha
    if ((min_val_red == 255) && (max_val_red == 255)) return 0;

    // caso a cor seja verde
    else if ((min_val_green == 255) && (max_val_green == 255)) return 1;

    // caso nao seja nenhuma das duas
    else return 2;
}

// funcao para enviar a flag pelo ros para a garra com base na cor identificada 
void colorFlag(const cv::Mat image)
{
    // identifica a cor, sendo '0' para o vermelho, '1' para o verde, e '2' nenhuma das duas
    int flag = colorIdentify(image);

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
        const cv::Mat image_bgr = cv_bridge::toCvShare(msg, "bgr8")->image;

        // agora, executamos a funcao que envia uma flag para a garra com base na cor presente na imagem
        colorFlag(image_bgr);

        // exibimos a imagem da camera na janela aberta
        cv::imshow("view", image_bgr);
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
