#include <iostream>
#include <numeric>
#include <cstdlib>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


using namespace std;

bool flag_parada = false;

// flag referente a cor identificada
std_msgs::Int32 flag_cor;

// funcao que envia um 'int' com base na cor detectada pela camera 
// 0 - vermelho, 1 - verde, 2- nenhuma das duas
int identificar_cor(const cv::Mat imagem)
{
    // convertendo a imagem para o padrao 'hsv'
    cv::Mat imagem_HSV;
    cv::cvtColor(imagem, imagem_HSV, cv::COLOR_BGR2HSV);

    // criando mascaras para identificar a cor vermelha
    cv::Mat mascara_vermelha_inferior, mascara_vermelha_superior;
    cv::inRange(imagem_HSV, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), mascara_vermelha_inferior);
    cv::inRange(imagem_HSV, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mascara_vermelha_superior);
    const cv::Mat mascara_vermelha = mascara_vermelha_inferior + mascara_vermelha_superior;

    // criando a mascara para identificar a cor verde
    cv::Mat mascara_verde;
    cv::inRange(imagem_HSV, cv::Scalar(36, 0, 0), cv::Scalar(86, 255, 255), mascara_verde);

    // identificando os maiores e menores valores para cada mascara
    double val_min_vermelho, val_max_vermelho, val_min_verde, val_max_verde;
    cv::minMaxLoc(mascara_vermelha, &val_min_vermelho, &val_max_vermelho);
    cv::minMaxLoc(mascara_verde, &val_min_verde, &val_max_verde);

    // caso a cor seja vermelha
    if ((val_min_vermelho == 255) && (val_max_vermelho == 255)) return 0;

    // caso a cor seja verde
    else if ((val_min_verde == 255) && (val_max_verde == 255)) return 1;

    // caso nao seja nenhuma das duas
    else return 2;
}

// funcao para atualizar a flag enviada pelo ros para a garra com base na cor identificada 
void atualiza_flag_cor(const cv::Mat imagem)
{
    // identifica a cor, sendo '0' para o vermelho, '1' para o verde, e '2' nenhuma das duas
    flag_cor.data = identificar_cor(imagem);

    return;
}

void receber_parada(const std_msgs::Bool& comando_parar)
{
    flag_parada = comando_parar.data;
}

void receber_imagem(const sensor_msgs::ImageConstPtr &msg)
{   
    try
    {
        // pegamos a imagem do open cv atraves da mensagem enviada pelo ros
        // ela eh codificada inicialmente no padrao 'bgr8'
        const cv::Mat imagem_BGR = cv_bridge::toCvShare(msg, "bgr8")->image;

        // agora, executamos a funcao que atualiza a flag que eh enviada para a garra com base na cor presente na imagem
        atualiza_flag_cor(imagem_BGR);

        // exibimos a imagem da camera na janela aberta
        cv::imshow("view", imagem_BGR);
        cv::waitKey(30);
    }

    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Não foi possível converter de '%s' para o padrão 'BGR8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{   
    if (argc != 4)
    {
        cout << "Argumentos inválidos. A execução do programa deve conter 3 tópicos: topico_imagem, topico_mensagem e topico_cor" << endl;
        exit(1);
    }

    string topico_imagem = argv[1];
    string topico_mensagem = argv[2];
    string topico_cor = argv[3];
    cout << "Nome dos tópicos: " << topico_imagem << " " << topico_mensagem << " " << topico_cor << endl;
    
    string nome_no("CBGER_carregador");
    int _argc=0;
    char **_argv = NULL;
    ros::init(_argc, _argv, nome_no.c_str());
    ros::NodeHandle nh;

    cv::namedWindow("Visão do Sensor");
    cv::startWindowThread();
    
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subscriber_imagem = it.subscribe(topico_imagem.c_str(), 1, receber_imagem);
    
    ros::Subscriber subscriber_mensagem = nh.subscribe(topico_mensagem, 1, receber_parada);
    ros::Publisher publisher_cor = nh.advertise<std_msgs::Int32>(topico_cor, 1);

    while(ros::ok())
    {
        if(flag_parada == true)
        {
            cv::destroyWindow("Visão do Sensor");
            ros::shutdown();
        }

        publisher_cor.publish(flag_cor);
        ros::spinOnce();
    }
}
