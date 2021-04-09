#include <iostream>
#include <string>

#include <ros/ros.h>
#include "std_msgs/Bool.h"

using namespace std;

bool trabalho_finalizado = false;

void receber_trabalho_finalizado(const std_msgs::Bool& comando_finaliza_trabalho);

int main(int argc, char **argv) {
    if (argc != 3)
    {
        cout << "Argumentos inválidos. A execução do programa deve conter 2 tópicos: "<<
            "topico_trabalho_finalizado e topico_transportador_carregado" << endl;
        exit(1);
    }
    
    string topico_trabalho_finalizado = argv[1];
    string topico_transportador_carregado = argv[2];
    cout << "Nome do tópicos: " << topico_trabalho_finalizado << " " << topico_transportador_carregado << endl;
    
    string nome_no("CBGER_transportador");
    int _argc=0;
    char **_argv = NULL;
    ros::init(_argc, _argv, nome_no.c_str());
    ros::NodeHandle nh;
    ros::Subscriber subs_finalizado = nh.subscribe(topico_trabalho_finalizado, 1, receber_trabalho_finalizado);

    ros::Publisher pub_finaliza = nh.advertise<std_msgs::Bool>(topico_transportador_carregado, 1);

    while(ros::ok())
    {
        if(trabalho_finalizado == true)
        {
    
            std_msgs::Bool finaliza_trabalho;
            finaliza_trabalho.data = trabalho_finalizado;
            pub_finaliza.publish(finaliza_trabalho);
        }
        ros::spinOnce();
    }
}

void receber_trabalho_finalizado(const std_msgs::Bool& comando_finaliza_trabalho) {
    trabalho_finalizado = comando_finaliza_trabalho.data;
}
