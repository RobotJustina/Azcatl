/************************************************************
*	LABORATORIO BIOROBÓTICA 2018
*	Garcés Marín Daniel 	
*
*	PROYECTO <Xochitonal> :: V2
*		Nodo principal para el manejo del robot Xochitonal por medio de la aplicación Smart Things
*	
*	Ultima versión:: 12 de Diciembre del 2018
*********************************************************************/

//>>>>>>>> LIBRERÍAS <<<<<<<<<<<<<
#include "ros/ros.h"
// ダ・ガ・マ・jû-san
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <termio.h>
#include <sys/ioctl.h>
#include <string>
#include <iostream>
using namespace std;
//----------------------------------

//>>> VARIABLES GLOBALES

	//Variables de apoyo
int i=0,j=0; //varaibles contadores
float vel_motores=125; //Velocidad estándar para los motores

	//Variables paraa guardar información de los tópicos
string dirMotor; //Variable para almacenar la dirección de los motores función dirObtenida()
float dir_MotorA[2]={0.0,0.0}; //Variable para indicar los valores de dirección y velocidadesa los motores
//_____________________________________________________________________________________________________________________

//>>>>>>>>>>> FUNCIONES <<<<<<<<<<<

	//Obtención del la dirección del nodo SMART THINGS
void dirObtenida(const std_msgs::String::ConstPtr& dirM){
	dirMotor=dirM->data;
	std::cout<<"Dirección recibida::_"<<dirMotor<<std::endl;

	switch(dirMotor){ //Switch anidado para el Smart Things

						case "Fw":	//Avanza
								dir_MotorA[0]=vel_motores;
								dir_MotorA[1]=vel_motores;
								break;

						case "Bw"://Atras
								dir_MotorA[0]=(-1*vel_motores);
								dir_MotorA[1]=(-1*vel_motores);
								break;

						case "R"://Giro Derecha
								dir_MotorA[0]=vel_motores;
								dir_MotorA[1]=(-1*vel_motores);
								break;

						case "L"://Giro Izquierda
								dir_MotorA[0]=(-1*vel_motores);
								dir_MotorA[1]=vel_motores;
								break;

						case "stop"://Alto
								dir_MotorA[0]=0;
								dir_MotorA[1]=0;
						break;

						case "FwR":	//Giro Avanza Derecha
								dir_MotorA[0]=vel_motores;
								dir_MotorA[1]=(-1*vel_motores)/2;
								break

						case "FwL"://Giro Avanza Izquierda
								dir_MotorA[0]=(-1*vel_motores)/2;
								dir_MotorA[1]=vel_motores;
								break;

						case "BwR"://Giro Atras Izquierda
								dir_MotorA[0]=(-1*vel_motores);
								dir_MotorA[1]=vel_motores/2;
								break;

						case "BwL":	//Giro Avanza Derecha
								dir_MotorA[0]=vel_motores/2;
								dir_MotorA[1]=(-1*vel_motores);
								break

						default://Alto
								dir_MotorA[0]=0;
								dir_MotorA[1]=0;
								break;
	}	}//Fin de dirObtenida

______________________________________________________________________________________________________________
//Función principal
int main(int  argc, char** argv){

	std::cout<<">>>>>LABORATORIO DE BIOROBÓTICA<<<<<<"<<std::endl;
	std::cout<<"_>>XOCHITONAL Smarth Things en linea ..."<<std::endl;
	ros::init(argc,argv,"XochitonalV2");//********************************************
	ros:: NodeHandle n;

    //Obtención de los datos transmitidos por los diferentes nodos 
 	ros::Subscriber subDir = n.subscribe("/hardware/smartThings/dir", 1000,dirObtenida); //Nodo SmartThings

 	//Datos a publicar
	std_msgs::Float32MultiArray  D_Motor; //Dirección del motor
	D_Motor.data.resize(2);	

    //Publicación de las velocidades de los motores al Arduino
    ros::Publisher pubDir=n.advertise <std_msgs::Float32MultiArray>("/hardware/motor/direction",1);

	ros::Rate loop(1); //10
    ros::Rate r(10); //1000
	
	while(ros::ok()){

	//pruebaMotores(); //Se prueban los motores
		//Publicación de tópico de las direcciones
        D_Motor.data = dirMotor; 
		
		pubDir.publish(D_Motor);  //Envió de las direcciónes al nodo Motor.py

		ros::spinOnce();
		loop.sleep();
        std::cout<<""<<std::endl;

	} //Fin del while(ROS)

return 0;
}//Fin del main principal