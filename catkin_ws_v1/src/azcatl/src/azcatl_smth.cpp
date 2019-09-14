/************************************************************
*	LABORATORIO BIOROBÓTICA 2019
*	Garcés Marín Daniel 	
*
*	PROYECTO <ÁZCATL> 
*		Nodo principal para el manejo del robot Azcatl por medio de la aplicación Smart Things
*	
*	Ultima versión:: 6¿7 de Febrero del 2019
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


	if(dirMotor=="Fw"){ //Avanza
		dir_MotorA[0]=vel_motores;
		dir_MotorA[1]=vel_motores;}

	else if(dirMotor=="Bw"){ //Atras
		dir_MotorA[0]=(-1*vel_motores);
		dir_MotorA[1]=(-1*vel_motores);}

	else if(dirMotor=="R"){ //Giro Derecha
		dir_MotorA[0]=vel_motores;
		dir_MotorA[1]=(-1*vel_motores);}

	else if(dirMotor=="L"){ //Giro Izquierda
		dir_MotorA[0]=(-1*vel_motores);
		dir_MotorA[1]=vel_motores;}

	else if(dirMotor=="stop"){ //Alto
		dir_MotorA[0]=0;
		dir_MotorA[1]=0;}

	else if(dirMotor=="FwR"){ //Giro Avanza Derecha
		dir_MotorA[0]=vel_motores;
		dir_MotorA[1]=(-1*vel_motores)/2;}

	else if(dirMotor=="FwL"){ //Giro Avanza Izquierda
		dir_MotorA[0]=(-1*vel_motores)/2;
		dir_MotorA[1]=vel_motores;}

	else if(dirMotor=="BwR"){//Giro Atras Izquierda
		dir_MotorA[0]=(-1*vel_motores);
		dir_MotorA[1]=vel_motores/2;}

	else if(dirMotor=="BwL"){//Giro Avanza Derecha
		dir_MotorA[0]=vel_motores/2;
		dir_MotorA[1]=(-1*vel_motores);}

	else{//Alto
		dir_MotorA[0]=0;
		dir_MotorA[1]=0;}

		}//Fin de dirObtenida

//______________________________________________________________________________________________________________
//Función principal
int main(int  argc, char** argv){

	std::cout<<">>>>>LABORATORIO DE BIOROBÓTICA<<<<<<"<<std::endl;
	std::cout<<"_>>Azcatl Smarth Things en linea ..."<<std::endl;
	ros::init(argc,argv,"AZCATL");//********************************************
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
        D_Motor.data[0] = dir_MotorA[0]; 
	D_Motor.data[1] = dir_MotorA[1];
		
		pubDir.publish(D_Motor);  //Envió de las direcciónes al nodo Motor.py

		ros::spinOnce();
		loop.sleep();
        std::cout<<""<<std::endl;

	} //Fin del while(ROS)

return 0;
}//Fin del main principal