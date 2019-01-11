/************************************************************
*	LABORATORIO BIOROBÓTICA de la FACULTAD DE INGENIERÍA
*	Garcés Marín Daniel 		
*	Programa de apoyo para el tesista::
*		Programa central en la Raspberry
*	Ultima versión:: 20 de Mayo del 2018
*********************************************************************/

//>>>>>>>> LIBRERÍAS <<<<<<<<<<<<<
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <stdio.h>
//DaGaMa_jû-san 
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <termio.h>
#include <sys/ioctl.h>

//Variables globales
float vel_Der = 0.0, vel_Izq=0.0;
   //   Vel_Der     Vel_Izq
int j; //Contador auxiliar

//>>>>>>>>>>> FUNCIONES <<<<<<<<<<<
void velocidadesObtenidas(const std_msgs::Float32MultiArray::ConstPtr& velO){
    vel_Der = velO->data[0];
    vel_Izq= velO->data[1];
    std::cout<<" >>> Velocidades obtenidas:: Vel_Der " << vel_Der <<" -- Vel_Izq"<< vel_Izq << std::endl;
                               }//Fin velocidaddesObtenidos

//_______________________________________________________________________________
//Función principal
int main(int  argc, char** argv){

	std::cout<<"Laboratorio de BioRobótica >> TEPORINGO en linea"<<std::endl;
	ros::init(argc,argv,"Teporingo");//********************************************
	ros:: NodeHandle n;

    //Obtención de los datos transmitidos del ARDUINO--2 [Velocidades de los motores]
 	ros::Subscriber sub = n.subscribe("/hardware/mobile_base/motor_speeds", 1000, velocidadesObtenidas); //<<<<<<<<<<----------------------------

	std_msgs::Float32MultiArray D_Motor; //Velocidades del motor
	D_Motor.data.resize(2);	

    //Publicación de las velocidades de los motores al Arduino
    ros::Publisher pubCount=n.advertise <std_msgs::Float32MultiArray>("/Teporingo/hardware/motor_speeds",1);

	ros::Rate loop(10);
    ros::Rate r(1000);
	
	while(ros::ok()){


        D_Motor.data[0]=vel_Der;
        D_Motor.data[1]=vel_Izq;


		pubCount.publish(D_Motor);  //Envió de datos al arduino

		ros::spinOnce();
		loop.sleep();
        std::cout<<""<<std::endl;

	} //Fin del while(ROS)

return 0;
} //Fin del main

