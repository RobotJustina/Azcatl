/************************************************************
*	LABORATORIO BIOROBÓTICA 2018
*	Garcés Marín Daniel 	
*
*	PROYECTO <TEPORINGO> :: Xochitonal
*		Nodo principal para el manejo del robot de 6 ruedas Xoxhitonal, el cual partiendo de Teporingo ha sido mejorado y 
*		actualizado con nuevos elementos y dispositivos, ademas de darles nuevas funcionalidades.
		-Revisar documentación del Reporte de la plataforma
*
*	Ultima versión:: 18 de Agosto del 2018
*********************************************************************/

//DaGaMa_jû-san 
//>>>>>>>> LIBRERÍAS <<<<<<<<<<<<<
#include "ros/ros.h"
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

//>>> VARAIBLES GLOBALES

	//Variables de apoyo
int i=0,j=0; //varaibles contadores
int temp=0,cont=0; //Varaibles para la dirección de prueba de los motores, función pruebaMotores()
int num_fot=0; //Variable para la función valorFoto()
float valor_foto=0.0; //Variable para la función valorFoto()

	//Variables paraa guardar información de los tópicos
float datos_Foto[8]={0,0,0,0,0,0,0,0}; //Variable para la función valorFoto()
float datos_Enc[12]={0,0,0,0,0,0,0,0,0,0,0,0}; //Variables para la funcion valorEnc()
string dirMotor; //Variable para almacenar la dirección de los motores función dirObtenida()
string dirMotorTemp= "Fw"; //Variable para a función pruebaMotores()
//_____________________________________________________________________________________________________________________

//>>>>>>>>>>> FUNCIONES <<<<<<<<<<<

	//Obtención del la dirección del nodo SMART THINGS
void dirObtenida(const std_msgs::String::ConstPtr& dirM){
	dirMotor=dirM->data;
	std::cout<<"Dirección recibida::_"<<dirMotor<<std::endl;	}//Fin de dirObtenida

	//Obtención de los valores de los fotoresistores
void valorFoto(const std_msgs::Float32MultiArray::ConstPtr& dFoto){

	for(i=0;i<8;i++){ //Vaciado de los valores fotoresitores
		datos_Foto[i]=dFoto->data[i];
		std::cout<<"Fotoresitor["<<i<<"]:_ "<<datos_Foto[i]<<std::endl;	}//Fin Vaciado de los valores fotoresitores

	//Tratamiento de los datos:: Obteniedo valor más alto;
	for(i=0;i<8;i++){ //comparación de todos los valores prra saber el más alto

		if(valor_foto<=datos_Foto[i]){ //valor actual mayor
			num_fot=i;
			valor_foto=datos_Foto[i]; }	} //Fin de la busqueda mayor

	std::cout<<"Fuente de luz ubicada en fotoresitor["<<num_fot<<"]:: "<<valor_foto<<std::endl;

}//Fin de valorFoto

	//Obtención de los valores de los encoders
void valorEnc(const std_msgs::Float32MultiArray::ConstPtr& dEnc){

	std::cout<<"Encoders ::";
	for(i=0;i<12;i++){ //Vaciando los datos de los encoders
		datos_Enc[i]=dEnc->data[i];
		std::cout<<"["<<datos_Enc<<"]"; 	} //Fin del vaciado de los encoders

	std::cout<<" --"<<std::endl;

	}//Fin de valorEnc

//_______________________________________________________________________________________________________________
//Función principal
int main(int  argc, char** argv){

	std::cout<<">>>>>LABORATORIO DE BIOROBÓTICA<<<<<<"<<std::endl;
	std::cout<<"_>>XOCHITONAL(base) en linea ..."<<std::endl;
	ros::init(argc,argv,"TeporingoV3");//********************************************
	ros:: NodeHandle n;

    //Obtención de los datos transmitidos por los diferentes nodos 
 	ros::Subscriber subDir = n.subscribe("/hardware/smartThings/dir", 1000,dirObtenida); //Nodo SmartThings
 	ros::Subscriber subFoto = n.subscribe("/hardware/sensors/foto",1000,valorFoto); //Nodo Sensors
 	ros::Subscriber subEnc = n.subscribe("/hardware/sensors/enc",1000,valorEnc);; //Nodo sensors

 	//Datos a publicar
	std_msgs::String D_Motor; //Dirección del motor	

    //Publicación de las velocidades de los motores al Arduino
    ros::Publisher pubDir=n.advertise <std_msgs::String>("/hardware/motor/direction",1);

	ros::Rate loop(10);
    ros::Rate r(1000);
	
	while(ros::ok()){

		//Publicación de tópico de las direcciones
        D_Motor.data = dirMotor; //<-----
		std::cout<<D_Motor<<std::endl;

		pubDir.publish(D_Motor);  //Envió de las direcciónes al nodo Motor.py

		ros::spinOnce();
		loop.sleep();
        std::cout<<""<<std::endl;

	} //Fin del while(ROS)

return 0;
}//Fin del main principal