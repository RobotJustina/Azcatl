/************************************************************
*	LABORATORIO BIOROBÓTICA 2019
*	Garcés Marín Daniel 	
*
*	PROYECTO <ÁZCATL> 
*		Nodo principal para la prueba de hardware del robot de 6 ruedas Ázcatl.
*			El principal objetivo de este nodo es el comprobar que cada uno de los componentes estén funcionando adecuadamente.
*
*	Ultima versión:: 7 de Febrero del 2019
*********************************************************************/

//>>>>>>>> LIBRERÍAS <<<<<<<<<<<<<
// ダ・ガ・マ・jû-san
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
int datos_Obs[2]={0,0};
float datos_Vel[2]={0,0}; //Variables para la funcion valorVel()
float dir_MotorA[2]={0.0,0.0}; //Variable para indicar los valores de dirección y velocidadesa los motores

string dirMotor; //Variable para almacenar la dirección de los motores trasnmitida por SmartThings en función dirObtenida()
//_____________________________________________________________________________________________________________________

//>>>>>>>>>>> FUNCIONES <<<<<<<<<<<

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

	std::cout<<" --"<<std::endl;									}//Fin de valorFoto
//----------------------------------------------------------------------------------------

	//Obtención de los valores de los encoders
void valorVel(const std_msgs::Float32MultiArray::ConstPtr& dVel){

	std::cout<<"Encoders ::";
	for(i=0;i<2;i++){ //Vaciando los datos de los encoders
		datos_Vel[i]=dVel->data[i];
		std::cout<<"["<<datos_Vel<<"]"; } //Fin del vaciado de los encoders

	std::cout<<" --"<<std::endl;  								}//Fin de valorEnc
//-----------------------------------------------------------------------------------------

	//Obtención del la dirección del nodo SMART THINGS
void dirObtenida(const std_msgs::String::ConstPtr& dirM){
	dirMotor=dirM->data;
	std::cout<<"Dirección recibida::_"<<dirMotor<<std::endl;	}//Fin de dirObtenida
//-----------------------------------------------------------------------------------------

	//Función para la pruba de los motores
void pruebaMotores(){

	float vel_prueba=125; //Velocidad con la que probarán los motores

	switch(temp){

					case 0:	//Avanza
						std::cout<<">>ROBOT AVANZA"<<std::endl;
						dir_MotorA[0]=vel_prueba;
						dir_MotorA[1]=vel_prueba;
					break;

					case 1://Atras
					std::cout<<">>ROBOT RETROCEDE"<<std::endl;
						dir_MotorA[0]=(-1*vel_prueba);
						dir_MotorA[1]=(-1*vel_prueba);
					break;

					case 2://Giro Derecha
					std::cout<<">>ROBOT GIRA DERECHA"<<std::endl;
						dir_MotorA[0]=vel_prueba;
						dir_MotorA[1]=(-1*vel_prueba);
					break;

					case 3://Giro Izquierda
					std::cout<<">>ROBOT GIRA IZQUIERDA"<<std::endl;
						dir_MotorA[0]=(-1*vel_prueba);
						dir_MotorA[1]=vel_prueba;
					break;

					case 4://Alto
					std::cout<<">>ROBOT ALTO"<<std::endl;
						dir_MotorA[0]=0;
						dir_MotorA[1]=0;
					break;

					case 5: //Prueba de SMART THINGS

							if(dirMotor=="Fw"){ //Avanza
								std::cout<<">>ROBOT SMART THINGS AVANZA"<<std::endl;
								dir_MotorA[0]=vel_prueba;
								dir_MotorA[1]=vel_prueba;}

							else if(dirMotor=="Bw"){ //Atras
								std::cout<<">>ROBOT SMART THINGS RETROCEDE"<<std::endl;
									dir_MotorA[0]=(-1*vel_prueba);
									dir_MotorA[1]=(-1*vel_prueba);}

							else if(dirMotor=="R"){ //Giro Derecha
								std::cout<<">>ROBOT SMART THINGS GIRO DERECHA"<<std::endl;
									dir_MotorA[0]=vel_prueba;
									dir_MotorA[1]=(-1*vel_prueba);}

							else if(dirMotor=="L"){ //Giro Izquierda
								std::cout<<">>ROBOT SMART THINGS GIRO IZQUIERDA"<<std::endl;
									dir_MotorA[0]=(-1*vel_prueba);
									dir_MotorA[1]=vel_prueba;}

							else if(dirMotor=="stop"){ //Alto
								std::cout<<">>ROBOT SMART THINGS ALTO"<<std::endl;
									dir_MotorA[0]=0;
									dir_MotorA[1]=0;}
							else{//Alto
								std::cout<<">>ROBOT SMART THINGS DETENIDO"<<std::endl;
									dir_MotorA[0]=0;
									dir_MotorA[1]=0;}
						break;

					default://Alto
						std::cout<<">>ROBOT DETENIDO"<<std::endl;
						dir_MotorA[0]=0;
						dir_MotorA[1]=0;
					break;
	}//fin del switch-case
	cont++;
	temp=cont/100;

}//Fin de prueba motores


//_______________________________________________________________________________________________________________
//Función principal
int main(int  argc, char** argv){

	std::cout<<">>>>>LABORATORIO DE BIOROBÓTICA<<<<<<"<<std::endl;
	std::cout<<"_>>AZCATL en linea para prueba de hardware ..."<<std::endl;
	ros::init(argc,argv,"AZCATL");//********************************************
	ros:: NodeHandle n;

    //Obtención de los datos transmitidos por los diferentes nodos 
 	ros::Subscriber subFoto = n.subscribe("/hardware/sensors/foto",1000,valorFoto); //Nodo Sensors/Fotoresistores
 	ros::Subscriber subEnc = n.subscribe("/hardware/sensors/enc",1000,valorEnc);; //Nodo Sensors/Encoders 
 	ros::Subscriber subDir = n.subscribe("/hardware/smartThings/dir", 1000,dirObtenida); //Nodo SmartThings


 	//Datos a publicar
	std_msgs::Float32MultiArray  D_Motor; //Dirección del motor
	D_Motor.data.resize(2);	

    //Publicación de las velocidades de los motores al Arduino
    ros::Publisher pubDir=n.advertise <std_msgs::Float32MultiArray>("/hardware/motor/direction",1);

	ros::Rate loop(1);
    ros::Rate r(10);
	
	while(ros::ok()){

	pruebaMotores(); //Se prueban los motores

		//Publicación de tópico de las direcciones
    	D_Motor.data[0] = dir_MotorA[0]; //<-----
	D_Motor.data[1] = dir_MotorA[1]; //<-----
	std::cout<<D_Motor<<std::endl;

		pubDir.publish(D_Motor);  //Envió de las direcciónes al nodo Motor.py

		ros::spinOnce();
		loop.sleep();
        std::cout<<""<<std::endl;

	} //Fin del while(ROS)

return 0;
}//Fin del main principal