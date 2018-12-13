/************************************************************
*	LABORATORIO BIOROBÓTICA 2018
*	Garcés Marín Daniel 	
*
*	PROYECTO <TEPORINGO> :: Xochitonal
*		Nodo principal para el manejo del robot de 6 ruedas Xoxhitonal, el cual partiendo de Teporingo ha sido mejorado y 
*		actualizado con nuevos elementos y dispositivos, ademas de darles nuevas funcionalidades.
		-Revisar documentación del Reporte de la plataforma
		-Nod para uso único del módulo Smarth Things y su funcionalidades de dirección
		-Agrega la fuuncionalidad de seguimiento de luz, incluyendo un boton extra a la aplicación; algoritmo mejorado
*
*	Ultima versión:: 6 de Noviembre del 2018
//DaGaMa_jû-san 
*********************************************************************/

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

//>>> VARIABLES GLOBALES

	//Variables de apoyo
int i=0,j=0; //varaibles contadores
int temp=0,cont=0; //Varaibles para la dirección de prueba de los motores, función pruebaMotores()
int num_fot=0; //Variable para la función valorFoto()
float valor_foto=0; //Variable para la función valorFoto()
int flag=0;// Esta bandera se usará para modificar el problema con el modo de seguidor de luz :: 0:NO 1:SI

	//Variables paraa guardar información de los tópicos
float datos_Foto[8]={0,0,0,0,0,0,0,0}; //Variable para la función valorFoto()
string dirMotor; //Variable para almacenar la dirección de los motores función dirObtenida()
string dirMotor_l; //Variable para almacenar la dirección en función de los fotoresistores
//_____________________________________________________________________________________________________________________

//>>>>>>>>>>> FUNCIONES <<<<<<<<<<<

	//Obtención del la dirección del nodo SMART THINGS
void dirObtenida(const std_msgs::String::ConstPtr& dirM){
	dirMotor=dirM->data;
		std::cout<<"Instruccion recibida::_"<<dirMotor<<std::endl;
		if(dirMotor!="light"){
			flag=0;	}  }//Fin de dirObtenida

//Analisis de los valores de los Fotoresistores para el seguimiento de luz
void valorFoto(const std_msgs::Float32MultiArray::ConstPtr& dFoto){

	for(i=0;i<8;i++){ //Vaciado de los valores fotoresitores
		datos_Foto[i]=dFoto->data[i];
		std::cout<<"Fotoresitor["<<i<<"]:_ "<<datos_Foto[i]<<std::endl;	}//Fin Vaciado de los valores fotoresitores

	//Tratamiento de los datos:: Obteniedo valor más alto;
	valor_foto=0.0; //reinicio del valor mas alto
	for(i=0;i<8;i++){ //comparación de todos los valores prra saber el más alto

		if(valor_foto<=datos_Foto[i]){ //valor actual mayor
			num_fot=i;
			valor_foto=datos_Foto[i]; }	} //Fin de la busqueda mayor

	std::cout<<"Fuente de luz ubicada en fotoresitor["<<num_fot<<"]:: "<<valor_foto<<std::endl;

	/*switch(num_fot){
			case 0: //Fuente de luz frente al objetivo
					dirMotor_l="Fw";
				break;
			case 1: //Fuente de luz frente derecha al objetivo
					dirMotor_l="FwR";
				break;
			case 2: //Fuente de luz derecha al objetivo
					dirMotor_l="R";
				break;
			case 3: //Fuente de luz atras derecha al objetivo
					dirMotor_l="BwR";
				break;
			case 4: //Fuente de luz detras al objetivo
					dirMotor_l="Bw";
				break;
			case 5: //Fuente de luz atras izquierda al objetivo
					dirMotor_l="BwL";
				break;
			case 6: //Fuente de luz izquierda al objetivo
					dirMotor_l="L";
				break;					
			case 7: //Fuente de luz frente izquierda al objetivo
					dirMotor_l="FwL";
				break;
			default:
					dirMotor_l="stop";
					break;
		}//Fin del switch case
	*/
}//Fin de valorFoto

	//Funcion para el seguimiento de la luz 
void lightSearch(){

	float v_parcialFoto=0.0; //Valor que representa el 70% del valor mas alto de los 8 fotoresistores
	float valor_FotoIzq=0.0,valor_FotoDer=0.0; // Valores para almacenar los valores de los fotoresistores adyacentes
	int mov_temp=8;  //indicará el movimiento del robót que se implementará, está habilitado para la opción por default

	v_parcialFoto = (valor_foto*0.70); //Foto resistores con una diferencia de 10% del valor mas álto
	valor_FotoDer=datos_Foto[num_fot+1];
	valor_FotoIzq=datos_Foto[num_fot-1];
	std::cout<<" moviendose"<<std::endl;

	//Analisis de los datos recibidos

	if(valor_FotoIzq >= v_parcialFoto && valor_FotoDer >= v_parcialFoto){//Caso fuente de luz centrada (1° if)
		mov_temp=num_fot;	}//fin primer if

	//Caso en que la fuente de luz esta inclinada hacia el lado izquierda (2° if)
	else if(valor_FotoIzq>=v_parcialFoto && valor_FotoIzq > valor_FotoDer){
		mov_temp=valor_foto-1;	}//fin segundo if

	//Caso en que la fuente de luz esta inclinada hacia el lado izquierda (3° if)
	else if(valor_FotoDer >= v_parcialFoto && valor_FotoDer > valor_FotoIzq){
		mov_temp=valor_foto+1;	}//fin tercer if

	else{ //Caso en que no se puede verificar correctamente la ubicación de la fuente de luz
	 	mov_temp=8; }


	//Asignando el movimiento del robot
	switch(mov_temp){

					case 0: //Fuente de luz frente al objetivo
							dirMotor_l="Fw";
						break;

					case 1: //Fuente de luz frente derecha al objetivo
							dirMotor_l="FwR";
						break;

					case 2: //Fuente de luz derecha al objetivo
							dirMotor_l="R";
						break;

					case 3: //Fuente de luz atras derecha al objetivo
							dirMotor_l="BwR";
						break;

					case 4: //Fuente de luz detras al objetivo
							dirMotor_l="Bw";
						break;

					case 5: //Fuente de luz atras izquierda al objetivo
							dirMotor_l="BwL";
						break;

					case 6: //Fuente de luz izquierda al objetivo
							dirMotor_l="L";
						break;	

					case 7: //Fuente de luz frente izquierda al objetivo
							dirMotor_l="FwL";	
						break;

					default:
							dirMotor_l="stop";
							break;
	}//Fin del switch case "mov_temp"

}//Fin de la FUNCIÓN LIGHT SEARCH
//_______________________________________________________________________________________________________________
//Función principal
int main(int  argc, char** argv){

	std::cout<<">>>>>LABORATORIO DE BIOROBÓTICA<<<<<<"<<std::endl;
	std::cout<<"_>>XOCHITONAL Smarth Things en linea ..."<<std::endl;
	ros::init(argc,argv,"TeporingoV3");//********************************************
	ros:: NodeHandle n;

    //Obtención de los datos transmitidos por los diferentes nodos 
 	ros::Subscriber subDir = n.subscribe("/hardware/smartThings/dir", 1000,dirObtenida); //Nodo SmartThings
 	ros::Subscriber subFoto = n.subscribe("/hardware/sensors/foto",1000,valorFoto); //Nodo Sensors

 	//Datos a publicar
	std_msgs::String D_Motor; //Dirección del motor	

    //Publicación de las velocidades de los motores al Arduino
    ros::Publisher pubDir=n.advertise <std_msgs::String>("/hardware/motor/direction",1);

	ros::Rate loop(1); //10
    ros::Rate r(10); //1000
	
	while(ros::ok()){

		//Revisión del comando enviado por la app móvi
		if(dirMotor=="light"||flag==1){  //Modo de seguimiento de luz
			std::cout<<"Modo de seguimiento de luz"<<std::endl;
			lightSearch();
			dirMotor=dirMotor_l;
			flag=1;	} //fin del if del modo

		else{//No hay instrucció para el seguimiento de luz
			std::cout<<"Dirección recibida::_"<<dirMotor<<std::endl;	
			flag=0;		}//fin del else

		//Publicación de tópico de las direcciones
        D_Motor.data = dirMotor; 
		std::cout<<D_Motor<<std::endl;
		
		pubDir.publish(D_Motor);  //Envió de las direcciónes al nodo Motor.py

		ros::spinOnce();
		loop.sleep();
        std::cout<<""<<std::endl;

	} //Fin del while(ROS)

return 0;
}//Fin del main principal