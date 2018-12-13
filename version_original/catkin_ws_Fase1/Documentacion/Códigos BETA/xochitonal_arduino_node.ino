/************************************************************
*	LABORATORIO BIOROBÓTICA 2018
*	Garcés Marín Daniel 		
*	PROYECTO <TEPORINGO>:: Xochitonal
*
*		Programa de Arduino enfocado en dos objetivos principales:
			-Recolectar información de los sensores: Fotoresistencias [8], Encoders [6]
			-Utilizar el módulo SMART THINGS.

*	Ultima versión:: 8 de Agosto del 2018
*********************************************************************/

//DaGaMa_jû-san 

//LIBRERÍAS
#include <ros.h>
#include <std_msgs/Float32MultiArray.h> 

//-----------------------------------------------

//ASIGNACIÓN DE LOS PINES DEL ARDUINO MEGA

  //Entradas Analogicas
int fotoR[8]= {A0,A1,A2,A3,A4,A5,A6,A7};

  //Entradas digitales
//int encoders[12]={}; 

//-----------------------------------------------
//VARIABLES GLOBALES
	
	//Almacenamiento de los datos que se enviarán a la Raspberry
float data_arduino[20];
	// [8]::Fotoresistencias -- [12]::Encoders -- [] 

  //Almacenamiento de valores sensados de los fotoresistores
float valor_SL[8]={0.0,0.0,0.0,0.0};

	//Almacenamiento de los valores sensados de los encoders
float valor_E[12]={0,0,0,0,0,0,0,0,0,0,0,0};

//------------------------------------------------
  //ROS

std_msgs::Float32MultiArray d_robot;//Crea un arreglo de tipo multif32 de la libreria std_msgs
ros::NodeHandle n;//Crea un nodo en ros llamado n

 //Publica los valores registrados por el arduino para ser leidos po un nodo el la Raspberry
ros::Publisher data_robot("/hardware/arduino/data", &d_robot);

//____________________________________________________________________________________________________________________
	//SETUP
void setup(){ 

  n.getHardware()->setBaud(57600);
  
  //Iniciar nodo en ros
  n.initNode(); 
  
  n.advertise(data_robot); 
  d_robot.data_length = 20; //Declara tamaño del arreglo a publicar
    
   //Configuracion de los pines de entrada

  // for(i=0;i<12,i++){
   	//	pinMode(encoders[i],INPUT);    }

} //Fin del SETUP
//_____________________________________________________________________________________________________________________

//LOOP
void loop(){

	for(int i=0;i<8;i++){
	valor_SL[i]=analogRead(fotoR[i]);
	data_arduino[i]=valor_SL[i];       }//Fin de las lecturas de los fotoresistores

	//for(i=0;i<12;i++){
	//	valor_E[i]=analogRead(encoders[i]);
	//	data_arduino[i+8]=valor_E[i]; 	} //Fin de las lecturas de los enconders

	d_robot.data=data_arduino;

	//Publicación de la DATA general del arduino
  	data_robot.publish(&d_robot);
	n.spinOnce();  //Publica los datos de mi arreglo a mi topico

	
}//Fin del LOOP

