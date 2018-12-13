/************************************************************
*	LABORATORIO BIOROBÓTICA 2018
*	Garcés Marín Daniel 		
*	PROYECTO <TEPORINGO>:: Xochitonal
*
*		Programa de Arduino enfocado en dos objetivos principales:
			-Recolectar información de los sensores: Fotoresistencias [8], Encoders [6]
			-Utilizar el módulo SMART THINGS.

*	Ultima versión:: 13 de Agosto del 2018
*********************************************************************/

//DaGaMa_jû-san 

//>>>>CONEXIONES ARDUINO
///   Arduino SmartThings Shield LED Example 
///              ______________
///             |              |
///             |         SW[] |
///             |[]RST         |
///             |         AREF |--
///             |          GND |--
///             |           13 |--X LED
///             |           12 |--
///             |           11 |--
///           --| 3.3V      10 |--
///           --| 5V         9 |--
///           --| GND        8 |--
///           --| GND          |
///           --| Vin        7 |--
///             |            6 |--
///           --| A0         5 |--
///           --| A1    ( )  4 |--
///           --| A2         3 |--X THING_RX
///           --| A3  ____   2 |--X THING_TX
///           --| A4 |    |  1 |--
///           --| A5 |    |  0 |--
///             |____|    |____|
///                  |____|

//__________________________________________________________________________

//>>>LIBRERÍAS
	//ROS
#include <ros.h>
#include <std_msgs/Float32MultiArray.h> 
	//Smart Things
#include <SoftwareSerial.h> 
#include <SmartThings.h>
//_________________________________________________________________________________

//>>>ASIGNACIÓN DE LOS PINES DEL ARDUINO MEGA

  //Entradas Analogicas
int fotoR[8]= {A0,A1,A2,A3,A4,A5,A6,A7};

  //Entradas digitales
//int encoders[12]={};
//--------------------------------------------------------- 
	//Pines Necesarios para el uso del Módulo de Smart THings
#define PIN_THING_RX    3
#define PIN_THING_TX    2

//_________________________________________________________________________________

//>>>VARIABLES GLOBALES
	
	//Almacenamiento de los datos que se enviarán a la Raspberry
float data_arduino[20];
	// [8]::Fotoresistencias -- [12]::Encoders -- [] 

  //Almacenamiento de valores sensados de los fotoresistores
float valor_SL[8]={0.0,0.0,0.0,0.0};

	//Almacenamiento de los valores sensados de los encoders
floar valor_E[12]={0,0,0,0,0,0,0,0,0,0,0,0};
//-------------------------------------------------------------

	//Módulo Smart Things
SmartThingsCallout_t messageCallout;    // call out function forward decalaration
SmartThings smartthing(PIN_THING_RX, PIN_THING_TX, messageCallout);  // constructor

char inChar;
String data;
//------------------------------------------------

  //ROS

std_msgs::Float32MultiArray d_robot;//Crea un arreglo de tipo multif32 de la libreria std_msgs
ros::NodeHandle n;//Crea un nodo en ros llamado n

 //Publica los valores registrados por el arduino para ser leidos po un nodo el la Raspberry
ros::Publisher data_robot("/hardware/arduino/data", &d_robot);

//____________________________________________________________________________________________________________________

//>>>>SETUP
void setup(){ 

	//Serial.begin(115200); //Módulo Smart Things
  n.getHardware()->setBaud(500000);
  
  //Iniciar nodo en ros
  n.initNode(); 
  
  n.advertise(data_robot); 
  d_robot.data_length = 20; //Declara tamaño del arreglo a publicar
    
   //Configuracion de los pines de entrada

  // for(i=0;i<12,i++){
   	//	pinMode(encoders[i],INPUT);    }

} //Fin del SETUP
//_____________________________________________________________________________________________________________________

//>>>>FUNCIONES

void messageCallout(String message){// message received from cloud

  Serial.println(message);	}//Fin de la función messageCallout

void serialEvent(){

  while (Serial.available()){
    inChar = Serial.read();
    delay(2);
    data+=inChar;
      if(Serial.available()==0){
        smartthing.send(data);// send message to cloud
        data="";       }     }    } //Fin de la función serialEvent
//_________________________________________________________________________________________________________________

//>>>>>LOOP
void loop(){

	for(i=0;i<8;i++){
	valor_SL[i]=analogRead(fotoR[i]);
	data_arduino[i]=valor_SL[I];       }//Fin de las lecturas de los fotoresistores

	//for(i=0;i<12;i++){
	//	valor_E[i]=analogRead(encoders[i]);
	//	data_arduino[i+8]=valor_E[i]; 	} //Fin de las lecturas de los enconders

	d_robot.data=data_arduino;

	// run smartthing logic
  	smartthing.run();

	//Publicación de la DATA general del arduino
  	data_robot.publish(&d_robot);
	n.spinOnce();  //Publica los datos de mi arreglo a mi topico

	
}//Fin del LOOP

