/************************************************************
*	LABORATORIO BIOROBÓTICA 2018
*	Garcés Marín Daniel 	
*
*	PROYECTO <Xochitonal> :: V2 
*		Nodo que se encarga del manejo de los datos recibidos del arduino para separarlos y publicarlos en tópicos diferentes.
*			Datos disponibles a trasnimitir: Valores de los Fotoresistores, Valores de los encoders
*			[!!] Tiene deshabilidato la trasnmisición de los datos de los Encoders
*	
*	Ultima versión:: 12 de Diciembre del 2018
*********************************************************************/

//>>>>>>>> LIBRERÍAS <<<<<<<<<<<<<
#include "ros/ros.h"
// ダ・ガ・マ・jû-san
#include "std_msgs/Float32MultiArray.h"
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <termio.h>
#include <sys/ioctl.h>
//----------------------------------------

	//>>>VARIABLES GLOBALES
int i=0, j=0; //varaibles para contadores
float val_Foto[8]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; //Valores de los fotoresistores
//float val_Enc[6]={0.0,0.0,0.0,0.0,0.0,0.0}; //Valores de los enconders

//_____________________________________________________________________
	//>>>FUNCIONES

void callbackArduino(const std_msgs::Float32MultiArray::ConstPtr& dataArduino){

	for(i=0;i<8;i++){ //Vaciando los datos de los fotoresistores
		val_Foto[i]=dataArduino->data[i]; 
		//std::cout<<"Fotoresitor-"<<i<<":_ "<<val_Foto[i]<< std::endl;
			}  //Fin del vaciado fotoresistores

	/*for(i=0;i<6;i++){//Vaciado de los datos de los encoders
		val_Enc[i]=dataArduino->data[i+8]; 
		std::cout<<"Encoder-"<<i<<":_ "<<val_Enc[i]<< std::endl;	} //fin del vaciado encoders
	*/
}//fin del callbackArduino

//_________________________________________________________________________

	//>>>FUNCIÓN MAIN PRINCIPAL
int main(int  argc, char** argv){

	//std::cout<<"     >>>>>LABORATORIO DE BIOROBÓTICA<<<<<<"<<std::endl;
	//std::cout<<">_Xochitonal en línea obteniendo datos de los sensores"<<std::endl;
	std::cout<<">_Recolectando datos...."<<std::endl;

	//_>Inicialiación del nodo de ROS); //Publicar datos enconders

	ros::init(argc,argv,"XochitonalV2");
	ros:: NodeHandle n;

	//_>Obtención de los datos proporcionados por el arduino
 	ros::Subscriber subArd = n.subscribe("/hardware/arduino/data", 1000, callbackArduino); //Obtener los datos de lso sensores

	//_>Variables a publicar
	std_msgs::Float32MultiArray data_foto; //Datos de los fotoresistores
	data_foto.data.resize(8);	

	//std_msgs::Float32MultiArray data_enc; //Datos de los encoders
	//data_enc.data.resize(6);

	//_>Tópicos que se van a publicar con la información por separado
	ros::Publisher pubFoto=n.advertise <std_msgs::Float32MultiArray> ("/hardware/sensors/foto",10); //Publicar datos fotoresistores
	//ros::Publisher pubEnc=n.advertise <std_msgs::Float32MultiArray> ("/hardware/sensors/enc",100); //Publicar datos enconders

	ros::Rate loop(10);
    ros::Rate r(1000);
	
	//Ciclo ROS
	while(ros::ok()){

		//Asignación de los valores para publicar
		for(i=0;i<8;i++){
		data_foto.data[i]=val_Foto[i];} //Fotoresitores
		//for(i=0;i<6;i++){
		//data_enc.data[i]=val_Enc[i];} //Encoders

		//Publicación de los tópicos
		pubFoto.publish(data_foto);
		//pubEnc.publish(data_enc);

		ros::spinOnce();
		loop.sleep();
        std::cout<<""<<std::endl;

	} //Fin del while(ROS)

}//Fin del MAIN 