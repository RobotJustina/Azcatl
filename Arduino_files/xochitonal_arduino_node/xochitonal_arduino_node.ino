/************************************************************
*	LABORATORIO BIOROBÓTICA 2018
*	Garcés Marín Daniel 		
*	PROYECTO <TEPORINGO>:: Xochitonal FASE 2 // Iteracion GAMMA
*
*		Programa de Arduino enfocado en dos objetivos principales:
			-Recolectar información de los sensores: Fotoresistencias [8]
			-Muestreo de los Encoders [6] por medio de interrupciones.
                        -Realiza un analisis de control PID
*	Ultima versión:: 8 de Noviembre del 2018
*********************************************************************/
//DaGaMa_jû-san 

//LIBRERÍAS
#include <ros.h>
#include <std_msgs/Float32MultiArray.h> 
//-----------------------------------------------

//ASIGNACIÓN DE LOS PINES DEL ARDUINO MEGA

  //Entradas Analogicas
int fotoR[8]= {A0,A1,A2,A3,A4,A5,A6,A7};

  //Control Motores
#define PWM_MotorD 6 //Control de velocidad de los motores Derechos
#define PWM_MotorI 7 //Control de velocidad de los motores Izquierdos

#define MotorD_A 52 //Control de direccion de los motores Derechos A
#define MotorD_B 53 //Control de direccion de los motores Derechos B
#define MotorI_A 50 //Control de direccion de los motores Izquierdos A
#define MotorI_B 51 //Control de direccion de los motores Izquierdos B

  //ENCODERS
//Motor 1 -- Enfrente Derecha
#define Enc_M1A 2//Interrup0
#define Enc_M1B 4
//Motor 2 -- Enfrente Izquierda
#define Enc_M2A 3//Interrup1
#define Enc_M2B 5
//Motor 3 -- Medio Derecha
#define Enc_M3A 18//Interrup 5
#define Enc_M3B 17
//Motor 4 -- Medio Izquierda
#define Enc_M4A 19//Interrup4
#define Enc_M4B 22
 //Motor 5 --  Atras Derecha
#define Enc_M5A 20//Interrup3
#define Enc_M5B 23
//Motor 6 -- Atras Izquierda
#define Enc_M6A 21//Interrup2
#define Enc_M6B 24

//----------------------------------------------------------------------------------
//VARIABLES GLOBALES

  //Variables control motores
 int MD_A=0; //Valor de dirección del MotorD_A
 int MD_B=0; //Valor de dirección del MotorD_B
 int MI_A=0; //Valor de dirección del MotorI_A
 int MI_B=0; //Valor de dirección del MotorI_B
 int pwm_D = 0; //Valor de dirección del Motor_D
 int pwm_I = 0; //Valor de dirección del Motor_I

  //Variables Topicos
float valor_SL[8]={0.0,0.0,0.0,0.0};  //Almacenamiento de valores sensados de los fotoresistores
int vueltas_Enc[6]={0,0,0,0,0,0}; //Alemacenamiento de la vueltas dadas con los enconders
	
  //Almacenamiento de los datos que se enviarán a la Raspberry
float data_arduino[14];
  // [8]::Fotoresistencias + [6] Datos encoders

//----------------------------------------------------------------------------------------
//ENCODERS Y PID

  //Alemacenaiento de pulsos de cada motor
volatile unsigned long ContM1=0;
volatile unsigned long ContM2=0;
volatile unsigned long ContM3=0;
volatile unsigned long ContM4=0;
volatile unsigned long ContM5=0;
volatile unsigned long ContM6=0;

//---------------------------------------------------------------------------------------------------
  //ROS

//Subscripcion al topico de direcciones y velocidades
void SpeedsCallback(const std_msgs::Float32MultiArray&msg){
  
  if (msg.data[0] > 0 && msg.data[1]>0){ //Adelante
    MD_A = 0;
    MD_B = 1;
    pwm_D = int(msg.data[0]);
    MI_A = 0;
    MI_B = 1;
    pwm_I = int(msg.data[1]);       }
  
  else if (msg.data[0] < 0 && msg.data[1] < 0){ //Atras
    MD_A = 1;
    MD_B = 0;
    pwm_D = int(-1 * msg.data[0]);
    MI_A = 1;
    MI_B = 0;
    pwm_I = int(-1 * msg.data[1]);          }

  else if (msg.data[0] > 0 && msg.data[1] < 0){ //Giro Derecho
    MD_A = 0;
    MD_B = 1;
    pwm_D = int(msg.data[0]);
    MI_A = 1;
    MI_B = 0;
    pwm_I = int(-1 * msg.data[1]);          }

  else if (msg.data[0] < 0 && msg.data[1]>0){  //Giro Izquierda
    MD_A = 1;
    MD_B = 0;
    pwm_D = int(-1 * msg.data[0]);
    MI_A = 0;
    MI_B = 1;
    pwm_I = int(msg.data[1]);          }

    else {  //ALTO
    MD_A = 0;
    MD_B = 0;
    MI_A = 0;
    MI_B = 0; }

}// Fin de SpeedsCallBack

//----------------------------------------------------------------------------------------------------

std_msgs::Float32MultiArray d_robot;//Crea un arreglo de tipo multif32 de la libreria std_msgs
ros::NodeHandle n;//Crea un nodo en ros llamado n

  //Se subscribe al topico publicado en la raspbery para el control de los motores
ros::Subscriber<std_msgs::Float32MultiArray> robot_Speeds("/hardware/motor/direction",&SpeedsCallback);
 //Publica los valores registrados por el arduino para ser leidos po un nodo el la Raspberry
ros::Publisher data_robot("/hardware/arduino/data", &d_robot);

//____________________________________________________________________________________________________________________

	//SETUP
void setup(){ 

  n.getHardware()->setBaud(500000);
  
  //Iniciar nodo en ros
  n.initNode(); 
  
  n.advertise(data_robot); 
  n.subscribe(robot_Speeds); // Subscribe al topico de los movimientos
  d_robot.data_length = 14; //Declara tamaño del arreglo a publicar [8]:FotoR, [6]: Enc
   
   //Configuracin de los pines para los Encoders
  pinMode(Enc_M1A,INPUT);
  pinMode(Enc_M1B,INPUT);
  pinMode(Enc_M2A,INPUT);
  pinMode(Enc_M2B,INPUT);
  pinMode(Enc_M3A,INPUT);
  pinMode(Enc_M3B,INPUT);
  pinMode(Enc_M4A,INPUT);
  pinMode(Enc_M4B,INPUT);
  pinMode(Enc_M5A,INPUT);
  pinMode(Enc_M5B,INPUT);
  pinMode(Enc_M6A,INPUT);
  pinMode(Enc_M6B,INPUT);
    
  //Configuracion Control Motores
  pinMode(PWM_MotorD,OUTPUT);
  pinMode(PWM_MotorI,OUTPUT);
  pinMode(MotorD_A,OUTPUT);
  pinMode(MotorD_B,OUTPUT);
  pinMode(MotorI_A,OUTPUT);
  pinMode(MotorI_A,OUTPUT);
  
  //Configuracion de las interrupciones
  attachInterrupt(0,encoderM1aEvent,CHANGE); //Int0 =Enc_M1A = pin2
  attachInterrupt(1,encoderM2aEvent,CHANGE); //Int1 =Enc_M2A = pin3
  attachInterrupt(5,encoderM3aEvent,CHANGE); //Int5 =Enc_M3A = pin18
  attachInterrupt(4,encoderM4aEvent,CHANGE); //Int4 =Enc_M4A = pin19
  attachInterrupt(3,encoderM5aEvent,CHANGE); //Int3 =Enc_M5A = pin20
  attachInterrupt(2,encoderM6aEvent,CHANGE); //Int2 =Enc_M6A = pin21
  
} //Fin del SETUP
//_____________________________________________________________________________________________________________________

//>>>>>>>>>>>>>>>>>>>>>>>>> FUNCIONES

void encoderM1aEvent(){ //MOTOR 1
  if(digitalRead(Enc_M1A)==HIGH)  {
    if(digitalRead(Enc_M1B)==LOW){
      ContM1++;                 }//Fin if anidado A=1&B=0
    else //if anidado A=1&B=1
      ContM1--;                    }//Fin primer if A=1
      
  else{
    if(digitalRead(Enc_M1B)==LOW)  {
      ContM1--;                 }//Fin if anidado A=0&B=0
      
    else //if anidado A=0&B=1
      ContM1++;                    }//Fin segundo if A=0
}//Fin encoderM1aEvent
//--------------------------------------------------------------

void encoderM2aEvent(){ //MOTOR 2
  if(digitalRead(Enc_M2A)==HIGH)  {
    if(digitalRead(Enc_M2B)==LOW){
      ContM2++;                 }//Fin if anidado A=1&B=0
    else //if anidado A=1&B=1
      ContM2--;                    }//Fin primer if A=1
      
  else{
    if(digitalRead(Enc_M2B)==LOW)  {
      ContM2--;                 }//Fin if anidado A=0&B=0
      
    else //if anidado A=0&B=1
      ContM2++;                    }//Fin segundo if A=0
}//Fin encoderM2aEvent
//--------------------------------------------------------------

void encoderM3aEvent(){ //MOTOR 3
  if(digitalRead(Enc_M3A)==HIGH)  {
    if(digitalRead(Enc_M3B)==LOW){
      ContM3++;                 }//Fin if anidado A=1&B=0
    else //if anidado A=1&B=1
      ContM3--;                    }//Fin primer if A=1
      
  else{
    if(digitalRead(Enc_M3B)==LOW)  {
      ContM3--;                 }//Fin if anidado A=0&B=0
      
    else //if anidado A=0&B=1
      ContM3++;                    }//Fin segundo if A=0
}//Fin encoderM3aEvent
//--------------------------------------------------------------

void encoderM4aEvent(){ //MOTOR 4
  if(digitalRead(Enc_M4A)==HIGH)  {
    if(digitalRead(Enc_M4B)==LOW){
      ContM4++;                 }//Fin if anidado A=1&B=0
    else //if anidado A=1&B=1
      ContM4--;                    }//Fin primer if A=1
      
  else{
    if(digitalRead(Enc_M4B)==LOW)  {
      ContM4--;                 }//Fin if anidado A=0&B=0
      
    else //if anidado A=0&B=1
      ContM4++;                    }//Fin segundo if A=0
}//Fin encoderM4aEvent
//--------------------------------------------------------------

void encoderM5aEvent(){ //MOTOR 5
  if(digitalRead(Enc_M5A)==HIGH)  {
    if(digitalRead(Enc_M5B)==LOW){
      ContM5++;                 }//Fin if anidado A=1&B=0
    else //if anidado A=1&B=1
      ContM5--;                    }//Fin primer if A=1
      
  else{
    if(digitalRead(Enc_M5B)==LOW)  {
      ContM5--;                 }//Fin if anidado A=0&B=0
      
    else //if anidado A=0&B=1
      ContM5++;                    }//Fin segundo if A=0
}//Fin encoderM5aEvent
//--------------------------------------------------------------

void encoderM6aEvent(){ //MOTOR 6
  if(digitalRead(Enc_M6A)==HIGH)  {
    if(digitalRead(Enc_M6B)==LOW){
      ContM6++;                 }//Fin if anidado A=1&B=0
    else //if anidado A=1&B=1
      ContM6--;                    }//Fin primer if A=1
      
  else{
    if(digitalRead(Enc_M6B)==LOW)  {
      ContM6--;                 }//Fin if anidado A=0&B=0
      
    else //if anidado A=0&B=1
      ContM6++;                    }//Fin segundo if A=0
}//Fin encoderM3aEvent
//--------------------------------------------------------------

//_____________________________________________________________________________________________________________________
//>>>>> LOOP
void loop(){

  //TOMA DE DATOS DE LOS SENSORES Y LOS ENCODERS
  for(int i=0;i<8;i++){
     valor_SL[i]=analogRead(fotoR[i]);
      data_arduino[i]=valor_SL[i];  //Asignacin al arreglo del topico a publicar
   }//Fin de las lecturas de los fotoresistores

  //Asginando los valores de la posicion de las llantas por el momento
  vueltas_Enc[0]=float(ContM1);
  vueltas_Enc[1]=float(ContM2);
  vueltas_Enc[2]=float(ContM3);
  vueltas_Enc[3]=float(ContM4);
  vueltas_Enc[4]=float(ContM5);
  vueltas_Enc[5]=float(ContM6);
        
  //Enviando los datos de los encoders a la raspberry
  for(int j=0; j<6;j++){       
    data_arduino[j+8]=vueltas_Enc[j];  //Asignacin al arreglo del topico a publicar
  }//Fin de la posicion de las llantas

  //PROCESO DE LA PUBLICACION DE TOPICO      
  d_robot.data=data_arduino;//Se le asignan los datos recolectados al topico que se va a publicar

  //Publicación de la DATA general del arduino
  data_robot.publish(&d_robot);
  n.spinOnce();  //Publica los datos de mi arreglo a mi topico

  //Direcciones y velocidad de lo motores
  digitalWrite(MotorD_A,MD_A); //Motores Derechos
  digitalWrite(MotorD_B,MD_B);
  analogWrite(PWM_MotorD, pwm_D);
  digitalWrite(MotorI_A,MI_A);  //Motores Izquierda
  digitalWrite(MotorI_B,MI_B);
  analogWrite(PWM_MotorI, pwm_I);
}//Fin del LOOP

