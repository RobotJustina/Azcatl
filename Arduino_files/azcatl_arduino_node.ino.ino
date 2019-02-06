/************************************************************
*  LABORATORIO BIOROBÓTICA 2018
* Garcés Marín Daniel     
* PROYECTO <ÁZCATL>
*
*   Programa de Arduino enfocado en dos objetivos principales:
      -Recolectar información de los sensores: Fotoresistencias [8]; Sensores infrarrojos de distancia [2]
      -Muestreo de los Encoders [6] por medio de interrupciones.
         -Realiza un analisis de control PID
* Ultima versión:: 22 de Enero del 209
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
#define PWM_MotorD 7 //Control de velocidad de los motores Derechos
#define PWM_MotorI 6 //Control de velocidad de los motores Izquierdos

#define MotorD_A 50 //Control de direccion de los motores Derechos A
#define MotorD_B 51 //Control de direccion de los motores Derechos B
#define MotorI_A 52 //Control de direccion de los motores Izquierdos A
#define MotorI_B 53 //Control de direccion de los motores Izquierdos B

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

//Sensores de contacto digitales
//Sensor derecho
#define Cont_Der 28 //Sensor de contacto derecho
//Sensor izquierdo
#define Cont_Izq 30 //Sensor de contacto derecho

//Constantes del motor
//Máximos y minimos valores de RPMs que puede conseguir el motor
#define maxPID 66
#define minPID 13
//Máximos y minimos valores de PWM que puede conseguir el motor
#define maxPWM 255
#define minPWM 17


//----------------------------------------------------------------------------------
//VARIABLES GLOBALES

  //Variables control motores
 int MD_A=0; //Valor de dirección del MotorD_A
 int MD_B=0; //Valor de dirección del MotorD_B
 int MI_A=0; //Valor de dirección del MotorI_A
 int MI_B=0; //Valor de dirección del MotorI_B
 int pwm_D = 0; //Valor de dirección del Motor_D
 int pwm_I = 0; //Valor de dirección del Motor_I
 float RPM_der = 0; //Revoluciones por minuto calculadas
 float RPM_izq = 0; //Revoluciones por minuto calculadas
 float velocidad_der = 0; //Velocidad en [m/s]
 float velocidad_izq = 0; //Velocidad en [m/s]

  int cambioderecha = 0;
  float velocidadder = 0;
  int cambioizquierda = 0;
  float velocidadizq = 0;
 //-----------------------

  //Variables Topicos
float valor_SL[8]={0.0,0.0,0.0,0.0};  //Almacenamiento de valores sensados de los fotoresistores
int vueltas_Enc[6]={0,0,0,0,0,0}; //Alemacenamiento de la vueltas dadas con los enconders
int obstaculos[2]={0,0}; //Almacenamiento de un valor binario para detectar la presencia de obstaculos
  
  //Almacenamiento de los datos que se enviarán a la Raspberry
float data_arduino[16];
  // [8]::Fotoresistencias + [6] Datos encoders + [2] Detector obstaculos

//----------------------------------------------------------------------------------------
//ENCODERS Y PID

  //Alemacenaiento de pulsos de cada motor
volatile long ContM1=0;
volatile long ContM2=0;
volatile long ContM3=0;
volatile long ContM4=0;
volatile long ContM5=0;
volatile long ContM6=0;

//---------------------------------------------------------------------------------------------------
 //Variables para calcular la velocidad
unsigned long timeold = 0;       // Tiempo
unsigned int pulsesperturn = 3267; // Número de pulsos por vuelta del motor, por canal = 3267.
float wheel_diameter = (123.40)/1000;// Diámetro de la rueda pequeña[mm]
int ratio = 1;                  // Relación de transmisión 
//---------------------------------------------------------------------------------------------------

//Definición de las constanytes para el PID
float error_D = 0;
float errorAnterior_D = 0; 
float P_D = 0;//Acción Proporcional
float I_D = 0;//Acción integral
float D_D = 0;//Acción /DErivativa
float Kp_D = 0.4; //Constante proporcional
float Ki_D = 0.025; //Constante integral
float Kd_D = 2.9;//Constante derivativa


float error_I = 0;
float errorAnterior_I = 0; 
float P_I = 0;//Acción Proporcional
float I_I = 0;//Acción integral
float D_I = 0;//Acción /DErivativa
float Kp_I = 0.4; //Constante proporcional
float Ki_I = 0.025; //Constante integral
float Kd_I = 2.9;//Constante derivativa

//----------------------------------------------------------------------------------------
//Definición de las señales de error, control y la salida del sistema.
float r_t_D = 0 ; //[m/s]
float r_t_I = 0 ; //[m/s]
float r_tder = 0;//[m/s]
float r_tizq = 0;//[m/s]
volatile float y_t_D = 0;
volatile float y_t_I = 0;
float ut_D = 0;
float ut_I = 0;
//----------------------------------------------------------------------------------------

//Tiempo del ciclo del PID
float cycleTime = 200; //ms
///Tiempo del ciclo del PID en segundo
//¡NO MODIFICAR, SE CALCULA SOLA!
float cycleTimeSeconds = 0; //s  // Variable auxiliar del timepo del ciclo del PID
//----------------------------------------------------------------------------------------


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
  d_robot.data_length = 16; //Declara tamaño del arreglo a publicar [8]:FotoR, [6]: Enc
   
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
  attachInterrupt(0,encoderM1aEvent,RISING); //Int0 =Enc_M1A = pin2
  attachInterrupt(1,encoderM2aEvent,RISING); //Int1 =Enc_M2A = pin3
  attachInterrupt(5,encoderM3aEvent,RISING); //Int5 =Enc_M3A = pin18
  attachInterrupt(4,encoderM4aEvent,RISING); //Int4 =Enc_M4A = pin19
  attachInterrupt(3,encoderM5aEvent,RISING); //Int3 =Enc_M5A = pin20
  attachInterrupt(2,encoderM6aEvent,RISING); //Int2 =Enc_M6A = pin21
  
  //Detectando obstaculos
   pinMode(Cont_Der,INPUT);
   pinMode(Cont_Izq,INPUT);

//Inicialización de parámetros
  ContM1 = 0;
  ContM2 = 0;
  ContM3 = 0;
  ContM4 = 0;
  ContM5 = 0;
  ContM6 = 0;
  RPM_der = 0;  
  RPM_izq = 0;  
  timeold = 0;  
  cycleTimeSeconds = cycleTime / 1000;
  //Serial.begin(9600);
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

void calculoVelocidad()
{
  if (millis() - timeold >= 1000)
  {  // Uptade every one second, this will be equal to reading frecuency (Hz). // Se actualiza cada segundo
      noInterrupts(); //Don't process interrupts during calculations // Desconectamos la interrupción para que no actué en esta parte del programa.
      //***************************************************************************************
      RPM_der = 120 * abs(ContM4) / pulsesperturn * 1000 / (millis() - timeold) ; // Note that this would be 60*1000/(millis() - timeold)*pulses if the interrupt. happened once per revolution // Calculamos las revoluciones por minuto
      //Ojo con la fórmula de arriba, la variable rpm tiene que ser tipo float porque salen decimales en medio de la operación.
      //velocidad_der = RPM_der *3.1416 * wheel_diameter  / 60; // Cálculo de la velocidad de la rueda en [Km/h]
      ContM4 = 0;  // Inicializamos los pulsos.
      
      //***************************************************************************************
      RPM_izq = 120 * abs(ContM3) / pulsesperturn * 1000 / (millis() - timeold) ; // Note that this would be 60*1000/(millis() - timeold)*pulses if the interrupt. happened once per revolution // Calculamos las revoluciones por minuto
      //Ojo con la fórmula de arriba, la variable rpm tiene que ser tipo float porque salen decimales en medio de la operación.
      //velocidad_izq = RPM_izq *3.1416 * wheel_diameter  / 60; // Cálculo de la velocidad de la rueda en [Km/h]
      ContM3 = 0;  // Inicializamos los pulsos.
      timeold = millis(); // Almacenamos el tiempo actual.
      interrupts(); // Restart the interrupt processing // Reiniciamos la interrupción
      
  }
}
void pid()
{
  //****************PID DERECHO****************************
  y_t_D = RPM_der;
  r_t_D =velocidadder*60/(3.1416 * wheel_diameter);
  error_D = r_t_D - y_t_D;//Cálculo del error
  P_D = Kp_D * error_D;//Acción proporcional
  I_D += Ki_D * error_D * cycleTimeSeconds;// /Calculo acción integrativa
  D_D = Kd_D * (error_D - errorAnterior_D) / cycleTimeSeconds;//cálculo acción derivativa
  ut_D = P_D + I_D + D_D;//suma de las tres acciones para obtener la señal de control
  //Si las RPMs (ut) de salida, son mas grandes que el máximo al que puede girar el motor
  //entonces se asigna el máximo
  if (ut_D > maxPID) ut_D = maxPID;
  //Si las RPMs (ut) de salida, son mas pequeñas que el mínimo al que puede girar el motor (0 RPMs)
  //entonces se asigna el mínimo
  if (ut_D < minPID) ut_D = minPID;
  errorAnterior_D = error_D;

    //****************PID IZQUIERDO****************************
  y_t_I = RPM_izq;
  r_t_I =velocidadizq*60/(3.1416 * wheel_diameter);
  error_I = r_t_I - y_t_I;//Cálculo del error
  P_I = Kp_I * error_I;//Acción proporcional
  I_I += Ki_I * error_I * cycleTimeSeconds;// /Calculo acción integrativa
  D_I = Kd_I * (error_I - errorAnterior_I) / cycleTimeSeconds;//cálculo acción derivativa
  ut_I = P_I + I_I + D_I;//suma de las tres acciones para obtener la señal de control
  //Si las RPMs (ut) de salida, son mas grandes que el máximo al que puede girar el motor
  //entonces se asigna el máximo
  if (ut_I > maxPID) ut_I = maxPID;
  //Si las RPMs (ut) de salida, son mas pequeñas que el mínimo al que puede girar el motor (0 RPMs)
  //entonces se asigna el mínimo
  if (ut_I < minPID) ut_I = minPID;
  errorAnterior_I = error_I;
}
  void rpmToPwm()
  {
  //Convertimos el valor de RPMs a PWM
  pwm_I = map(ut_I, minPID, maxPID, minPWM, maxPWM);
  pwm_D = map(ut_D, minPID, maxPID, minPWM, maxPWM);
  }

  void elecciongiro()
  {
        if (r_tder < 0)
  {
    MD_B = 1;
    MD_A = 0;
  }
  else
  {
    MD_A = 1;
    MD_B = 0;
  }
  
  if (r_tizq < 0)
  {
    MI_A = 1;
    MI_B = 0;
  }
  else
  {
    MI_B = 1;
    MI_A = 0;
  }
  
}
  void cambiovelocidades()
  {
    if (Serial.available())
    {
    cambioderecha = Serial.read();
    
    if(cambioderecha==112)
    r_tder = r_tder + 0.01;
    if(cambioderecha==109)
     r_tder = r_tder - 0.01;
    if(r_tder > 0.45)
    r_tder = 0.45;
    if(r_tder < - 0.45)
    r_tder = -0.45;
    }
    //***************************************************************************************
  
    if (Serial.available())
    {
    cambioizquierda = Serial.read();
    if(cambioizquierda==43)
    r_tizq = r_tizq + 0.01;
    if(cambioizquierda==45)
     r_tizq = r_tizq - 0.01;
    if(r_tizq > 0.45)
    r_tizq = 0.45;
    if(r_tizq < - 0.45)
    r_tizq = -0.45;
    }
      velocidadder = abs(r_tder);
     velocidadizq = abs(r_tizq);
   }
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
     
  //Leyendo sensores de contacto para detectar obstaculos
  obstaculos[1]=digitalRead(Cont_Der);
  obstaculos[2]=digitalRead(Cont_Izq);
  
  //Enviando los datos de los encoders a la raspberry
  for(int j=0; j<6;j++){       
    data_arduino[j+8]=vueltas_Enc[j];  //Asignacin al arreglo del topico a publicar
  }//Fin de la posicion de las llantas
  
  for(int k=0; k<2; k++)  {
  data_arduino[k+14]=obstaculos[k]; //Asignación de la presencia de obstaculos al arreglo del tópico
  }//Fin de la presencia de obstaculos
  
  //PROCESO DE LA PUBLICACION DE TOPICO      
  d_robot.data=data_arduino;//Se le asignan los datos recolectados al topico que se va a publicar

  //Publicación de la DATA general del arduino
  data_robot.publish(&d_robot);
  n.spinOnce();  //Publica los datos de mi arreglo a mi topico

//***********CONTROL**************************
  cambiovelocidades();  
  elecciongiro();
  calculoVelocidad();
  pid();
  rpmToPwm();


  //Direcciones y velocidad de lo motores
  digitalWrite(MotorD_A,MD_A); //Motores Derechos
  digitalWrite(MotorD_B,MD_B);
  digitalWrite(MotorI_A,MI_A);  //Motores Izquierda
  digitalWrite(MotorI_B,MI_B);


  analogWrite(PWM_MotorD, pwm_D);
  analogWrite(PWM_MotorI, pwm_I);
  
  //Serial.println(ContM4);
  
}//Fin del LOOP
