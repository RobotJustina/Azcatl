#include <ros.h>
#include <std_msgs/Float32MultiArray.h> 

#define PWM_MotorD 7 //Control de velocidad de los motores Derechos
#define PWM_MotorI 6 //Control de velocidad de los motores Izquierdos

#define MotorD_A 50 //Control de direccion de los motores Derechos A
#define MotorD_B 51 //Control de direccion de los motores Derechos B
#define MotorI_A 48 //Control de direccion de los motores Izquierdos A
#define MotorI_B 49 //Control de direccion de los motores Izquierdos B

//Enconders
//Motor 3 -- Medio Derecha
#define Enc_M3A 18//Interrup 5
#define Enc_M3B 17
//Motor 4 -- Medio Izquierda
#define Enc_M4A 19//Interrup4
#define Enc_M4B 22

//Alemacenaiento de pulsos de cada motor
volatile long ContM3=0;
volatile long ContM4=0;


//Variables para calcular la 
float pi = 3.1416;
float diameter = 0.1234; // diameter in m
float dist_wheel = 0.3; //longitud in m 
float resolution =  3267.0; //pulses per turn
float factor = 2*pi*diameter/resolution; //Converter factor
float dist_wheel_D = 0.0;
float dist_wheel_I = 0.0;
float dist_center_robot = 0.0;
bool spin=false;
bool straight=false;
int m3_aux, m4_aux = 0;


float data[9]={0,0,0,0,0,0,0,0,1};

float angle=0;
float dist=0;



void simpleMoveCallback(const std_msgs::Float32MultiArray& msg){
  
  if(msg.data_length < 2){
    return;
  }else{
    angle=msg.data[0];
    dist=msg.data[1];
    spin=true;
    straight=true;
    data[8]=0.0;
    ContM3 = 0;
    ContM4 = 0;
  }

  
}

//---------------------------------------------------------
void encoderM3aEvent(){ //MOTOR 3

      ContM3++;
}
//--------------------------------------------------------------

void encoderM4aEvent(){ //MOTOR 4
  
      ContM4++;  
}
//--------------------------------------------------------------



void odometry(float ang_goal,float goal){
  
  float dist_rest=0;
  float dist_ang=0.0;
  int delta_izq=abs(m3_aux-ContM3);
  int delta_der=abs(m4_aux-ContM4);

  dist_wheel_D = factor * delta_der;
  dist_wheel_I = factor * delta_izq;
  dist_center_robot = (dist_wheel_D + dist_wheel_I)/2;

  if(spin){
    if(ang_goal>0){
      digitalWrite(MotorD_A,HIGH);  //Motores Derecha
      digitalWrite(MotorD_B,LOW);
    
      digitalWrite(MotorI_A,HIGH);  //Motores Izquierda
      digitalWrite(MotorI_B,LOW);
    }
    else{   
      
      digitalWrite(MotorD_A,LOW);  //Motores Derecha
      digitalWrite(MotorD_B,HIGH);
    
      digitalWrite(MotorI_A,LOW);  //Motores Izquierda
      digitalWrite(MotorI_B,HIGH);
      
    }
    dist_ang=fabs(ang_goal)*0.4*pi/360;
    if(dist_center_robot<dist_ang){
   
      //dist_rest=goal-dist_center_robot;
      analogWrite(PWM_MotorD, 120);
      analogWrite(PWM_MotorI, 120);
    }
    else{
      analogWrite(PWM_MotorD, 0);
      analogWrite(PWM_MotorI, 0);

      dist_wheel_D = 0.0;
      dist_wheel_I = 0.0;
      dist_center_robot = 0.0;
      spin=false;
    }
  }else if(!spin && straight){

    if(goal>0){

      digitalWrite(MotorD_A,HIGH);  //Motores Derecha
      digitalWrite(MotorD_B,LOW);
    
      digitalWrite(MotorI_A,LOW);  //Motores Izquierda
      digitalWrite(MotorI_B,HIGH);
      
    }else{
      digitalWrite(MotorD_A,LOW);  //Motores Derecha
      digitalWrite(MotorD_B,HIGH);
    
      digitalWrite(MotorI_A,HIGH);  //Motores Izquierda
      digitalWrite(MotorI_B,LOW);
    }
    
  
    if(dist_center_robot<fabs(goal)-0.05){
      
      dist_rest=fabs(goal)-dist_center_robot;
//      analogWrite(PWM_MotorD, 80);
//      analogWrite(PWM_MotorI, 80);
      analogWrite(PWM_MotorD, 80-(dist_rest*30));
      analogWrite(PWM_MotorI, 80-(dist_rest*30));
    }
    else{
      analogWrite(PWM_MotorD, 0);
      analogWrite(PWM_MotorI, 0);
//      dist=0;
//      angle=0;
      dist_wheel_D = 0.0;
      dist_wheel_I = 0.0;
      straight=false;
      data[8]=1;
    }
  }

}

ros::NodeHandle nh;
std_msgs::Float32MultiArray msgSensors;
ros::Subscriber<std_msgs::Float32MultiArray> subSpeeds("/azcatl/hardware/simple_move", &simpleMoveCallback);
ros::Publisher pubSensors("/azcatl/hardware/sensors", &msgSensors);

void setup() {

  nh.getHardware()->setBaud(115200);
  nh.initNode(); 
  
  nh.advertise(pubSensors); 
  nh.subscribe(subSpeeds); 
  msgSensors.data_length = 9; 
   //Encoders
  pinMode(Enc_M3A,INPUT);
  pinMode(Enc_M3B,INPUT);
  pinMode(Enc_M4A,INPUT);
  pinMode(Enc_M4B,INPUT);
    
  //Motors
  pinMode(PWM_MotorD,OUTPUT);
  pinMode(PWM_MotorI,OUTPUT);
  pinMode(MotorD_A,OUTPUT);
  pinMode(MotorD_B,OUTPUT);
  pinMode(MotorI_A,OUTPUT);
  pinMode(MotorI_A,OUTPUT);
  
  //Interruptions

  attachInterrupt(5,encoderM3aEvent,RISING); //Int5 =Enc_M3A = pin18
  attachInterrupt(4,encoderM4aEvent,RISING); //Int4 =Enc_M4A = pin19
 


  //Set of values
  ContM3 = 0;
  ContM4 = 0;
}

void loop() {
   data[0]=analogRead(A0);
   data[1]=analogRead(A1);
   data[2]=analogRead(A2);
   data[3]=analogRead(A3);
   data[4]=analogRead(A4);
   data[5]=analogRead(A5);
   data[6]=analogRead(A6);
   data[7]=analogRead(A7);
  odometry(angle,dist);
  msgSensors.data = data;
  pubSensors.publish(&msgSensors);
  nh.spinOnce();
  delay(30);
 
}
