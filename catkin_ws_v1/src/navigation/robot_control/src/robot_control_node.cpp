#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"


//Declaración de variables
float refL_t = 0;
float refw_t = 0;
float r_t = 0;
float rw_t = 0;
float rL_t = 0;
float y_t = 0;
float yi_t = 0;
float yd_t = 0;
float ur = 0;

float Pr = 0;
float Ir = 0;
float Dr = 0;

float Kp = 1;
float Ki = 0;
float Kd = 0;

float error_r = 0;
float erroranterior = 0;

double tiempociclo = 200.0/1000.0;
float maxPIDr = 0.225;
float minPIDr = 0.045;

float r_tder = 0;
float r_tizq = 0;

int sentido = 0; 
int giro = 0; 

std_msgs::Float32MultiArray speed;

void Control()
{
		error_r = r_t - y_t; //Cálculo del error
		Pr = Kp * error_r;
		Ir += Ki * error_r *  tiempociclo;
		Dr = Kd * (error_r - erroranterior) / tiempociclo;
		ur = Pr + Ir + Dr;
		if (ur > maxPIDr)
		{
			ur = maxPIDr;
		}
		if (ur < minPIDr)
		{
			ur = minPIDr;
		}
}

void Velocidad()
{
    rL_t = fabs (refL_t);
    rw_t = fabs (refw_t);

    if (refL_t > 0)
    {
    	sentido = 1; //avanza hacia el frente
    }
    else
    {
    	sentido = 0; // avanza hacia atrás
    }

    if (refw_t > 0)
    {
    	giro = 1; //gira hacia la izquierda
    }
    else
    {
    	giro = 0; // gira hacia la derecha
    }

	if (rw_t > 0.617)
	{
		rw_t = 0.617;
	}
	if (rL_t > 0.225)
	{
		rw_t = 0.225;
	}
	
}

void VelocidadMotores()
{
	if (sentido == 1)
	{
		r_tder = ur * 2;
		r_tizq = ur * 2;
	}
	if (sentido == 0)
	{
		r_tder = ur * -2;
		r_tizq = ur * -2;
	}
	
}

void callback_sensors(const std_msgs::Float32MultiArray::ConstPtr& msg){

	yi_t=msg->data[10];
	yd_t=msg->data[11];
}


int main(int argc, char* argv[]){
	std::cout << "INITIALIZING robot control..." << std::endl;
    ros::init(argc, argv, "robot_control");
    ros::NodeHandle n;
    ros::Rate loop(10);

    ros::Publisher pubSpeeds = n.advertise<std_msgs::Float32MultiArray>("/azcatl/hardware/motor_speeds", 1);
    ros::Subscriber subSensors= n.subscribe("/azcatl/hardware/sensors",1, callback_sensors);

    speed.data.push_back(0.0);
    speed.data.push_back(0.0);


    while(ros::ok()){

		Velocidad();
		r_t = rL_t;
		y_t = (yi_t + yd_t) / 2;
		Control();
		//r_t = rw_t;
		//y_t = (yi_t + yd_t) / 0.18;
		//Control();
		VelocidadMotores();

		speed.data[0]=r_tizq;
		speed.data[1]=r_tder;
		pubSpeeds.publish(speed);
		ros::spinOnce();
		loop.sleep();
    }

	return 0;
}