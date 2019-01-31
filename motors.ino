//motor sequence - motors are numbered in increasing order of anticlockwise started from front left.
int motor_1=0;//motor 1
int motor_2=0//motor 2
int motor_3=0;//motor 3
int motor_4=0;//motor 4
int motor_5=0;//motor 5
int motor_6=0;//motor 6

int ch1=1500;//channel_1
int ch2=1500;//channel_2
int ch3=1500;//channel_3
int ch4=1500;//channel_4
int ch5=1500;//channel_5

const alpha; //constant for channel_1  Right Left
const beta;	//constant for channel_2	front back
const alpha_new; //new constant for channel 1

const value=1000;

//Function to update the values of motor_speed .
//input[] array for 5 channels.
void function(float input[6])
{
	motor_1 = input[3];
	motor_1 += (input[1]-ch1)*alpha;
	motor_1 += (input[2]-ch2)*(-beta);
	value = value*input[5];
	motor1 = motor1*value;

	motor_2 = input[3];
	motor_2 += (input[1]-ch1)*alpha_new;
	motor_2 = motor_2*value;

	motor_3 = input[3];
	motor_3 += (input[1]-ch1)*alpha;
	motor_3 += (input[2]-ch2)*beta;
	motor_3 = motor_3*value;

	motor_4 = input[3];
	motor_4 += (input[1]-ch1)*(-alpha);
	motor_4 += (input[2]-ch2)*beta;
	motor_4 = motor_4*value;

	motor_5 = input[3];
	motor_5 += (input[1]-ch1)*(-alpha_new);
	motor_5 = motor_5*value;

	motor_6 = input[3];
	motor_6 += (input[1]-ch1)*(-alpha);
	motor_6 += (input[2]-ch2)*(-beta);
	motor_6 = motor_6*value;
}
