#include <Wire.h>
#include <Servo.h>


Servo right_prop_X,left_prop_X,right_prop_Y,left_prop_Y,right_prop_Z,left_prop_Z;

/*MPU-6050 gives you 16 bits data so you have to create some 16int constants
 * to store the data for accelerations and gyro*/

int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
 

float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];




float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180/3.141592654;

float PID_X,PID_Y,PID_Z, pwmLeft_X, pwmRight_X, error_X, previous_error_X,pwmLeft_Y, pwmRight_Y, error_Y, previous_error_Y,pwmLeft_Z, pwmRight_Z, error_Z, previous_error_Z;
float pid_px=0;
float pid_ix=0;
float pid_dx=0;
float pid_iy=0;
float pid_iz=0;
float pid_dy=0;
float pid_dz=0;
float pid_py=0;
float pid_pz=0;
/////////////////PID CONSTANTS/////////////////
double kp=3.55;//3.55
double ki=0.005;//0.003
double kd=2.05;//2.05
///////////////////////////////////////////////

double throttle=1300; //initial value of throttle to the motors
float desired_angle = 0; //This is the angle in which we whant the
                         //balance to stay steady


void setup() {
  Wire.begin(); //begin the wire comunication
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(250000);
//  right_prop_X.attach(3); //attatch the right motor to pin 3
//  left_prop_X.attach(5);  //attatch the left motor to pin 5

  time = millis(); //Start counting time in milliseconds
  /*In order to start up the ESCs we have to send a min value
   * of PWM to them before connecting the battery. Otherwise
   * the ESCs won't start up or enter in the configure mode.
   * The min value is 1000us and max is 2000us, REMEMBER!*/

  delay(7000); /*Give some delay, 7s, to have time to connect
                *the propellers and let everything start up*/ 
}//end of setup void

void loop() {

/////////////////////////////I M U/////////////////////////////////////
    timePrev = time;  // the previous time is stored before the actual time read
    time = millis();  // actual time read
    elapsedTime = (time - timePrev) / 1000; 
  
  /*The tiemStep is the time that elapsed since the previous loop. 
   * This is the value that we will use in the formulas as "elapsedTime" 
   * in seconds. We work in ms so we haveto divide the value by 1000 
   to obtain seconds*/

  /*Reed the values that the accelerometre gives.
   * We know that the slave adress for this IMU is 0x68 in
   * hexadecimal. For that in the RequestFrom and the 
   * begin functions we have to put this value.*/
   
     Wire.beginTransmission(0x68);
     Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
     Wire.endTransmission(false);
     Wire.requestFrom(0x68,6,true); 
   
   /*We have asked for the 0x3B register. The IMU will send a brust of register.
    * The amount of register to read is specify in the requestFrom function.
    * In this case we request 6 registers. Each value of acceleration is made out of
    * two 8bits registers, low values and high values. For that we request the 6 of them  
    * and just make then sum of each pair. For that we shift to the left the high values 
    * register (<<) and make an or (|) operation to add the low values.*/
    
     Acc_rawX=Wire.read()<<8|Wire.read(); //each value needs two registres
     Acc_rawY=Wire.read()<<8|Wire.read();
     Acc_rawZ=Wire.read()<<8|Wire.read();

 
    /*///This is the part where you need to calculate the angles using Euler equations///*/
    
    /* - Now, to obtain the values of acceleration in "g" units we first have to divide the raw   
     * values that we have just read by 16384.0 because that is the value that the MPU6050 
     * datasheet gives us.*/
    /* - Next we have to calculate the radian to degree value by dividing 180º by the PI number
    * which is 3.141592654 and store this value in the rad_to_deg variable. In order to not have
    * to calculate this value in each loop we have done that just once before the setup void.
    */

    /* Now we can apply the Euler formula. The atan will calculate the arctangent. The
     *  pow(a,b) will elevate the a value to the b power. And finnaly sqrt function
     *  will calculate the rooth square.*/
     /*---X---*/
     Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
     /*---Y---*/
     Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
     /*----YAW---*/
     Acceleration_angle[2] = atan(-1*(Acc_rawZ/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
   /*Now we read the Gyro data in the same way as the Acc data. The adress for the
    * gyro data starts at 0x43. We can see this adresses if we look at the register map
    * of the MPU6050. In this case we request just 4 values..*/
    
   Wire.beginTransmission(0x68);
   Wire.write(0x43); //Gyro data first adress
   Wire.endTransmission(false);
   Wire.requestFrom(0x68,4,true); //Just 4 registers
   
   Gyr_rawX=Wire.read()<<8|Wire.read(); //Once again we shif and sum
   Gyr_rawY=Wire.read()<<8|Wire.read();
   Gyr_rawZ=Wire.read()<<8|Wire.read();
 
   /*Now in order to obtain the gyro data in degrees/seconda we have to divide first
   the raw value by 131 because that's the value that the datasheet gives us*/

   /*---X---*/
   Gyro_angle[0] = Gyr_rawX/131.0; 
   /*---Y---*/
   Gyro_angle[1] = Gyr_rawY/131.0;
  /*----Z---*/
   Gyro_angle[2] = Gyr_rawZ/131.0;

   /*Now in order to obtain degrees we have to multiply the degree/seconds
   *value by the elapsedTime.*/
   /*Finnaly we can apply the final filter where we add the acceleration
   *part that afects the angles and ofcourse multiply by 0.98 */

   /*---X axis angle---*/
   Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
   /*---Y axis angle---*/
   Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];
   /*---Z axis Angl*/
   Total_angle[2] = 0.98 *(Total_angle[2] + Gyro_angle[2]*elapsedTime) + 0.02*Acceleration_angle[2];
   
   /*Now we have our angles in degree and values from -10º0 to 100º aprox*/
    //Serial.println(Total_angle[1]);

   
  
/*///////////////////////////P I D///////////////////////////////////*/
/*Remember that for the balance we will use just one axis. I've choose the x angle
to implement the PID with. That means that the x axis of the IMU has to be paralel to
the balance*/

/*First calculate the error between the desired angle and 
*the real measured angle*/

error_Y = Total_angle[1] - desired_angle;
error_Z = Total_angle[2]-desired_angle;
error_X = Total_angle[0]-desired_angle;
    
/*Next the proportional value of the PID is just a proportional constant
*multiplied by the error*/

pid_px = kp*error_X;
pid_py = kp*error_Y;
pid_pz = kp*error_Z;

/*The integral part should only act if we are close to the
desired position but we want to fine tune the error. That's
why I've made a if operation for an error between -2 and 2 degree.
To integrate we just sum the previous integral value with the
error multiplied by  the integral constant. This will integrate (increase)
the value each loop till we reach the 0 point*/
if(-3 <error_X <3)
{
  pid_ix = pid_ix+(ki*error_X);  
}

if(-3 <error_Y <3)
{
  pid_iy = pid_iy+(ki*error_Y);  
}

if(-3 <error_Z <3)
{
  pid_iz = pid_iz+(ki*error_Z);  
}

/*The last part is the derivate. The derivate acts upon the speed of the error.
As we know the speed is the amount of error that produced in a certain amount of
time divided by that time. For taht we will use a variable called previous_error.
We substract that value from the actual error and divide all by the elapsed time. 
Finnaly we multiply the result by the derivate constant*/

pid_dx = kd*((error_X - previous_error_X)/elapsedTime);

pid_dy = kd*((error_Y - previous_error_Y)/elapsedTime);

pid_dz = kd*((error_Z - previous_error_Z)/elapsedTime);

/*The final PID values is the sum of each of this 3 parts*/
PID_X = pid_px + pid_ix + pid_dx;

PID_Y = pid_py + pid_iy + pid_dy;

PID_Z = pid_pz + pid_iz + pid_dz;


/*We know taht the min value of PWM signal is 1000us and the max is 2000. So that
tells us that the PID value can/s oscilate more than -1000 and 1000 because when we
have a value of 2000us the maximum value taht we could sybstract is 1000 and when
we have a value of 1000us for the PWM sihnal, the maximum value that we could add is 1000
to reach the maximum 2000us*/
if(PID_X < -1000)
{
  PID_X=-1000;
}
if(PID_X > 1000)
{
  PID_X=1000;
}

if(PID_Y < -1000)
{
  PID_Y=-1000;
}
if(PID_Y > 1000)
{
  PID_Y=1000;
}


if(PID_Z < -1000)
{
  PID_Z=-1000;
}
if(PID_Z > 1000)
{
  PID_Z=1000;
}

/*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
pwmLeft_X = throttle + PID_X;
pwmRight_X = throttle - PID_X;

pwmLeft_Y = throttle + PID_Y;
pwmRight_Y = throttle - PID_Y;

pwmLeft_Z = throttle + PID_Z;
pwmRight_Z = throttle - PID_Z;


/*Once again we map the PWM values to be sure that we won't pass the min
and max values. Yes, we've already maped the PID values. But for example, for 
throttle value of 1300, if we sum the max PID value we would have 2300us and
that will mess up the ESC.*/
//Right
if(pwmRight_X < 1000)
{
  pwmRight_X= 1000;
}
if(pwmRight_X > 2000)
{
  pwmRight_X=2000;
}
//Left
if(pwmLeft_X < 1000)
{
  pwmLeft_X= 1000;
}
if(pwmLeft_X > 2000)
{
  pwmLeft_X=2000;
}


//Right
if(pwmRight_Y < 1000)
{
  pwmRight_Y= 1000;
}
if(pwmRight_Y > 2000)
{
  pwmRight_Y=2000;
}
//Left
if(pwmLeft_Y < 1000)
{
  pwmLeft_Y= 1000;
}
if(pwmLeft_Y > 2000)
{
  pwmLeft_Y=2000;
}


//Right
if(pwmRight_Z < 1000)
{
  pwmRight_Z= 1000;
}
if(pwmRight_Z > 2000)
{
  pwmRight_Z=2000;
}
//Left
if(pwmLeft_Z < 1000)
{
  pwmLeft_Z= 1000;
}
if(pwmLeft_Z > 2000)
{
  pwmLeft_Z=2000;
}

/*Finnaly using the servo function we create the PWM pulses with the calculated
width for each pulse*/

previous_error_X = error_X; //Remember to store the previous error.

previous_error_Y = error_Y; //Remember to store the previous error.


previous_error_Z = error_Z; //Remember to store the previous error.

}//end of loop void
