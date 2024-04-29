#include <iostream>
#include "pid.h"
#include "mpu6050.h"
#include "helperMath.h"
#include "l298nMotor.h"

using namespace std;
using namespace robot;



 int main () {
     // MPU control/status vars
     robot::imu::MPU6050 device(0x68);

// orientation/motion vars
     Quaternion q;           // [w, x, y, z]         quaternion container
     VectorFloat gravity;    // [x, y, z]            gravity vector
     float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//PID
     double originalSetpoint = 175.8;
     double setpoint = originalSetpoint;
     double movingAngleOffset = 0.1;
     double input, output {};
     int moveState=0; //0 = balance; 1 = back; 2 = forth
     double Kp = 50;
     double Kd = 1.4;
     double Ki = 60;

     double motorSpeedFactorLeft = 0.6;
     double motorSpeedFactorRight = 0.5;
//MOTOR CONTROLLER
     int ENA = 5;
     int IN1 = 6;
     int IN2 = 7;
     int IN3 = 8;
     int IN4 = 9;
     int ENB = 10;
     robot::l298n::L298n motorControl(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

     while(true)
     {
         float ax, ay, az, gr, gp, gy; //Variables to store the accel, gyro and angle values

         sleep(1); //Wait for the MPU6050 to stabilize

        /*
	        //Calculate the offsets
	        std::cout << "Calculating the offsets...\n    Please keep the accelerometer level and still\n    This could take a couple of minutes...";
	        device.getOffsets(&ax, &ay, &az, &gr, &gp, &gy);
	        std::cout << "Gyroscope R,P,Y: " << gr << "," << gp << "," << gy << "\nAccelerometer X,Y,Z: " << ax << "," << ay << "," << az << "\n";
        */

         //Read the current yaw angle
         device.calc_yaw = true;

         device.getAngle(0, &gr);
         device.getAngle(1, &gp);
         device.getAngle(2, &gy);
         std::cout << "Current angle around the roll axis: " << gr << "\n";
         std::cout << "Current angle around the pitch axis: " << gp << "\n";
         std::cout << "Current angle around the yaw axis: " << gy << "\n";
         usleep(250000); //0.25sec

         //Get the current accelerometer values
         device.getAccel(&ax, &ay, &az);
         std::cout << "Accelerometer Readings: X: " << ax << ", Y: " << ay << ", Z: " << az << "\n";

         //Get the current gyroscope values
         device.getGyro(&gr, &gp, &gy);
         std::cout << "Gyroscope Readings: X: " << gr << ", Y: " << gp << ", Z: " << gy << "\n";
         input = gr;

         pid::Pid pid(input, output, setpoint, Kp, Ki, Kd, robot::pid::controllerDirection::Direct);
         pid.setMode(pid::pidMode::Automatic);
         pid.setSampleTime(10);
         pid.setOutput(-255, 255);
         pid.compute();
         motorControl.move(output);


     }
 }