#include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include "SPI.h"
#include <Servo.h>
#include <math.h>
LSM6DS3 myIMU; //Default constructor is I2C, addr 0x6B
//program Parms
double ComputeConst;
int ProgIni, exits=0;
//GyroParms
float Angle, value;
double Angle_Prev;
double denom;
double AccelVal;
double AccelAng;
double RotVelocity;
double RotVelocity_Prev;
double RotAccel;
float GyroVal;
//Motor Control Parms
Servo RightMotor;
Servo LeftMotor;
double LeftRpm;
double RightRpm;
//gains
double pGain;
double dGain;
double aGain;
double iGain;  //NEW

//Timer?
int loopTime;  //NEW
int prevLoopTime;

//Angular Displacement
double angInt; //deg

//new
float velAvg[5];
float posAvg[5];
int count;
float avgVel;
float avgAng;
int avgNum;

void setup() {
Serial.begin(9600);
//Program Parms
ComputeConst=0.001;
ProgIni=0;
//Imu
myIMU.begin();
Angle=0;
Angle_Prev=0;
RotVelocity=0;
RotVelocity_Prev=0;
RotAccel=0;
//Motor Control
RightMotor.attach(3);
LeftMotor.attach(5);
RightMotor.write(1000);
LeftMotor.write(1000);
while(ProgIni==0){if (Serial.available() > 0) {ProgIni=Serial.read();}}
delay(1000);
RightRpm=1100;
LeftRpm=1100;
RightMotor.write(RightRpm);
LeftMotor.write(LeftRpm);
//Gains
pGain=1;
dGain=0.5;
aGain=0;
iGain=0; //NEW
loopTime = millis();
float velAvg[5] = {0,0,0,0,0};
//float posAvg[5] = {0,0,0,0,0};
int count = 0;
float avgVel = 0;
//int avgNum = 3;
}

int compTime(int currTime)
{}


void loop()
{       
        prevLoopTime = loopTime; //New, setting a start time for loop timing
        ComputeConst=(loopTime-prevLoopTime)*0.001;
        Angle_Prev=Angle;
        denom=abs(0.1+(myIMU.readFloatAccelX()*myIMU.readFloatAccelX()) + (myIMU.readFloatAccelZ())*myIMU.readFloatAccelZ());
        
        if (denom<0.0001){denom=0.0001;}
          AccelVal=((myIMU.readFloatAccelY())/(sqrt(denom)));
          AccelAng=atan(AccelVal)*(180/3.1415);
          GyroVal=myIMU.readFloatGyroX();
          Angle=(GyroVal*ComputeConst)+Angle;
          Angle=0.5*Angle+0.5*AccelAng;

        loopTime = millis();
        
        RotVelocity_Prev=RotVelocity;
        RotVelocity=(Angle-Angle_Prev)/((loopTime-prevLoopTime)*0.001);
        RotAccel=(RotVelocity-RotVelocity_Prev)/((loopTime-prevLoopTime)*0.001);

        if(fabs(RotVelocity)>150)
        
        {
          RotVelocity = RotVelocity/fabs(RotVelocity)*150;
        }

        velAvg[count] = RotVelocity;
        //posAvg[count] = Angle;
        
        count++;
        if(count>=3)
          count = 0;

        for(int index = 0;index<3;index++)
        {
          avgVel+=velAvg[index];
          //avgAng+=posAvg[index];
        }
        avgVel/=3;
        //avgAng/=3;

        angInt+=Angle*((loopTime-prevLoopTime)*0.001);//finding angle traveled in time it took to loop*0.001 cuz ms        
    
        RightRpm=1100-(Angle*pGain)+(avgVel*dGain)-(RotAccel*aGain)-angInt*iGain;  //Edited  changed rot velocity to -
        LeftRpm=1110+(Angle*pGain)-(avgVel*dGain)+(RotAccel*aGain)+angInt*iGain;    //Edited
        
        if (RightRpm>1500) RightRpm=1500;
        if (LeftRpm>1500) LeftRpm=1500;
        if (RightRpm<1050) RightRpm=1050;
        if (LeftRpm<1050) LeftRpm=1050;
        
        //write to motors
         RightMotor.write(RightRpm);
         LeftMotor.write(LeftRpm);
         
         if (Serial.available() > 0) {
         exits = Serial.read();}
         //type p followed by value to change pGain
         if (exits == 112) {
           if (Serial.available() > 0) {
             value = Serial.read();
             }
           pGain = value;
         }
         //type d followed by value to change dGain
         if (exits == 100) {
           if (Serial.available() > 0) {
             value = Serial.read();
             }
           dGain = value;
         }
         //type 9 to stop the program
         if (exits == 57) {
          RightMotor.write(1000);
          LeftMotor.write(1000);
          delay(1000);
          exit(0);
         }
         //type 5 to pause and 1 to start up again
         if (exits == 53) {
          RightMotor.write(1000);
          LeftMotor.write(1000);
          exits = 0;
           while (1) {
             if (Serial.available() > 0) {
             exits = Serial.read();}
             if (exits == 49) {
               break;
             }
           }
         }
         //Serial.print(RightRpm);
         //Serial.print(",");
         //Serial.print(LeftRpm);
        // Serial.print(", ,");
        Serial.print(Angle);
        /*Serial.print(",");
        Serial.print(RotVelocity*0.01);
        Serial.print(",");
        Serial.println(avgVel);
        Serial.print("      ");
        Serial.print(loopTime-prevLoopTime);  
        // Serial.print(",");     
        // Serial.println(RotAccel*0.01);*/
        Serial.print(" , ");
        Serial.println(pGain);
}
