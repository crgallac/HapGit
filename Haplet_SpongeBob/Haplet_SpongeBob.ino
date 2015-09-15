



//--------------------------------------------------------------------------
// Colin Gallacher and Steve Ding
// 7/17/15
// Code to test basic Haplet functionality (sensing and force output)
//--------------------------------------------------------------------------

// Defines
//#define ENCODER_OPTIMIZE_INTERRUPTS

// Includes
#include<stdio.h>
#include <math.h>
#include <Encoder.h>
//#include "pins_arduino.h" // Speed Test
 

// Declare structures 

struct Balls
{
   float  x;
   float  y;
   float  radius;
   float   stiffness;
};

//const int outputPin=4; // SPEED TEST 

// Pin declares
int pwmPin = 5; // PWM output pin for motor 1
int dirPin = 8; // direction output pin for motor 1
int pwmPin1 = 6; // PWM output pin for motor 1
int dirPin1 = 7; // direction output pin for motor 1
//int sensorPosPin = A2; // input pin for MR sensor
Encoder myEnc(2,9);
Encoder myEnc1(3, 10);
//int fsrPin = A3; // input pin for FSR sensor

// Position tracking variables
int updatedPos = 0;     // keeps track of the latest updated value of the Encoder sensor reading
int rawPos = 0;         // current raw reading from Encoder sensor
int lastRawPos = 0;     // last raw reading from MR sensor
//int lastLastRawPos = 0; // last last raw reading from MR sensor
//int tempOffset = 0;
int rawDiff = 0;
//int lastRawDiff = 0;
//int rawOffset = 0;
//int lastRawOffset = 0;

int updatedPos1 = 0;     // keeps track of the latest updated value of the Encoder sensor reading
int rawPos1 = 0;         // current raw reading from Encoder sensor
int lastRawPos1 = 0;     // last raw reading from MR sensor
//int lastLastRawPos1 = 0; // last last raw reading from MR sensor
//int tempOffset1 = 0;
int rawDiff1 = 0;
//int lastRawDiff1 = 0;
//int rawOffset1 = 0;
//int lastRawOffset1 = 0;
//const int flipThresh = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred
//boolean flipped = false;

// Kinematics variables

float ts1; // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
float ts; // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
      
float      th1; 
float     th2; 
    
float c1;
float c2;
float s1;
float s2;

float xA;
float yA;
float xB;
float yB;
float R;
float S;
float M;
float N;
float a;
float b;
float c;
float Delta;
float y; 
float x;
float phi1;
float phi2;
float xP;
float yP;


// Kinematic vars

float xh = 0;           // position of the handle [m]
float yh= 0;
float lastXh = 0;     //last x position of the handle
float lastYh= 0;
float vhx = 0;         //velocity of the handle
float vhy = 0;
//float lastVhx = 0;     //last velocity of the handle
//float lastVhy = 0;     //last velocity of the handle
//float lastLastVhx = 0; //last last velocity of the handle
//float lastLastVhy = 0; //last last velocity of the handle

// Force output variables
float xforce = 0;           // force at the handle
float yforce = 0;           // force at the handle
float xforce_spring=0; 
float yforce_spring=0;
float xforce_damper=0; 
float yforce_damper=0; 
float Tp = 0;              // torque of the motor
float Tp1 = 0;              // torque of the motor
float duty = 0;            // duty cylce (between 0 and 255)
float duty1 = 0;            // duty cylce (between 0 and 255)
float constant; 
unsigned int output = 0;    // output command to the motor
unsigned int output1 = 0;    // output command to the motor

struct Balls balls[4];
struct Balls EE; 
const int numOfBalls= sizeof(balls)/sizeof(EE); 

float distance[numOfBalls]; 
float vectorX[numOfBalls]; 
float vectorY[numOfBalls]; 
float forceX[numOfBalls]; 
float forceY[numOfBalls]; 





// --------------------------------------------------------------
// Setup function -- NO NEED TO EDIT
// --------------------------------------------------------------
void setup()
{
  pinMode(4, OUTPUT);
  // Set up serial communication
  Serial.begin(57600);
  Serial.println("Haplet:");
  Serial.print("Number Of Objects:");
    Serial.println(numOfBalls);
  // Set PWM frequency
  setPwmFrequency(pwmPin,1);
  setPwmFrequency(pwmPin1,1);

  // Output pins
  pinMode(pwmPin, OUTPUT);  // PWM pin for motor A
  pinMode(dirPin, OUTPUT);  // dir pin for motor A
  pinMode(pwmPin1, OUTPUT);  // PWM pin for motor B
  pinMode(dirPin1, OUTPUT);  // dir pin for motor B

  // Initialize motor
  analogWrite(pwmPin, 0);     // set to not be spinning (0/255)
  analogWrite(pwmPin1, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPin, LOW);  // set direction
  digitalWrite(dirPin1, LOW);  // set direction

//Initialize Encoder Values
  myEnc.write(-169/(360.0/13824.0));
  myEnc1.write(-11/(360.0/13824.0));
  //-360.0/13824.0
  //.583

//balls[0].x=0.040; 
//balls[0].y=0.103;
//balls[0].radius=0.02; 
//balls[0].stiffness=5; 
//
//balls[1].x=.0438;
//balls[1].y=.0835; 
//balls[1].radius=.01; 
//balls[1].stiffness=5; 
//balls[2].x=-.028;
//balls[2].y=0.098; 
//balls[2].radius= .02;
//balls[2].stiffness=10;  
//
balls[0].x = 0.0181; 
balls[0].y=.0446; 
balls[0].radius=0.015; 
balls[0].stiffness=1.0; 
//balls[0].damping=.005;

}

//// Initialize position valiables
int newPosition  = -999;
int oldPosition = -999;
//int oldOldPosition= -999;
//
int newPosition1  = -999;
int oldPosition1 = -999;
//int oldOldPosition1= -999;

// Define kinematic parameters you may need
//float rh = 0.0725;   //[m]
//float l=0.0494; 
float l=0.0646; 
//float L=0.0656;
float L=0.0750;
float d=0.0210; 
float l3=0.015; 

//#if defined(__AVR__)
//#define REGTYPE unsigned char
//#elif defined(__PIC32MX__)
//#define REGTYPE unsigned long
//#endif

float forceXtotal; 
float forceYtotal; 


// --------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------
void loop()
{
  //*************************************************************
  //*** Section 1. Compute position in counts (do not change) ***
  //*************************************************************

  // Get voltage output by MR sensor
  newPosition = myEnc.read();  //current raw position the rotary encoder
  newPosition1 = myEnc1.read();  //current raw position the rotary encoder
  if (newPosition != oldPosition || newPosition1 != oldPosition1) {
    // Calculate differences between subsequent rotory encoder readings
    rawDiff = newPosition - oldPosition;          //difference btwn current position and last position
//    lastRawDiff = newPosition - oldOldPosition;  //difference btwn current position and last last position
//    rawOffset = abs(rawDiff);
//    lastRawOffset = abs(lastRawDiff);

    rawDiff1 = newPosition1 - oldPosition1;          //difference btwn current position and last position
//    lastRawDiff1 = newPosition1 - oldOldPosition1;  //difference btwn current position and last last position
//    rawOffset1 = abs(rawDiff1);
//    lastRawOffset1 = abs(lastRawDiff1);
    // Update position record-keeping vairables
//    oldOldPosition = oldPosition;
    oldPosition = newPosition;

    // Update position record-keeping vairables
//    oldOldPosition1 = oldPosition1;
    oldPosition1 = newPosition1;

//        Serial.println(newPosition);
//          Serial.println(newPosition1);
//               Serial.println();

  }


  //*************************************************************
  //*** Section 2. Compute position in meters *******************
  //*************************************************************

  // ADD YOUR CODE HERE
  // Step 2.1: print updatedPos via serial monitor
  //Serial.println(updatedPos);
  // Step 2.6:

// ts1= 0.583*newPosition; // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
// ts = 0.583*newPosition1; // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
  
  
   ts=  -360.0/13824.0*newPosition; // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
 ts1 =  -360.0/13824.0*newPosition1; // Compute the angle of the sector pulley (ts) in degrees based on updatedPos

//     Serial.println("angles");
//          Serial.println(ts);
//          Serial.println(ts1);

          
          
 th1=ts*3.14159/180.0; 
 th2=ts1*3.14159/180.0; 
 
//     Serial.println("angles(rad)");
//          Serial.println(th1);
//          Serial.println(th2);

// Forward Kinematics

 c1=cos(th1);
 c2=cos(th2);
 s1=sin(th1);
 s2=sin(th2);

 xA=l*c1;
 yA=l*s1;
 xB=d+l*c2;
 yB=l*s2;
 R=pow(xA,2) +pow(yA,2);
 S=pow(xB,2)+pow(yB,2);
 M=(yA-yB)/(xB-xA);
 N=0.5*(S-R)/(xB-xA);
 a=pow(M,2)+1;
 b=2*(M*N-M*xA-yA);
 c=pow(N,2)-2*N*xA+R-pow(L,2);
 Delta=pow(b,2)-4*a*c;
 y=(-b+sqrt(Delta))/(2*a); 
 x=M*y+N;
//phi1=angle((x-l*c1)/L+1i*(y-l*s1)/L);
//phi2=angle((x-d-l*c2)/L+1i*(y-l*s2)/L);
 phi1=acos((x-l*c1)/L);
 phi2=acos((x-d-l*c2)/L);
 xP=x;
 yP=y;
 
//xh=xP+l3*cos(phi1+3.14/3.0); 
//yh=yP+l3*sin(phi1+3.14/3.0); 
xh=xP-.011; 
yh=yP; 

//    Serial.println("positions");
//          Serial.println(xh,4);
//          Serial.println(yh,4);
//          Serial.println(); 

//WRITE FORCES HERE
  forceXtotal = 0;
  forceYtotal = 0; 
//
for(int i=0; i<numOfBalls; i++){
  vectorX[i]=xh-balls[i].x; 
  vectorY[i]=yh-balls[i].y;
  distance[i]= pow(vectorX[i],2)+pow(vectorY[i],2); 
  distance[i]= sqrt(distance[i]); 
  float norm= 1/distance[i];
 

  if(distance[i]<balls[i].radius) { // Coliision detected
    
   forceX[i]=vectorX[i]*balls[i].stiffness; 
   forceY[i]=vectorY[i]*balls[i].stiffness; 
}else{
  forceX[i]=0; 
  forceY[i]=0; 
}
}
// Serial.println(forceX[1],4);

for (int i=0; i<numOfBalls; i++){
  forceXtotal += forceX[i];
  forceYtotal += forceY[i];  
}


//
//if(xh>0.03){
//
//  forceXtotal=10*(xh-.03);
//}
//if(xh<-0.03){
//  forceXtotal=10*(xh+.03); 
//}
//if(yh>.07){
//  forceYtotal=10*(yh-.07); 
//}



//
//Serial.println("angles(rad)");
//          Serial.println(forceXtotal,4);
//          Serial.println(forceYtotal,4);
   
  //
  // Step 2.8: print xh via serial monitor
  //  Serial.println(ts,5);
  // Lab 4 Step 2.3: compute handle velocity
//  vhx = -(.95*.95)*lastLastVhx + 2*.95*lastVhx + (1-.95)*(1-.95)*(xh-lastXh)/.0001;  // filtered velocity (2nd-order filter)
//  vhy = -(.95*.95)*lastLastVhy + 2*.95*lastVhy + (1-.95)*(1-.95)*(yh-lastYh)/.0001;  // filtered velocity (2nd-order filter)
//  vhx= (lastVhx+2*(xh-lastXh)/.0015)/3;
  vhx= (xh-lastXh)/.0015;
//  vhy=(lastVhy+2* (yh-lastYh)/.0015)/3;
  vhy=(yh-lastYh)/.0015; 
  lastXh = xh;
  lastYh = yh;
//  lastLastVhx = lastVhx;
//  lastLastVhy = lastVhy;
//  lastVhx = vhx;
//  lastVhy = vhy;
  
 
  //   Serial.println(ts,5);


  //*************************************************************
  //*** Section 3. Assign a motor output force in Newtons *******
  //*************************************************************

forceXtotal-=.005*vhx; 
forceYtotal-=.005*vhy; 

//for (int i=0; i<numOfBalls; i++){
//  forceXtotal += forceX[i];
//  forceYtotal += forceY[i];  
//}

xforce=forceXtotal; 
yforce=forceYtotal; 
 
constant = -1.0/sin(phi2-phi1);
Tp= (s1*sin(phi2-phi1)+sin(phi1)*sin(th1-phi2))* xforce +(-c1*sin(phi2-phi1)-cos(phi1)*sin(th1-phi2))*yforce; 
Tp1= -(sin(phi1)*sin(th2-phi2))*xforce+(cos(phi1)*sin(th2-phi2))*yforce; 

Tp=constant*Tp; 
Tp1=constant*Tp1;
//
//Serial.println(Tp,6);
//Serial.println(Tp1,6);
//Serial.println();

//Tp=0; 
Tp=-Tp;

//Tp1=1; 
Tp1=-Tp1; 


  //*************************************************************
  //*** Section 5. Force output (do not change) *****************
  //*************************************************************

  // Determine correct direction for motor torque
  if(Tp < 0) {
    digitalWrite(dirPin, HIGH);
  }
  else {
    digitalWrite(dirPin, LOW);
  }

  if(Tp1 <  0) {
    digitalWrite(dirPin1, HIGH);
  }
  else {
    digitalWrite(dirPin1, LOW);
  }
//Serial.println(Tp,6);
//Serial.println(Tp1,6);
//Serial.println();

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)

  duty = sqrt(abs(Tp)/0.06);
  duty1 = sqrt(abs(Tp1)/0.06);
//  Serial.println(duty);
//  Serial.println(duty1);
//  Serial.println();

  // Make sure the duty cycle is between 0 and 100%
  if (duty > 1) {
    duty = 1;
  }
  else if (duty < 0) {
    duty = 0;
  }
  if (duty1 > 1) {
    duty1 = 1;
  }
  else if (duty1 < 0) {
    duty1 = 0;
  }

//  Serial.println(duty);
//  Serial.println(duty1);
//  Serial.println();

//  duty=0;
//  duty1=0;

  output = (int)(duty* 255*1);   // convert duty cycle to output signal
  analogWrite(pwmPin,output);  // output the signal
  output1 = (int)(duty1* 255*1 );   // convert duty cycle to output signal
  analogWrite(pwmPin1,output1);  // output the signal

}



// --------------------------------------------------------------
// Function to set PWM Freq -- DO NOT EDIT
// --------------------------------------------------------------
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
    case 1:
      mode = 0x01;
      break;
    case 8:
      mode = 0x02;
      break;
    case 64:
      mode = 0x03;
      break;
    case 256:
      mode = 0x04;
      break;
    case 1024:
      mode = 0x05;
      break;
    default:
      return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    }
    else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  }
  else if(pin == 3 || pin == 11) {
    switch(divisor) {
    case 1:
      mode = 0x01;
      break;
    case 8:
      mode = 0x02;
      break;
    case 32:
      mode = 0x03;
      break;
    case 64:
      mode = 0x04;
      break;
    case 128:
      mode = 0x05;
      break;
    case 256:
      mode = 0x06;
      break;
    case 1024:
      mode = 0x7;
      break;
    default:
      return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}




