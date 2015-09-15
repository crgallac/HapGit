



//--------------------------------------------------------------------------
// Colin Gallacher
// 4.30.15
// Code to test basic Haplet functionality (sensing and force output)
//--------------------------------------------------------------------------

// Defines
//#define ENCODER_OPTIMIZE_INTERRUPTS

// Includes
#include <math.h>
#include <Encoder.h>
//#include "pins_arduino.h" // Speed Test


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
int lastLastRawPos = 0; // last last raw reading from MR sensor
int tempOffset = 0;
int rawDiff = 0;
int lastRawDiff = 0;
int rawOffset = 0;
int lastRawOffset = 0;

int updatedPos1 = 0;     // keeps track of the latest updated value of the Encoder sensor reading
int rawPos1 = 0;         // current raw reading from Encoder sensor
int lastRawPos1 = 0;     // last raw reading from MR sensor
int lastLastRawPos1 = 0; // last last raw reading from MR sensor
int tempOffset1 = 0;
int rawDiff1 = 0;
int lastRawDiff1 = 0;
int rawOffset1 = 0;
int lastRawOffset1 = 0;
//const int flipThresh = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred
//boolean flipped = false;

// Kinematics variables

double xh = 0;           // position of the handle [m]
double yh= 0;
double lastXh = 0;     //last x position of the handle
double lastYh= 0;
double vhx = 0;         //velocity of the handle
double vhy = 0;
double lastVhx = 0;     //last velocity of the handle
double lastVhy = 0;     //last velocity of the handle
double lastLastVhx = 0; //last last velocity of the handle
double lastLastVhy = 0; //last last velocity of the handle

// Force output variables
double xforce = 0;           // force at the handle
double yforce = 0;           // force at the handle
double xforce_spring=0; 
double yforce_spring=0;
double xforce_damper=0; 
double yforce_damper=0; 
double Tp = 0;              // torque of the motor
double Tp1 = 0;              // torque of the motor
double duty = 0;            // duty cylce (between 0 and 255)
double duty1 = 0;            // duty cylce (between 0 and 255)
unsigned int output = 0;    // output command to the motor
unsigned int output1 = 0;    // output command to the motor

// --------------------------------------------------------------
// Setup function -- NO NEED TO EDIT
// --------------------------------------------------------------
void setup()
{
  pinMode(4, OUTPUT);
  // Set up serial communication
    Serial.begin(57600);
    Serial.println("Haplet:");
  // Set PWM frequency
  setPwmFrequency(pwmPin,1);
  setPwmFrequency(pwmPin1,1);
  // Input pins
  //  pinMode(sensorPosPin, INPUT); // set MR sensor pin to be an input
  //  pinMode(fsrPin, INPUT);       // set FSR sensor pin to be an input

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

double offset=18.0; 

  myEnc.write(-(180.0-offset)*13856.0/360.0);
  myEnc1.write(-offset*13856.0/360.0);

}

// Initialize position valiables
long newPosition  = -999;
long oldPosition = -999;
long oldOldPosition= -999;

long newPosition1  = -999;
long oldPosition1 = -999;
long oldOldPosition1= -999;

// Define kinematic parameters you may need
//double rh = 0.0725;   //[m]
double l=0.067; 
double L=0.073;
double d=0.020; 

//#if defined(__AVR__)
//#define REGTYPE unsigned char
//#elif defined(__PIC32MX__)
//#define REGTYPE unsigned long
//#endif



// --------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------
void loop()
{
  // SPEED TEST 
  //--------------------------
  //  volatile int count = 0;
  //  volatile REGTYPE *reg = portOutputRegister(digitalPinToPort(outputPin));
  //  REGTYPE mask = digitalPinToBitMask(outputPin);
  //  
  //while(1){
  //-------------------

  //*************************************************************
  //*** Section 1. Compute position in counts (do not change) ***
  //*************************************************************

  // Get voltage output by MR sensor
  newPosition = myEnc.read();  //current raw position the rotary encoder
  newPosition1 = myEnc1.read();  //current raw position the rotary encoder
  if (newPosition != oldPosition || newPosition1 != oldPosition1) {
    // Calculate differences between subsequent rotory encoder readings
    rawDiff = newPosition - oldPosition;          //difference btwn current position and last position
    lastRawDiff = newPosition - oldOldPosition;  //difference btwn current position and last last position
    rawOffset = abs(rawDiff);
    lastRawOffset = abs(lastRawDiff);

    rawDiff1 = newPosition1 - oldPosition1;          //difference btwn current position and last position
    lastRawDiff1 = newPosition1 - oldOldPosition1;  //difference btwn current position and last last position
    rawOffset1 = abs(rawDiff1);
    lastRawOffset1 = abs(lastRawDiff1);

    // Update position record-keeping vairables
    oldOldPosition = oldPosition;
    oldPosition = newPosition;

    // Update position record-keeping vairables
    oldOldPosition1 = oldPosition1;
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
  //30:1 motors
  //  double ts = -1.0046*newPosition; // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
  //  double ts1 = -1.0046*newPosition1; // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
 double ts1= -360.0/13824.0*newPosition1; // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
  double ts = -360.0/13824.0*newPosition; // Compute the angle of the sector pulley (ts) in degrees based on updatedPos

//       Serial.println("angles");
//            Serial.println(ts1);
//            Serial.println(ts);


  double      th1=ts*3.14159/180.0; 
  double     th2=ts1*3.14159/180.0; 
//       Serial.println("angles(rad)");
//            Serial.println(th1);
//            Serial.println(th2);

  //delay(1000); 
  // Forward Kinematics

  double c1=cos(th1);
  double c2=cos(th2);
  double s1=sin(th1);
  double s2=sin(th2);

  double xA=l*c1;
  double yA=l*s1;
  double xB=d+l*c2;
  double yB=l*s2;
  double R=pow(xA,2) +pow(yA,2);
  double S=pow(xB,2)+pow(yB,2);
  double M=(yA-yB)/(xB-xA);
  double N=0.5*(S-R)/(xB-xA);
  double a=pow(M,2)+1;
  double b=2*(M*N-M*xA-yA);
  double c=pow(N,2)-2*N*xA+R-pow(L,2);
  double Delta=pow(b,2)-4*a*c;
  double y=(-b+sqrt(Delta))/(2*a); 
  double x=M*y+N;
  //phi1=angle((x-l*c1)/L+1i*(y-l*s1)/L);
  //phi2=angle((x-d-l*c2)/L+1i*(y-l*s2)/L);
  double phi1=acos((x-l*c1)/L);
  double phi2=acos((x-d-l*c2)/L);
  double xP=x;
  double yP=y;

  xh=xP; 
  yh=yP; 


  // Step 2.7:
  //  xh = rh*(cos(ts*3.14159/180)-cos(ts1*3.14159/180));       // Compute the position of the handle based on ts
  //  yh= rh*(sin(ts*3.14159/180)+sin(ts1*3.14159/180));
//  Serial.println("postions");
//         Serial.println(xh,6);
//        Serial.println(yh);

  //
  // Step 2.8: print xh via serial monitor
  //  Serial.println(ts,5);
  // Lab 4 Step 2.3: compute handle velocity
//  vhx = -(.95*.95)*lastLastVhx + 2*.95*lastVhx + (1-.95)*(1-.95)*(xh-lastXh)/.0001;  // filtered velocity (2nd-order filter)
//  vhy = -(.95*.95)*lastLastVhy + 2*.95*lastVhy + (1-.95)*(1-.95)*(yh-lastYh)/.0001;  // filtered velocity (2nd-order filter)
//  vhx= (lastVhx+2*(xh-lastXh)/.0015)/3;
  vhx= (xh-lastXh)/.0015; ///.0015;
//  vhy=(lastVhy+2* (yh-lastYh)/.0015)/3;
    vhy=(yh-lastYh)/.0015; 
  lastXh = xh;
  lastYh = yh;
  lastLastVhx = lastVhx;
  lastLastVhy = lastVhy;
  lastVhx = vhx;
  lastVhy = vhy;

//  Serial.println("velocities");
//         Serial.println(vhx,6);
//        Serial.println(vhy,6);
//        Serial.println(); 


  //   Serial.println(ts,5);


  //*************************************************************
  //*** Section 3. Assign a motor output force in Newtons *******
  //*************************************************************

  // ADD YOUR CODE HERE
  // Define kinematic parameters you may need
  //     double rp = 0.004191;   //[m]
  //     double rs = 0.073152;   //[m]
  // Step 3.1:
  //  force = -0.5; // In lab 3, you will generate a force by simply assigning this to a constant number (in Newtons)

  //  xforce=0.5;
  // Lab 4 Step 1.4: render a VIRTUAL SPRING
  //  double k_spring = 20; //[N/m]
  //  force = -k_spring*xh;
  //  Serial.println(force);
  // Lab 4 Step 2.4: render a VIRTUAL DAMPER
  //double b_damper = 0.35;
  //force = -b_damper*vh;
  // Lab 4 Step 3.3: render a VIRTUAL TEXTURE using the damping approach (there is a more clever way to do this)
  //       double w = .005;  //width of damping area
  //       double b_damper = 0.5;
  //       for(int i=0; i<9; i=i+2){
  //         if((abs(xh)>i*w)&&(abs(xh)<(i+1)*w)){
  //           force = -b_damper*vh;
  //         } else if((abs(xh)>(i+1)*w)&&(abs(xh)<(i+2)*w)){
  //           force = 0;
  //         }
  //       }
  //  // Lab 4 Step 4.3: render a VIRTUAL WALL
  //       double x_wall = .01;
  //       double k_wall = 50;
  //       if(xh>x_wall) {
  //         force = -k_wall*(xh-x_wall);
  //       } else {
  //         force = 0;
  //       }

  //  // Lab 5 Step 4.3: render a VIRTUAL BOX
  //  double x_wallL = .03;
  //  double x_wallR= -.03;
  //  double k_wall = 50;
  ////
  //  if(xh>x_wallL) {
  //    xforce = -k_wall*(xh-x_wallL);
  //  }
  //  else if (xh<x_wallR )
  //  {
  //    xforce = -k_wall*(xh-x_wallR);
  //  }
  //  else {
  //    xforce = 0;
  //  }
  //          Serial.println(xh);
  //         Serial.println(xforce);
  //         Serial.println();
  //         xforce=0;
  //
  //          double y_wallT = .06;
  //         double y_wallB= 0.0;
  ////         double k_wall = 10;
  //        if(yh>y_wallT) {
  //         yforce = -k_wall*(yh-y_wallT);
  //       } else if(yh<y_wallB) {
  //         yforce = -k_wall*(yh-y_wallB);
  //       }else{
  //         yforce = 0;
  //       }

  //      Serial.println(yh,8);
  //     Serial.println(yforce);
  //     Serial.println();
  //     yforce=0; //

  //   Fix up
  //  //  // Lab 6 Step 4.3: render a SOLID BALL
  //
  // double kball= 50;
  // double ball_radius = 0.03; // 3 cm
  // double squared= sq(xh)+sq(yh);
  //double penatration =sqrt(squared-.03)
  //if (squared< sq(.03))){
  //  double theta= atan2(yh, xh);
  //  xforce=
  //
  //}

  //Lab 7

  double x_wallL = .036;
  double x_wallR= -.02;
  double b_damperL= 0.005;
  double b_damperR = 0.0205;
  double k_wall = 10;

  if(xh>x_wallL) {
    xforce_spring = -k_wall*(xh-x_wallL);
    xforce_damper = -b_damperL*vhx;
    yforce_damper= -b_damperL*vhy; 
    xforce=xforce_spring+xforce_damper;
    yforce=yforce_damper; 

  }
  else if (xh<x_wallR )
  {
    //    xforce = -k_wall*(xh-x_wallR);
    xforce_damper = -b_damperR*vhx;
    yforce_damper=-b_damperR*vhy; 
    xforce=xforce_damper; 
    yforce=yforce_damper; 
  }
  else {
    xforce = 0;
    yforce=0; 
  }

//  Serial.println("forces"); 
//           Serial.println(xforce);
//          Serial.println(yforce);
  // Step 3.2:
  //  Using the Jacobian of the pantograph
  //      Tp1 = rh * (-sin(ts*3.14159/180)*xforce+cos(ts*3.14159/180)*yforce);    // Compute the require motor pulley torque (Tp) to generate that force
  //      Tp = rh * (sin(ts1*3.14159/180)*xforce+cos(ts1*3.14159/180)*yforce);


  //   Tp1 =(-l.0/sin(phi2-phi1))*((s1*sin(phi2-phi1)+sin(phi1)*sin(th1-phi2))* xforce +(-1.0*sin(phi1)*sin(th2-phi2))*yforce); 
  // Tp= (-1.0/sin(phi2-phi1))*((-c1*sin(phi2-phi1)-cos(phi1)*sin(th1-phi2))*xforce+(cos(phi1)*sin(th2-phi2))*yforce); 

  double constant = -1.0/sin(phi2-phi1);
//   Serial.println(yforce,6);
//   Serial.println();
  Tp= (s1*sin(phi2-phi1)+sin(phi1)*sin(th1-phi2))* xforce +(-c1*sin(phi2-phi1)-cos(phi1)*sin(th1-phi2))*yforce; 
  Tp1= -(sin(phi1)*sin(th2-phi2))*xforce+(cos(phi1)*sin(th2-phi2))*yforce; 
  
  
//  Serial.println(Tp,6);
//  Serial.println(Tp1,6);
//  Serial.println();
  Tp=-constant*Tp; 
  Tp1=-constant*Tp1;


//  Serial.println(Tp,6);
//  Serial.println(Tp1,6);
//  Serial.println();
  //Tp1=-Tp1; //backwards



//  Tp=0; 
//  Tp1=0; 
  //Tp=1.0;
//  Tp1=1.0; 

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
//  Serial.println(Tp,6);
//  Serial.println(Tp1,6);
//  Serial.println();

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  //  duty = sqrt(abs(Tp)/0.06355);
  //  duty1 = sqrt(abs(Tp1)/0.06355);
  duty = sqrt(abs(Tp)/0.1);
  duty1 = sqrt(abs(Tp1)/0.1);
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


  //noInterrupts();
  //    *reg |= mask;	// Pulse the pin high, while interrupts are disabled.
  //    count = count + 1;
  //    *reg &= ~mask;
  //    interrupts();
  //}

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





