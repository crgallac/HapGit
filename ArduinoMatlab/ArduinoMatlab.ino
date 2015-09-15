
/*
 * Sketch to interface with the Haplet via serial interface
 *
 *
 * Colin Gallacher 07/15/2015
 */


// Includes

#include<stdlib.h>
#include <math.h>
#include <Encoder.h>


//Data Structures and Unions 


 
union binary_float {
  float floating_point;
  char binary_array[4];
};

union binary_float torques[2];
union binary_float angles[2];



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

// Force output variables
float Tp = 0;              // torque of the motor
float Tp1 = 0;              // torque of the motor
float duty = 0;            // duty cycle (between 0 and 255)
float duty1 = 0;            // duty cycle (between 0 and 255)
unsigned int output = 0;    // output command to the motor
unsigned int output1 = 0;    // output command to the motor



char operation; // Holds operation (R, W, ...)
char mode; // Holds the mode (F, T, ...)
int wait_for_transmission = 1000; // Delay in ms in order to receive the serial data

float th1;
float th2;
float ts1; 
float ts; 

float tau1=0.0;
float tau2=0.0;

//int led = 13;


//void return_position(float x, float y, float xdot, float ydot){
//  /*
//     * Performs a digital read on pin_number and returns the value` read to serial
//   * in this format: D{pin_number}:{value}\n where value can be 0 or 1
//   */
//
//  //    x = xh;
//  //    y=yh;
//  //    xdot=vhx;
//  //    ydot=vhy;
//  //
//  Serial.print('P');
//  Serial.print(':');
//  Serial.print(x);
//  Serial.print(':');
//  Serial.print(y);
//  Serial.print(':');
//  Serial.print(xdot);
//  Serial.print(':');
//  Serial.println(ydot);// Adds a trailing \n
//    Serial.flush();
//}

void return_joints(float th1, float th2){
  /*
     * Performs an analog read on pin_number and returns the value read to serial
   * in this format: A{pin_number}:{value}\n where value ranges from 0 to 1023
   */
angles[0].floating_point= th1; 
angles[1].floating_point=th2; 
   
Serial.print('J');
     for(int i=0; i<4; i++){
           Serial.print(angles[0].binary_array[i]);
          }
          for(int i=0; i<4; i++){
                    Serial.print(angles[1].binary_array[i]);
 }

//  Serial.print('J');
//  Serial.print(':');
//  Serial.print(th1);
//  Serial.print(':');
//  Serial.print(th2);
//  Serial.print(':');
//  Serial.print(th1dot);
//  Serial.print(':');
//  Serial.println(th2dot);// Adds a trailing \n
//  Serial.flush();
}

//still not done
//void write_force(float fx, float fy){
//  /*
//     * Performs a digital write on pin_number with the digital_value
//   * The value must be 1 or 0
//   */
//
//  digitalWrite(led, HIGH);
//
//}

void write_torque(float tau1, float tau2){
  /*
     * Performs an analog write on pin_number with the analog_value
        * The value must be range from 0 to 255
   */

   Tp=-torques[0].floating_point; 
   Tp1=-torques[1].floating_point;
//
//   if (tau1 == 0){
//      digitalWrite(led, LOW);
//   }
//   else {
//       digitalWrite(led, HIGH);
//   }

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
  duty = sqrt(abs(Tp)/0.3);
  duty1 = sqrt(abs(Tp1)/0.3);
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


return_joints(angles[0].floating_point, angles[1].floating_point);
//  return_joints(angles[0].floating_point, angles[1].floating_point);
}



// SETUP

void setup() {
 pinMode(4, OUTPUT);
  // Set up serial communication
    Serial.begin(57600);
    Serial.setTimeout(100); 
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



// LOOP

void loop() {
  // Check if characters available in the buffer


  //*************************************************************
  //*** Section 1. Compute position in counts (do not change) ***
  //*************************************************************

  // Get voltage output by MR sensor
  newPosition = myEnc.read();  //current raw position the rotary encoder
  newPosition1 = myEnc1.read();  //current raw position the rotary encoder
//  if (newPosition != oldPosition || newPosition1 != oldPosition1) {
//    // Calculate differences between subsequent rotory encoder readings
//    
//    // Update position record-keeping vairables
//    oldPosition = newPosition;
//
//    // Update position record-keeping vairables
//    oldPosition1 = newPosition1;
//
//    //        Serial.println(newPosition);
//    //          Serial.println(newPosition1);
//    //               Serial.println();
//
//  }

   ts1= -360.0/13824.0*newPosition1; // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
   ts = -360.0/13824.0*newPosition; // Compute the angle of the sector pulley (ts) in degrees based on updatedPos

    th1=ts*3.14159/180.0; 
    th2=ts1*3.14159/180.0; 

angles[0].floating_point = th1;
angles[1].floating_point= th2;

  if (Serial.available() > 0) {
    operation = Serial.read();
    delayMicroseconds(wait_for_transmission); // If not delayed, second character is not correctly read
    mode = Serial.read();
   delayMicroseconds(wait_for_transmission); // If not delayed, second character is not correctly read
    if (Serial.read()==':'){

          for(int i=0; i<4; i++){
           torques[0].binary_array[i]= Serial.read();
          }

            for(int i=0; i<4; i++){
           torques[1].binary_array[i]= Serial.read();
          }


    }

    switch (operation){
    case 'R': // Read operation, e.g. RJ, RP
      if (mode == 'P'){ //  read position
//        return_position(x,y,xdot,ydot);  // this is a function
      }
      else if (mode == 'J'){ // Analog read
        return_joints(angles[0].floating_point, angles[1].floating_point);
      }
      else {
        break; // Unexpected mode
      }
      break;

    case 'W': // Write operation, e.g. WD3:1, WA8:255
      if (mode == 'F'){ //  write force
//        write_force(fx, fy);
      }
      else if (mode == 'T'){ //  write torque
        write_torque(torques[0].floating_point, torques[1].floating_point);
      }
      else {
        break; // Unexpected mode
      }
      break;

    default: // Unexpected char
      break;
    }


}
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




