// Alex Marlow & Jake McKenzie    this program balances a ball on a beam. bceause of the PID controller it
// is specific to our ball and beam setup.

//Ki = 3.5394              original gain terms that did not work
//Kp = 36.4019
//Kd = 93.5953

#include <Servo.h>
#include <Wire.h>

#include "SparkFun_VL53L1X_Arduino_Library.h"
VL53L1X distanceSensor;

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int Time;
int LastTime;
int DelT;
float sum = 0;
float angle = 0;
float yLast = 0;
//255
int center = 255;
//93
int level = 93;
//1
float a = .2;

//gain terms
int Kp = 36.4019;
int Ki = 4.5394;
int Kd = 93.5953;

void setup() {

  LastTime = 0;

   Wire.begin();

  Serial.begin(9600);
  
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object

 myservo.write(level); 
 delay(500);
 
  while (distanceSensor.begin() == false)
    Serial.println("Sensor offline!");
//  distanceSensor.setDistanceMode(1);
  myservo.write(level);              // tell servo to go to position in variable 'pos'
    delay(15);
   // distanceSensor.setDistanceMode();
    Time = millis();
}

void loop() {
 while (distanceSensor.newDataReady() == false) {
    delay(5);
 }
    Time = millis();
    DelT = Time - LastTime;
    
   while(DelT < 50) {      // ensuring 50 ms has passed
      Time = millis();
      DelT = Time - LastTime;
   }
   int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor

   LastTime = Time;                                 //giving the center a little bit of a wider rainge
  if(distance < center +1 && distance > center -1) {    //because of sensor variance
    distance = center;
  }
  angle = getAngle(distance);               //get angle
  if(angle >15) {                           //turn result into angle
    myservo.write(level +75);
    delay(5);
  } else if(angle < -15) {
    myservo.write(level - 75);
    delay(5);
  } else {
    myservo.write(angle*10 + level);
    delay(5);
  }
 //Serial.print(((float(distance - 235)))/1000,5);
 // Serial.print(",");
//  Serial.print(angle, 5);

// Serial.print(",");
//   Serial.print(DelT);
  Serial.println();
//  } else {
//    Serial.print("yeet got 0");
  

}

float getAngle(int Dist) {
  float yCurrent;
  float maxSum = 10;
  float dist = ((float)Dist-center)/1000;  //mm to m and zero offset
  
  float first = Kp*dist;  //first term
  Serial.print(first,5);
  Serial.print(",");
  //second term stuff
  sum = sum + dist*0.05;      //integral sum
  
  sum = min(max(sum, -maxSum), maxSum);
  float second = sum*Ki;   //second term
   Serial.print(second,5);
   Serial.print(",");

  yCurrent = (a*dist +(1-a)*yLast);          //filter

  float Der = (yCurrent - yLast)/.05;      //derivative

  float third = Der*Kd;            //third term
  yLast = yCurrent;
  Serial.print(third,5);
  
  return first + second + third;
}
