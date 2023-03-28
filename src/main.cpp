#include <HCSR04.h>
#include <Servo.h>

UltraSonicDistanceSensor distanceSensor(A0, A1);  // Initialize sensor that uses digital pins 13 and 12.

///////////////////////Inputs/outputs///////////////////////
Servo myservo;  // create servo object to control a servo, later attatched to D9
///////////////////////////////////////////////////////
//
////////////////////////Variables///////////////////////
int Read = 0;
float distance = 0.0;
float elapsedTime, temps, timePrev;        //Variables for time control
float distance_previous_error, distance_error;
int period = 50;  //Refresh rate period of the loop is 50ms
///////////////////////////////////////////////////////

///////////////////PID constants///////////////////////
float kp=8; //Mine was 8
float ki=0.2; //Mine was 0.2
float kd=3100; //Mine was 3100
float distance_setpoint = 21;           //Should be the distance from sensor to the middle of the bar in mm
float PID_p, PID_i, PID_d, PID_total;
///////////////////////////////////////////////////////

float measureDistance ()
{
  return distanceSensor.measureDistanceCm();
}

void setup () {
    Serial.begin(9600);  // We initialize serial connection so that we could print values from sensor.
    myservo.attach(9);  // attaches the servo on pin 9 to the servo object
    myservo.write(125); //Put the servco at angle 125, so the balance is in the middle
    temps = millis();
}

void loop () {

  if (millis() > temps+period)
  {
    temps = millis();    
    distance = measureDistance();   
    Serial.println(distance);
    distance_error = distance_setpoint - distance;   
    PID_p = kp * distance_error;
    float dist_diference = distance_error - distance_previous_error;     
    PID_d = kd*((distance_error - distance_previous_error)/period);
      
    if(-3 < distance_error && distance_error < 3)
    {
      PID_i = PID_i + (ki * distance_error);
    }
    else
    {
      PID_i = 0;
    }
  
    PID_total = PID_p + PID_i + PID_d;  
    PID_total = map(PID_total, -150, 150, 0, 150);
  
    if(PID_total < 20){PID_total = 20;}
    if(PID_total > 160) {PID_total = 160; } 
  
    myservo.write(PID_total+30);  
    distance_previous_error = distance_error;
  }
}
