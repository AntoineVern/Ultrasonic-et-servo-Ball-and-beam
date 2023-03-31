#include <Arduino.h>
#include "A4988.h"
//hi
// using a 200-step motor (most common)
#define MOTOR_STEPS 200
// configure the pins connected///////////////////////Inputs/outputs///////////////////////
// Define the pin connections for the A4988 driver
#define DIR_PIN 14
#define STEP_PIN 32
#define MS1_PIN 13
#define MS2_PIN 12
#define MS3_PIN 27

#define MAX 45
#define MIN -45

#define motorInterfaceType 1
A4988 stepper(MOTOR_STEPS, DIR_PIN, STEP_PIN, MS1_PIN, MS2_PIN, MS3_PIN);
double angle_P, angle_I, angle_D, angle_PID, angle_now;

#include <HCSR04.h>
UltraSonicDistanceSensor distanceSensor(A0, A1); // Initialize sensor that uses digital pins 13 and 12.

double distance = 0.0;
double temps; // Variables for time control
double distance_previous_error, distance_error;
int period = 50; // Refresh rate period of the loop is 50ms

///////////////////PID constants///////////////////////
double kp = 2;                 // Mine was 8
double ki = 0.2;               // Mine was 0.2
double kd = 3100;              // Mine was 3100
double distance_setpoint = 20; // Should be the distance from sensor to the middle of the bar in mm
///////////////////////////////////////////////////////

double limit(void);
float measureDistance();
void Kpot(void);

void setup()
{
  Serial.begin(115200);
  stepper.begin(60, 1); // vitesse est de 1.8°/s
  temps = millis();
  angle_now = 0;
}

void loop()
{

  if (millis() > temps + period)
  {
    temps = millis();
    Kpot();
    distance = measureDistance();
    distance_error = distance_setpoint - distance;

    angle_P = kp * distance_error;

    float dist_diference = distance_error - distance_previous_error;
    angle_D = kd * ((distance_error - distance_previous_error) / period);

    if (-3 < distance_error && distance_error < 3)
    {
      angle_I = angle_I + (ki * distance_error);
    }
    else
    {
      angle_I = 0;
    }

    angle_PID = angle_P + angle_I + angle_D;
    angle_PID = map(angle_PID, -150, 150, 0, 150);

    if (angle_PID < 20)
    {
      angle_PID = 20;
    }
    if (angle_PID > 160)
    {
      angle_PID = 160;
    }

    stepper.rotate(limit());
    distance_previous_error = distance_error;
    printf("distance: %f error: %f angle: %f ", distance, distance_error, angle_P);
    printf("kp: %f ki: %f kd: %f \n", kp, ki, kd);
  }
}
float measureDistance()
{
  return distanceSensor.measureDistanceCm();
}

double limit(void)
{
  double target_angle = angle_now + angle_P;
  target_angle = target_angle > MAX ? MAX : target_angle;
  target_angle = target_angle < MIN ? MIN : target_angle;
  double rotation = target_angle - angle_now;
  angle_now = target_angle;
  return rotation;
}

void Kpot(void)
{
  uint16_t ADC_Value1 = analogRead(A2);
  uint16_t ADC_Value2 = analogRead(A3);
  uint16_t ADC_Value3 = analogRead(A4);

  // kp = 10.0 * ADC_Value1/ 4096.0;
  // ki = 1.0 * ADC_Value2/ 4096.0;
  // kd = 10000.0 * ADC_Value3/ 4096.0;
}