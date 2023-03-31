#include <Arduino.h>
#include "A4988.h"

// using a 200-step motor (most common)
#define MOTOR_STEPS 200
// configure the pins connected///////////////////////Inputs/outputs///////////////////////
// Define the pin connections for the A4988 driver
#define DIR_PIN 14
#define STEP_PIN 32
#define MS1_PIN 13
#define MS2_PIN 12
#define MS3_PIN 27
#define set_point 20

#define MAX 45
#define MIN -45

#define motorInterfaceType 1
A4988 stepper(MOTOR_STEPS, DIR_PIN, STEP_PIN, MS1_PIN, MS2_PIN, MS3_PIN);
double angle_P, angle_I, angle_D, angle_PI, angle_PID, angle_now;

#include <HCSR04.h>
UltraSonicDistanceSensor distanceSensor(A0, A1); // Initialize sensor that uses digital pins 13 and 12.

double distance = 0.0;
double temps, tempsPrint; // Variables for time control
double distance_previous_error, distance_error;
int period = 50; // Refresh rate period of the loop is 50ms

///////////////////PID constants///////////////////////
#define KP_MAX 10
#define KI_MAX 2
#define KD_MAX 2000

float target;
float target_now;

double kp = 4;                 // Mine was 8
double ki = 0.2;               // Mine was 0.2
double kd = 2000;              // Mine was 3100
double distance_setpoint = 25; // Should be the distance from sensor to the middle of the bar in mm
///////////////////////////////////////////////////////

float current_angle = 0;

double limit(void);
float measureDistance();
void Kpot(void);
float position(float distance);

void setup()
{
  Serial.begin(115200);
  stepper.begin(60, 2); // vitesse est de 1.8°/s
  temps = millis();
  tempsPrint = millis();
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

    if (-5 < distance_error && distance_error < 8)
    {
      angle_I = angle_I + (ki * distance_error);
    }
    else
    {
      angle_I = 0;
    }
    //angle_I = angle_I + (ki * distance_error);
    angle_PI = angle_P + angle_I;
    angle_PID = angle_P + angle_I + angle_D;

    // stepper.rotate(target);double target_angle = angle_now + angle;
    double target_angle = angle_now + angle_PID;
    target_angle = target_angle > MAX ? MAX : target_angle;
    target_angle = target_angle < MIN ? MIN : target_angle;
    double rotation = target_angle - angle_now;
    stepper.rotate(rotation);
    angle_now = target_angle;
    distance_previous_error = distance_error;
  }

  if (millis() > tempsPrint + 300)
  {
    tempsPrint = millis();
    printf("distance: %6.2f error: %6.2f angle: %6.2f angle_now: %6.2f ", distance, distance_error, angle_PID, angle_now);
    printf("kp: %f ki: %f kd: %f \n", angle_P, angle_I, angle_D);
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

float position(float distance)
{
  target_now = set_point - distance;
  return target;
}

void Kpot(void)
{
  uint16_t ADC_Value1 = analogRead(A2);
  uint16_t ADC_Value2 = analogRead(A3);
  uint16_t ADC_Value3 = analogRead(A4);

  // kp = KP_MAX * ADC_Value1/ 4096.0;
  // ki = KI_MAX * ADC_Value2/ 4096.0;
  // kd = KD_MAX * ADC_Value3/ 4096.0;
}