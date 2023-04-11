#include <Arduino.h>
#include "A4988.h"
#include <CircularBuffer.h>
#include <HCSR04.h>

CircularBuffer<double, 6> buffer;
// using a 200-step motor (most common)
#define MOTOR_STEPS 200
// configure the pins connected///////////////////////Inputs/outputs///////////////////////
// Define the pin connections for the A4988 driver
#define DIR_PIN 14
#define STEP_PIN 32
#define MS1_PIN 13
#define MS2_PIN 12
#define MS3_PIN 27

#define MAX 35
#define MIN -35

#define motorInterfaceType 1
A4988 stepper(MOTOR_STEPS, DIR_PIN, STEP_PIN, MS1_PIN, MS2_PIN, MS3_PIN);
double angle_P, angle_I, angle_D, angle_PI, angle_PID, angle_now;

UltraSonicDistanceSensor distanceSensor(A0, A1); // Initialize sensor that uses digital pins 13 and 12.

double distanceLive = 0.0;
float distancePrecedente;
double temps, tempsPrint; // Variables for time control
double distance_previous_error, distance_error;
int period = 10; // Refresh rate period of the loop is 50ms

///////////////////PID constants///////////////////////
#define KP_MAX 10
#define KI_MAX 2
#define KD_MAX 2000

float target;
float target_now;

double kp = 2;   // Mine was 8
double ki = 0.2; // Mine was 0.2
double kd = 100; // Mine was 3100
double distance_setpoint = 25;
///////////////////////////////////////////////////////

void measureDistance(void);
void Kpot(void);

void setup()
{
  Serial.begin(115200);
  stepper.begin(30, 1); // vitesse est de 1.8°/s
  temps = millis();
  tempsPrint = millis();
  angle_now = 0;
  stepper.setSpeedProfile(stepper.LINEAR_SPEED, 5000, 5000);
}

void loop()
{
  stepper.getAcceleration();
  if (millis() > temps + period)
  {
    temps = millis();
    Kpot();
    measureDistance();

    distance_error = distance_setpoint - distanceLive;

    angle_P = kp * distance_error;

    // Derivée
    float dist_diference = distance_error - distance_previous_error;
    angle_D = kd * ((distance_error - distance_previous_error) / period);

    if (-2 < distance_error && distance_error < 2)
    {
      angle_I = angle_I + (ki * distance_error);
    }
    else
    {
      angle_I = 0;
    }
    // angle_I = angle_I + (ki * distance_error);
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
    printf("distance: %6.2f error: %6.2f angle_PID: %6.2f angle_now: %6.2f ", distanceLive, distance_error, angle_PID, angle_now);
    printf("kp: %f ki: %f kd: %f \n", angle_P, angle_I, angle_D);
  }
}

void measureDistance(void)
{

  while (!buffer.isFull())
  {
    distanceLive = distanceSensor.measureDistanceCm();
    buffer.push(distanceLive);
  }

  if (buffer.isFull())
  {
    // trouver la disntance en ce moment
    distanceLive = 0.00;
    for (int i = 0; i < buffer.size() - 1; i++)
    {
      distanceLive += buffer[i];
    }
    distanceLive = distanceLive / (float)buffer.size();

    // trouver la distance precedente

    distancePrecedente = 0.00;
    for (int i = 1; i < buffer.size(); i++)
    {
      distancePrecedente += buffer[i];
    }
    distancePrecedente = distancePrecedente / (float)buffer.size();

    buffer.shift(); // enleve la derniere valeur pour fifo
  }
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