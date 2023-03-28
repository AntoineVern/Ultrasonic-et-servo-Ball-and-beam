#include <HCSR04.h>
#include <AccelStepper.h>
#include <PID_v1.h>
#include <A4988.h>

UltraSonicDistanceSensor distanceSensor(A0, A1); // Initialize sensor that uses digital pins 13 and 12.

#define PIN_INPUT A1
#define PIN_OUTPUT 32

// Define Variables we'll be connecting to
double Setpoint, Input, Output;

// Specify the links and initial tuning parameters
double Kp = 0.5, Ki = 0.0, Kd = 0.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

///////////////////////Inputs/outputs///////////////////////
// Define the pin connections for the A4988 driver
#define DIR_PIN 14
#define STEP_PIN 32
#define MS1_PIN 13
#define MS2_PIN 12
#define MS3_PIN 27

#define MAX 40
#define MIN 0
// Define the number of steps per revolution of the stepper motor
int microsteps = 1;
const int stepsPerRevolution = 200 * microsteps;

#define motorInterfaceType 1
AccelStepper stepper(motorInterfaceType, STEP_PIN, DIR_PIN);
int encAngle;
///////////////////////////////////////////////////////

////////////////////////Variables///////////////////////
double distance;
double lastDistance;
int Read = 0;
float elapsedTime, temps, timePrev; // Variables for time control
float distance_previous_error, distance_error;
int period = 50; // Refresh rate period of the loop is 50ms
int steps = 0;
///////////////////////////////////////////////////////

float measureDistance()
{
  return distanceSensor.measureDistanceCm();
}

void setMicrostep(int steps)
{
  if (steps == 1)
  {
    digitalWrite(MS1_PIN, LOW);
    digitalWrite(MS2_PIN, LOW);
    digitalWrite(MS3_PIN, LOW);
  }
  else if (steps == 2)
  {
    digitalWrite(MS1_PIN, HIGH);
    digitalWrite(MS2_PIN, LOW);
    digitalWrite(MS3_PIN, LOW);
  }
  else if (steps == 4)
  {
    digitalWrite(MS1_PIN, LOW);
    digitalWrite(MS2_PIN, HIGH);
    digitalWrite(MS3_PIN, LOW);
    microsteps = 4;
  }
  else if (steps == 8)
  {
    digitalWrite(MS1_PIN, HIGH);
    digitalWrite(MS2_PIN, HIGH);
    digitalWrite(MS3_PIN, LOW);
    microsteps = 8;
  }
  else if (steps == 16)
  {
    digitalWrite(MS1_PIN, HIGH);
    digitalWrite(MS2_PIN, HIGH);
    digitalWrite(MS3_PIN, HIGH);
  }
}

void safety(int steps)
{
  steps = steps > MAX ? MAX : steps;
  steps = steps < MIN ? MIN : steps;
}

void rotate(int steps, bool clockwise)
{
  // Set the direction of rotation
  digitalWrite(DIR_PIN, clockwise ? HIGH : LOW);
  safety(steps);
  // Step the motor the specified number of steps
  for (int i = 0; i < steps; i++)
  {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(500);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(500);
  }
}

void tourner_angle(double angle, bool clockwise)
{
  //deplacer(angle);
  int steps = (angle * stepsPerRevolution) / 360;
  rotate(steps, clockwise);
}

void setup()
{
  // Initialize the serial port
  Serial.begin(115200);
  Input = measureDistance();
  Setpoint = 20;
  // Set the A4988 driver pins as outputs
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(MS1_PIN, OUTPUT);
  pinMode(MS2_PIN, OUTPUT);
  pinMode(MS3_PIN, OUTPUT);
  temps = millis();
  setMicrostep(1);
  // turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-40, 40);
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);
}

void loop()
{
  // Set the target position for the stepper motor
  int targetPosition = 100;

  // Move the stepper motor to the target position
  stepper.moveTo(targetPosition);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }

  // Delay for 1 second before moving to the next position
  delay(1000);
  // if (millis() > temps + period)
  // {
  //   if (Input < 0)
  //   {
  //     tourner_angle(Output, true);
  //     steps++;
  //   }
  //   else
  //   {
  //     tourner_angle(Output, false);
  //     steps--;
  //   }
  //   Serial.print("steps: ");
  //   Serial.print(steps);
  //   rotate(1, true);
  //   Input = measureDistance();
  //   Serial.print("  Input: ");
  //   Serial.print(Input);
  //   myPID.Compute();
  //   // analogWrite(PIN_OUTPUT, Output);

    
  //   Serial.print(" Output: ");
  //   Serial.print(Output);
  //   Serial.print(" error: ");
  //   Serial.println(Setpoint - Input);
    
    // myservo.write(PID_total+30);
  // }
}
