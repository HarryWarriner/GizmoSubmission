#ifndef PIO_UNIT_TESTING
#include <Arduino.h>
#endif
#include <stdio.h>
#include <math.h>
#include "debug.h"
#include "motors.h"
#include "brain.h"

// Local functions

void stepMotor(int stepPin);
//
//
unsigned long lastStepTimeX = 0;
unsigned long lastStepTimeY = 0;
unsigned long lastStepTimeZ = 0;
#ifndef PIO_UNIT_TESTING

// Pin Definitions for Motors
const int stepX = 2;
const int dirX = 5;

const int stepY = 3;
const int dirY = 6;

const int stepZ = 4;
const int dirZ = 7;
const int enPin = 8;
//
void motor_init()
{
  // Initialize motor pins
  pinMode(stepX, OUTPUT);
  pinMode(dirX, OUTPUT);
  pinMode(stepY, OUTPUT);
  pinMode(dirY, OUTPUT);
  pinMode(stepZ, OUTPUT);
  pinMode(dirZ, OUTPUT);
  pinMode(enPin, OUTPUT);

  // Enable motors
  digitalWrite(enPin, LOW);

  //
  debug_printf("Motor init complete\n");
}
#endif

extern float gx, gy, gz;
void print_movement(Movement &movement)
{
  debug_printf("MOVEMENT:: gx,gy: %.2f, %.2f",
               gx, gy);

  debug_printf("Speed X: %.2f, Speed Y: %.2f, Speed Z: %.2f ", movement.speedX, movement.speedY, movement.speedZ);
  debug_printf(" Interval %lu, %lu, %lu\n", speed_to_step_interval(movement.speedX),
               speed_to_step_interval(movement.speedY),
               speed_to_step_interval(movement.speedZ));
}
unsigned long speed_to_step_interval(double speed)
{
  if (fabs(speed) < 0.01)
    return 0;
  speed = fabs(speed);
  if (speed > 1.0)
    speed = 1.0;
  //
  double stepinterval = (1 - fabs(speed)) * 10000 + 5000;
  return stepinterval;
}

#ifndef PIO_UNIT_TESTING
#define motorInterval 500
unsigned long lastMotorTime = 0;

void motor_move_if_needed(Movement &movementData, bool debug)
{
  unsigned long currentTime = micros();
  if (currentTime - lastMotorTime < motorInterval)
  {
    return;
  }
  //
  // Change this to reverse X and Y axis
  //
  bool reverseXY = false;
  if (reverseXY)
  {
    double temp = movementData.speedX;
    movementData.speedX = movementData.speedY;
    movementData.speedY = temp;
  }
  digitalWrite(dirX, (movementData.speedX >= 0) ? HIGH : LOW);
  digitalWrite(dirY, (movementData.speedY >= 0) ? HIGH : LOW);
  digitalWrite(dirZ, (movementData.speedZ >= 0) ? HIGH : LOW);

  unsigned long stepIntervalX = speed_to_step_interval(movementData.speedX);
  unsigned long stepIntervalY = speed_to_step_interval(movementData.speedY);
  unsigned long stepIntervalZ = speed_to_step_interval(movementData.speedZ);
  if (debug)
  {
    debug_printf("MotorRunning: gx,gy: %.2f, %.2f", gx, gy);
    debug_printf(" :: %.2f, %.2f, %.2f, ",movementData.speedX, movementData.speedY, movementData.speedZ);
    debug_printf("Intervals: %lu, %lu, %lu\n",stepIntervalX, stepIntervalY, stepIntervalZ);
  }

  if (stepIntervalX != 0 && (currentTime - lastStepTimeX >= stepIntervalX))
  {
    lastStepTimeX = currentTime; // Update to current time
    stepMotor(stepX);            // Step Motor X
  }

  if (stepIntervalY != 0 && (currentTime - lastStepTimeY >= stepIntervalY))
  {
    lastStepTimeY = currentTime; // Update to current time
    stepMotor(stepY);            // Step Motor Y
  }

  if (stepIntervalZ != 0 && (currentTime - lastStepTimeZ >= stepIntervalZ))
  {
    lastStepTimeZ = currentTime; // Update to current time
    stepMotor(stepZ);            // Step Motor Z
  }
}

void sleep(unsigned long time)
{
  unsigned long startTime = micros();
  while (micros() - startTime < time)
  {
  }
}

void spin_motor(char motor)
{

  switch (motor)
  {
  case '1':
    debug_printf("Spinning motor X\n");
    digitalWrite(dirX, HIGH);
    for (int i = 0; i < 200; i++)
    {
      digitalWrite(dirX, HIGH);
      stepMotor(stepX);
      sleep(5000);
    }
    break;
  case '2':
    debug_printf("Spinning motor Y\n");
    digitalWrite(dirY, HIGH);
    for (int i = 0; i < 200; i++)
    {
      digitalWrite(dirY, HIGH);
      stepMotor(stepY);
      sleep(5000);
    }
    break;
  case '3':
    debug_printf("Spinning motor Z\n");
    digitalWrite(dirZ, HIGH);
    for (int i = 0; i < 200; i++)
    {
      stepMotor(stepZ);
      sleep(5000);
    }
    break;
  case 'A':
    debug_printf("Spinning motor ALL\n");
    digitalWrite(dirX, HIGH);
    digitalWrite(dirY, HIGH);
    digitalWrite(dirZ, HIGH);
    for (int i = 0; i < 200; i++)
    {
      stepMotor(stepX);
      stepMotor(stepY);
      stepMotor(stepZ);
      sleep(5000);
    }
    break;
  case 'B':
    debug_printf("Spinning motor ALL BACK\n");
    digitalWrite(dirX, LOW);
    digitalWrite(dirY, LOW);
    digitalWrite(dirZ, LOW);
    for (int i = 0; i < 200; i++)
    {
      stepMotor(stepX);
      stepMotor(stepY);
      stepMotor(stepZ);
      sleep(5000);
    }
    break;
  }
}

void stepMotor(int stepPin)
{
  digitalWrite(stepPin, LOW);
  unsigned long pulseStartTime = micros();
  while (micros() - pulseStartTime < 10)
  {
  }
  digitalWrite(stepPin, HIGH);
  pulseStartTime = micros();
  while (micros() - pulseStartTime < 10)
  {
  }
  digitalWrite(stepPin, LOW);
}
#endif