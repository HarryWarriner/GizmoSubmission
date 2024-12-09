#ifndef _MOTORS_H
#define _MOTORS_H
#include "brain.h"

void motor_init();
void motor_dump();
void motor_move_if_needed(Movement &movementData, bool debug);

unsigned long speed_to_step_interval(double speed);
void spin_motor(char motor);


#endif