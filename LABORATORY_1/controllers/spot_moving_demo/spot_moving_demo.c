/*
 * Copyright 1996-2024 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <webots/camera.h>
#include <webots/device.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define NUMBER_OF_LEDS 8
#define NUMBER_OF_JOINTS 12
#define NUMBER_OF_CAMERAS 5

static WbDeviceTag motors[NUMBER_OF_JOINTS];
static const char *motor_names[NUMBER_OF_JOINTS] = {
  "front left shoulder abduction motor",  "front left shoulder rotation motor",  "front left elbow motor",
  "front right shoulder abduction motor", "front right shoulder rotation motor", "front right elbow motor",
  "rear left shoulder abduction motor",   "rear left shoulder rotation motor",   "rear left elbow motor",
  "rear right shoulder abduction motor",  "rear right shoulder rotation motor",  "rear right elbow motor"};

static void step() {
  const double time_step = wb_robot_get_basic_time_step();
  if (wb_robot_step(time_step) == -1) {
    wb_robot_cleanup();
    exit(0);
  }
}

static void movement_decomposition(const double *target, double duration) {
  const double time_step = wb_robot_get_basic_time_step();
  const int n_steps_to_achieve_target = duration * 1000 / time_step;
  double step_difference[NUMBER_OF_JOINTS];
  double current_position[NUMBER_OF_JOINTS];

  for (int i = 0; i < NUMBER_OF_JOINTS; ++i) {
    current_position[i] = wb_motor_get_target_position(motors[i]);
    step_difference[i] = (target[i] - current_position[i]) / n_steps_to_achieve_target;
  }

  for (int i = 0; i < n_steps_to_achieve_target; ++i) {
    for (int j = 0; j < NUMBER_OF_JOINTS; ++j) {
      current_position[j] += step_difference[j];
      wb_motor_set_position(motors[j], current_position[j]);
    }
    step();
  }
}

static void stand_up(double duration) {
  const double motors_target_pos[NUMBER_OF_JOINTS] = {-0.1, 0.0, 0.0,  
                                                       0.1,  0.0, 0.0,  
                                                      -0.1, 0.0, 0.0,  
                                                       0.1,  0.0, 0.0};
  movement_decomposition(motors_target_pos, duration);
}

static void walk_forward(double duration) {
  const double shift_pos[NUMBER_OF_JOINTS] = {
    -0.1, 0.1, 0.0,   
     0.1, -0.1, 0.0,  
    -0.1, 0.1, 0.0,   
     0.1, -0.1, 0.0   
  };
  movement_decomposition(shift_pos, duration / 3);

  const double motors_target_pos_1[NUMBER_OF_JOINTS] = {
    -0.15, 0.2, -0.2,   
     0.1, 0.0,  0.0,   
    -0.1, 0.0,  0.0,   
     0.15, -0.2,  0.2   
  };
  movement_decomposition(motors_target_pos_1, duration / 3);

  movement_decomposition(shift_pos, duration / 3);

  const double motors_target_pos_2[NUMBER_OF_JOINTS] = {
     0.1, 0.0,  0.0,   
    -0.15, -0.2, 0.2,   
     0.15, 0.2, -0.2,   
    -0.1, 0.0,  0.0    
  };
  movement_decomposition(motors_target_pos_2, duration / 3);
}

int main(int argc, char **argv) {
  wb_robot_init();
  const double time_step = wb_robot_get_basic_time_step();

  for (int i = 0; i < NUMBER_OF_JOINTS; ++i)
    motors[i] = wb_robot_get_device(motor_names[i]);

  while (true) {
    stand_up(1.0);
    walk_forward(1.0);
  }

  wb_robot_cleanup();
  return EXIT_FAILURE;
}
