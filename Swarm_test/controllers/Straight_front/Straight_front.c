
#include <webots/motor.h>
#include <webots/robot.h>
#include <stdlib.h>

#define TIME_STEP 64

int main(int argc, char **argv) {
  wb_robot_init();
  int i;
  float max_speed = 6.28;
  
  float a = 0.1;
  
  WbDeviceTag wheels[2];
  char wheels_names[2][8] = {"motor_1", "motor_2"};
  for (i = 0; i < 2; i++) {
    wheels[i] = wb_robot_get_device(wheels_names[i]);
    wb_motor_set_position(wheels[i], INFINITY);
  }
  
  while (wb_robot_step(TIME_STEP) != -1) {
  
    float rand_speed_1 = ((float)rand()/(float)(RAND_MAX)) * a;
    float rand_speed_2 = ((float)rand()/(float)(RAND_MAX)) * a;
    
    double left_speed = (0.5+rand_speed_1) * max_speed;
    double right_speed = (0.5+rand_speed_2) * max_speed;

    wb_motor_set_velocity(wheels[0], left_speed);
    wb_motor_set_velocity(wheels[1], right_speed);
    
    printf("%1.3f \n",rand_speed_1);
    printf("%1.3f \n",rand_speed_2);
    
  }
  wb_robot_cleanup();
  return 0;  // EXIT_SUCCESS
}