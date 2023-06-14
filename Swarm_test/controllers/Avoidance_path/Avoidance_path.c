#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <stdlib.h>
#include <webots/keyboard.h>

#define TIME_STEP 64

int main(int argc, char **argv) {
  wb_robot_init();
  
  // Initialisation sensor & led
  WbDeviceTag sensor = wb_robot_get_device("front_sensor");
  WbDeviceTag led = wb_robot_get_device("led_avoid");
  wb_distance_sensor_enable(sensor, TIME_STEP);
  
  wb_keyboard_enable(TIME_STEP);
  
  wb_led_set(led, 0);
 
  // Initialisation variables
  int i;
  //int key = 0;
  //unsigned int tim = 0;
  bool avoid_obstacle_counter = 0;
  float max_speed = 6.28;
 
 
  // Initialisation motor
  WbDeviceTag wheels[2];
  char wheels_names[2][8] = {"motor_1", "motor_2"};
  for (i = 0; i < 2; i++) {
    wheels[i] = wb_robot_get_device(wheels_names[i]);
    wb_motor_set_position(wheels[i], INFINITY);
  }
  
  // Main loop
  while (wb_robot_step(TIME_STEP) != -1) {
    // Forward direction
    double left_speed = max_speed*0.5;
    double right_speed = max_speed*0.5;
    
    //key = wb_keyboard_get_key();
    
    // Turn until the counter reach 0
    if (avoid_obstacle_counter > 0) {
      avoid_obstacle_counter--;
      wb_led_set(led, 1); // Led ON (Avoiding state)
      left_speed = max_speed*0.5;
      right_speed = -max_speed*0.5;
    } else { // read sensors
       //tim += TIME_STEP;
       wb_led_set(led, 0); // Led OFF (Normal state)
      double ds_values = wb_distance_sensor_get_value(sensor);
      if (ds_values < 950.0) { // Obstacle detected
        avoid_obstacle_counter = 25;
      }
    }
    wb_motor_set_velocity(wheels[0], left_speed);
    wb_motor_set_velocity(wheels[1], right_speed);
    /*
    if(key == 83){
      wb_led_set(led, 1);
      max_speed = 0;
      printf("%s :\t",wb_robot_get_name());
      printf("%d\t ms performence\n",tim);
      }
     if(key == 90){
      wb_led_set(led, 1);
      max_speed = 6.28;
      
    }*/
  }
  wb_robot_cleanup();
  return 0;  // EXIT_SUCCESS
}