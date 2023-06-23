#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <stdio.h>
#include <string.h>
#include <webots/gps.h>
#include <math.h>
#include <webots/pen.h>

#define TIME_STEP 64
#define PI 3.14159

int area_detection(double sensor_area) {
  int turn_back = 0;
  if (sensor_area <= 350){turn_back = 2;}
  else if(sensor_area <= 800){turn_back = 1;}
  return turn_back;
}

int main(int argc, char **argv) {
  wb_robot_init();
  
  // Initialisation variables
  int i;
  bool avoid_obstacle_counter_left = 0;
  bool avoid_obstacle_counter_right = 0;
  bool avoid_obstacle_counter_waiting = 0;
  bool trun_counter = 0;
  bool alreadyWaiting = false;
  bool counterClock_turn = false;
  bool carringFood = false;

  
  float max_speed = 6.28;
  int area_localisation = 3;

  
  // Initialisation led
  WbDeviceTag leds[2];
  char leds_name[2][10] = {"led_avoid","led_food"};
  for (i = 0; i < 2;i++){
    leds[i] = wb_robot_get_device(leds_name[i]);
    wb_led_set(leds[i],0);
  }
 
  // Initialisation sensor
  WbDeviceTag sensors[5];
  char sensors_name[5][20] = {"left_sensor", "right_sensor","area_detector"};
  for (i = 0; i < 3; i++) {
    sensors[i] = wb_robot_get_device(sensors_name[i]);
    wb_distance_sensor_enable(sensors[i], TIME_STEP);
  }
 
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

    // Turn until the counter reach 0
    if (avoid_obstacle_counter_waiting >0){ // Obstacle in both side
      avoid_obstacle_counter_waiting--;
      wb_led_set(leds[0], 1); // Led ON (Avoiding state)
      left_speed = 0.0;
      right_speed = 0.0;
      alreadyWaiting = true;
    }
    else if (avoid_obstacle_counter_left > 0){ // Turn to the left
      avoid_obstacle_counter_left--;
      wb_led_set(leds[0], 1); // Led ON (Avoiding state)
      left_speed = -max_speed*0.5;
      right_speed = max_speed*0.5;
    }
    else if(avoid_obstacle_counter_right > 0){ // Turn to the right
      avoid_obstacle_counter_right--;
      wb_led_set(leds[0], 1); // Led ON (Avoiding state)
      left_speed = max_speed*0.5;
      right_speed = -max_speed*0.5;
    }
     else if(trun_counter > 0){
       trun_counter--;
       if (counterClock_turn == false){// alternate the rotation for avoiding shifting
         left_speed = -max_speed*0.5;
         right_speed = max_speed*0.5;
       }
       else{
         left_speed = max_speed*0.5;
         right_speed = -max_speed*0.5;   
       }
     }
     else { // read sensors
      wb_led_set(leds[0], 0); // Led OFF (Normal state)
      double ds_values_left = wb_distance_sensor_get_value(sensors[0]);
      double ds_values_right = wb_distance_sensor_get_value(sensors[1]);
      double ds_values_area = wb_distance_sensor_get_value(sensors[2]);
      
      // Store the area in the last state
      int last_area_localisation = area_localisation;
      area_localisation = area_detection(ds_values_area);
      
      if (last_area_localisation != area_localisation){ // Detect transition area
        if (area_localisation == 2 && carringFood == false){ // In the food area
          wb_led_set(leds[1], 1);
          carringFood = true;
          trun_counter = 31; // turn back
          counterClock_turn = !counterClock_turn;
        }
        else if(area_localisation == 1 && carringFood == true){ // In the nest area
          wb_led_set(leds[1], 0);
          carringFood = false;
          trun_counter = 31; // turn back
          counterClock_turn = !counterClock_turn;
        } 
      }
      
      // --- Avoidance state ---
      if (ds_values_left < 700.0 && ds_values_right < 700.0){ // Obstacle detected on both side
        if (alreadyWaiting == true){trun_counter=31;} // if we have already wait in the last state, turn back
        else {avoid_obstacle_counter_waiting = 20;} // Waiting state
      }
      else if (ds_values_left < 950.0){avoid_obstacle_counter_right = 6;} // Obstacle detected on the left 
 
      else if (ds_values_right < 950.0){avoid_obstacle_counter_left = 6;}  // Obstacle detected on the right

      alreadyWaiting = false;
    }
    // Move the robot
    wb_motor_set_velocity(wheels[0], left_speed);
    wb_motor_set_velocity(wheels[1], right_speed);
    
  }// end main loop
  wb_robot_cleanup();
  return 0;  // EXIT_SUCCESS
}
