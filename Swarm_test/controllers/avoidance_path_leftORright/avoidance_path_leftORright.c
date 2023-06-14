#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <stdlib.h>
#include <webots/keyboard.h>


#define TIME_STEP 64

int area_detection(double sensor_area) {
  int turn_back = 0;
  if (sensor_area <= 350){turn_back = 2;}
  else if(sensor_area <= 800){turn_back = 1;}
  return turn_back;
}

int main(int argc, char **argv) {
  wb_robot_init();
  
  wb_keyboard_enable(TIME_STEP);
  int key = 0;
  
  // Initialisation variables
  int i;
  unsigned int total_food = 0;
  bool avoid_obstacle_counter_left = 0;
  bool avoid_obstacle_counter_right = 0;
  bool avoid_obstacle_counter_waiting = 0;
  bool trun_counter = 0;
  bool carrying_food = false;
  bool alreadyWaiting = false;
  
  float max_speed = 6.28;
  int area_localisation = 3;
  
  //char area_name[3][10] = {"path_zone","nest_zone","food_zone"};
  
  // Initialisation led
  WbDeviceTag avoiding_state = wb_robot_get_device("led_avoid");
  WbDeviceTag briging_state = wb_robot_get_device("led_food");
  wb_led_set(avoiding_state, 0);
  wb_led_set(briging_state, 0);
 
  // Initialisation sensor
  WbDeviceTag sensors[3];
  char sensors_name[3][14] = {"left_sensor", "right_sensor","area_detector"};
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
      wb_led_set(avoiding_state, 1); // Led ON (Avoiding state)
      left_speed = 0.0;
      right_speed = 0.0;
      alreadyWaiting = true;
    }
    else if (avoid_obstacle_counter_left > 0){ // Turn to the left
      avoid_obstacle_counter_left--;
      wb_led_set(avoiding_state, 1); // Led ON (Avoiding state)
      left_speed = -max_speed*0.5;
      right_speed = max_speed*0.5;
     }
     else if(avoid_obstacle_counter_right > 0){ // Turn to the right
      avoid_obstacle_counter_right--;
      wb_led_set(avoiding_state, 1); // Led ON (Avoiding state)
      left_speed = max_speed*0.5;
      right_speed = -max_speed*0.5;
      }
     else if(trun_counter > 0){
       trun_counter--;
       if (carrying_food == false){// alternate the rotation for avoiding useless path
       left_speed = -max_speed*0.5;
       right_speed = max_speed*0.5;
       }
       else{
       left_speed = max_speed*0.5;
       right_speed = -max_speed*0.5;
       }
     }
     else { // read sensors
      wb_led_set(avoiding_state, 0); // Led OFF (Normal state)
      double ds_values_left = wb_distance_sensor_get_value(sensors[0]);
      double ds_values_right = wb_distance_sensor_get_value(sensors[1]);
      double ds_values_area = wb_distance_sensor_get_value(sensors[2]);
      
      int last_area_localisation = area_localisation;
      area_localisation = area_detection(ds_values_area);
      
      if (last_area_localisation != area_localisation){ // Detect transition area
        //printf("actual area : %s\n",area_name[area_localisation]);
        if (area_localisation == 2 && carrying_food == false){ // No food and in the food area
          wb_led_set(briging_state, 1);
          carrying_food = true;
          trun_counter=31;
          }    
        }
        if (area_localisation == 1  && carrying_food == true){ // food and in the nest area
        wb_led_set(briging_state, 0);
        carrying_food = false;
        total_food++;
        trun_counter=31;
        //printf("%s Food deposit in the nest !\n",wb_robot_get_name());
        }

      // Avoidance state
      if (ds_values_left < 950.0 && ds_values_right < 950.0){ // Obstacle detected on both side
        if (alreadyWaiting == true){trun_counter=31;} // if we have already wait in the last state, turn back
        else {avoid_obstacle_counter_waiting = 20;} // Waiting state
      }
      else if (ds_values_left < 950.0){avoid_obstacle_counter_left = 5;} // Obstacle detected on the left 
      else if (ds_values_right < 950.0){ avoid_obstacle_counter_right = 5;} // Obstacle detected on the right
      alreadyWaiting = false;
    } 
    wb_motor_set_velocity(wheels[0], left_speed);
    wb_motor_set_velocity(wheels[1], right_speed);
    key = wb_keyboard_get_key();
    if(key == 83){
      printf("%s :\t",wb_robot_get_name());
      printf("%d\t food\n",total_food);
      }
    
  }// end main loop
  wb_robot_cleanup();
  return 0;  // EXIT_SUCCESS
}