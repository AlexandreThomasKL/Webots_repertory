#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <stdio.h>
#include <string.h>
#include <webots/keyboard.h>
#include <webots/gps.h>
#include <math.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

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
  bool trun_counter = 0;
  bool carrying_food = false;
  
  
  float max_speed = 0;
  int area_localisation = 3;
  
  // Initilaisation GPS
  WbDeviceTag gp = wb_robot_get_device("gps");
  wb_gps_enable(gp,TIME_STEP);
  
  // Initialisation antenna
  WbDeviceTag tx = wb_robot_get_device("emitter");
  WbDeviceTag rx = wb_robot_get_device("receiver");
  
  wb_receiver_enable(rx, TIME_STEP);
  wb_receiver_set_channel(rx,5);
  wb_emitter_set_channel(tx,5);
  
  // Initialisation led
  WbDeviceTag leds[2];
  char leds_name[2][10] = {"led_avoid","led_food"};
  for (i = 0; i < 2;i++){
    leds[i] = wb_robot_get_device(leds_name[i]);
    wb_led_set(leds[i],0);
  }
 
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

   
    
    
       if(trun_counter > 0){
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
      wb_led_set(leds[0], 0); // Led OFF (Normal state)
      double ds_values_left = wb_distance_sensor_get_value(sensors[0]);
      double ds_values_right = wb_distance_sensor_get_value(sensors[1]);
      double ds_values_area = wb_distance_sensor_get_value(sensors[2]);
      
      if (area_detection(ds_values_area)==2){ 
          wb_led_set(leds[1], 1);
          carrying_food = true;
          max_speed = 0.0;
          double gps_value[2] = {wb_gps_get_values(gp)[0],wb_gps_get_values(gp)[2]};
          char message[128];
          //printf("%s found the Source ! broadcast the coorinates...\n",wb_robot_get_name());
          sprintf(message, "%.3lf %.3lf\n", gps_value[0],gps_value[1]);
          wb_emitter_send(tx,message,strlen(message)+1);
    
              
        }
    }

    wb_motor_set_velocity(wheels[0], left_speed);
    wb_motor_set_velocity(wheels[1], right_speed);
  
  }// end main loop
  wb_robot_cleanup();
  return 0;  // EXIT_SUCCESS
}

