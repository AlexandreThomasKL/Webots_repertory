#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <stdio.h>
#include <string.h>
#include <webots/gps.h>
#include <math.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/compass.h>
#include <webots/pen.h>

#define TIME_STEP 64
#define PI 3.14159

int area_detection(double sensor_area) {
  int turn_back = 0;
  if (sensor_area <= 350){turn_back = 2;}
  else if(sensor_area <= 800){turn_back = 1;}
  return turn_back;
}

int rotation_indicator (double food_position_x,double food_position_y,WbDeviceTag cp,WbDeviceTag gp){
      double gps_diff[2] = {food_position_x-wb_gps_get_values(gp)[0],food_position_y-wb_gps_get_values(gp)[1]};
      const double *north = wb_compass_get_values(cp);
      
      
      /*
      double rad = fmod(-(atan2(north[1], north[0])-PI/2),PI);
      double angle_cal = fmod((atan2(gps_diff[1],gps_diff[0])-PI/2),PI);
      double rad = -(atan2(north[1], north[0])-PI/2);
      if (fmod(rad,PI) >= 0){rad = rad -2*PI;}
      double angle_cal = fmod((atan2(gps_diff[1],gps_diff[0])-PI/2),PI);
      */
      
      double rad =atan2(north[1], north[0])-PI/2;
      double angle_cal = atan2(gps_diff[1],gps_diff[0])-PI/2;
      
      printf("%s\n",wb_robot_get_name());
      printf("vect : %lf %lf\n",gps_diff[0],gps_diff[1]);
      printf("angle North : %lf\n",rad);
      printf("angle Source : %lf\n",angle_cal);

      //double rotation = fmod(angle_cal - rad,2*PI);
      double rotation = fmod(angle_cal + rad,PI*2);
      //if (fmod(rotation,PI) >= 0){rotation = -rotation;}
      printf("angle to rotate :%lf\n", rotation);
      
      int n = round((rotation) / 0.101);

  return n;
}

int main(int argc, char **argv) {
  wb_robot_init();
  
  // Initialisation variables
  int i;
  bool avoid_obstacle_counter_left = 0;
  bool avoid_obstacle_counter_right = 0;
  bool avoid_obstacle_counter_waiting = 0;
  bool trun_counter = 0;
  bool counterClock_turn = false;
  bool alreadyWaiting = false;
  bool sourceHasBefound = false;
  
  double food_position[2];
  
  char message[128];
  
  float max_speed = 6.28;
  int area_localisation = 3;
  
  // Initilaisation GPS & compass
  WbDeviceTag gp = wb_robot_get_device("gps");
  WbDeviceTag cp = wb_robot_get_device("compass");
  wb_gps_enable(gp,TIME_STEP);
  wb_compass_enable(cp,TIME_STEP);
  
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
  WbDeviceTag sensors[5];
  char sensors_name[5][20] = {"left_sensor", "right_sensor","area_detector","left_sensor_side", "right_sensor_side"};
  for (i = 0; i < 5; i++) {
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
 
 pen_track = wb_robot_get_device("pen");
 wb_pen_set_ink_color(pen_track,0x000000, 0.9);
 
 
 
 
 
 
 
  // Main loop
  while (wb_robot_step(TIME_STEP) != -1) {
    wb_pen_write(pen_track,true);
  
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
       if (counterClock_turn == false){// alternate the rotation for avoiding useless path
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
      double ds_values_left_side = wb_distance_sensor_get_value(sensors[3]);
      double ds_values_right_side = wb_distance_sensor_get_value(sensors[4]);
      
      
      // Store the area in the last state
      int last_area_localisation = area_localisation;
      area_localisation = area_detection(ds_values_area);
      
      // --- Emission infirmation ---
      if (last_area_localisation != area_localisation){ // Detect transition area
        if (area_localisation == 2 ){ // In the food area
          wb_led_set(leds[1], 1);
          sourceHasBefound = true;
          max_speed = 0.0; // stop
          
          //Get & send coodinates of the food area
          double gps_value_send[2] = {wb_gps_get_values(gp)[0],wb_gps_get_values(gp)[1]};
          printf("%s found the Source ! broadcast the coorinates...\n",wb_robot_get_name());
          sprintf(message, "%.5lf %.5lf\n", gps_value_send[0],gps_value_send[1]);
          wb_emitter_send(tx,message,strlen(message)+1);
    
          }else if(area_localisation == 1){ /* In the nest area*/}  
        } 
      


      // --- Avoidance state ---
      if (ds_values_left < 700.0 && ds_values_right < 700.0){ // Obstacle detected on both side
        if (alreadyWaiting == true){trun_counter=31;} // if we have already wait in the last state, turn back
        else {avoid_obstacle_counter_waiting = 20;} // Waiting state
      }
      else if (ds_values_left < 950.0){
        avoid_obstacle_counter_right = 6; 
        if(ds_values_left_side < 950.0){
          avoid_obstacle_counter_right = avoid_obstacle_counter_right + 3;
        }
      } // Obstacle detected on the left 
      else if (ds_values_right < 950.0){ 
        avoid_obstacle_counter_left = 6;
        if(ds_values_right_side < 950.0){
          avoid_obstacle_counter_left = avoid_obstacle_counter_left + 3;
        }
      } // Obstacle detected on the right
      alreadyWaiting = false;
    } 
    
    
    
    // --- Reception information ---
    // Statment ture if a package is recepted and if the source has not be found
    if (wb_receiver_get_queue_length(rx) > 0 && sourceHasBefound == false) {
      //sourceHasBefound = true;
      char *message = wb_receiver_get_data(rx);
      //const double *dir = wb_receiver_get_emitter_direction(rx);
      //double signal = wb_receiver_get_signal_strength(rx);
      /*
      printf("%s receive data %s: (signal=%g, dir=[%g %g %g])\n",
            wb_robot_get_name(),message, signal, dir[0], dir[1], dir[2]);
        */    
      wb_receiver_next_packet(rx); // next package (end)
      
      char * strToken = strtok(message," "); 
      i = 0;
      while ( strToken != NULL ) {
        sscanf(strToken,"%lf",&food_position[i]);
        strToken = strtok ( NULL, " " );
        i++;
      }
      //printf("food pos : %lf %lf\n",food_position[0],food_position[1]);
      int n = rotation_indicator (food_position[0],food_position[1],cp, gp);
      printf("n : %d\n", n);
      
      if (n < 0 ){
        n =-n; 
        counterClock_turn = true;
      }else{
        counterClock_turn = false;
      }
      
      trun_counter = n; // Rotate to the source signal
    }
    
    // Move the robot
    wb_motor_set_velocity(wheels[0], left_speed);
    wb_motor_set_velocity(wheels[1], right_speed);
    
  }// end main loop
  wb_robot_cleanup();
  return 0;  // EXIT_SUCCESS
}
