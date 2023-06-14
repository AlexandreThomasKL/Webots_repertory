#include <webots/Supervisor.h>
#include <webots/receiver.h>
#include <webots/emitter.h>
#include <webots/robot.h>
#include <stdio.h>
#include <string.h>

#define TIME_STEP 64

int main(int argc, char **argv) {
  wb_robot_init();
   
  WbDeviceTag rx_c = wb_robot_get_device("receiver");
  WbDeviceTag tx_c = wb_robot_get_device("emitter");
  
  wb_receiver_enable(rx_c, TIME_STEP);
 
  wb_receiver_set_channel(rx_c,5);
  wb_emitter_set_channel(tx_c,5);
  
  double food_position[6];
  int nb_reception = 0;
  int i =0;

  double start_delay = 0.0;
  double delay = 0.0;
  
  char message[128];
  
  double x_cercle = 0.0;    
  double y_cercle = 0.0;  
  double radius_cercle = 0.0;  
  
  bool alreadyGetValues = false;

  // Main loop
  while (wb_robot_step(TIME_STEP) != -1) {

    if (wb_receiver_get_queue_length(rx_c) > 0) {
      start_delay = wb_robot_get_time();
      nb_reception++;
      const char *message = wb_receiver_get_data(rx_c);
      const double *dir = wb_receiver_get_emitter_direction(rx_c);
      double signal = wb_receiver_get_signal_strength(rx_c);
      /*
      printf("%s receive data %s: (signal=%g, dir=[%g %g %g])\n",
            wb_robot_get_name(),message, signal, dir[0], dir[1], dir[2]);
            */
      printf("%s receive data\n",wb_robot_get_name());
            
      wb_receiver_next_packet(rx_c); // next package (end)
      
      char * strToken = strtok(message," "); 
      while ( strToken != NULL ) {
        sscanf(strToken,"%lf",&food_position[i]);
        strToken = strtok ( NULL, " " );
        i++;
      }    
    }
    // Estimate the circule with 3 dots
    if(nb_reception == 3 && alreadyGetValues == false){
      start_delay = wb_robot_get_time();
      alreadyGetValues = true;
      /*
      printf("pt n°1: %lf %lf\n",food_position[0],food_position[1]);
      printf("pt n°2: %lf %lf\n",food_position[2],food_position[3]);
      printf("pt n°3: %lf %lf\n",food_position[4],food_position[5]);
      */
      double den_factor = (food_position[2]-food_position[0])*(food_position[5]-food_position[1])
                        -(food_position[4]-food_position[0])*(food_position[3]-food_position[1]);
                        
      double xy1 =  pow(food_position[0],2)+pow(food_position[1],2);
      double xy2 =  pow(food_position[2],2)+pow(food_position[3],2);
      double xy3 =  pow(food_position[4],2)+pow(food_position[5],2);
    
      double num_y_factor = (xy2-xy1)*(food_position[4]-food_position[0])
                          -(xy3-xy1)*(food_position[2]-food_position[0]);
                        
      double num_x_factor = (xy2-xy1)*(food_position[5]-food_position[1])
                          -(xy3-xy1)*(food_position[3]-food_position[1]); 
                        
      x_cercle = 0.5*(num_x_factor/den_factor);     
      y_cercle = -0.5*(num_y_factor/den_factor);  
      
      radius_cercle = sqrt(pow((food_position[2]-x_cercle),2)+pow((food_position[3]-y_cercle),2));
                        
      printf("circle center : (%lf,%lf)\n",x_cercle,y_cercle);
      printf("circle radius : %lf\n",radius_cercle);         
    }
    
    
    if (radius_cercle >0 && delay > 15.0){
      start_delay = wb_robot_get_time();
      double gps_value_send[2] = {x_cercle,y_cercle};
      printf("Center area found, brodcast to everyone...\n");
      sprintf(message, "%.5lf %.5lf\n", gps_value_send[0],gps_value_send[1]);
      wb_emitter_send(tx_c,message,strlen(message)+1);
    }else if((nb_reception > 0 && nb_reception < 3) && delay > 15.0){
      start_delay = wb_robot_get_time();
      printf("Robot has found area found, recending brodcast to everyone...\n");
      sprintf(message, "%.5lf %.5lf\n", food_position[0],food_position[1]);
      wb_emitter_send(tx_c,message,strlen(message)+1);
    }  
    if (start_delay != 0.0){ delay = wb_robot_get_time() - start_delay;}
  }
  wb_robot_cleanup();
  return 0;  // EXIT_SUCCESS
}
