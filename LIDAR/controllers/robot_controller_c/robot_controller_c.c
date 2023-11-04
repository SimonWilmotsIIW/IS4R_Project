/*
 * File:          robot_controller_c.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/lidar.h>
#include <webots/keyboard.h>
#include <stdio.h>
/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  

  wb_keyboard_enable(1000);  
  // MOTOR CODE
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");

  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  
  float left_motor_velocity = 0.0;
  float right_motor_velocity = 0.0;
  
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  
  // LIDAR
  WbDeviceTag lidar = wb_robot_get_device("LDS-01");
  wb_lidar_enable(lidar, TIME_STEP);
  wb_lidar_enable_point_cloud(lidar);

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */

    /* Process sensor data here */
    const float *range_image = wb_lidar_get_range_image(lidar);
    
    for (int i=0; i<180; i++)
    {
      printf("%.2f ", *(range_image + i));
    }
    
    printf("\n");

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
     
    int key = wb_keyboard_get_key();
    
    // Define control based on key input
    if (key == WB_KEYBOARD_UP) {
      // Move forward
      left_motor_velocity = 6.28 * 0.5;
      right_motor_velocity = 6.28 * 0.5;
    } else if (key == WB_KEYBOARD_DOWN) {
      // Move backward
      left_motor_velocity = -6.28 * 0.5;
      right_motor_velocity = -6.28 * 0.5;
    } else if (key == WB_KEYBOARD_LEFT) {
      // Turn left
      left_motor_velocity = -6.28 * 0.5;
      right_motor_velocity = 6.28 * 0.5;
    } else if (key == WB_KEYBOARD_RIGHT) {
      // Turn right
      left_motor_velocity = 6.28 * 0.5;
      right_motor_velocity = -6.28 * 0.5;
    } else {
      // Stop if no key is pressed
      left_motor_velocity = 0.0;
      right_motor_velocity = 0.0;
    }
     wb_motor_set_velocity(left_motor, left_motor_velocity);
     wb_motor_set_velocity(right_motor, right_motor_velocity);
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
