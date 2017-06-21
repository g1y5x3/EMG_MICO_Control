/*  Author: Yixiang Gao   
 *  Email: yg5d6@mail.missouri.edu
 *  Copyright: ViGIR Lab, University of Missouri - Columbia
 *
 *  Description:
 *  This application controls the MICO robotic arm to perform
 *  simple movements, for example, moving forward/backward,
 *  left/right, up/down, gripper open/close, by using a keyboard.
 *  However, the keyboard part is totally replaceable such as joystick,
 *  EMG, etc. It is used for an illustration for control.
 */

/*  This file uses libkindrv.
 *
 *  libkindrv is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  libkindrv is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser Public License
 *  along with libkindrv.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <libkindrv/kindrv.h>
#include <pthread.h>
#include <stdio.h>

using namespace KinDrv;

// ========== Global Variables ==========
JacoArm *arm;
int gesture = 0;
int x_dir = -1;
int y_dir = -1;
int z_dir = -1;
int finger_dir = -1;
int moving_status = 0;

// ========== Control Function for MICO ==========
int goto_retract(JacoArm *arm) {
  // this can only be achieved from HOME position. Otherwise the arm
  // will move to HOME. You'll probably need to uncomment the gripper movements
  // in order for this to work. Or even better, implement moving to HOME position,
  // which could be triggered before going to RETRACT ;)
  jaco_retract_mode_t mode = arm->get_status();
  switch( mode ) {
    case MODE_READY_TO_RETRACT:
      // is currently on the way to RETRACT. Need 2 button presses,
      // 1st moves towards HOME, 2nd brings it back to its way to RETRACT
      arm->push_joystick_button(2);
      arm->release_joystick();
      arm->push_joystick_button(2);
      break;

    case MODE_READY_STANDBY:
    case MODE_RETRACT_TO_READY:
      // just 1 button press needed
      arm->push_joystick_button(2);
      break;

    case MODE_NORMAL_TO_READY:
    case MODE_NORMAL:
    case MODE_NOINIT:
      printf("cannot go from NORMAL/NOINIT to RETRACT \n");
      return 0;
      break;

    case MODE_ERROR:
      printf("some error?! \n");
      return 0;
      break;

    case MODE_RETRACT_STANDBY:
      printf("nothing to do here \n");
      return 1;
      break;
  }

  while( mode != MODE_RETRACT_STANDBY ) {
    usleep(1000*10); // 10 ms
    mode = arm->get_status();
  }
  arm->release_joystick();

  return 1;
}

int goto_home(JacoArm *arm) {
  // going to HOME position is possible from all positions. Only problem is,
  // if there is some kinfo of error
  jaco_retract_mode_t mode = arm->get_status();
  switch( mode ) {
    case MODE_RETRACT_TO_READY:
      // is currently on the way to HOME. Need 2 button presses,
      // 1st moves towards RETRACT, 2nd brings it back to its way to HOME
      arm->push_joystick_button(2);
      arm->release_joystick();
      arm->push_joystick_button(2);
      break;

    case MODE_NORMAL_TO_READY:
    case MODE_READY_TO_RETRACT:
    case MODE_RETRACT_STANDBY:
    case MODE_NORMAL:
    case MODE_NOINIT:
      // just 1 button press needed
      arm->push_joystick_button(2);
      break;

    case MODE_ERROR:
      printf("some error?! \n");
      return 0;
      break;

    case MODE_READY_STANDBY:
      printf("nothing to do here \n");
      return 1;
      break;
  }

  while( mode != MODE_READY_STANDBY ) {
    usleep(1000*10); // 10 ms
    mode = arm->get_status();
    if( mode == MODE_READY_TO_RETRACT ) {
      arm->release_joystick();
      arm->push_joystick_button(2);
    }
  }
  arm->release_joystick();

  return 1;
}

// ========== Thread for Controlling the Robot ==========
void *RobotControl(void *t) {
    jaco_position_t pos;        // store the current position of the jaco arm
    arm->set_control_cart();

    while(gesture != 5) {
        pos = arm->get_cart_pos();

        if((gesture == 1) && (x_dir == 1)) {        // moving forward or backward
//            arm->set_control_cart();
            pos.position[0] = pos.position[0] + 0.01;
            arm->set_target_cart(pos.position[0], pos.position[1], pos.position[2],
         		                 pos.rotation[0], pos.rotation[1], pos.rotation[2],
         		                 pos.finger_position[0], pos.finger_position[1], pos.finger_position[2]);
            usleep(1000*50);
        }     
        else if((gesture == 1) && (x_dir == -1)) {
//            arm->set_control_cart();
            pos.position[0] = pos.position[0] - 0.01;
            arm->set_target_cart(pos.position[0], pos.position[1], pos.position[2],
         		                 pos.rotation[0], pos.rotation[1], pos.rotation[2],
         		                 pos.finger_position[0], pos.finger_position[1], pos.finger_position[2]);
            usleep(1000*50);
        }     
        if((gesture == 2) && (y_dir == 1)) {        // moving forward or backward
//            arm->set_control_cart();
            pos.position[1] = pos.position[1] + 0.01;
            arm->set_target_cart(pos.position[0], pos.position[1], pos.position[2],
         		                 pos.rotation[0], pos.rotation[1], pos.rotation[2],
         		                 pos.finger_position[0], pos.finger_position[1], pos.finger_position[2]);
            usleep(1000*50);
        }     
        else if((gesture == 2) && (y_dir == -1)) {
//            arm->set_control_cart();
            pos.position[1] = pos.position[1] - 0.01;
            arm->set_target_cart(pos.position[0], pos.position[1], pos.position[2],
         		                 pos.rotation[0], pos.rotation[1], pos.rotation[2],
         		                 pos.finger_position[0], pos.finger_position[1], pos.finger_position[2]);
            usleep(1000*50);
        }     
 
    }
    printf("Movement finished!\n");
    pthread_exit(NULL);
}

int main() {
    
    printf("Using keyboard to controlling the arm...\n");

    // ========== Initialize the Robot ==========
    // explicitly initialize a libusb context; optional
    KinDrv::init_usb();

    printf("Create a MicoArm \n");
    try {
      arm = new JacoArm();
      printf("Successfully connected to arm! \n");
    } catch( KinDrvException &e ) {
      printf("error %i: %s \n", e.error(), e.what());
      return 0; 
    }

    printf("Gaining API control over the arm \n");
    arm->start_api_ctrl();

    //check if we need to initialize arm
    jaco_retract_mode_t mode = arm->get_status();
    printf("Arm is currently in state: %i \n", mode);
    if( mode == MODE_NOINIT ) {
        //push the "HOME/RETRACT" button until arm is initialized
        arm->push_joystick_button(2);

        while( mode == MODE_NOINIT ) {
            usleep(1000*10); // 10 ms
            mode = arm->get_status();
        }
        // *****TO DO*****CHECK WHETHER CAN USE JOYSTICK TO SEND A COMMAND
        arm->release_joystick();
    }
    printf("Arm is initialized now, state: %i \n", mode);

    // ========== Initialize the Pthread ==========
    pthread_t thread_read;
    pthread_attr_t attr;
    int rc;
    long t;

    //set thread joinable
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    rc = pthread_create(&thread_read, &attr, RobotControl, (void * )t);
    if(rc) {
        printf("Error: unable to create thread %d.\n", rc);
        exit(-1);
    }

    printf("Pthread is initialized now.\n");

    // ========== Asking for Input ==========
    while( gesture != 5 ) { 
        printf("Control Command:");
        scanf("%d", &gesture);

        switch ( gesture ) {
            case 1:
                x_dir = x_dir * -1;
                moving_status = 1;                
                break;
            case 2:
                y_dir = y_dir * -1;
                moving_status = 1;
                break;
            case 3:
                z_dir = z_dir * -1;
                moving_status = 1;
                break;
            case 4:
                if( moving_status == 1 )
                    moving_status = 0;
                else {
                    moving_status = 0;
                    finger_dir = finger_dir * -1;
                }
                break;
            default:
                break;
        }
    }
//  printf("Opening gripper...");
//  // set control type to angular
//  arm->set_control_ang();
//  // current joint values + target finger values
//  jaco_position_t pos_angular = arm->get_ang_pos();
//  float finger_open[3] = {1900, 1900, 0};
//  // set target values; this moves the arm towards the target position
//  arm->set_target_ang(pos_angular.joints, finger_open);
//  // wait a little bit, until movement is finished
//  usleep(1000*2000);
//  printf("DONE\n");
//
//  printf("Closing gripper ...");
//  float finger_close[3] = {5400, 5400, 0};
//  arm->set_target_ang(pos_angular.joints, finger_close);
//  usleep(1000*2000);
//  printf("DONE\n");
//
//  // set control type to cartesian
//  arm->set_control_cart();
//  jaco_position_t pos = arm->get_cart_pos();
//  printf("Moving forward...");
//  pos.position[0] = pos.position[0] + 0.1;
//  arm->set_target_cart(	pos.position[0], pos.position[1], pos.position[2],
//         		pos.rotation[0], pos.rotation[1], pos.rotation[2],
//         		pos.finger_position[0], pos.finger_position[1], pos.finger_position[2]);
//  usleep(1000*2000);
//  printf("DONE\n");
//
//  arm->set_control_cart();
//  pos = arm->get_cart_pos();
//  printf("Moving backward...");
//  pos.position[0] = pos.position[0] - 0.1;
//  arm->set_target_cart(	pos.position[0], pos.position[1], pos.position[2],
//         		pos.rotation[0], pos.rotation[1], pos.rotation[2],
//         		pos.finger_position[0], pos.finger_position[1], pos.finger_position[2]);
//  usleep(1000*2000);
//  printf("DONE\n");
//
//  arm->set_control_cart();
//  pos = arm->get_cart_pos();
//  printf("Moving right...");
//  pos.position[1] = pos.position[1] - 0.1;
//  arm->set_target_cart(	pos.position[0], pos.position[1], pos.position[2],
//         		pos.rotation[0], pos.rotation[1], pos.rotation[2],
//         		pos.finger_position[0], pos.finger_position[1], pos.finger_position[2]);
//  usleep(1000*2000);
//  printf("DONE\n");
//
//  arm->set_control_cart();
//  pos = arm->get_cart_pos();
//  printf("Moving left...");
//  pos.position[1] = pos.position[1] + 0.1;
//  arm->set_target_cart(	pos.position[0], pos.position[1], pos.position[2],
//         		pos.rotation[0], pos.rotation[1], pos.rotation[2],
//         		pos.finger_position[0], pos.finger_position[1], pos.finger_position[2]);
//  usleep(1000*2000);
//  printf("DONE\n");
//
//  arm->set_control_cart();
//  pos = arm->get_cart_pos();
//  printf("Moving up...");
//  pos.position[2] = pos.position[2] + 0.1;
//  arm->set_target_cart(	pos.position[0], pos.position[1], pos.position[2],
//         		pos.rotation[0], pos.rotation[1], pos.rotation[2],
//         		pos.finger_position[0], pos.finger_position[1], pos.finger_position[2]);
//  usleep(1000*2000);
//  printf("DONE\n");
//
//  arm->set_control_cart();
//  pos = arm->get_cart_pos();
//  printf("Moving down...");
//  pos.position[2] = pos.position[2] - 0.1;
//  arm->set_target_cart(	pos.position[0], pos.position[1], pos.position[2],
//         		pos.rotation[0], pos.rotation[1], pos.rotation[2],
//         		pos.finger_position[0], pos.finger_position[1], pos.finger_position[2]);
//  usleep(1000*2000);
//  printf("DONE\n");
//
//  printf("Move arm back to RETRACT position \n");
//  if( !goto_retract(arm) ) {
//    printf("Failed to go to RETRACT. Go to HOME first \n");
//    if( goto_home(arm) ) {
//      printf("Try RETRACT again\n");
//      goto_retract(arm);
//    } else {
//      printf("Also failed going to HOME. Might be some serious problem...hmm \n");
//    }
//  }

    pthread_join(thread_read, NULL);
    // explicitly close libusb context (only needed if explicitly openede before)
    KinDrv::close_usb();
    printf("Exiting the program...\n");
    return 0;
}
