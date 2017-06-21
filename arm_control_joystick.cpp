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
int x_dir   = -1;
int y_dir   = -1;
int z_dir   = -1;
int finger_dir  = -1;
int moving_status = 0;
jaco_position_t pos;

// ========== Control Function for MICO ==========
int goto_retract(JacoArm *arm) 
{
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

int goto_home(JacoArm *arm) 
{
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
void *RobotControl(void *t) 
{
    jaco_joystick_axis_t axes = {0};
    float finger_open[3] = {0, 0, 0};

    while(gesture != 5) {

        if((gesture == 1) && (x_dir == 1)) {        // moving forward
            arm->set_control_cart();
            axes.trans_fb = -0.6f;
            arm->move_joystick_axis(axes);
            usleep(2000);
            arm->release_joystick();
            axes.trans_fb = 0.f;
        }     
        else if((gesture == 1) && (x_dir == -1)) {  // moving backward
            arm->set_control_cart();
            axes.trans_fb = 0.6f;
            arm->move_joystick_axis(axes);
            usleep(2000);
            arm->release_joystick();
            axes.trans_fb = 0.f;
        }     
        if((gesture == 2) && (y_dir == 1)) {        // moving left        
            arm->set_control_cart();
            axes.trans_lr = 0.6f;
            arm->move_joystick_axis(axes);
            usleep(2000);
            arm->release_joystick();
            axes.trans_lr = 0.f;
        }     
        else if((gesture == 2) && (y_dir == -1)) {  // moving right
            arm->set_control_cart();
            axes.trans_lr = -0.6f;
            arm->move_joystick_axis(axes);
            usleep(2000);
            arm->release_joystick();
            axes.trans_lr = 0.f;
        }     
        if((gesture == 3) && (z_dir == 1)) {        // moving up 
            arm->set_control_cart();
            axes.trans_rot = 0.6f;
            arm->move_joystick_axis(axes);
            usleep(2000);
            arm->release_joystick();
            axes.trans_rot = 0.f;
        }     
        else if((gesture == 3) && (z_dir == -1)) {  // moving down
            arm->set_control_cart();
            axes.trans_rot = -0.6f;
            arm->move_joystick_axis(axes);
            usleep(2000);
            arm->release_joystick();
            axes.trans_rot = 0.f;
        }
        if((gesture == 4) && (finger_dir == 1)) {       // close grep
            axes.trans_fb = 0.f;
            axes.trans_lr = 0.f;
            axes.trans_rot = 0.f;
            arm->move_joystick_axis(axes);
            usleep(2000);
            arm->release_joystick();

            // set control type to angular
            arm->set_control_ang();
            finger_open[0] = 1900;
            finger_open[1] = 1900;
            arm->set_target_ang(pos.joints, finger_open);
            usleep(1e6);
            arm->release_joystick();
        }     
        else if((gesture == 4) && (finger_dir == -1)) { // open grep
            axes.trans_fb = 0.f;
            axes.trans_lr = 0.f;
            arm->move_joystick_axis(axes);
            usleep(2000);
            arm->release_joystick();
            
            // set control type to angular
            arm->set_control_ang();
            finger_open[0] = 5400;
            finger_open[1] = 5400;
            arm->set_target_ang(pos.joints, finger_open);
            usleep(1e6);
        }     
    }
    printf("Movement finished!\n");
    pthread_exit(NULL);
}

// ========== Main Thread ==========
int main() 
{
    
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

    // need cartesian-control for joystick simulation.
    // Angular-control is also possible, then you would control each joint!

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

        switch (gesture) {
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
                pos = arm->get_ang_pos();
                if(moving_status == 1)
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

    pthread_join(thread_read, NULL);
    // explicitly close libusb context (only needed if explicitly openede before)
    KinDrv::close_usb();
    printf("Exiting the program...\n");
    return 0;
}
