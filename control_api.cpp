/* The state machine for using 4 commands to control MICO moving
 * up/down
 * left/right
 * forward/backward
 * grib/release
 */

#include <stdlib.h>
#include <iostream>
#include <unistd.h>

using namespace std;

int main() 
{
    // [X+/-, Y+/-, Z+/-, R/G]
    bool arm_status[4] = {false};
    int input, output;
    int state = 0;
  
    while(1) {

        cout << "X: " << arm_status[0] << endl;
        cout << "Y: " << arm_status[1] << endl;
        cout << "Z: " << arm_status[2] << endl;
        cout << "R/G: " << arm_status[3] << endl;

        // Initial the state machine
        switch(state) {
            case 0:
                cout << "Stop!" << endl;
                cin >> input;

                if(input == 1) 
                    state = 1;
                else
                    state = 4;

                break;

            case 1:
                cout << "Wait!" << endl;
                cin >> input;

                if(input == 1)
                    state = 3;
                else
                    state = 2;

                break;

            case 2:
                if(input == 2) {
                    arm_status[0] = !arm_status[0];
                    if(arm_status[0] == false)
                        cout << "Move Forward!" << endl;
                    else
                        cout << "Move Backward!" << endl;
                }

                else if(input == 3) {
                    arm_status[1] = !arm_status[1];
                    if(arm_status[1] == false)
                        cout << "Move Right!" << endl;
                    else
                        cout << "Move Left!" << endl;
                }

                else if(input == 3) {
                    arm_status[2] = !arm_status[2];
                    if(arm_status[2] == false)
                        cout << "Move Up!" << endl;
                    else
                        cout << "Move Down!" << endl;
                }

                cin >> input;
                state = 0;

                break;
               
            case 3:
                arm_status[3] = !arm_status[3];
                if(arm_status[3] == false)
                    cout << "Release Gripper!" << endl;
                else
                    cout << "Close Gripper" << endl;

                state = 0;
                break;
            
            case 4:
                if(input == 2) {
                    if(arm_status[0] == false)
                        cout << "Move Forward!" << endl;
                    else
                        cout << "Move Backward!" << endl;
                }

                else if(input == 3) {
                    if(arm_status[1] == false)
                        cout << "Move Right!" << endl;
                    else
                        cout << "Move Left!" << endl;
                }

                else if(input == 4) {
                    if(arm_status[2] == false)
                        cout << "Move Up!" << endl;
                    else
                        cout << "Move Down!" << endl;
                }

                cin >> input;
                state = 0;

                break;

            default:
               break;
        }
                          

    }

    return 0;
}

