#ifndef PHICUBE_CONFIGURATION
#define PHICUBE_CONFIGURATION

#define PHICUBE_ROBOT_PERIOD_MS        1           // PhiCube motors control cycle running at 1000 Hz
#define PHICUBE_MOTORSTATE_PERIOD_MS   20          // ROS2 topic for motors state msg published at 50 Hz
#define PHICUBE_DEVICESTATE_PERIOD_MS  500         // ROS2 

#define CANCOMM_TIMEOUT_MS             10          // Max time to wait between sending a CAN command and receiving the response. Retry if passed.
#define CANCOMM_MAX_RETRIES            3           // Max number of attempts when sending a CAN command and not receiving response. Discard if exceeded.
#define CANCOMM_QUEUE_SIZE             20          // Max number of CAN commands that can be enqueued. Discard if exceeded.
#define CANCOMM_M1_CANID               0x101       // CAN ID for Motor 1
#define CANCOMM_M2_CANID               0x102       // CAN ID for Motor 2

#endif