#include <math.h>

#define HANDADMIN_VERSION 	"v5.0.0"

#define NUM_OF_SENSORS 		3
#define NUM_OF_MOTORS 		2
#define NUM_OF_EMGS 		2
#define PI 					3.14159265359

#define DEFAULT_RESOLUTION 	3
#define BROADCAST_ID 		0
#define DEFAULT_INCREMENT 	100 //in tick

#define DEG_TICK_MULTIPLIER (65536.0 / (360.0 * (pow(2, DEFAULT_RESOLUTION))))

// Just thinking of any time in the future the soft hand might have more than one synergy
#define N_SYN 				1
#define MAX_HAND_MEAS 		19000.0

// Qb_interface topics
#define HAND_MEAS_TOPIC 	"/qb_class/hand_measurement"
#define HAND_CURR_TOPIC 	"/qb_class/hand_current"
#define HAND_REF_TOPIC 		"/qb_class/hand_ref"

// For debugging: prints out additional info if 1
#define DEBUG 				1