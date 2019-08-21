#include <arm_neon.h>
#include <arm_acle.h>
#define DP_PI (3.1415926535897932384626433)	// pi
/* function prototypes */
void sim_motion(void);
/*void Find_Nearest_Waypoint(float cur_pos_lat, float cur_pos_lon, 
                            float * distance, float * bearing, 
	                          char  * * name);*/
float cos_12(float x);
float32x4_t v_cos_12(float32x4_t v4_x);
float cos_32(float x);
float cos_52(float x);
float32x4_t v_cos_73(float32x4_t v4_x);
extern inline double cos_73(double x);
double cos_121(double x);

float sin_32(float x);
float sin_52(float x);
double sin_73(double x);
double sin_121(double x);
