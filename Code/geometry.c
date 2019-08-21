#include "geometry.h"
#include <math.h>
#include <string.h>
#include <time.h>
#include <stdio.h>
#include "proto.h"
#include <arm_neon.h>
#include <arm_acle.h>

#define PI 3.14159265f
#define PI_OVER_180 (0.017453293f) // (3.1415927/180.0)
double const twopi;
double const two_over_pi;
extern const PT_T waypoints[];
extern vector_PT values[];

#define Epsilon 0.0010557f

/*
11: radians in table, precalc'd sin, cos
12: Calc_Closeness
13: Don't do bearing
*/

// Table holds precalculated sin/cos for p2. Table Lat/Lon values are in radians

float Calc_Bearing( PT_T * p1,  const PT_T * p2){
  // calculates bearing in degrees between locations (represented in radians)
  float term1, term2;
  float angle;

  term1 = sinf(p1->Lon - p2->Lon)*p2->CosLat;
  term2 = p1->CosLat*p2->SinLat -
    p1->SinLat*p2->CosLat*cosf(p1->Lon - p2->Lon);
  angle = atan2f(term1, term2) * (180/PI);
  if (angle < 0.0)
    angle += 360;
  return angle;
}
float Calc_Bearing_validation( PT_T * p1,  const PT_T * p2){
  // calculates bearing in degrees between locations (represented in radians)
  float term1, term2;
  float angle;

  term1 = sin(p1->Lon - p2->Lon)*p2->CosLat;
  term2 = p1->CosLat*p2->SinLat -
    p1->SinLat*p2->CosLat*cos(p1->Lon - p2->Lon);
  angle = atan2(term1, term2) * (180/PI);
  if (angle < 0.0)
    angle += 360;
  return angle;
}
float Calc_Closeness_accurate( PT_T * p1,  const PT_T * p2) { 
  // calculates closeness (decreases as distance increases) of locations

  return p1->SinLat * p2->SinLat +
    p1->CosLat * p2->CosLat*
    cos_73(p2->Lon - p1->Lon);
}
float Calc_Closeness_fastest( PT_T * p1,  const PT_T * p2) { 
  // calculates closeness (decreases as distance increases) of locations

  return p1->SinLat * p2->SinLat +
    p1->CosLat * p2->CosLat*
    cos_12(p2->Lon - p1->Lon);
}
float Calc_Closeness_validation( PT_T * p1,  const PT_T * p2) { 
  // calculates closeness (decreases as distance increases) of locations

  return p1->SinLat * p2->SinLat +
    p1->CosLat * p2->CosLat*
    cosf(p2->Lon - p1->Lon);
}
void Find_Nearest_Waypoint(PT_T *ref, float * distance, float * bearing, 
         char  * * name){
  // cur_pos_lat and cur_pos_lon are in degrees
  // distance is in kilometers
  // bearing is in degrees

  int i=0, closest_i=0;
  float d, b, c, max_c=0, closest_d=1E10;
  struct timespec prestart, start, end1, end2;
  unsigned to;
  *distance = 0.0f;
  *bearing = 0;
  *name = '\0';
  float32x4_t v4_waypoints_Lon,v4_waypoints_SinLat,v4_waypoints_CosLat,v4_CosLat, v4_SinLat, v4_Term2,v4_max_c,v4_c,v4_prev_max;
  uint32x4_t v4_temp,v4_index,v4_temp_index;
  //int *out;
  float SinLat[165],CosLat[165],Lon[165];
  int temp_index[165];
  v4_max_c = vdupq_n_f32(-1e30);
  #if VER>=3
  for (int j=0;j<164;j++)
  {
  	SinLat[j]=waypoints[j].SinLat;
  	CosLat[j]=waypoints[j].CosLat;
  	Lon[j]=waypoints[j].Lon-ref->Lon;
  	temp_index[j]=j;
  }
  for (int i=0;i<164;i+=4)
    {
        
        v4_waypoints_SinLat=vld1q_f32(&SinLat[i]);
        v4_waypoints_CosLat=vld1q_f32(&CosLat[i]);
        v4_waypoints_Lon=vld1q_f32(&Lon[i]);
        v4_waypoints_Lon=v_cos_73(v4_waypoints_Lon);
        v4_temp_index=vld1q_u32(&temp_index[i]);
        v4_SinLat=vmulq_n_f32(v4_waypoints_SinLat,ref->SinLat);
        v4_CosLat=vmulq_n_f32(v4_waypoints_CosLat,ref->CosLat);
        v4_Term2=vmulq_f32(v4_waypoints_Lon,v4_CosLat);
        v4_c=vaddq_f32(v4_SinLat,v4_Term2);
        v4_prev_max=v4_max_c;
        v4_max_c=vmaxq_f32(v4_max_c,v4_c);
        v4_temp=vceqq_f32(v4_prev_max,v4_max_c);
        v4_index=vbslq_u32(v4_temp,v4_index,v4_temp_index);
    }    
    float32x2_t v2_u, v2_l,v2_temp_u;
    uint32x2_t v2_compare,v2_index1,v2_index2,v2_index,v2_index_rev,v2_index_1,v2_index_2;
    float32x2_t v2_zero = vdup_n_f32(0.0);
    v2_u = vget_high_f32(v4_max_c);
    v2_l = vget_low_f32(v4_max_c);
    v2_index1 = vget_high_u32(v4_index);
    v2_index2 = vget_low_u32(v4_index);
    v2_temp_u=v2_u;
    v2_u = vpmax_f32(v2_l, v2_u);
    v2_compare=vceq_f32(v2_u,v2_temp_u);
    v2_index_rev=vrev64_u32(v2_index1);
    v2_index_1=vbsl_u32(v2_compare,v2_index1,v2_index_rev);
    v2_compare=vceq_f32(v2_l,v2_u);
    v2_index_rev=vrev64_u32(v2_index2);
    v2_index_2=vbsl_u32(v2_compare,v2_index2,v2_index_rev);
    v2_index=vext_u32(v2_index_1,v2_index_2,1);
    v2_index_rev=v2_index;
    v2_index=vrev64_u32(v2_index);
    v2_temp_u=v2_u;
    v2_u = vpmax_f32(v2_u, v2_zero);
    v2_compare=vceq_f32(v2_u,v2_temp_u);
    v2_index=vbsl_u32(v2_compare,v2_index,v2_index_rev);
    max_c= vget_lane_f32(v2_u, 0);
    closest_i= vget_lane_u32(v2_index, 0);
    #else
    while (strcmp(waypoints[i].Name, "END")) {
      c = Calc_Closeness_accurate(ref, &(waypoints[i]) );
      if (c>max_c) {
        max_c = c;
        closest_i = i;
      }
      i++;
    }
    #endif
  // Finish calcuations for the closest point
  d = acosf(max_c)*6371; // finish distance calcuation
  b = Calc_Bearing(ref, &(waypoints[closest_i]) );
  // return information to calling function about closest waypoint 
  *distance = d;
  *bearing = b;
  *name = (char * ) (waypoints[closest_i].Name);
}
int Find_Nearest_Waypoint_Pass_1(PT_T *ref,float *c, float *max_c){
  // cur_pos_lat and cur_pos_lon are in degrees
  // distance is in kilometers
  // bearing is in degrees

  int i=0;
  struct timespec prestart, start, end1, end2;
  unsigned to;
  *max_c=0;
while (strcmp(waypoints[i].Name, "END")) {

    c[i] = Calc_Closeness_fastest(ref, &(waypoints[i]) );
    if (c[i]>*max_c) {
      *max_c = c[i];
    }
    i++;
  }
  return i;
}
void Find_Nearest_Waypoint_Pass(PT_T *ref,float * distance, float * bearing, 
         char  * * name){
	int i=0,m=0,closest_i2=0,count=0;
  float d, b,max_c=0,max_c2=0,upper_bound,lower_bound;
  *distance = 0.0f;
  *bearing = 0;
  *name = '\0';
  float c[200];
  float c_final;
  struct timespec start, end;
  uint64_t diff, total=0, min=1234567890;
  float32x4_t v4_waypoints_Lon,v4_waypoints_SinLat,v4_waypoints_CosLat,v4_CosLat, v4_SinLat, v4_Term2,v4_max_c,v4_c,v4_prev_max;
  uint32x4_t v4_temp,v4_index,v4_temp_index;
  float32x2_t v2_u, v2_l,v2_temp_u;
  uint32x2_t v2_compare,v2_index1,v2_index2,v2_index,v2_index_rev,v2_index_1,v2_index_2;
  float32x2_t v2_zero = vdup_n_f32(0.0);
  const float c1= 0.99940307;
  const float c2=-0.49558072;
  float32x4_t v4_case2,v4_case3,v4_case4,v4_result_temp,v4_result,v4_term;
  uint32x4_t v4_zero,v4_quad,v4_compare,v4_compare_neg1,v4_compare_neg2,v4_and;
  float32x4x3_t v4_waypoints;
  v4_max_c = vdupq_n_f32(-1e30);
  for (int i=0;i<164;i+=4)
    {
        v4_waypoints=vld3q_f32(&values[i]);
        v4_waypoints_Lon=vsubq_f32(v4_waypoints.val[2],vdupq_n_f32(ref->Lon));

        v4_zero = vdupq_n_u32(0);
        v4_result=vdupq_n_f32(0.0);
        v4_quad=vcvtq_u32_f32(vmulq_n_f32(v4_waypoints_Lon,two_over_pi));
        v4_case2=vsubq_f32(vdupq_n_f32(DP_PI),v4_waypoints_Lon);
        v4_case3=vsubq_f32(v4_waypoints_Lon,vdupq_n_f32(DP_PI));
        v4_case4=vsubq_f32(vdupq_n_f32(twopi),v4_waypoints_Lon);
            
        v4_compare=vceqq_u32(vdupq_n_u32(1),v4_quad);
        v4_compare_neg1=vorrq_u32(v4_compare,v4_zero);
        v4_result=vbslq_f32(v4_compare,v4_case2,v4_waypoints_Lon);
        
        v4_compare=vceqq_u32(vdupq_n_u32(2),v4_quad);
        v4_compare_neg1=vorrq_u32(v4_compare,v4_compare_neg1);
        v4_result=vbslq_f32(v4_compare,v4_case3,v4_result);
        
        v4_compare=vceqq_u32(vdupq_n_u32(3),v4_quad);
        v4_result=vbslq_f32(v4_compare,v4_case4,v4_result);

        v4_result=vmulq_f32(v4_result,v4_result);
        v4_term=vmulq_n_f32(v4_result,c2);
        v4_result=vaddq_f32(vdupq_n_f32(c1),v4_term);
          
        v4_result_temp=v4_result;
        v4_result_temp=vmulq_n_f32(v4_result,-1);
        v4_result=vbslq_f32(v4_compare_neg1,v4_result_temp,v4_result);

        v4_SinLat=vmulq_n_f32(v4_waypoints.val[0],ref->SinLat);
        v4_CosLat=vmulq_n_f32(v4_waypoints.val[1],ref->CosLat);
        v4_Term2=vmulq_f32(v4_result,v4_CosLat);
        v4_c=vaddq_f32(v4_SinLat,v4_Term2);
        vst1q_f32(&c[i],v4_c);
        v4_max_c=vmaxq_f32(v4_max_c,v4_c);
        
    }    
  v2_u = vget_high_f32(v4_max_c);
  v2_l = vget_low_f32(v4_max_c);
  v2_u = vpmax_f32(v2_l, v2_u);
  v2_u = vpmax_f32(v2_u, v2_zero);
  max_c= vget_lane_f32(v2_u, 0);
  lower_bound=(max_c)*((100-Epsilon)/(100+Epsilon));
  max_c2=max_c;
  for(int j=0;j<164;j++)
	{
		if(c[j]>lower_bound)
		{
			c_final = Calc_Closeness_accurate(ref, &(waypoints[j]));
			if(c_final>max_c2)
			{	
				max_c2=c_final;
				closest_i2=j;
			}
		}
	}
  	// Finish calcuations for the closest point
  d = acosf(max_c2)*6371; // finish distance calcuation
  b = Calc_Bearing(ref, &(waypoints[closest_i2]) );
  *distance = d;
	*bearing = b;
 	*name = (char * ) (waypoints[closest_i2].Name);
}
void Find_Nearest_Waypoint_validation(PT_T *ref, float * distance, float * bearing, 
         char  * * name){
  // cur_pos_lat and cur_pos_lon are in degrees
  // distance is in kilometers
  // bearing is in degrees

  int i=0, closest_i=0;
  //PT_T ref;
  float d, b, c, max_c=0, closest_d=1E10;
  struct timespec prestart, start, end1, end2;
  unsigned to;

  *distance = 0.0f;
  *bearing = 0;
  *name = '\0';

  while (strcmp(waypoints[i].Name, "END")) {
//    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
    c = Calc_Closeness_validation(ref, &(waypoints[i]) );
 //   clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end1);
    if (c>max_c) {
      max_c = c;
      closest_i = i;
    }
 //   clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end2);
   i++;
  }

/*
  printf("Start to End 1: %d\t", end1.tv_nsec - start.tv_nsec - to);
  printf("Start to End 2: %d\n", end2.tv_nsec - start.tv_nsec - to);
*/
  // Finish calcuations for the closest point

d = acos(max_c)*6371; // finish distance calcuation
b = Calc_Bearing_validation(ref, &(waypoints[closest_i]) );
 // return information to calling function about closest waypoint 
  *distance = d;
  *bearing = b;
  *name = (char * ) (waypoints[closest_i].Name);
}
