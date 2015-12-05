/*
 *
 *		mcl_tools.h
 *
 */
#ifndef MCL_TOOLS_H
#define MCL_TOOLS_H
//================== Includes ======================================//
#include "particle.h"
//============== External Functions Declaration ====================//
 void initMclTools(int argc, char **argv);
 void mcl_run_viz();
 u16_t map_hit(f64_t wx, f64_t wy);
 f64_t map_range(Pos_t particle, f64_t phi);

 void cleanBadParticles(void);
 vector<particle> resample( vector<particle> oldParticles, vector<f64_t> cdf_arr);
 vector<f64_t> norm_cdf_vector(vector<particle> arr);

 #endif
