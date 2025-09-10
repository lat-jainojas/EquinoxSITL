#pragma once
#include <stddef.h>

#define N_SAMPLES_CRUISE 3300
#define N_SAMPLES_TAKEOFF 3894
#define N_FEATURES 6
#define N_OUTPUTS 6
#define K_NEIGHBORS 10


extern double X_DATA_TAKEOFF[N_SAMPLES_TAKEOFF][6];
extern double Y_DATA_TAKEOFF[N_SAMPLES_TAKEOFF][6];
extern double X_DATA_CRUISE[N_SAMPLES_CRUISE][6];
extern double Y_DATA_CRUISE[N_SAMPLES_CRUISE][6];