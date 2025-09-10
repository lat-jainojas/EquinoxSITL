#pragma once
// Adaptive-k KNN distance-weighted regression (ratio trigger)
// Predicts {CL, CD, CS, CMl, CMm, CMn} from inputs
// Inputs: throttle, aileron, elevator, rudder, beta, aoa

// ---- Tunables (baked-in defaults) ----
#ifndef K_DENSE
#define K_DENSE 10      // k to use in dense regions
#endif
#ifndef K_SPARSE
#define K_SPARSE 5      // k to use in sparse regions
#endif
#ifndef K_SEARCH
#define K_SEARCH 20     // neighbors gathered before choosing k
#endif
#ifndef EPS_DIST
#define EPS_DIST 1e-12  // safety epsilon to avoid divide-by-zero
#endif

// Use the ratio trigger r = d_k / d_1 (recommended for grid-like datasets)
#define ADAPT_USE_RATIO
#define K_FOR_GAP   10            // which neighbor defines d_k
#define RATIO_SPLIT 3.16227766    // 90th percentile of r = d10/d1 from TRAIN (≈ √10)

// (Legacy d1 trigger left here only for reference; not used when ADAPT_USE_RATIO is defined)
#ifndef NN_GAP_SPLIT
#define NN_GAP_SPLIT 0.75
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Basic API (prediction only)
void aero_interpolate(double throttle, double aileron, double elevator,
                      double rudder, double beta, double aoa,
                      double coeffs[6],int);

// Extended API: also returns chosen k and the adaptive metric
// NOTE: with ADAPT_USE_RATIO, the "nn_gap" output is the ratio r = d_k/d_1.
//       If an exact match occurs, it returns the second-neighbor distance (d1).
void aero_interpolate_ex(double throttle, double aileron, double elevator,
                         double rudder, double beta, double aoa,
                         double coeffs[6], int *chosen_k, double *nn_gap, int);

#ifdef __cplusplus
}
#endif
