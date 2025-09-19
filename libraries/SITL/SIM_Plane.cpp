// Using the generated header file from barycentric interpolation
#include <sstream>
#include <iomanip>
#include <sys/select.h>
#include <AP_HAL/AP_HAL.h>
#include "SIM_Plane.h"
#include <vector>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <signal.h>
#include <ctime>
#include <inttypes.h>
#include <AP_Filesystem/AP_Filesystem_config.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_AHRS/AP_AHRS.h>
#include<stdio.h>
// The following code is for EQUINOX's SITL Model
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif



// Writing the code for Mr. Rajat Patel to read
// What I have done is made separate functions for the different coefficients which can just be changed later on without affecting any other part of the code.

// =============================================================================
// NEW AERODYNAMIC MODEL FUNCTIONS
// =============================================================================

// clamp helper
static inline double clamp_double(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}


/*
 * Cruise Unified CL
 * Inputs: alpha (deg), delta_e (deg), delta_f (deg), delta_a (deg), C_mu
 * Output: CL
 */
 // ...existing code...
static inline double compute_CL_cruise_unified(double alpha_deg, double de, double df, double da_L, double da_R, double Cmu) {
    double alpha_rad = alpha_deg * M_PI / 180.0;
    double sigma = (1 + exp(-18.080 * (alpha_rad - (16.345 * M_PI / 180.0))) + exp(18.080 * (alpha_rad + (16.345 * M_PI / 180.0))))
                 / ((1 + exp(-18.080 * (alpha_rad - (16.345 * M_PI / 180.0)))) * (1 + exp(18.080 * (alpha_rad + (16.345 * M_PI / 180.0)))));
    double CL = (1 - sigma) * (0.195343 + 0.111981 * alpha_deg)
              + sigma * (2.0 * copysign(1.0, alpha_rad) * pow(sin(alpha_rad), 2) * cos(alpha_rad))
              + (0.015900 * de + 0.009675 * df + 0.006162 * da_L + 0.006168 * da_R
                 + 2.252691 * Cmu + 0.031598 * (alpha_deg * Cmu));
    return clamp_double(CL, -1.5, 5);
}

static inline double compute_CL_takeoff_unified(double alpha_deg, double de, double df, double da_L, double da_R, double Cmu) {
    double alpha_rad = alpha_deg * M_PI / 180.0;
    double sigma = (1 + exp(-18.080 * (alpha_rad - (16.345 * M_PI / 180.0))) + exp(18.080 * (alpha_rad + (16.345 * M_PI / 180.0))))
                 / ((1 + exp(-18.080 * (alpha_rad - (16.345 * M_PI / 180.0)))) * (1 + exp(18.080 * (alpha_rad + (16.345 * M_PI / 180.0)))));
    double CL = (1 - sigma) * (0.195343 + 0.111981 * alpha_deg)
              + sigma * (2.0 * copysign(1.0, alpha_rad) * pow(sin(alpha_rad), 2) * cos(alpha_rad))
              + (0.015900 * de + 0.009675 * df + 0.006162 * da_L + 0.006168 * da_R
                 + 2.252691 * Cmu + 0.031598 * (alpha_deg * Cmu));
    return clamp_double(CL, -1.5, 5);
}
// ...existing code...

/*
 * Cruise Unified CD
 * Inputs: alpha (deg), delta_e (deg), delta_f (deg), delta_a (deg), C_mu, CL_measured
 * Output: CD
 */
// static inline double compute_CD_cruise_unified(double alpha, double delta_e, double delta_f, double delta_a, double C_mu, double CL_meas) {
//     // Use your existing CD calculation with modifications for cruise configuration
//     double CD0c = -0.51403 * C_mu + 0.107575;
//     double CD0_del_f_term = 0.01285 * delta_f + 0.0021;
//     double CD0 = CD0c + 0.0038 * delta_e + CD0_del_f_term + 0.0015 * delta_a + 1.322175 * C_mu;
    
//     // k interpolation for cruise (0 deg flaps)
//     double k = 0.046;
    
//     double CD = CD0 + k * (CL_meas * CL_meas);
//     return clamp_double(CD, 0.0, 3.0);
// }

// /*
//  * Takeoff Unified CD  
//  * Inputs: alpha (deg), delta_e (deg), delta_f (deg), delta_a (deg), C_mu, CL_measured
//  * Output: CD
//  */
// static inline double compute_CD_takeoff_unified(double alpha, double delta_e, double delta_f, double delta_a, double C_mu, double CL_meas) {
//     // Use your existing CD calculation with modifications for takeoff configuration
//     double CD0c = -0.51403 * C_mu + 0.107575;
//     double CD0_del_f_term = 0.01285 * delta_f + 0.0021;
//     double CD0 = CD0c + 0.0038 * delta_e + CD0_del_f_term + 0.0015 * delta_a + 1.322175 * C_mu;
    
//     // k interpolation for takeoff (40 deg flaps)
//     double k = 0.056;
    
//     double CD = CD0 + k * (CL_meas * CL_meas);
//     return clamp_double(CD, 0.0, 3.0);
// }

static inline double compute_CD_cruise_unified(double alpha_deg, double de, double df, double da_L, double da_R, double Cmu, double CL_meas) {
    // Effective angle of attack
    double alpha_eff_deg = alpha_deg + ((alpha_deg >= 0) ? (-0.005509 * de) : (0.2 * de));
    double alpha_eff_rad = alpha_eff_deg * M_PI / 180.0;

    // Switching function sigma
    const double M = 120.0;
    const double c_pos = 8.48 * M_PI / 180.0;
    const double c_neg = 3.00 * M_PI / 180.0;

    double sigma_pos = (1 + exp(-M * (alpha_eff_rad - c_pos)) + exp(M * (alpha_eff_rad + c_pos)))
                    / ((1 + exp(-M * (alpha_eff_rad - c_pos))) * (1 + exp(M * (alpha_eff_rad + c_pos))));
    double sigma_neg = (1 + exp(-M * (alpha_eff_rad - c_neg)) + exp(M * (alpha_eff_rad + c_neg)))
                    / ((1 + exp(-M * (alpha_eff_rad - c_neg))) * (1 + exp(M * (alpha_eff_rad + c_neg))));
    double sigma = (alpha_deg < 0) ? sigma_neg : sigma_pos;

    // CD0 base drag
    double CD0 = 0.071087
        + 0.000139 * de
        + 0.005921 * df
        + 0.001166 * da_L
        + 0.003391 * da_R
        + 1.273205 * Cmu
        + 0.064856 * (alpha_deg * Cmu);

    // k factor
    double k = 0.046 + (0.01 / 120.0) * (df + da_L + da_R);

    // Total drag coefficient
    double CD = CD0 + (1 - sigma) * (k * (CL_meas * CL_meas))
        + sigma * (2.0 * pow(sin(alpha_eff_rad), 2));

    return clamp_double(CD, 0.0, 3.0);
}

static inline double compute_CD_takeoff_unified(double alpha_deg, double de, double df, double da_L, double da_R, double Cmu, double CL_meas) {
    // Effective angle of attack
    double alpha_eff_deg = alpha_deg + ((alpha_deg >= 0) ? (-0.005509 * de) : (0.2 * de));
    double alpha_eff_rad = alpha_eff_deg * M_PI / 180.0;

    // Switching function sigma
    const double M = 120.0;
    const double c_pos = 8.48 * M_PI / 180.0;
    const double c_neg = 3.00 * M_PI / 180.0;

    double sigma_pos = (1 + exp(-M * (alpha_eff_rad - c_pos)) + exp(M * (alpha_eff_rad + c_pos)))
                    / ((1 + exp(-M * (alpha_eff_rad - c_pos))) * (1 + exp(M * (alpha_eff_rad + c_pos))));
    double sigma_neg = (1 + exp(-M * (alpha_eff_rad - c_neg)) + exp(M * (alpha_eff_rad + c_neg)))
                    / ((1 + exp(-M * (alpha_eff_rad - c_neg))) * (1 + exp(M * (alpha_eff_rad + c_neg))));
    double sigma = (alpha_deg < 0) ? sigma_neg : sigma_pos;

    // CD0 base drag
    double CD0 = 0.071087
        + 0.000139 * de
        + 0.005921 * df
        + 0.001166 * da_L
        + 0.003391 * da_R
        + 1.273205 * Cmu
        + 0.064856 * (alpha_deg * Cmu);

    // k factor
    double k = 0.046 + (0.01 / 120.0) * (df + da_L + da_R);

    // Total drag coefficient
    double CD = CD0 + (1 - sigma) * (k * (CL_meas * CL_meas))
        + sigma * (2.0 * pow(sin(alpha_eff_rad), 2));

    return clamp_double(CD, 0.0, 3.0);
}


/*
 * Cruise Unified Cm
 * Inputs: alpha (deg), delta_e (deg), delta_f (deg), delta_a (deg), C_mu
 * Output: Cm
 */
static inline double compute_Cm_cruise_unified(double alpha_deg, double de, double df, double da_L, double da_R, double Cmu) {
    // Effective angle of attack
    double alpha_eff_deg = alpha_deg + ((alpha_deg >= 0) ? (-0.008112 * de) : (0.000009 * de));
    double alpha_eff_rad = alpha_eff_deg * M_PI / 180.0;

    // Switching function sigma
    const double Mpos = 59.547506;
    const double cpos = 0.153102;
    const double Mneg = 116.594978;
    const double cneg = 0.348137;

    double sigma_pos = (1 + exp(-Mpos * (alpha_eff_rad - cpos)) + exp(Mpos * (alpha_eff_rad + cpos)))
                    / ((1 + exp(-Mpos * (alpha_eff_rad - cpos))) * (1 + exp(Mpos * (alpha_eff_rad + cpos))));
    double sigma_neg = (1 + exp(-Mneg * (alpha_eff_rad - cneg)) + exp(Mneg * (alpha_eff_rad + cneg)))
                    / ((1 + exp(-Mneg * (alpha_eff_rad - cneg))) * (1 + exp(Mneg * (alpha_eff_rad + cneg))));
    double sigma = (alpha_deg < 0) ? sigma_neg : sigma_pos;

    // Flat plate CL
    double CL_flat = 2.0 * copysign(1.0, alpha_eff_rad) * pow(sin(alpha_eff_rad), 2) * cos(alpha_eff_rad);

    // Post-stall moment
    double Cm_post = (alpha_deg >= 0) ? (-0.343986 - 1.359063 * fabs(CL_flat)) : 0.0;

    // Linear moment
    double Cm_lin = 0.340124 + (-0.035189) * alpha_deg;

    // Control terms
    double controls = (-0.077253) * de
                    + 0.008949 * df
                    + 0.001488 * da_L
                    + 0.000651 * da_R
                    + 0.603072 * Cmu
                    + 0.079124 * (alpha_deg * Cmu)
                    + (-0.000863) * (alpha_deg * de);

    double Cm = (1 - sigma) * Cm_lin + sigma * Cm_post + controls;

    return clamp_double(Cm, -3.0, 3.0);
}

/*
 * Takeoff Unified Cm
 * Inputs: alpha (deg), delta_e (deg), delta_f (deg), delta_a (deg), C_mu
 * Output: Cm
 */
static inline double compute_Cm_takeoff_unified(double alpha_deg, double de, double df, double da_L, double da_R, double Cmu) {
    // Effective angle of attack
    double alpha_eff_deg = alpha_deg + ((alpha_deg >= 0) ? (-0.008112 * de) : (0.000009 * de));
    double alpha_eff_rad = alpha_eff_deg * M_PI / 180.0;

    // Switching function sigma
    const double Mpos = 59.547506;
    const double cpos = 0.153102;
    const double Mneg = 116.594978;
    const double cneg = 0.348137;

    double sigma_pos = (1 + exp(-Mpos * (alpha_eff_rad - cpos)) + exp(Mpos * (alpha_eff_rad + cpos)))
                    / ((1 + exp(-Mpos * (alpha_eff_rad - cpos))) * (1 + exp(Mpos * (alpha_eff_rad + cpos))));
    double sigma_neg = (1 + exp(-Mneg * (alpha_eff_rad - cneg)) + exp(Mneg * (alpha_eff_rad + cneg)))
                    / ((1 + exp(-Mneg * (alpha_eff_rad - cneg))) * (1 + exp(Mneg * (alpha_eff_rad + cneg))));
    double sigma = (alpha_deg < 0) ? sigma_neg : sigma_pos;

    // Flat plate CL
    double CL_flat = 2.0 * copysign(1.0, alpha_eff_rad) * pow(sin(alpha_eff_rad), 2) * cos(alpha_eff_rad);

    // Post-stall moment
    double Cm_post = (alpha_deg >= 0) ? (-0.343986 - 1.359063 * fabs(CL_flat)) : 0.0;

    // Linear moment
    double Cm_lin = 0.340124 + (-0.035189) * alpha_deg;

    // Control terms
    double controls = (-0.077253) * de
                    + 0.008949 * df
                    + 0.001488 * da_L
                    + 0.000651 * da_R
                    + 0.603072 * Cmu
                    + 0.079124 * (alpha_deg * Cmu)
                    + (-0.000863) * (alpha_deg * de);

    double Cm = (1 - sigma) * Cm_lin + sigma * Cm_post + controls;

    return clamp_double(Cm, -3.0, 3.0);
}




/*
 * Determine flight mode based on aircraft state
 * Returns true for takeoff mode, false for cruise mode
 * Logic: 
 * - If aircraft is on ground -> takeoff mode (regardless of throttle/speed)
 * - If aircraft is airborne and speed <= 35 m/s -> takeoff mode (regardless of throttle)
 * - If aircraft is airborne and speed > 35 m/s and throttle <= 0.6 -> cruise mode
 * - If aircraft is airborne and speed > 35 m/s and throttle > 0.6 -> takeoff mode (climb)
 */
static inline bool is_takeoff_mode(bool is_on_ground, double throttle, double airspeed) {
    if (is_on_ground) {
        return true;  // Always takeoff mode on ground
    }
    
    if (airspeed <= 35.0) {
        return true;  // Always takeoff mode at low speeds
    }
    
    // At high speeds (> 35 m/s), use throttle to determine mode
    return (throttle > 0.6);  // High throttle = takeoff/climb, low throttle = cruise
}



// Smooth transition function using sigmoid blending
// Returns 0 at speed_low, 1 at speed_high, smooth transition in between
static inline double smooth_transition(double speed, double speed_low, double speed_high) {
    if (speed <= speed_low) return 0.0;
    if (speed >= speed_high) return 1.0;
    
    // Sigmoid transition between speed_low and speed_high
    double normalized = (speed - speed_low) / (speed_high - speed_low);
    // Use smooth sigmoid: 3*x^2 - 2*x^3 (smooth at both ends)
    return normalized * normalized * (3.0 - 2.0 * normalized);
}

// Clamp alpha to reasonable flight envelope
static inline double clamp_alpha(double alpha_deg) {
    double clamped = clamp_double(alpha_deg, -20.0, 20.0);  // Reasonable flight range
    if (fabs(alpha_deg - clamped) > 0.001) {  // If clamping occurred
        // printf("ALPHA CLAMPED: raw=%.3f° -> clamped=%.3f°\n", alpha_deg, clamped);
    }
    return clamped;
}

/*
 * compute_n_from_throttle
 * - Uses expression: n = (19771*throttle - 1896.4)/60
 * - throttle expected in [0,1]. We clamp n to a safe minimum to avoid negative or zero rpm.
 */
static inline double compute_n_from_throttle(double throttle) {
    double n = (19771.0 * throttle - 1896.4) / 60.0;
    // guard against negative or zero spin; choose 1.0 rps as safe minimum
    if (n < 1.0) n = 1.0;
    return n;
}

/*
 * compute_Cmu - Original equation with J clamping
 * J = V_inf / (n * D)
 * D is in meters (user specified D = 0.120 m)
 * Cmu = 0.376/(J^2.025) + 0.0359
 *
 * When J < 0.70, use C_mu value computed at J = 0.70
 */
static inline double compute_Cmu(double V_inf, double n, double D = 0.120) {
    // avoid division by zero / extremely small values
    double Vsafe = (V_inf < 0.1) ? 0.1 : V_inf;       // 0.1 m/s min
    double n_safe = (n < 1e-3) ? 1e-3 : n;           // Prevent division by very small rotor speed
    double J_raw = Vsafe / (n_safe * D);              // Raw advance ratio
    
    // Clamp J to minimum of 0.70
    const double J_min = 0.40;
    double J_clamped = (J_raw < J_min) ? J_min : J_raw;
    
    double Cmu = 0.376284 / pow(J_clamped, 2.05254) + 0.035986;

    if(Cmu > 0.80){
        Cmu = 0.80;
    }
    return Cmu;
}


// // Old analytical CL and CD models removed - now using unified models above



using namespace SITL;

// Global variables for shutdown logging
static FILE* g_csv_file = nullptr;
static char g_current_filename[256] = {0};

// ====================================================================
// LOGGING CONTROL FLAG
// ====================================================================
// Set this flag to true to enable CSV data logging during simulation
// When enabled, logs flight data at 10 Hz to timestamped CSV files
// Files are automatically saved when SITL disconnects
// Set to false to disable logging completely (saves CPU and disk space)
static const bool ENABLE_CSV_LOGGING = false;  // Change to false to disable logging

// Shutdown handler - called when SITL disconnects/terminates
static void cleanup_csv_logging() {
    if (g_csv_file) {
        fclose(g_csv_file);
        g_csv_file = nullptr;
        printf("LOG: Saved and closed log file on SITL shutdown: %s\n", g_current_filename);
    }
}

// Signal handler for graceful shutdown
static void signal_handler(int signal) {
    printf("LOG: Received signal %d, saving log file...\n", signal);
    cleanup_csv_logging();
    exit(signal);
}


using namespace SITL;

Plane::Plane(const char *frame_str) :
    Aircraft(frame_str)
{

    const char *colon = strchr(frame_str, ':');
    size_t slen = strlen(frame_str);
    // The last 5 letters are ".json"
    if (colon != nullptr && slen > 5 && strcmp(&frame_str[slen-5], ".json") == 0) {
        load_coeffs(colon+1);
    } else {
        coefficient = default_coefficients;
    }

    //mass = 2.0f;
    mass = 65.0f;

    //inertias = 3,4,5;
    /*
       scaling from motor power to Newtons. Allows the plane to hold
       vertically against gravity when the motor is at hover_throttle
    */
    // float hover_const = 0.937;
    thrust_scale = 1000.0f; //400.0f; //1000.0f; //400.0f;`
    frame_height = 0.1f;

    ground_behavior = GROUND_BEHAVIOR_FWD_ONLY;
    lock_step_scheduled = true;

    if (strstr(frame_str, "-heavy")) {
        mass = 8;
    }
    if (strstr(frame_str, "-jet")) {
        // a 22kg "jet", level top speed is 102m/s
        mass = 22;
        thrust_scale = (mass * GRAVITY_MSS) / hover_throttle;
    }
    if (strstr(frame_str, "-revthrust")) {
        reverse_thrust = true;
    }
    if (strstr(frame_str, "-elevon")) {
        elevons = true;
    } else if (strstr(frame_str, "-vtail")) {
        vtail = true;
    } else if (strstr(frame_str, "-dspoilers")) {
        dspoilers = true;
    } else if (strstr(frame_str, "-redundant")) {
        redundant = true;
    }
    if (strstr(frame_str, "-elevrev")) {
        reverse_elevator_rudder = true;
    }
    if (strstr(frame_str, "-catapult")) {
        have_launcher = true;
        launch_accel = 15;
        launch_time = 2;
    }
    if (strstr(frame_str, "-bungee")) {
        have_launcher = true;
        launch_accel = 7;
        launch_time = 4;
    }
    if (strstr(frame_str, "-throw")) {
        have_launcher = true;
        launch_accel = 25;
        launch_time = 0.4;
    }
    if (strstr(frame_str, "-tailsitter")) {
        tailsitter = true;
        ground_behavior = GROUND_BEHAVIOR_TAILSITTER;
        thrust_scale *= 1.5;
    }
    if (strstr(frame_str, "-steering")) {
        have_steering = true;
    }

#if AP_FILESYSTEM_FILE_READING_ENABLED
    if (strstr(frame_str, "-3d")) {
        aerobatic = true;
        thrust_scale *= 1.5;
        // setup parameters for plane-3d
        AP_Param::load_defaults_file("@ROMFS/models/plane.parm", false);
        AP_Param::load_defaults_file("@ROMFS/models/plane-3d.parm", false);
    }
#endif

    if (strstr(frame_str, "-ice")) {
        ice_engine = true;
    }

    if (strstr(frame_str, "-soaring")) {
        mass = 2.0;
        coefficient.c_drag_p = 0.05;
    }
}

void Plane::load_coeffs(const char *model_json)
{
    char *fname = nullptr;
    struct stat st;
    if (AP::FS().stat(model_json, &st) == 0) {
        fname = strdup(model_json);
    } else {
        IGNORE_RETURN(asprintf(&fname, "@ROMFS/models/%s", model_json));
        if (AP::FS().stat(model_json, &st) != 0) {
            AP_HAL::panic("%s failed to load", model_json);
        }
    }
    if (fname == nullptr) {
        AP_HAL::panic("%s failed to load", model_json);
    }
    AP_JSON::value *obj = AP_JSON::load_json(model_json);
    if (obj == nullptr) {
        AP_HAL::panic("%s failed to load", model_json);
    }

    enum class VarType {
        FLOAT,
        VECTOR3F,
    };

    struct json_search {
        const char *label;
        void *ptr;
        VarType t;
    };
    
    json_search vars[] = {
#define COFF_FLOAT(s) { #s, &coefficient.s, VarType::FLOAT }
        COFF_FLOAT(s),
        COFF_FLOAT(b),
        COFF_FLOAT(c),
        COFF_FLOAT(c_lift_0),
        COFF_FLOAT(c_lift_deltae),
        COFF_FLOAT(c_lift_a),
        COFF_FLOAT(c_lift_q),
        COFF_FLOAT(mcoeff),
        COFF_FLOAT(oswald),
        COFF_FLOAT(alpha_stall),
        COFF_FLOAT(c_drag_q),
        COFF_FLOAT(c_drag_deltae),
        COFF_FLOAT(c_drag_p),
        COFF_FLOAT(c_y_0),
        COFF_FLOAT(c_y_b),
        COFF_FLOAT(c_y_p),
        COFF_FLOAT(c_y_r),
        COFF_FLOAT(c_y_deltaa),
        COFF_FLOAT(c_y_deltar),
        COFF_FLOAT(c_l_0),
        COFF_FLOAT(c_l_p),
        COFF_FLOAT(c_l_b),
        COFF_FLOAT(c_l_r),
        COFF_FLOAT(c_l_deltaa),
        COFF_FLOAT(c_l_deltar),
        COFF_FLOAT(c_m_0),
        COFF_FLOAT(c_m_a),
        COFF_FLOAT(c_m_q),
        COFF_FLOAT(c_m_deltae),
        COFF_FLOAT(c_n_0),
        COFF_FLOAT(c_n_b),
        COFF_FLOAT(c_n_p),
        COFF_FLOAT(c_n_r),
        COFF_FLOAT(c_n_deltaa),
        COFF_FLOAT(c_n_deltar),
        COFF_FLOAT(deltaa_max),
        COFF_FLOAT(deltae_max),
        COFF_FLOAT(deltar_max),
        { "CGOffset", &coefficient.CGOffset, VarType::VECTOR3F },
    };

    for (uint8_t i=0; i<ARRAY_SIZE(vars); i++) {
        auto v = obj->get(vars[i].label);
        if (v.is<AP_JSON::null>()) {
            // use default value
            continue;
        }
        if (vars[i].t == VarType::FLOAT) {
            parse_float(v, vars[i].label, *((float *)vars[i].ptr));

        } else if (vars[i].t == VarType::VECTOR3F) {
            parse_vector3(v, vars[i].label, *(Vector3f *)vars[i].ptr);

        }
    }

    delete obj;

    ::printf("Loaded plane aero coefficients from %s\n", model_json);
}

void Plane::parse_float(AP_JSON::value val, const char* label, float &param) {
    if (!val.is<double>()) {
        AP_HAL::panic("Bad json type for %s: %s", label, val.to_str().c_str());
    }
    param = val.get<double>();
}

void Plane::parse_vector3(AP_JSON::value val, const char* label, Vector3f &param) {
    if (!val.is<AP_JSON::value::array>() || !val.contains(2) || val.contains(3)) {
        AP_HAL::panic("Bad json type for %s: %s", label, val.to_str().c_str());
    }
    for (uint8_t j=0; j<3; j++) {
        parse_float(val.get(j), label, param[j]);
    }
}

/*
  the following functions are from last_letter
  https://github.com/Georacer/last_letter/blob/master/last_letter/src/aerodynamicsLib.cpp
  many thanks to Georacer!
 */
float Plane::liftCoeff(float alpha) const
{
    const float alpha0 = coefficient.alpha_stall;
    const float M = coefficient.mcoeff;
    const float c_lift_0 = coefficient.c_lift_0;
    const float c_lift_a0 = coefficient.c_lift_a;

    // clamp the value of alpha to avoid exp(90) in calculation of sigmoid
    const float max_alpha_delta = 0.8f;
    if (alpha-alpha0 > max_alpha_delta) {
        alpha = alpha0 + max_alpha_delta;
    } else if (alpha0-alpha > max_alpha_delta) {
        alpha = alpha0 - max_alpha_delta;
    }
	double sigmoid = ( 1+exp(-M*(alpha-alpha0))+exp(M*(alpha+alpha0)) ) / (1+exp(-M*(alpha-alpha0))) / (1+exp(M*(alpha+alpha0)));
	double linear = (1.0-sigmoid) * (c_lift_0 + c_lift_a0*alpha); //Lift at small AoA
	double flatPlate = sigmoid*(2*copysign(1,alpha)*pow(sin(alpha),2)*cos(alpha)); //Lift beyond stall

	float result  = linear+flatPlate;
	return result;
}

float Plane::dragCoeff(float alpha) const
{
    const float b = coefficient.b;
    const float s = coefficient.s;
    const float c_drag_p = coefficient.c_drag_p;
    const float c_lift_0 = coefficient.c_lift_0;
    const float c_lift_a0 = coefficient.c_lift_a;
    const float oswald = coefficient.oswald;
    
	double AR = pow(b,2)/s;
	double c_drag_a = c_drag_p + pow(c_lift_0+c_lift_a0*alpha,2)/(M_PI*oswald*AR);

	return c_drag_a;
}


SITL::Wrench Plane::getForcesAndMoments(float inputAileron, float inputElevator, float inputRudder, float inputThrust,const struct sitl_input &input, bool fm)
{
    const bool default_mode = false;
    if (default_mode) {
   float alpha = angle_of_attack;
    float radtodeg = 180/(3.14);
    // printf("alpha = %.3f",alpha*radtodeg);
	//calculate aerodynamic torque
    float effective_airspeed = airspeed;

    if (tailsitter || aerobatic) {
        /*
          tailsitters get airspeed from prop-wash
         */
        effective_airspeed += inputThrust * 20;

        // reduce effective angle of attack as thrust increases
        alpha *= constrain_float(1 - inputThrust, 0, 1);
    }


    const float c_drag_q = coefficient.c_drag_q;
    const float c_lift_q = coefficient.c_lift_q;
    const float s = coefficient.s;
    const float c = coefficient.c;
    const float b = coefficient.b;
    const float c_drag_deltae = coefficient.c_drag_deltae;
    const float c_lift_deltae = coefficient.c_lift_deltae;
    const float c_y_0 = coefficient.c_y_0;
    const float c_y_b = coefficient.c_y_b;
    const float c_y_p = coefficient.c_y_p;
    const float c_y_r = coefficient.c_y_r;
    const float c_y_deltaa = coefficient.c_y_deltaa;
    const float c_y_deltar = coefficient.c_y_deltar;
    const float c_drag_0 = coefficient.c_drag_0;
    const float c_lift_0 = coefficient.c_lift_0;
    const float c_l_0 = coefficient.c_l_0;
    const float c_l_b = coefficient.c_l_b;
    const float c_l_p = coefficient.c_l_p;
    const float c_l_r = coefficient.c_l_r;
    const float c_l_deltaa = coefficient.c_l_deltaa;
    const float c_l_deltar = coefficient.c_l_deltar;
    const float c_m_0 = coefficient.c_m_0;
    // const float c_m_a = coefficient.c_m_a;
    float c_m_a;
    if (alpha<0){
        c_m_a = 0.06*radtodeg;
    }
    else if (alpha>0 && alpha<(5/radtodeg))
    {
        c_m_a = 0.15*radtodeg;
    }
    else{
        c_m_a = -0.1*radtodeg;
    }
    // printf("Cm_alpha=%0.3f",c_m_a);
    const float c_m_q = coefficient.c_m_q;
    const float c_m_deltae = coefficient.c_m_deltae;
    const float c_n_0 = coefficient.c_n_0;
    const float c_n_b = coefficient.c_n_b;
    const float c_n_p = coefficient.c_n_p;
    const float c_n_r = coefficient.c_n_r;
    const float c_n_deltaa = coefficient.c_n_deltaa;
    const float c_n_deltar = coefficient.c_n_deltar;
    const float Lf = 0.858f;    // CG to nose gear (+x) 
    const float Lb = 0.122f;    // CG to main gear (aft)

    float rho = air_density;

    //const float rho = 1.225; // air density at sea level

	//request lift and drag alpha-coefficients from the corresponding functions
	double c_lift_a = liftCoeff(alpha);
	double c_drag_a = dragCoeff(alpha);

	// === CONSOLIDATED AERODYNAMIC COEFFICIENTS ===
    double p = gyro.x;
	double q = gyro.y;
	double r = gyro.z;
	// Calculate complete lift coefficient CL
	double CL = c_lift_0 + c_lift_a + c_lift_q*c*q/(2*airspeed) + c_lift_deltae*inputElevator;
	
	// Calculate complete drag coefficient CD
	double CD = c_drag_0 + c_drag_a + c_drag_q*c*q/(2*airspeed) + c_drag_deltae*fabs(inputElevator);
	
	// Calculate complete side force coefficient CY
	double CY = c_y_0 + c_y_b*beta + c_y_p*b*p/(2*airspeed) + c_y_r*b*r/(2*airspeed) + c_y_deltaa*inputAileron + c_y_deltar*inputRudder;
	
	// Calculate complete moment coefficients
	double Cl = c_l_0 + c_l_b*beta + c_l_p*b*p/(2*effective_airspeed) + c_l_r*b*r/(2*effective_airspeed) + c_l_deltaa*inputAileron + c_l_deltar*inputRudder;
	double Cm = c_m_0 + c_m_a*alpha + c_m_q*c*q/(2*effective_airspeed) + c_m_deltae*inputElevator;
	double Cn = c_n_0 + c_n_b*beta + c_n_p*b*p/(2*effective_airspeed) + c_n_r*b*r/(2*effective_airspeed) + c_n_deltaa*inputAileron + c_n_deltar*inputRudder;

    float throttle;
    if (reverse_thrust) {
        throttle = filtered_servo_angle(input, 2);
    } else {
        throttle = filtered_servo_range(input, 2);
    }
    
    float thrust     = throttle;
    thrust *= thrust_scale;

    // double aerocfs[6];
    
    // float deg_inputa = inputAileron*radtodeg;
    // float deg_inpute = inputElevator*radtodeg;
    // float deg_inputr = inputRudder*radtodeg;
    // float deg_beta = beta*radtodeg;
    // float deg_alpha = alpha*radtodeg;

    //printf("thrust = %.3f, throttle = %.3f,inputAileron=%.3f,inputElevator=%.3f",thrust, throttle*100,deg_inputa,deg_inpute);
    fm = 0;

    const float theta = AP::ahrs().get_pitch();


    
    float thrust_offset = 0.091;

	double qbar = 1.0/2.0*rho*pow(airspeed,2)*s; //Calculate dynamic pressure
    gcs().send_text(MAV_SEVERITY_INFO, "SAREA(ref area)=%.3f m^2", (double)s);
    float ax = 0.0f, ay = 0.0f, az = 0.0f;   // body forces
    float la = 0.0f, ma = 0.0f, na = 0.0f;   // body moments

    //double Nf,Nb;
    if (is_zero(airspeed))
	{
		ax = 0;
		ay = 0;
		az = 0;
        la = 0;
		ma = 0;
		na = 0;
	}
    else{
        // === NEW CONSOLIDATED FORCE CALCULATIONS ===
        // Body frame forces using complete coefficients
        float fx_aero_b = qbar * (CL*sin(alpha) - CD*cos(alpha));
        float fy_aero_b = qbar * CY;
        float fz_aero_b = qbar * (-CL*cos(alpha) - CD*sin(alpha));
        
        // Legacy calculations (kept for reference/comparison)
        // float fx_aero_b_legacy = qbar*(c_x_0 + c_x_a + c_x_q*c*q/(2*airspeed) - c_drag_deltae*cos(alpha)*fabs(inputElevator) + c_lift_deltae*sin(alpha)*inputElevator);
        // float fy_aero_b_legacy = qbar*(c_y_0 + c_y_b*beta + c_y_p*b*p/(2*airspeed) + c_y_r*b*r/(2*airspeed) + c_y_deltaa*inputAileron + c_y_deltar*inputRudder);
        // float fz_aero_b_legacy = qbar*(c_z_0 + c_z_a + c_z_q*c*q/(2*airspeed) - c_drag_deltae*sin(alpha)*fabs(inputElevator) - c_lift_deltae*cos(alpha)*inputElevator);
        
        float fz_aero_e = sin(theta)*fx_aero_b + cos(theta)*fz_aero_b;
        float fz_thrust_e = -thrust*sin(theta);
        
        // === NEW CONSOLIDATED MOMENT CALCULATIONS ===
        // Moments using complete coefficients
        float l_aero = qbar*b*Cl;
		float m_aero = qbar*c*Cm;
		float n_aero = qbar*b*Cn;
        float m_thrust = thrust*thrust_offset;
    

        // -------------------- Liftoff/landing state machine --------------------
       // Tunables
        const float FORCE_MARGIN         = 0.02f;   // 2% margin above/below mg
        const int   FORCE_HOLD_FRAMES    = 5;       // consecutive frames to confirm force trigger
        const float ALT_HYSTERESIS       = 0.010f;  // 1 cm AGL to latch airborne (wheel clearance)
        const float PEN_ENGAGE           = 0.005f;  // 5 mm engage window (up-positive)
        const float MIN_LOAD_N           = 0.1f;    // treat tiny normals as zero (reduced for debug)

        // altitude-stability detector (frames/tolerance)
        const int   ALT_STABLE_FRAMES    = 5000;       // consecutive frames altitude must be steady
        const float ALT_STABLE_TOL       = 0.002f;   // meters tolerance (2 mm)
        const float VZ_TOUCH_THRESH      = 0.6f;    // m/s; vertical speed threshold for landing detection

        // 1) Geometry
        const Vector3f rNose_b(+Lf, 0.0f, frame_height);
        const Vector3f rMain_b(-Lb, 0.0f, frame_height);
        const Vector3f rNose_e = dcm.transposed() * rNose_b;   // body -> earth (NED)
        const Vector3f rMain_e = dcm.transposed() * rMain_b;

        // 1a) relative position (NED: z down-positive)
        Vector3f locned;
        bool alt_gotten = AP::ahrs().get_relative_position_NED_origin_float(locned);
        // origin_hagl_up: up-positive aircraft origin height (if alt_gotten false -> 0)
        const float origin_hagl_up = alt_gotten ? -locned.z : 0.0f;

        // wheel world Z (down-positive)
        const float wheelNose_world_z = locned.z + rNose_e.z;   // down-positive
        const float wheelMain_world_z = locned.z + rMain_e.z;   // down-positive

        // latched touchdown wheel world Z (down-positive)
        static float ground_wheel_z_touch = 0.0f;
        static bool  ground_wheel_z_latched = false;

        // compute wheel clearance (up-positive) relative to latched wheel world z if latched,
        // otherwise relative to immediate world z=0
        float wheelNose_AGL_up, wheelMain_AGL_up;
        if (ground_wheel_z_latched) {
            wheelNose_AGL_up = ground_wheel_z_touch - wheelNose_world_z; // up-positive
            wheelMain_AGL_up = ground_wheel_z_touch - wheelMain_world_z;
        } else {
            wheelNose_AGL_up = -wheelNose_world_z;
            wheelMain_AGL_up = -wheelMain_world_z;
        }
        wheelNose_AGL_up = std::max(0.0f, wheelNose_AGL_up);
        wheelMain_AGL_up = std::max(0.0f, wheelMain_AGL_up);
        const float minWheel_AGL_up = std::min(wheelNose_AGL_up, wheelMain_AGL_up);
        const float AGL_up = minWheel_AGL_up;

        // 2) S1/S2 provisional split (unchanged)
        const float S1 = mass*GRAVITY_MSS + fz_aero_e + fz_thrust_e; // down-positive
        float cosph_raw = std::cos(theta);
        float cosph = (std::fabs(cosph_raw) < 0.02f) ? (copysign(0.02f, cosph_raw)) : cosph_raw;
        const float S2 = -(m_aero + m_thrust) / cosph;
        float Nf_prov = ( S2 + Lb*S1 ) / (Lf + Lb);
        float Nb_prov = ( Lf*S1 - S2 ) / (Lf + Lb);
        Nf_prov = std::max(0.0f, Nf_prov);
        Nb_prov = std::max(0.0f, Nb_prov);

        // 3) Force trigger (unchanged)
        const float lift_up   = -fz_aero_e;    // up-positive
        const float thrust_up = -fz_thrust_e;  // up-positive
        const float Fz_up     = lift_up + thrust_up; // up-positive net upward force

        static float Fz_up_f = 0.0f;
        const float EMA = 0.3f;
        Fz_up_f = (1.0f - EMA) * Fz_up_f + EMA * Fz_up;

        const bool force_exceeds = (Fz_up_f > mass*GRAVITY_MSS * (1.0f + FORCE_MARGIN));
        const bool force_below   = (Fz_up_f < mass*GRAVITY_MSS * (1.0f - FORCE_MARGIN));

        // 4) altitude-stability bookkeeping (shared across states)
        static float alt_last = 0.0f;
        static int   alt_stable_cnt = 0;
        static bool has_taken_off = false;   
        // update altitude-stability counter if we have valid HAGL
        if (alt_gotten) {
            const float dalt = fabsf(origin_hagl_up - alt_last);
            if (dalt <= ALT_STABLE_TOL) {
                alt_stable_cnt++;
            } else {
                alt_stable_cnt = 0;
            }
            alt_last = origin_hagl_up;
        } else {
            // no alt available, reset stability counter
            alt_stable_cnt = 0;
        }

        // 5) State machine
        enum class AirState : uint8_t { GROUND, FORCE_OK, AIRBORNE };
        static AirState air_state = AirState::GROUND;
        static int force_cnt = 0;

        float Nf = 0.0f, Nb = 0.0f;

        switch (air_state) {
        case AirState::GROUND:
        {
            // If not latched yet (start-on-ground), latch now
            if (!ground_wheel_z_latched && alt_gotten) {
                // choose the wheel with the smallest world z (closest to ground in down-positive)
                ground_wheel_z_touch = std::min(wheelNose_world_z, wheelMain_world_z);
                ground_wheel_z_latched = true;
                std::printf("DBG: initial-ground latch wheel_z_touch=%.3f (down-pos)\n", ground_wheel_z_touch);
            }

            Nf = (Nf_prov > MIN_LOAD_N) ? Nf_prov : 0.0f;
            Nb = (Nb_prov > MIN_LOAD_N) ? Nb_prov : 0.0f;

            // Advance to FORCE_OK only if net upward force persistently > mg
            force_cnt = force_exceeds ? (force_cnt + 1) : 0;
            if (force_cnt >= FORCE_HOLD_FRAMES) {
                air_state = AirState::FORCE_OK;
                force_cnt = 0;
            }
            break;
        }

        case AirState::FORCE_OK:
        {
            // still treat as ground until clearance observed
            Nf = (Nf_prov > MIN_LOAD_N) ? Nf_prov : 0.0f;
            Nb = (Nb_prov > MIN_LOAD_N) ? Nb_prov : 0.0f;
            fm = 1;
            // commit to AIRBORNE only if we see sustained wheel clearance
            if (AGL_up >= ALT_HYSTERESIS) {
                air_state = AirState::AIRBORNE;
                has_taken_off = true; 
                // clear latched touchdown reference so next landing re-latches
                ground_wheel_z_latched = false;
                Nf = Nb = 0.0f;
                break;
            }

            // if upward force collapses before clearance, return to GROUND
            if (!has_taken_off && force_below) {
                air_state = AirState::GROUND;
                force_cnt = 0;
                // keep ground_wheel_z_latched as-is (do not clear)
            }
            break;
        }

        case AirState::AIRBORNE:
        {
            // normals zero while airborne
            Nf = Nb = 0.0f;
            fm = 1;

            // Wheel-penetration touchdown detection (fast)
            const bool nose_touch = (wheelNose_AGL_up <= PEN_ENGAGE);
            const bool main_touch = (wheelMain_AGL_up <= PEN_ENGAGE);
            if (nose_touch || main_touch) {
                air_state = AirState::GROUND;
                printf("CONDITION1\n");
                //exit(0);
                Nf = (Nf_prov > MIN_LOAD_N) ? Nf_prov : 0.0f;
                Nb = (Nb_prov > MIN_LOAD_N) ? Nb_prov : 0.0f;
                // latch world wheel z at touchdown (prefer main wheel)
                ground_wheel_z_touch = main_touch ? wheelMain_world_z : wheelNose_world_z;
                ground_wheel_z_latched = true;
                has_taken_off = false; // landed -> allow next ground latch on next takeoff
                std::printf("DBG: wheel-touch touchdown latched wheel_z_touch=%.3f (down-pos)\n", ground_wheel_z_touch);
                force_cnt = 0;
                break;
            }

            // Robust fallback: force_below + low vertical speed + stable altitude for several frames
            const float vz_down = velocity_ef.z; // NED down-positive
            static int touchdown_force_cnt = 0;
            if (Fz_up_f<10 && fabsf(vz_down) < VZ_TOUCH_THRESH && alt_stable_cnt >= ALT_STABLE_FRAMES) {
                touchdown_force_cnt++;
            } else {
                touchdown_force_cnt = 0;
            }
            if (touchdown_force_cnt >= FORCE_HOLD_FRAMES) {
                printf("CONDITION2\n");
                //exit(0);
                air_state = AirState::GROUND;
                fm=0;
                Nf = (Nf_prov > MIN_LOAD_N) ? Nf_prov : 0.0f;
                Nb = (Nb_prov > MIN_LOAD_N) ? Nb_prov : 0.0f;
                ground_wheel_z_touch = std::min(wheelNose_world_z, wheelMain_world_z);
                ground_wheel_z_latched = true;
                has_taken_off = false;
                touchdown_force_cnt = 0;
                std::printf("DBG: force+stable-alt touchdown latched wheel_z_touch=%.3f (down-pos)\n", ground_wheel_z_touch);
                break;
            }
            break;
        } // end AIRBORNE
        } // end switch
        const float fx_norm_b =  (Nf + Nb) * std::sin(theta);
        const float fz_norm_b = -(Nf + Nb) * std::cos(theta);

        ax = fx_aero_b + fx_norm_b;   // you return aero + ground only
        ay = fy_aero_b;
        az = fz_aero_b + fz_norm_b;

        la = l_aero;
        ma = m_aero + Nb*Lb*std::cos(theta) - Nf*Lf*std::cos(theta) + m_thrust;
        na = n_aero;


    }
    return Wrench{ Vector3f(ax, ay, az), Vector3f(la, ma, na) };

    } else {
        // === ALTERNATIVE FDM MODE ===
        float alpha = angle_of_attack;
        const float phi = AP::ahrs().get_roll();
        const float psi = AP::ahrs().get_yaw();

        float test = phi + psi;
        test = 0;
        float radtodeg = 180/(3.14);
        // printf("alpha = %.3f",alpha*radtodeg);
        
        float effective_airspeed = airspeed + test;
        if (tailsitter || aerobatic) {
            effective_airspeed += inputThrust * 20;
            alpha *= constrain_float(1 - inputThrust, 0, 1);
        }

        // Basic aircraft geometry parameters
        const float s = coefficient.s;
        const float c = coefficient.c;
        const float Lf = 0.858f;    // CG to nose gear (+x) 
        const float Lb = 0.122f;    // CG to main gear (aft)
        float rho = air_density;
        //const float rho = 1.225; // air density at sea level

        // Throttle handling
        float throttle;
        if (reverse_thrust) {
            throttle = filtered_servo_angle(input, 2);
        } else {
            throttle = filtered_servo_range(input, 2);
        }
        
        float thrust = throttle;
        thrust *= thrust_scale;

        // Angular rates
        // double p = gyro.x;
        double q = gyro.y;
        // double r = gyro.z;
        
        // ...existing code...
        // Takeoff/cruise mode determination based on speed
        bool takeoff_mode = (airspeed <= 30.0f);

        // Control surface angles based on mode
        double del_e_deg = inputElevator * radtodeg;
        double del_a_deg = takeoff_mode ? 40.0 : 0.0;  // ailerons

        // Hardcoded individual ailerons for future use
        double aileron_r_deg = takeoff_mode ? 40.0 : 0.0;  // right aileron
        double aileron_l_deg = takeoff_mode ? 40.0 : 0.0;  // left aileron

        double del_f_deg = takeoff_mode ? 40.0 : 0.0;  // flaps
        // ...existing code...
        
        // Alpha conditioning: force to 0 at low speeds, then clamp to ±20°
        double alpha_deg_raw = alpha * radtodeg;
        double alpha_deg;
        if (airspeed < 5.0f) {
            alpha_deg = 0.0;  // Force alpha to 0 below 5 m/s
        } else {
            alpha_deg = clamp_alpha(alpha_deg_raw);  // Clamp to ±20° at higher speeds
        }
        
        // Compute C_mu for propeller effects
        double n = compute_n_from_throttle(throttle);
        double D_prop = 0.120;  // Propeller diameter in meters
        double V_inf = (double)airspeed;
        double C_mu = compute_Cmu(V_inf, n, D_prop);
        
        // Calculate J for display purposes
        double Vsafe = (V_inf < 0.1) ? 0.1 : V_inf;
        double n_safe = (n < 1e-3) ? 1e-3 : n;
        double J_display = Vsafe / (n_safe * D_prop);
        
        // === UNIFIED MODEL CALCULATIONS ===
        double CL_unified, CD_unified;
        
        if (takeoff_mode) {
            CL_unified = compute_CL_takeoff_unified(alpha_deg, del_e_deg, del_f_deg, aileron_l_deg, aileron_r_deg, C_mu);
            CD_unified = compute_CD_takeoff_unified(alpha_deg, del_e_deg, del_f_deg, aileron_l_deg, aileron_r_deg, C_mu, CL_unified);
        } else {
            CL_unified = compute_CL_cruise_unified(alpha_deg, del_e_deg, del_f_deg, aileron_l_deg, aileron_r_deg, C_mu);
            CD_unified = compute_CD_cruise_unified(alpha_deg, del_e_deg, del_f_deg, aileron_l_deg, aileron_r_deg, C_mu, CL_unified);
        }
        

        const float theta = AP::ahrs().get_pitch();
        float thrust_offset = 0.091;
        double qbar = 1.0/2.0*rho*pow(airspeed,2)*s;
        float ax = 0.0f, ay = 0.0f, az = 0.0f;
        float la = 0.0f, ma = 0.0f, na = 0.0f;

        if (is_zero(airspeed)) {
            ax = ay = az = la = ma = na = 0.0f;
        } 
        
        else {
            // === ALTERNATIVE FORCE CALCULATIONS ===
            // Body frame forces using unified coefficients (use conditioned alpha)
            double alpha_rad_conditioned = alpha_deg * M_PI / 180.0;
            float fx_aero_b = qbar * (CL_unified*sin(alpha_rad_conditioned) - CD_unified*cos(alpha_rad_conditioned));
            float fy_aero_b = 0.0f;  // Set to zero as requested
            float fz_aero_b = qbar * (-CL_unified*cos(alpha_rad_conditioned) - CD_unified*sin(alpha_rad_conditioned));
            
            float fz_aero_e = sin(theta)*fx_aero_b + cos(theta)*fz_aero_b;
            float fz_thrust_e = -thrust*sin(theta);
            
            // === MOMENT CALCULATIONS ===
            float l_aero = 0.0f;  // Set to zero as requested
            
            // Compute moment coefficient using unified functions
            double Cm_unified;
            if (takeoff_mode) {
                Cm_unified = compute_Cm_takeoff_unified(alpha_deg, del_e_deg, del_f_deg, aileron_l_deg, aileron_r_deg, C_mu);
            } else {
                Cm_unified = compute_Cm_cruise_unified(alpha_deg, del_e_deg, del_f_deg, aileron_l_deg, aileron_r_deg, C_mu);
            }

            const double Cm_q = -95.2255;  // Pitch damping coefficient
            //const double Cm_q = 0.0;  // Pitch damping coefficient
            double airspeed_safe = (airspeed < 1e-3) ? 1e-3 : airspeed;  // Prevent division by very small airspeed
            double CM_q_contribution = Cm_q * q * c / (2.0 * airspeed_safe);
         
             float m_aero = qbar*c*(Cm_unified+CM_q_contribution) ;
             
             float n_aero = 0.0f;  // Set to zero as requested
             float m_thrust = thrust*thrust_offset;



            // === GROUND MODEL (same as default mode) ===
            // Tunables
            const float FORCE_MARGIN         = 0.02f;
            const int   FORCE_HOLD_FRAMES    = 5;
            const float ALT_HYSTERESIS       = 0.010f;
            const float PEN_ENGAGE           = 0.005f;
            const float MIN_LOAD_N           = 0.1f;
            const int   ALT_STABLE_FRAMES    = 5000;
            const float ALT_STABLE_TOL       = 0.002f;
            const float VZ_TOUCH_THRESH      = 0.6f;

            // Geometry
            const Vector3f rNose_b(+Lf, 0.0f, frame_height);
            const Vector3f rMain_b(-Lb, 0.0f, frame_height);
            const Vector3f rNose_e = dcm.transposed() * rNose_b;
            const Vector3f rMain_e = dcm.transposed() * rMain_b;

            Vector3f locned;
            bool alt_gotten = AP::ahrs().get_relative_position_NED_origin_float(locned);
            const float origin_hagl_up = alt_gotten ? -locned.z : 0.0f;

            const float wheelNose_world_z = locned.z + rNose_e.z;
            const float wheelMain_world_z = locned.z + rMain_e.z;

            static float ground_wheel_z_touch = 0.0f;
            static bool  ground_wheel_z_latched = false;

            float wheelNose_AGL_up, wheelMain_AGL_up;
            if (ground_wheel_z_latched) {
                wheelNose_AGL_up = ground_wheel_z_touch - wheelNose_world_z;
                wheelMain_AGL_up = ground_wheel_z_touch - wheelMain_world_z;
            } else {
                wheelNose_AGL_up = -wheelNose_world_z;
                wheelMain_AGL_up = -wheelMain_world_z;
            }
            wheelNose_AGL_up = std::max(0.0f, wheelNose_AGL_up);
            wheelMain_AGL_up = std::max(0.0f, wheelMain_AGL_up);
            const float minWheel_AGL_up = std::min(wheelNose_AGL_up, wheelMain_AGL_up);
            const float AGL_up = minWheel_AGL_up;

            const float S1 = mass*GRAVITY_MSS + fz_aero_e + fz_thrust_e;
            float cosph_raw = std::cos(theta);
            float cosph = (std::fabs(cosph_raw) < 0.02f) ? (copysign(0.02f, cosph_raw)) : cosph_raw;
            const float S2 = -(m_aero + m_thrust) / cosph;
            float Nf_prov = ( S2 + Lb*S1 ) / (Lf + Lb);
            float Nb_prov = ( Lf*S1 - S2 ) / (Lf + Lb);
            Nf_prov = std::max(0.0f, Nf_prov);
            Nb_prov = std::max(0.0f, Nb_prov);

            const float lift_up   = -fz_aero_e;
            const float thrust_up = -fz_thrust_e;
            const float Fz_up     = lift_up + thrust_up;

            static float Fz_up_f = 0.0f;
            const float EMA = 0.3f;
            Fz_up_f = (1.0f - EMA) * Fz_up_f + EMA * Fz_up;

            const bool force_exceeds = (Fz_up_f > mass*GRAVITY_MSS * (1.0f + FORCE_MARGIN));
            const bool force_below   = (Fz_up_f < mass*GRAVITY_MSS * (1.0f - FORCE_MARGIN));

            static float alt_last = 0.0f;
            static int   alt_stable_cnt = 0;
            static bool has_taken_off = false;
            if (alt_gotten) {
                const float dalt = fabsf(origin_hagl_up - alt_last);
                if (dalt <= ALT_STABLE_TOL) {
                    alt_stable_cnt++;
                } else {
                    alt_stable_cnt = 0;
                }
                alt_last = origin_hagl_up;
            } else {
                alt_stable_cnt = 0;
            }

            enum class AirState : uint8_t { GROUND, FORCE_OK, AIRBORNE };
            static AirState air_state = AirState::GROUND;
            static int force_cnt = 0;

            float Nf = 0.0f, Nb = 0.0f;

            switch (air_state) {
            case AirState::GROUND:
            {
                if (!ground_wheel_z_latched && alt_gotten) {
                    ground_wheel_z_touch = std::min(wheelNose_world_z, wheelMain_world_z);
                    ground_wheel_z_latched = true;
                    std::printf("DBG: initial-ground latch wheel_z_touch=%.3f (down-pos)\n", ground_wheel_z_touch);
                }

                Nf = (Nf_prov > MIN_LOAD_N) ? Nf_prov : 0.0f;
                Nb = (Nb_prov > MIN_LOAD_N) ? Nb_prov : 0.0f;

                force_cnt = force_exceeds ? (force_cnt + 1) : 0;
                if (force_cnt >= FORCE_HOLD_FRAMES) {
                    air_state = AirState::FORCE_OK;
                    force_cnt = 0;
                }
                break;
            }

            case AirState::FORCE_OK:
            {
                Nf = (Nf_prov > MIN_LOAD_N) ? Nf_prov : 0.0f;
                Nb = (Nb_prov > MIN_LOAD_N) ? Nb_prov : 0.0f;
                fm = 1;
                if (AGL_up >= ALT_HYSTERESIS) {
                    air_state = AirState::AIRBORNE;
                    has_taken_off = true; 
                    ground_wheel_z_latched = false;
                    Nf = Nb = 0.0f;
                    break;
                }

                if (!has_taken_off && force_below) {
                    air_state = AirState::GROUND;
                    force_cnt = 0;
                }
                break;
            }

            case AirState::AIRBORNE:
            {
                Nf = Nb = 0.0f;
                fm = 1;

                const bool nose_touch = (wheelNose_AGL_up <= PEN_ENGAGE);
                const bool main_touch = (wheelMain_AGL_up <= PEN_ENGAGE);
                if (nose_touch || main_touch) {
                    air_state = AirState::GROUND;
                    printf("CONDITION1\n");
                    Nf = (Nf_prov > MIN_LOAD_N) ? Nf_prov : 0.0f;
                    Nb = (Nb_prov > MIN_LOAD_N) ? Nb_prov : 0.0f;
                    ground_wheel_z_touch = main_touch ? wheelMain_world_z : wheelNose_world_z;
                    ground_wheel_z_latched = true;
                    has_taken_off = false;
                    std::printf("DBG: wheel-touch touchdown latched wheel_z_touch=%.3f (down-pos)\n", ground_wheel_z_touch);
                    force_cnt = 0;
                    break;
                }

                const float vz_down = velocity_ef.z;
                static int touchdown_force_cnt = 0;
                if (Fz_up_f<10 && fabsf(vz_down) < VZ_TOUCH_THRESH && alt_stable_cnt >= ALT_STABLE_FRAMES) {
                    touchdown_force_cnt++;
                } else {
                    touchdown_force_cnt = 0;
                }
                if (touchdown_force_cnt >= FORCE_HOLD_FRAMES) {
                    printf("CONDITION2\n");
                    air_state = AirState::GROUND;
                    fm=0;
                    Nf = (Nf_prov > MIN_LOAD_N) ? Nf_prov : 0.0f;
                    Nb = (Nb_prov > MIN_LOAD_N) ? Nb_prov : 0.0f;
                    ground_wheel_z_touch = std::min(wheelNose_world_z, wheelMain_world_z);
                    ground_wheel_z_latched = true;
                    has_taken_off = false;
                    touchdown_force_cnt = 0;
                    std::printf("DBG: force+stable-alt touchdown latched wheel_z_touch=%.3f (down-pos)\n", ground_wheel_z_touch);
                    break;
                }
                break;
            }
            }

            // std::printf("ALT_FDM: state=%d fm=%d AGL=%.3f Fz_up=%.1f Nf=%.1f Nb=%.1f\n",
                        // int(air_state), int(fm), AGL_up, Fz_up, Nf, Nb);

            const float fx_norm_b =  (Nf + Nb) * std::sin(theta);
            const float fz_norm_b = -(Nf + Nb) * std::cos(theta);

            ax = fx_aero_b + fx_norm_b;
            ay = fy_aero_b;
            az = fz_aero_b + fz_norm_b;

            la = l_aero;
            ma = m_aero - Nb*Lb*std::cos(theta) + Nf*Lf*std::cos(theta) + m_thrust;
            na = n_aero;

             // Calculate actual forces for debugging
             float total_lift_force = qbar * CL_unified;
             float total_drag_force = qbar * CD_unified;
             float pitch_deg = theta * radtodeg;  // Convert pitch from radians to degrees
             float v_z = velocity_ef.z;  // Vertical speed (down-positive in NED)
             
             printf("ALT_FDM: Mode=%s Vinf=%.2f Pitch=%.2f Vz=%.2f | Alpha=%.2f(raw=%.2f) Del_e=%.2f Del_a=%.2f Del_f=%.2f Del_r=%.2f | J=%.3f Cmu=%.4f | CL=%.4f Lift=%.1f CD=%.4f Drag=%.1f Cm=%.4f PitchMom=%.1f | S=%.3f\n",
                     takeoff_mode ? "TKOFF" : "CRUISE", airspeed, pitch_deg, v_z, alpha_deg, alpha_deg_raw, del_e_deg, del_a_deg, del_f_deg, inputRudder*radtodeg, 
                     J_display, C_mu, CL_unified, total_lift_force, CD_unified, total_drag_force, ma/(0.5*rho*sq(airspeed)*c*s), ma, s);

            // === CSV LOGGING (CONTROLLED BY ENABLE_CSV_LOGGING FLAG) ===
            // Only execute logging code if explicitly enabled via flag at top of file
            if (ENABLE_CSV_LOGGING) {
                // CSV Logging at 10 Hz with timestamped files - saved automatically on SITL disconnect/shutdown
                static bool csv_header_written = false;
                static bool log_initialized = false;
                static uint64_t last_log_time_ms = 0;
                const uint64_t LOG_INTERVAL_MS = 100;  // 100ms = 10 Hz logging rate
                
                uint64_t current_time_ms = AP_HAL::millis64();
             
             // Initialize logging on first call - create timestamped filename and register shutdown handlers
             if (!log_initialized) {
                 time_t rawtime;
                 struct tm * timeinfo;
                 time(&rawtime);
                 timeinfo = localtime(&rawtime);
                 snprintf(g_current_filename, sizeof(g_current_filename), 
                         "alt_fdm_log_%04d%02d%02d_%02d%02d%02d.csv",
                         timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
                         timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
                 
                 g_csv_file = fopen(g_current_filename, "w");
                 log_initialized = true;
                 last_log_time_ms = current_time_ms;  // Initialize timing
                 
                 // Register shutdown handlers to save file on SITL disconnect
                 atexit(cleanup_csv_logging);
                 signal(SIGINT, signal_handler);   // Ctrl+C
                 signal(SIGTERM, signal_handler);  // Termination request
                 signal(SIGHUP, signal_handler);   // Hang up (connection lost)
                 
                 printf("LOG: Started new ALT_FDM session log file at 10 Hz: %s\n", g_current_filename);
                 printf("LOG: Registered shutdown handlers - file will be saved on SITL disconnect\n");
             }
             
             // Write header if needed
             if (g_csv_file && !csv_header_written) {
                 fprintf(g_csv_file, "timestamp_ms,mode,airspeed,pitch_deg,v_z,alpha_deg,alpha_raw,del_e,del_a,del_f,del_r,n,J,C_mu,CL,lift_force,CD,drag_force,Cm,pitch_moment,throttle\n");
                 csv_header_written = true;
             }
             
             // Write data row only at 10 Hz (every 100ms)
             if (g_csv_file && (current_time_ms - last_log_time_ms >= LOG_INTERVAL_MS)) {
                 fprintf(g_csv_file, "%" PRIu64 ",%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.3f,%.4f,%.4f,%.1f,%.4f,%.1f,%.4f,%.1f,%.3f\n",
                         current_time_ms, takeoff_mode ? "TKOFF" : "CRUISE", airspeed, pitch_deg, v_z, alpha_deg, alpha_deg_raw, del_e_deg, del_a_deg, del_f_deg, inputRudder*radtodeg,
                         n, J_display, C_mu, CL_unified, total_lift_force, CD_unified, total_drag_force, ma/(0.5*rho*sq(airspeed)*c*s), ma, throttle);
                 fflush(g_csv_file);  // Ensure data is written immediately
                 last_log_time_ms = current_time_ms;  // Update last log time
             }
            }  // End of ENABLE_CSV_LOGGING flag check
        }
        return Wrench{ Vector3f(ax, ay, az), Vector3f(la, ma, na) };
    }
    return Wrench{ Vector3f(0, 0, 0), Vector3f(0, 0, 0) };
}
void Plane::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, bool fm)
{
    float aileron  = filtered_servo_angle(input, 0);
    float elevator = filtered_servo_angle(input, 1);
    float rudder   = filtered_servo_angle(input, 3);
    bool launch_triggered = input.servos[6] > 1700;
    float throttle;
    if (reverse_elevator_rudder) {
        elevator = -elevator;
        rudder = -rudder;
    }
    if (elevons) {
        // fake an elevon plane
        float ch1 = aileron;
        float ch2 = elevator;
        aileron  = (ch2-ch1)/2.0f;
        // the minus does away with the need for RC2_REVERSED=-1
        elevator = -(ch2+ch1)/2.0f;

        // assume no rudder
        rudder = 0;
    } else if (vtail) {
        // fake a vtail plane
        float ch1 = elevator;
        float ch2 = rudder;
        // this matches VTAIL_OUTPUT==2
        elevator = (ch2-ch1)/2.0f;
        rudder   = (ch2+ch1)/2.0f;
    } else if (dspoilers) {
        // fake a differential spoiler plane. Use outputs 1, 2, 4 and 5
        float dspoiler1_left = filtered_servo_angle(input, 0);
        float dspoiler1_right = filtered_servo_angle(input, 1);
        float dspoiler2_left = filtered_servo_angle(input, 3);
        float dspoiler2_right = filtered_servo_angle(input, 4);
        float elevon_left  = (dspoiler1_left + dspoiler2_left)/2;
        float elevon_right = (dspoiler1_right + dspoiler2_right)/2;
        aileron  = (elevon_right-elevon_left)/2;
        elevator = (elevon_left+elevon_right)/2;
        rudder = fabsf(dspoiler1_right - dspoiler2_right)/2 - fabsf(dspoiler1_left - dspoiler2_left)/2;
    } else if (redundant) {
        // channels 1/9 are left/right ailierons
        // channels 2/10 are left/right elevators
        // channels 4/12 are top/bottom rudders
        aileron  = (filtered_servo_angle(input, 0) + filtered_servo_angle(input, 8)) / 2.0;
        elevator = (filtered_servo_angle(input, 1) + filtered_servo_angle(input, 9)) / 2.0;
        rudder   = (filtered_servo_angle(input, 3) + filtered_servo_angle(input, 11)) / 2.0;
    }
    //printf("Aileron: %.1f elevator: %.1f rudder: %.1f\n", aileron, elevator, rudder);

    if (reverse_thrust) {
        throttle = filtered_servo_angle(input, 2);
    } else {
        throttle = filtered_servo_range(input, 2);
    }
    
    float thrust     = throttle;

    battery_voltage = sitl->batt_voltage - 0.7*throttle;
    battery_current = (battery_voltage/sitl->batt_voltage)*50.0f*sq(throttle);

    if (ice_engine) {
        thrust = icengine.update(input);
    }

    // calculate angle of attack
    angle_of_attack = atan2f(velocity_air_bf.z, velocity_air_bf.x);
    beta = atan2f(velocity_air_bf.y,velocity_air_bf.x);

    if (tailsitter || aerobatic) {
        /*
          tailsitters get 4x the control surfaces
         */
        aileron *= 4;
        elevator *= 4;
        rudder *= 4;
    }
    
     // simulate engine RPM
    motor_mask |= (1U<<2);
    rpm[2] = thrust * 7000;

    // scale thrust to newtons
    thrust *= thrust_scale;
    //float thrust_offset = 0.091;

    // Vector3f force = getForce(aileron, elevator, rudder);
    // rot_accel = getTorque(aileron, elevator, rudder, thrust, force);

    //rot_accel[1] = rot_accel[1]+thrust*thrust_offset;
    Wrench w = getForcesAndMoments(aileron, elevator, rudder, thrust, input,fm);

    Vector3f force = w.F;
    rot_accel = w.M;
    rot_accel[0] = rot_accel[0]/3.0f;
    rot_accel[1] = rot_accel[1]/4.0f;
    rot_accel[2] = rot_accel[2]/5.0f;

    if (have_launcher) {
        /*
          simple simulation of a launcher
         */
        if (launch_triggered) {
            uint64_t now = AP_HAL::millis64();
            if (launch_start_ms == 0) {
                launch_start_ms = now;
            }
            if (now - launch_start_ms < launch_time*1000) {
                force.x += mass * launch_accel;
                force.z += mass * launch_accel/3;
            }
        } else {
            // allow reset of catapult
            launch_start_ms = 0;
        }
    }

    

    accel_body = Vector3f(thrust, 0, 0) + force;
    accel_body /= mass;

    // add some noise
    if (thrust_scale > 0) {
        add_noise(fabsf(thrust) / thrust_scale);
    }

    if (on_ground() && !tailsitter) {
        // add some ground friction
        Vector3f vel_body = dcm.transposed() * velocity_ef;
        accel_body.x -= vel_body.x * 0.3f;
    }
}
    
/*
  update the plane simulation by one time step
 */
void Plane::update(const struct sitl_input &input)
{
    Vector3f rot_accel;

    bool flightmode = update_flight_mode(input);

    update_wind(input);
    
    calculate_forces(input, rot_accel,flightmode);
    
    update_dynamics(rot_accel);

    /*
      add in ground steering, this should be replaced with a proper
      calculation of a nose wheel effect
    */
    if (have_steering && on_ground()) {
        const float steering = filtered_servo_angle(input, 4);
        const Vector3f velocity_bf = dcm.transposed() * velocity_ef;
        const float steer_scale = radians(5);
        gyro.z += steering * velocity_bf.x * steer_scale;
    }

    update_external_payload(input);

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}







