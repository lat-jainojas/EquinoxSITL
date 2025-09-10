/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  very simple plane simulator class. Not aerodynamically accurate,
  just enough to be able to debug control logic for new frame types
*/

#include "SIM_Plane.h"
#include<vector>
#include <stdio.h>
#include <AP_Filesystem/AP_Filesystem_config.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_AHRS/AP_AHRS.h>
#include<stdio.h>
#include "aero_data.h"
#include "aero_knn.cpp"
#include "aero_knn.h"
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
    float hover_const = 0.937;
    thrust_scale = (mass * GRAVITY_MSS) / hover_const;
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
{   float alpha = angle_of_attack;
    float radtodeg = 180/(3.14);
    printf("alpha = %.3f",alpha*radtodeg);
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

    // const float c_drag_q = coefficient.c_drag_q;
    // const float c_lift_q = coefficient.c_lift_q;
    const float s = coefficient.s;
    const float c = coefficient.c;
    const float b = coefficient.b;
    // const float c_drag_deltae = coefficient.c_drag_deltae;
    // const float c_lift_deltae = coefficient.c_lift_deltae;
    // const float c_y_0 = coefficient.c_y_0;
    // const float c_y_b = coefficient.c_y_b;
    // const float c_y_p = coefficient.c_y_p;
    // const float c_y_r = coefficient.c_y_r;
    // const float c_y_deltaa = coefficient.c_y_deltaa;
    // const float c_y_deltar = coefficient.c_y_deltar;
    // const float c_drag_0 = coefficient.c_drag_0;
    // const float c_lift_0 = coefficient.c_lift_0;
    // const float c_l_0 = coefficient.c_l_0;
    // const float c_l_b = coefficient.c_l_b;
    // const float c_l_p = coefficient.c_l_p;
    // const float c_l_r = coefficient.c_l_r;
    // const float c_l_deltaa = coefficient.c_l_deltaa;
    // const float c_l_deltar = coefficient.c_l_deltar;
    // const float c_m_0 = coefficient.c_m_0;
    //const float c_m_a = coefficient.c_m_a;
    // float c_m_a;
    // if (alpha<0){
    //     c_m_a = 0.06*radtodeg;
    // }
    // else if (alpha>0 && alpha<(5/radtodeg))
    // {
    //     c_m_a = 0.15*radtodeg;
    // }
    // else{
    //     c_m_a = -0.1*radtodeg;
    // }
    // printf("Cm_alpha=%0.3f",c_m_a);
    // const float c_m_q = coefficient.c_m_q;
    // const float c_m_deltae = coefficient.c_m_deltae;
    // const float c_n_0 = coefficient.c_n_0;
    // const float c_n_b = coefficient.c_n_b;
    // const float c_n_p = coefficient.c_n_p;
    // const float c_n_r = coefficient.c_n_r;
    // const float c_n_deltaa = coefficient.c_n_deltaa;
    // const float c_n_deltar = coefficient.c_n_deltar;
    const float Lf = 0.858f;    // CG to nose gear (+x) 
    const float Lb = 0.122f;    // CG to main gear (aft)

    float rho = air_density;

	// //request lift and drag alpha-coefficients from the corresponding functions
	// double c_lift_a = liftCoeff(alpha);
	// double c_drag_a = dragCoeff(alpha);

	// //convert coefficients to the body frame
    // double c_x_0 = -c_drag_0*cos(alpha)+c_lift_0*sin(alpha);
	// double c_x_a = -c_drag_a*cos(alpha)+c_lift_a*sin(alpha);
	// double c_x_q = -c_drag_q*cos(alpha)+c_lift_q*sin(alpha);
    // double c_z_0 = -c_drag_0*sin(alpha)-c_lift_0*cos(alpha);
	// double c_z_a = -c_drag_a*sin(alpha)-c_lift_a*cos(alpha);
	// double c_z_q = -c_drag_q*sin(alpha)-c_lift_q*cos(alpha);
    float throttle;
    if (reverse_thrust) {
        throttle = filtered_servo_angle(input, 2);
    } else {
        throttle = filtered_servo_range(input, 2);
    }
    
    float thrust     = throttle;
    thrust *= thrust_scale;

    double aerocfs[6];
    
    float deg_inputa = inputAileron*radtodeg;
    float deg_inpute = inputElevator*radtodeg;
    float deg_inputr = inputRudder*radtodeg;
    float deg_beta = beta*radtodeg;
    float deg_alpha = alpha*radtodeg;

    //printf("thrust = %.3f, throttle = %.3f,inputAileron=%.3f,inputElevator=%.3f",thrust, throttle*100,deg_inputa,deg_inpute);
    fm = 0;
    aero_interpolate(throttle*100, deg_inputa, deg_inpute, deg_inputr, deg_beta, deg_alpha, aerocfs,fm);

    float CL_direct = aerocfs[0];
	float CD_direct = aerocfs[1];
	float CY_direct = aerocfs[2];
	float Cl_direct = aerocfs[3];
	float Cm_direct = aerocfs[4];
	float Cn_direct = aerocfs[5];
    const float phi = AP::ahrs().get_pitch();
    float CX_direct = CL_direct*sin(alpha)-CD_direct*cos(alpha)*cos(beta)-CY_direct*cos(alpha)*sin(beta);
    float CZ_direct = -CL_direct*cos(alpha)-CD_direct*sin(alpha)*cos(beta)-CY_direct*sin(alpha)*sin(beta);    
    printf("CL=%.3f,CD=%.3f,CY=%.3f,Cl=%.3f,Cm=%.3f,Cn=%.3f",CL_direct,CD_direct,CY_direct,Cl_direct,Cm_direct,Cn_direct);
    printf("throttle = %.3f, aileron = %.3f, elevator = %.3f, rudder = %.3f, beta = %.3f, alpha = %.3f",throttle*100, deg_inputa, deg_inpute, deg_inputr, deg_beta, deg_alpha);
	//read angular rates
	// double p = gyro.x;
	// double q = gyro.y;
	// double r = gyro.z;
    
    float thrust_offset = 0.091;
	//calculate aerodynamic force
	double qbar = 1.0/2.0*rho*pow(airspeed,2)*s; //Calculate dynamic pressure
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
        //float fx_aero_b = qbar*(c_x_0 + c_x_a + c_x_q*c*q/(2*airspeed) - c_drag_deltae*cos(alpha)*fabs(inputElevator) + c_lift_deltae*sin(alpha)*inputElevator);
		// split c_x_deltae to include "abs" term
		//float fy_aero_b = qbar*(c_y_0 + c_y_b*beta + c_y_p*b*p/(2*airspeed) + c_y_r*b*r/(2*airspeed) + c_y_deltaa*inputAileron + c_y_deltar*inputRudder);
		//float fz_aero_b = qbar*(c_z_0 + c_z_a + c_z_q*c*q/(2*airspeed) - c_drag_deltae*sin(alpha)*fabs(inputElevator) - c_lift_deltae*cos(alpha)*inputElevator);
        float fx_aero_b = qbar*CX_direct;
        float fy_aero_b = qbar*CY_direct;
        float fz_aero_b = qbar*CZ_direct;
        float fz_aero_e = sin(phi)*fx_aero_b + cos(phi)*fz_aero_b;
        float fz_thrust_e = -thrust*sin(phi);
        //float l_aero = qbar*b*(c_l_0 + c_l_b*beta + c_l_p*b*p/(2*effective_airspeed) + c_l_r*b*r/(2*effective_airspeed) + c_l_deltaa*inputAileron + c_l_deltar*inputRudder);
		//float m_aero = qbar*c*(c_m_0 + c_m_a*alpha + c_m_q*c*q/(2*effective_airspeed) + c_m_deltae*inputElevator);
		//float n_aero = qbar*b*(c_n_0 + c_n_b*beta + c_n_p*b*p/(2*effective_airspeed) + c_n_r*b*r/(2*effective_airspeed) + c_n_deltaa*inputAileron + c_n_deltar*inputRudder);
        float l_aero = qbar*b*Cl_direct;
        float m_aero = qbar*c*Cm_direct;
        float n_aero = qbar*b*Cn_direct;
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
        float cosph_raw = std::cos(phi);
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

        // debug print
        // std::printf("DBG: alt_gotten=%d origin_hagl_up=%.3f ground_latched=%d wheel_z_touch=%.3f "
        //             "wheelNose_z=%.3f wheelMain_z=%.3f wheelNose_AGL=%.3f wheelMain_AGL=%.3f AGL=%.3f "
        //             "Fz_up_f=%.2f force_exceeds=%d force_below=%d alt_stable_cnt=%d state=%d\n",
        //             int(alt_gotten),
        //             origin_hagl_up,
        //             int(ground_wheel_z_latched),
        //             ground_wheel_z_touch,
        //             wheelNose_world_z,
        //             wheelMain_world_z,
        //             wheelNose_AGL_up,
        //             wheelMain_AGL_up,
        //             AGL_up,
        //             Fz_up_f,
        //             int(force_exceeds),
        //             int(force_below),
        //             alt_stable_cnt,
        //             int(air_state)
        // );

        // 5) Finish as before: project normals back to BODY and add to aero-only forces
        std::printf("state=%d fm =%d AGL=%.3f Fz_up=%.1f Nf=%.1f Nb=%.1f\n",
                    int(air_state),int(fm), AGL_up, Fz_up, Nf, Nb);
        //std::printf("flightmode = %d,CL=%.5f,CD=%.5f,CM=%.5f,CY=%.5f\n",fm,CL_direct,CD_direct,Cm_direct,CY_direct);

        const float fx_norm_b =  (Nf + Nb) * std::sin(phi);
        const float fz_norm_b = -(Nf + Nb) * std::cos(phi);

        ax = fx_aero_b + fx_norm_b;   // you return aero + ground only
        ay = fy_aero_b;
        az = fz_aero_b + fz_norm_b;

        la = l_aero;
        ma = m_aero + Nb*Lb*std::cos(phi) - Nf*Lf*std::cos(phi) + m_thrust;
        na = n_aero;


    }
    return Wrench{ Vector3f(ax, ay, az), Vector3f(la, ma, na) };

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















// Torque calculation function
// Vector3f Plane::getTorque(float inputAileron, float inputElevator, float inputRudder, float inputThrust, const Vector3f &force) const
// {
//     float alpha = angle_of_attack;

// 	//calculate aerodynamic torque
//     float effective_airspeed = airspeed;

//     if (tailsitter || aerobatic) {
//         /*
//           tailsitters get airspeed from prop-wash
//          */
//         effective_airspeed += inputThrust * 20;

//         // reduce effective angle of attack as thrust increases
//         alpha *= constrain_float(1 - inputThrust, 0, 1);
//     }
    
//     const float s = coefficient.s;
//     const float c = coefficient.c;
//     const float b = coefficient.b;
//     const float c_l_0 = coefficient.c_l_0;
//     const float c_l_b = coefficient.c_l_b;
//     const float c_l_p = coefficient.c_l_p;
//     const float c_l_r = coefficient.c_l_r;
//     const float c_l_deltaa = coefficient.c_l_deltaa;
//     const float c_l_deltar = coefficient.c_l_deltar;
//     const float c_m_0 = coefficient.c_m_0;
//     const float c_m_a = coefficient.c_m_a;
//     const float c_m_q = coefficient.c_m_q;
//     const float c_m_deltae = coefficient.c_m_deltae;
//     const float c_n_0 = coefficient.c_n_0;
//     const float c_n_b = coefficient.c_n_b;
//     const float c_n_p = coefficient.c_n_p;
//     const float c_n_r = coefficient.c_n_r;
//     const float c_n_deltaa = coefficient.c_n_deltaa;
//     const float c_n_deltar = coefficient.c_n_deltar;
//     const Vector3f &CGOffset = coefficient.CGOffset;
    
//     float rho = air_density;

// 	//read angular rates
// 	double p = gyro.x;
// 	double q = gyro.y;
// 	double r = gyro.z;

// 	double qbar = 1.0/2.0*rho*pow(effective_airspeed,2)*s; //Calculate dynamic pressure
// 	double la, na, ma;
// 	if (is_zero(effective_airspeed))
// 	{
// 		la = 0;
// 		ma = 0;
// 		na = 0;
// 	}
// 	else
// 	{
// 		la = qbar*b*(c_l_0 + c_l_b*beta + c_l_p*b*p/(2*effective_airspeed) + c_l_r*b*r/(2*effective_airspeed) + c_l_deltaa*inputAileron + c_l_deltar*inputRudder);
// 		ma = qbar*c*(c_m_0 + c_m_a*alpha + c_m_q*c*q/(2*effective_airspeed) + c_m_deltae*inputElevator);
// 		na = qbar*b*(c_n_0 + c_n_b*beta + c_n_p*b*p/(2*effective_airspeed) + c_n_r*b*r/(2*effective_airspeed) + c_n_deltaa*inputAileron + c_n_deltar*inputRudder);
// 	}


// 	// Add torque to force misalignment with CG
// 	// r x F, where r is the distance from CoG to CoL
// 	la +=  CGOffset.y * force.z - CGOffset.z * force.y;
// 	ma += -CGOffset.x * force.z + CGOffset.z * force.x;
// 	na += -CGOffset.y * force.x + CGOffset.x * force.y;

// 	return Vector3f(la, ma, na);
// }

// // Force calculation function from last_letter
// Vector3f Plane::getForce(float inputAileron, float inputElevator, float inputRudder) const
// {
//     const float alpha = angle_of_attack;
//     const float c_drag_q = coefficient.c_drag_q;
//     const float c_lift_q = coefficient.c_lift_q;
//     const float s = coefficient.s;
//     const float c = coefficient.c;
//     const float b = coefficient.b;
//     const float c_drag_deltae = coefficient.c_drag_deltae;
//     const float c_lift_deltae = coefficient.c_lift_deltae;
//     const float c_y_0 = coefficient.c_y_0;
//     const float c_y_b = coefficient.c_y_b;
//     const float c_y_p = coefficient.c_y_p;
//     const float c_y_r = coefficient.c_y_r;
//     const float c_y_deltaa = coefficient.c_y_deltaa;
//     const float c_y_deltar = coefficient.c_y_deltar;
//     const float c_drag_0 = coefficient.c_drag_0;
//     const float c_lift_0 = coefficient.c_lift_0;
    


//     float rho = air_density;

// 	//request lift and drag alpha-coefficients from the corresponding functions
// 	double c_lift_a = liftCoeff(alpha);
// 	double c_drag_a = dragCoeff(alpha);

// 	//convert coefficients to the body frame
//     double c_x_0 = -c_drag_0*cos(alpha)+c_lift_0*sin(alpha);
// 	double c_x_a = -c_drag_a*cos(alpha)+c_lift_a*sin(alpha);
// 	double c_x_q = -c_drag_q*cos(alpha)+c_lift_q*sin(alpha);
//     double c_z_0 = -c_drag_0*sin(alpha)-c_lift_0*cos(alpha);
// 	double c_z_a = -c_drag_a*sin(alpha)-c_lift_a*cos(alpha);
// 	double c_z_q = -c_drag_q*sin(alpha)-c_lift_q*cos(alpha);

// 	//read angular rates
// 	double p = gyro.x;
// 	double q = gyro.y;
// 	double r = gyro.z;

// 	//calculate aerodynamic force
// 	double qbar = 1.0/2.0*rho*pow(airspeed,2)*s; //Calculate dynamic pressure
// 	double ax, ay, az;
// 	if (is_zero(airspeed))
// 	{
// 		ax = 0;
// 		ay = 0;
// 		az = 0;
// 	}
// 	else
// 	{
// 		ax = qbar*(c_x_0 + c_x_a + c_x_q*c*q/(2*airspeed) - c_drag_deltae*cos(alpha)*fabs(inputElevator) + c_lift_deltae*sin(alpha)*inputElevator);
// 		// split c_x_deltae to include "abs" term
// 		ay = qbar*(c_y_0 + c_y_b*beta + c_y_p*b*p/(2*airspeed) + c_y_r*b*r/(2*airspeed) + c_y_deltaa*inputAileron + c_y_deltar*inputRudder);
// 		az = qbar*(c_z_0 + c_z_a + c_z_q*c*q/(2*airspeed) - c_drag_deltae*sin(alpha)*fabs(inputElevator) - c_lift_deltae*cos(alpha)*inputElevator);
// 		// split c_z_deltae to include "abs" term
// 	}
//     return Vector3f(ax, ay, az);
// }