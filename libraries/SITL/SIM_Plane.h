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
  simple plane simulator class
*/

#pragma once

#include "SIM_Aircraft.h"
#include "SIM_ICEngine.h"
#include <Filter/LowPassFilter.h>
#include <AP_JSON/AP_JSON.h>

namespace SITL {

/*
  a very simple plane simulator
 */
struct Wrench {
        Vector3f F; // body forces [Nx, Ny, Nz]
        Vector3f M; // body moments [L, M, N]
    };
class Plane : public Aircraft {
public:
    Plane(const char *frame_str);

    /* update model by one time step */
    virtual void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return NEW_NOTHROW Plane(frame_str);
    }

    const struct Coefficients {
        float s = 0.9;
        float b = 3;
        float c = 0.3;
        float c_lift_0 = 0.56;
        float c_lift_deltae = 0;
        float c_lift_a = 6.9;
        float c_lift_q = 0;
        float mcoeff = 50;
        float oswald = 0.9;
        float alpha_stall = 0.4712;
        float c_drag_0 = 0;
        float c_drag_q = 0;
        float c_drag_deltae = 0.0;
        float c_drag_p = 0.1;
        float c_y_0 = 0;
        float c_y_b = -0.98;
        float c_y_p = 0;
        float c_y_r = 0;
        float c_y_deltaa = 0;
        float c_y_deltar = -0.2;
        float c_l_0 = 0;
        float c_l_p = -1.0;
        float c_l_b = -0.12;
        float c_l_r = 0.14;
        float c_l_deltaa = 0.25;
        float c_l_deltar = -0.037;
        float c_m_0 = 0.045;
        float c_m_a = -0.7;
        float c_m_q = -20;
        float c_m_deltae = 1.0;
        float c_n_0 = 0;
        float c_n_b = 0.25;
        float c_n_p = 0.022;
        float c_n_r = -1;
        float c_n_deltaa = 0.00;
        float c_n_deltar = 0.1;
        float deltaa_max = 0.3491;
        float deltae_max = 0.3491;
        float deltar_max = 0.3491;

        // ====================================================================
        // UNIFIED AERODYNAMIC MODEL COEFFICIENTS
        // ====================================================================
        // CL coefficients for unified model
        float cl_base_0 = 0.195343;
        float cl_base_alpha = 0.111981;
        float cl_deltae = 0.015900;
        float cl_deltaf = 0.009675;
        float cl_deltaa_L = 0.006162;
        float cl_deltaa_R = 0.006168;
        float cl_cmu = 2.252691;
        float cl_alpha_cmu = 0.031598;
        
        // CD coefficients for unified model
        float cd_base_0 = 0.071087;
        float cd_deltae = 0.000139;
        float cd_deltaf = 0.005921;
        float cd_deltaa_L = 0.001166;
        float cd_deltaa_R = 0.003391;
        float cd_cmu = 1.273205;
        float cd_alpha_cmu = 0.064856;
        float cd_k_base = 0.046;
        float cd_k_controls = 0.01 / 120.0;
        
        // CM coefficients for unified model
        float cm_base_0 = 0.340124;
        float cm_base_alpha = -0.035189;
        float cm_deltae = -0.077253;
        float cm_deltaf = 0.008949;
        float cm_deltaa_L = 0.001488;
        float cm_deltaa_R = 0.000651;
        float cm_cmu = 0.603072;
        float cm_alpha_cmu = 0.079124;
        float cm_alpha_deltae = -0.000863;
        float cm_post_stall_0 = -0.343986;
        float cm_post_stall_cl = -1.359063;

        Vector3f CGOffset{0, 0, 0};

    } default_coefficients;

    struct Coefficients coefficient;

protected:
    const float hover_throttle = 0.7f;
    float angle_of_attack;
    float beta;

    float thrust_scale;
    bool reverse_thrust;
    // bool default_mode;
    bool elevons;
    bool vtail;
    bool dspoilers;
    bool redundant;
    bool reverse_elevator_rudder;
    bool ice_engine;
    bool tailsitter;
    bool interp_ok;
    bool aerobatic;
    bool copter_tailsitter;
    bool have_launcher;
    bool have_steering;
    float launch_accel;
    float launch_time;
    uint64_t launch_start_ms;

    const uint8_t throttle_servo = 2;
    const int8_t choke_servo = 14;
    const int8_t ignition_servo = 12;
    const int8_t starter_servo = 13;
    const float slewrate = 100;
    ICEngine icengine{
        throttle_servo,
        choke_servo,
        ignition_servo,
        starter_servo,
        slewrate,
        true
    };
    
    
    // load aero coefficients from a json model file
    void load_coeffs(const char *model_json);
    float liftCoeff(float alpha) const;
    float liftCoeff(float alpha, float CL_linear) const;
    float dragCoeff(float alpha) const;
    Vector3f getForce(float inputAileron, float inputElevator, float inputRudder) const;
    Vector3f getTorque(float inputAileron, float inputElevator, float inputRudder, float inputThrust, const Vector3f &force) const;
    SITL::Wrench getForcesAndMoments(float inputAileron, float inputElevator,
                               float inputRudder, float inputThrust,
                               const struct sitl_input &input,bool fm);
    void calculate_forces(const struct sitl_input &input, Vector3f &rot_accel,bool fm);

private:
    // json parsing helpers (TODO reduce code duplication)
    void parse_float(AP_JSON::value val, const char* label, float &param);
    void parse_vector3(AP_JSON::value val, const char* label, Vector3f &param);
};

} // namespace SITL