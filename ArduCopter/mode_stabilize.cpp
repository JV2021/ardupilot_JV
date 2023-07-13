#include "Copter.h"
// #include <GCS_MAVLink/GCS.h>            // Added this to debug JV

// Init and run calls for stabilize flight mode. Added this init JV 

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void ModeStabilize::run()
{    
    // Get pilot PWM inputs (from roll/pitch to forward/lateral) thrust JV
    float target_lateral, target_forward;

    // get pilot's desired yaw rate. Set to zero at first. When piloting we want a yaw rate instead of an angle JV
    float target_yaw_rate = 0.0f;

    // PCS booleans
    bool auto_yaw_ON = false;
    bool enabled_pcs_rfc = false;

    if (!motors->armed() || pcs_killswitch() || !pcs_homeset() ) {
        // Motors should be Stopped
        target_lateral = 0.0f;
        target_forward = 0.0f;
        target_yaw_rate = 0.0f;     // See AP_MotorsMulticopter.cpp JV
        auto_yaw_ON = false;
        enabled_pcs_rfc = false;
    } else {
        get_pilot_desired_planar_movement(target_lateral, target_forward, target_yaw_rate);      // (PWM) JV
        auto_yaw_ON = true;
        enabled_pcs_rfc = true;
    }

    /* static uint8_t counter = 0;         // Use to debug
    counter++;
    if (counter > 50) {
        counter = 0;
        gcs().send_text(MAV_SEVERITY_CRITICAL, "lateral= %5.3f", (float)target_lateral);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "forward= %5.3f", (float)target_forward);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "yaw= %5.3f", (float)target_yaw_rate);
    } */

    Vector3f dist_vec;              // vector to lead vehicle NED (m)
    Vector3f dist_vec_offs;         // vector to lead vehicle + offset. Null offsets NED (m)
    Vector3f vel_of_target;         // velocity of lead vehicle NED (m/s)
    Vector3f dist_vec_tar_ned;      // Distance vector to pass to RF controller NED (m)
    if (copter.g2.follow_pcs.get_target_dist_and_vel_ned(dist_vec, dist_vec_offs, vel_of_target)) {
        dist_vec_tar_ned.x = dist_vec.x;
        dist_vec_tar_ned.y = dist_vec.y;
        dist_vec_tar_ned.z = dist_vec.z;
    } else {
        dist_vec_tar_ned.x = 0.0f;
        dist_vec_tar_ned.y = 0.0f;
        dist_vec_tar_ned.z = 0.0f;
    }

    // call attitude controller. Will need to call my controller JV
    // attitude_control->pcs_manual_bypass(target_lateral, target_forward, target_yaw_rate);
    attitude_control->pcs_rf_controller( enabled_pcs_rfc, target_forward, target_lateral, dist_vec_tar_ned);      // forward = latitude (North), lateral = longitude (East)
    attitude_control->pcs_auto_yaw(auto_yaw_ON, target_yaw_rate);
    }