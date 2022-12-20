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
    float target_yaw_rate = 0.5f; // Change to 0.5f to avoid max reverse for an instant JV

    if (!motors->armed()) {
        // Motors should be Stopped
        target_lateral = 0.0f;
        target_forward = 0.0f;
        target_yaw_rate = 0.5f;     // 0.5f means 0 yaw rate. See also AP_MotorsMulticopter.cpp JV
    } else {
        get_pilot_desired_planar_movement(target_lateral, target_forward, target_yaw_rate);      // (PWM) JV
    }

/* static uint8_t counter = 0;         // Use to debug
counter++;
if (counter > 50) {
    counter = 0;
    gcs().send_text(MAV_SEVERITY_CRITICAL, "lateral= %5.3f", (float)target_lateral);
    gcs().send_text(MAV_SEVERITY_CRITICAL, "forward= %5.3f", (float)target_forward);
    gcs().send_text(MAV_SEVERITY_CRITICAL, "yaw= %5.3f", (float)target_yaw_rate);
} */

    // call attitude controller. Will need to call my controller JV
    attitude_control->pcs_manual_bypass(target_lateral, target_forward, target_yaw_rate);
}
