#include "AC_AttitudeControl_Multi.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h> // Added this to debug JV

// table of user settable parameters
const AP_Param::GroupInfo AC_AttitudeControl_Multi::var_info[] = {
    // parameters from parent vehicle
    AP_NESTEDGROUPINFO(AC_AttitudeControl, 0),

    // @Param: RAT_RLL_P
    // @DisplayName: Roll axis rate controller P gain
    // @Description: Roll axis rate controller P gain.  Converts the difference between desired roll rate and actual roll rate into a motor speed output
    // @Range: 0.01 0.5
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_RLL_I
    // @DisplayName: Roll axis rate controller I gain
    // @Description: Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_RLL_IMAX
    // @DisplayName: Roll axis rate controller I gain maximum
    // @Description: Roll axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_RLL_D
    // @DisplayName: Roll axis rate controller D gain
    // @Description: Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_RLL_FF
    // @DisplayName: Roll axis rate controller feed forward
    // @Description: Roll axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_RLL_FLTT
    // @DisplayName: Roll axis rate controller target frequency in Hz
    // @Description: Roll axis rate controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_FLTE
    // @DisplayName: Roll axis rate controller error frequency in Hz
    // @Description: Roll axis rate controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_FLTD
    // @DisplayName: Roll axis rate controller derivative frequency in Hz
    // @Description: Roll axis rate controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_SMAX
    // @DisplayName: Roll slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_roll, "RAT_RLL_", 1, AC_AttitudeControl_Multi, AC_PID),

    // @Param: RAT_PIT_P
    // @DisplayName: Pitch axis rate controller P gain
    // @Description: Pitch axis rate controller P gain.  Converts the difference between desired pitch rate and actual pitch rate into a motor speed output
    // @Range: 0.01 0.50
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_PIT_I
    // @DisplayName: Pitch axis rate controller I gain
    // @Description: Pitch axis rate controller I gain.  Corrects long-term difference in desired pitch rate vs actual pitch rate
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_PIT_IMAX
    // @DisplayName: Pitch axis rate controller I gain maximum
    // @Description: Pitch axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_PIT_D
    // @DisplayName: Pitch axis rate controller D gain
    // @Description: Pitch axis rate controller D gain.  Compensates for short-term change in desired pitch rate vs actual pitch rate
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_PIT_FF
    // @DisplayName: Pitch axis rate controller feed forward
    // @Description: Pitch axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_PIT_FLTT
    // @DisplayName: Pitch axis rate controller target frequency in Hz
    // @Description: Pitch axis rate controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_FLTE
    // @DisplayName: Pitch axis rate controller error frequency in Hz
    // @Description: Pitch axis rate controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_FLTD
    // @DisplayName: Pitch axis rate controller derivative frequency in Hz
    // @Description: Pitch axis rate controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_SMAX
    // @DisplayName: Pitch slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_pitch, "RAT_PIT_", 2, AC_AttitudeControl_Multi, AC_PID),

    // @Param: RAT_YAW_P
    // @DisplayName: Yaw axis rate controller P gain
    // @Description: Yaw axis rate controller P gain.  Converts the difference between desired yaw rate and actual yaw rate into a motor speed output
    // @Range: 0.10 2.50
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_YAW_I
    // @DisplayName: Yaw axis rate controller I gain
    // @Description: Yaw axis rate controller I gain.  Corrects long-term difference in desired yaw rate vs actual yaw rate
    // @Range: 0.010 1.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_YAW_IMAX
    // @DisplayName: Yaw axis rate controller I gain maximum
    // @Description: Yaw axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_YAW_D
    // @DisplayName: Yaw axis rate controller D gain
    // @Description: Yaw axis rate controller D gain.  Compensates for short-term change in desired yaw rate vs actual yaw rate
    // @Range: 0.000 0.02
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_YAW_FF
    // @DisplayName: Yaw axis rate controller feed forward
    // @Description: Yaw axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_YAW_FLTT
    // @DisplayName: Yaw axis rate controller target frequency in Hz
    // @Description: Yaw axis rate controller target frequency in Hz
    // @Range: 1 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_FLTE
    // @DisplayName: Yaw axis rate controller error frequency in Hz
    // @Description: Yaw axis rate controller error frequency in Hz
    // @Range: 0 20
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_FLTD
    // @DisplayName: Yaw axis rate controller derivative frequency in Hz
    // @Description: Yaw axis rate controller derivative frequency in Hz
    // @Range: 5 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_SMAX
    // @DisplayName: Yaw slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_yaw, "RAT_YAW_", 3, AC_AttitudeControl_Multi, AC_PID),

    // @Param: THR_MIX_MIN
    // @DisplayName: Throttle Mix Minimum
    // @Description: Throttle vs attitude control prioritisation used when landing (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.1 0.25
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MIN", 4, AC_AttitudeControl_Multi, _thr_mix_min, AC_ATTITUDE_CONTROL_MIN_DEFAULT),

    // @Param: THR_MIX_MAX
    // @DisplayName: Throttle Mix Maximum
    // @Description: Throttle vs attitude control prioritisation used during active flight (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.5 0.9
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MAX", 5, AC_AttitudeControl_Multi, _thr_mix_max, AC_ATTITUDE_CONTROL_MAX_DEFAULT),

    // @Param: THR_MIX_MAN
    // @DisplayName: Throttle Mix Manual
    // @Description: Throttle vs attitude control prioritisation used during manual flight (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.1 0.9
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MAN", 6, AC_AttitudeControl_Multi, _thr_mix_man, AC_ATTITUDE_CONTROL_MAN_DEFAULT),

    // @Param: ayaw_P
    // @DisplayName: PCS yaw controller proportional gain on angular position error in 1/rad
    // @Description: P Gain which produces an output value that is proportional to the current error value
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("ayaw_P", 7, AC_AttitudeControl_Multi, _ayaw_kp, AC_ATTITUDE_CONTROL_ayaw_kp_DEFAULT),

    // @Param: ayaw_plt
	// @DisplayName: Pilot input proportional gain in rad/sec
	// @Description: Pilot proportional gain which converts an input -1 to +1 to the desired output range in rad/sec
	// @Range: 0.7 4.0
	// @User: Advanced
	AP_GROUPINFO("ayaw_plt", 12, AC_AttitudeControl_Multi, _ayaw_plt, AC_ATTITUDE_CONTROL_ayaw_plt_DEFAULT),

    // @Param: rfc_vel_plt
	// @DisplayName: Rotating force controller pilot input scaler in m/s
	// @Description: Pilot proportional gain which converts an input -1 to +1 to the desired output range in m/s
	// @Range: 0.0 4.0
	// @User: Advanced
	AP_GROUPINFO("rfc_vel_plt", 13, AC_AttitudeControl_Multi, _rfc_vel_plt, AC_ATTITUDE_CONTROL_rfc_vel_plt_DEFAULT),

    // @Param: rfc_vel_kp
	// @DisplayName: Rotating force controller velocity kp N*s/m
	// @Description: Rotating force controller velocity proportional gain in N*s/m
	// @Range: 4.0 10.0
	// @User: Advanced
	AP_GROUPINFO("rfc_vel_kp", 14, AC_AttitudeControl_Multi, _rfc_vel_kp, AC_ATTITUDE_CONTROL_rfc_vel_kp_DEFAULT),

    // @Param: rfc_pos_kp
	// @DisplayName: Rotating force controller position kp N/m
	// @Description: Rotating force controller position proportional gain in N/m
	// @Range: 1.0 10.0
	// @User: Advanced
	AP_GROUPINFO("rfc_pos_kp", 15, AC_AttitudeControl_Multi, _rfc_pos_kp, AC_ATTITUDE_CONTROL_rfc_pos_kp_DEFAULT),

    // @Param: idle_on
	// @DisplayName: Motor idling boolean parameter
	// @Description: Motor idling boolean parameter
	// @Range: 0 1
	// @User: Advanced
	AP_GROUPINFO("idle_on", 16, AC_AttitudeControl_Multi, _idle_on, AC_ATTITUDE_CONTROL_idle_on_DEFAULT),

    // @Param: idle_thrust
	// @DisplayName: Motor idling thrust parameter N
	// @Description: Motor idling thrust parameter N
	// @Range: 0.0 0.5
	// @User: Advanced
	AP_GROUPINFO("idle_thrust", 17, AC_AttitudeControl_Multi, _idle_thrust, AC_ATTITUDE_CONTROL_idle_thrust_DEFAULT),

    // @Param: ayaw_D
    // @DisplayName: PCS yaw controller proportional gain on the derivative of the angular position error in s/rad
    // @Description: D Gain which produces an output value that is proportional to the derivative of the error value
    // @Range: 0.0 3.0
    // @User: Advanced
    AP_GROUPINFO("ayaw_D", 18, AC_AttitudeControl_Multi, _ayaw_kd, AC_ATTITUDE_CONTROL_ayaw_kd_DEFAULT),

    // @Param: side_force
	// @DisplayName: Norm of rotating side force N
	// @Description: Norm of rotating side force N
	// @Range: 0.0 10.0
	// @User: Advanced
	AP_GROUPINFO("side_force", 19, AC_AttitudeControl_Multi, _side_force, AC_ATTITUDE_CONTROL_side_force_DEFAULT),

    // @Param: ayaw_off
    // @DisplayName: PCS yaw heading offset in deg
    // @Description: PCS yaw heading offset in deg
    // @Range: 0.0 359.0
    // @User: Advanced
    AP_GROUPINFO("ayaw_off", 20, AC_AttitudeControl_Multi, _ayaw_off, AC_ATTITUDE_CONTROL_ayaw_off_DEFAULT),

    AP_GROUPEND
};          // PCS new parameters were added at the end. Param JV

AC_AttitudeControl_Multi::AC_AttitudeControl_Multi(AP_AHRS_View &ahrs, const AP_Vehicle::MultiCopter &aparm, AP_MotorsMulticopter& motors, float dt) :
    AC_AttitudeControl(ahrs, aparm, motors, dt),
    _motors_multi(motors),
    _pid_rate_roll(AC_ATC_MULTI_RATE_RP_P, AC_ATC_MULTI_RATE_RP_I, AC_ATC_MULTI_RATE_RP_D, 0.0f, AC_ATC_MULTI_RATE_RP_IMAX, AC_ATC_MULTI_RATE_RP_FILT_HZ, 0.0f, AC_ATC_MULTI_RATE_RP_FILT_HZ, dt),
    _pid_rate_pitch(AC_ATC_MULTI_RATE_RP_P, AC_ATC_MULTI_RATE_RP_I, AC_ATC_MULTI_RATE_RP_D, 0.0f, AC_ATC_MULTI_RATE_RP_IMAX, AC_ATC_MULTI_RATE_RP_FILT_HZ, 0.0f, AC_ATC_MULTI_RATE_RP_FILT_HZ, dt),
    _pid_rate_yaw(AC_ATC_MULTI_RATE_YAW_P, AC_ATC_MULTI_RATE_YAW_I, AC_ATC_MULTI_RATE_YAW_D, 0.0f, AC_ATC_MULTI_RATE_YAW_IMAX, AC_ATC_MULTI_RATE_RP_FILT_HZ, AC_ATC_MULTI_RATE_YAW_FILT_HZ, 0.0f, dt)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// Update Alt_Hold angle maximum
void AC_AttitudeControl_Multi::update_althold_lean_angle_max(float throttle_in)
{
    // calc maximum tilt angle based on throttle
    float thr_max = _motors_multi.get_throttle_thrust_max();

    // divide by zero check
    if (is_zero(thr_max)) {
        _althold_lean_angle_max = 0.0f;
        return;
    }

    float althold_lean_angle_max = acosf(constrain_float(throttle_in / (AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX * thr_max), 0.0f, 1.0f));
    _althold_lean_angle_max = _althold_lean_angle_max + (_dt / (_dt + _angle_limit_tc)) * (althold_lean_angle_max - _althold_lean_angle_max);
}

void AC_AttitudeControl_Multi::set_throttle_out(float throttle_in, bool apply_angle_boost, float filter_cutoff)
{
    _throttle_in = throttle_in;
    update_althold_lean_angle_max(throttle_in);
    _motors.set_throttle_filter_cutoff(filter_cutoff);
    if (apply_angle_boost) {
        // Apply angle boost
        throttle_in = get_throttle_boosted(throttle_in);
    } else {
        // Clear angle_boost for logging purposes
        _angle_boost = 0.0f;
    }
    _motors.set_throttle(throttle_in);
    _motors.set_throttle_avg_max(get_throttle_avg_max(MAX(throttle_in, _throttle_in)));
}

void AC_AttitudeControl_Multi::set_throttle_mix_max(float ratio)
{
    ratio = constrain_float(ratio, 0.0f, 1.0f);
    _throttle_rpy_mix_desired = (1.0f - ratio) * _thr_mix_min + ratio * _thr_mix_max;
}

// returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1
float AC_AttitudeControl_Multi::get_throttle_boosted(float throttle_in)
{
    if (!_angle_boost_enabled) {
        _angle_boost = 0;
        return throttle_in;
    }
    // inverted_factor is 1 for tilt angles below 60 degrees
    // inverted_factor reduces from 1 to 0 for tilt angles between 60 and 90 degrees

    float cos_tilt = _ahrs.cos_pitch() * _ahrs.cos_roll();
    float inverted_factor = constrain_float(10.0f * cos_tilt, 0.0f, 1.0f);
    float cos_tilt_target = cosf(_thrust_angle);
    float boost_factor = 1.0f / constrain_float(cos_tilt_target, 0.1f, 1.0f);

    float throttle_out = throttle_in * inverted_factor * boost_factor;
    _angle_boost = constrain_float(throttle_out - throttle_in, -1.0f, 1.0f);
    return throttle_out;
}

// returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1
float AC_AttitudeControl_Multi::get_throttle_avg_max(float throttle_in)
{
    throttle_in = constrain_float(throttle_in, 0.0f, 1.0f);
    return MAX(throttle_in, throttle_in * MAX(0.0f, 1.0f - _throttle_rpy_mix) + _motors.get_throttle_hover() * _throttle_rpy_mix);
}

// update_throttle_rpy_mix - slew set_throttle_rpy_mix to requested value
void AC_AttitudeControl_Multi::update_throttle_rpy_mix()
{
    // slew _throttle_rpy_mix to _throttle_rpy_mix_desired
    if (_throttle_rpy_mix < _throttle_rpy_mix_desired) {
        // increase quickly (i.e. from 0.1 to 0.9 in 0.4 seconds)
        _throttle_rpy_mix += MIN(2.0f * _dt, _throttle_rpy_mix_desired - _throttle_rpy_mix);
    } else if (_throttle_rpy_mix > _throttle_rpy_mix_desired) {
        // reduce more slowly (from 0.9 to 0.1 in 1.6 seconds)
        _throttle_rpy_mix -= MIN(0.5f * _dt, _throttle_rpy_mix - _throttle_rpy_mix_desired);
    }
    _throttle_rpy_mix = constrain_float(_throttle_rpy_mix, 0.1f, AC_ATTITUDE_CONTROL_MAX);
}

void AC_AttitudeControl_Multi::rate_controller_run()
{
    // move throttle vs attitude mixing towards desired (called from here because this is conveniently called on every iteration)
    update_throttle_rpy_mix();

    _ang_vel_body += _sysid_ang_vel_body;

    Vector3f gyro_latest = _ahrs.get_gyro_latest();

    _motors.set_roll(get_rate_roll_pid().update_all(_ang_vel_body.x, gyro_latest.x, _motors.limit.roll) + _actuator_sysid.x);
    _motors.set_roll_ff(get_rate_roll_pid().get_ff());

    _motors.set_pitch(get_rate_pitch_pid().update_all(_ang_vel_body.y, gyro_latest.y, _motors.limit.pitch) + _actuator_sysid.y);
    _motors.set_pitch_ff(get_rate_pitch_pid().get_ff());

    _motors.set_yaw(get_rate_yaw_pid().update_all(_ang_vel_body.z, gyro_latest.z, _motors.limit.yaw) + _actuator_sysid.z);
    _motors.set_yaw_ff(get_rate_yaw_pid().get_ff()*_feedforward_scalar);

    _sysid_ang_vel_body.zero();
    _actuator_sysid.zero();

    control_monitor_update();
}

// Get pilot input and and apply limit in the range -1 ~ +1. JV
void AC_AttitudeControl_Multi::pcs_manual_bypass(float lateral_temp, float forward_temp, float yaw_rate_temp)
{
    float target_lateral = lateral_temp;
    float target_forward = forward_temp;
    float target_yaw_rate = yaw_rate_temp;

    if ((target_lateral < -1.000f) || (target_lateral > 1.000f)){
        target_lateral = 0.0f;
    } else if ((target_forward < -1.000f) || (target_forward > 1.000f)){
        target_forward = 0.0f;
    } else if ((target_yaw_rate < -1.000f) || (target_yaw_rate > 1.000f)){
        target_yaw_rate = 0.0f;
    }

    // Logging JV
    _pcscmd.pcs_tar_lat = target_lateral;
    _pcscmd.pcs_tar_fwd = target_forward;

    _motors.set_lateral(target_lateral);          // Set lateral. To be used in output()
    _motors.set_forward(target_forward);          // Set forward
    // _motors.set_yaw(target_yaw_rate);           // Now pcs_auto_yaw
    //    _motors.set_yaw_ff(get_rate_yaw_pid().get_ff()*_feedforward_scalar);          // Useful later JV
    /* static uint8_t counter = 0;         // Use to debug JV
    counter++;
    if (counter > 50) {
        counter = 0;
        gcs().send_text(MAV_SEVERITY_CRITICAL, "lateral= %5.3f", (float)_motors.get_lateral());
        gcs().send_text(MAV_SEVERITY_CRITICAL, "forward= %5.3f", (float)_motors.get_forward());
        gcs().send_text(MAV_SEVERITY_CRITICAL, "yaw= %5.3f", (float)_motors.get_yaw());
    } */
}

// Set a yaw command based only on the angular velocity from gyros. Ayaw JV
void AC_AttitudeControl_Multi::pcs_auto_yaw(bool enabled_auto_yaw, float yaw_rate_temp2)
{   // TODO JV: Replace ayaw_kp_vel by a parameter.
    float target_yaw_rate = yaw_rate_temp2 * _ayaw_plt;        // I want pilot input as target in the range -1.571 ~ +1.571 rad/s
    float angular_speed_z = _ahrs.get_gyro_latest().z;         // Gyros smoothed angular velocity in yaw (rad/s)
    float cmd_yaw = 0.0f;                                      // yaw command
    float ayaw_kp_vel = 0.4f;                                      // Temporary gain for angular velocity error term

    if (enabled_auto_yaw && ((angular_speed_z * angular_speed_z) <= 625.0f ))   // Second condition is a safety if angular speed in yaw exceeds 25 rad/s
    {
        cmd_yaw = ayaw_kp_vel * (target_yaw_rate - angular_speed_z);     // Proportional yaw_rate controller

        if (cmd_yaw > 1.0f) {
            cmd_yaw = 1.0f;
        } else if (cmd_yaw < -1.0f) {
                    cmd_yaw = -1.0f;
                }
    } else {
        cmd_yaw = 0.0f;
    }

    _pcscmd.pcs_tar_yaw = cmd_yaw;          // Logging JV

    _motors.set_yaw(cmd_yaw);           // Set yaw command
}

// Converts Thrust (N) to a cmd from -1 to +1. For BR2212 motor with 1045 prop. RFC JV
float AC_AttitudeControl_Multi::thrust_model_br2212(float thrust_body)
{
    float thrust_sign = 1.0f;
    float max_cmd = 0.681f;         // Maximum command for BR2212 (PWM: 1681 microsec)
    float cmd_0_to_1 = 0.0f;
    float coeff_x2 = 6.6109f;       // Quadratic approximation coefficient
    float coeff_x1 = 6.7972f;       // Quadratic approximation coefficient
    float coeff_x0 = -0.23523f;       // Quadratic approximation coefficient

    if ((thrust_body < 0.25f ) && (thrust_body > -0.25f)) {
        return cmd_0_to_1;

    } else if (thrust_body < 0.0f) {
        thrust_sign = -1.0f;

    } else if ((thrust_body <= -7.45f ) || (thrust_body >= 7.45f)) {
        return (max_cmd * thrust_sign);

    }

    cmd_0_to_1 = (-coeff_x1 + safe_sqrt(coeff_x1 * coeff_x1 - 4.0f * coeff_x2 * (coeff_x0 - thrust_sign * thrust_body))) / (2.0f * coeff_x2);

    return (thrust_sign * cmd_0_to_1);
} 

// Converts Thrust (N) to a cmd from -1 to +1. For BA2310-1220Kv motor with APC 7x5E prop. RFC JV
float AC_AttitudeControl_Multi::thrust_model_ba2310apc7x5(float thrust_body)
{   // TODO JV: Compensate for voltage.
    float thrust_sign = 1.0f;
    float max_cmd = 0.950f;         // Maximum command for BA2310-1220Kv with ESC Rebel V2 30A (PWM: 1950 microsec)
    float min_cmd = 0.160f;        // Minimum command for BA2310-1220Kv with ESC Rebel V2 30A (PWM: 1160 microsec)
    float thrust_max = 8.55f;       // Max thrust (N). Adjust for the -0.1 N offset in the data.
    float thrust_min = 0.11f;       // Min thrust when spinning (N). Adjust for the -0.1 N offset in the data.
    float cmd_0_to_1 = 0.0f;        // Initialization
    float coeff_b = -1.599f;       // Linear approximation coefficient
    float coeff_a = (thrust_max - thrust_min)/(max_cmd - min_cmd);

    if ((thrust_body < thrust_min ) && (thrust_body > -thrust_min)) {
        return cmd_0_to_1;

    } else if (thrust_body < 0.0f) {
        thrust_sign = -1.0f;

    } else if ((thrust_body <= -thrust_max ) || (thrust_body >= thrust_max)) {
        return (max_cmd * thrust_sign);

    }

    cmd_0_to_1 = (thrust_sign * thrust_body - coeff_b) / coeff_a;         // Linear approximation of thrust to 0_to_1 (Normalization of PWM)

    return (thrust_sign * cmd_0_to_1);
}

// Converts Thrust (N) to a cmd from -1 to +1. For BA2310-1220Kv motor with HQ 6x3.5x3 prop. RFC JV
float AC_AttitudeControl_Multi::thrust_model_ba2310hq6x35x3(float thrust_body)
{   // TODO JV: Compensate for voltage. 
    float thrust_sign = 1.0f;
    float max_cmd = 0.605f;           // Maximum command for BA2310-1220Kv with ESC VGood 60A (This limits the power at ~300 W)
    float min_cmd = 0.06f;            // Minimum command for BA2310-1220Kv with ESC VGood 60A (PWM: 1050 microsec)
    float thrust_max = 10.002f;       // Max thrust (N). 
    float thrust_min = 0.368f;        // Min thrust when spinning (N).
    float cmd_0_to_1 = 0.0f;          // Initialization
    float coeff_b = -0.693;           // Linear approximation coefficient
    float coeff_a = (thrust_max - thrust_min)/(max_cmd - min_cmd);      // Linear approximation coefficient

    if ((thrust_body < thrust_min ) && (thrust_body > -thrust_min)) {
        return cmd_0_to_1;

    } else if (thrust_body < 0.0f) {
        thrust_sign = -1.0f;

    } else if ((thrust_body <= -thrust_max ) || (thrust_body >= thrust_max)) {
        return (max_cmd * thrust_sign);

    }

    cmd_0_to_1 = (thrust_sign * thrust_body - coeff_b) / coeff_a;         // Linear approximation of thrust to 0_to_1 (Normalization of PWM)

    return (thrust_sign * cmd_0_to_1);
}

// Converts Thrust (N) to a cmd from -1 to +1. For Tmotor F1507-2700 Kv & GemFan3028. RFC JV
float AC_AttitudeControl_Multi::thrust_model_f1507gf3028(float thrust_body)
{ // TODO JV: Compensate for voltage. 
    float thrust_sign = 1.0f;
    float max_cmd = 0.565f;          // Upper command limit (3.00 N)
    // float min_cmd = 0.192f;          // Upper command limit (0.1 N)
    float cmd_0_to_1 = 0.0f;         // Initialization
    float thrust_max = 3.00f;        // Upper thrust limit (N). For the longitudinal props, it means that they will saturate at an angular position error of 9.46 ° if the desired side thrust is 18 N.
    float thrust_min = 0.100f;       // Lower thrust limit when spinning (N).
    float coeff_x2 = 12.6227f;       // Quadratic approximation coefficient
    float coeff_x1 = -1.7838f;       // Quadratic approximation coefficient
    float coeff_x0 = -0.02362f;      // Quadratic approximation coefficient

    if ((thrust_body < thrust_min ) && (thrust_body > -thrust_min)) {
        return cmd_0_to_1;

    } else if (thrust_body < 0.0f) {
        thrust_sign = -1.0f;

    } else if ((thrust_body <= -thrust_max ) || (thrust_body >= thrust_max)) {
        return (max_cmd * thrust_sign);

    }

    cmd_0_to_1 = (-coeff_x1 + safe_sqrt(coeff_x1 * coeff_x1 - 4.0f * coeff_x2 * (coeff_x0 - thrust_sign * thrust_body))) / (2.0f * coeff_x2);

    return (thrust_sign * cmd_0_to_1);
} 

// Sets motor commands based on velocity target (latitude/longitude) from pilot. RFC JV , Ayaw JV
void AC_AttitudeControl_Multi::pcs_rf_controller(bool enabled_rfc, float plt_latitude, float plt_longitude, Vector3f dist_vec_tar_ned, bool enabled_auto_yaw, float yaw_pilot)
{   // TODO JV: Adapt the idle parameter to only affect yaw and longitudinal props. Implement a low-pass filter for derivative of error?. Move thrust models function with the motor mixing (Having the thrust model here isn't ideal since it assumes that we already know the motor/prop config).

    // Variable initialization
    Vector3f vel_inertial; //_inav->get_velocity();    // Velocity in inertial frame (NED). Latitude, Longitude, Vertical down  (m/s)
    float cmd_vel_latitude = 0.0f;           // latitude command
    float cmd_vel_longitude = 0.0f;          // longitude command
    float cmd_lateral = 0.0f;                       // Lateral command
    float cmd_forward = 0.0f;                       // Forward command
    const Matrix3f &rot_body_to_NED = _ahrs.get_rotation_body_to_ned();               // Rotation matrix from body frame to NED
    Vector3f cmd_body;                  // Commands from North-East-0 to body frame (lateral/forward/yaw)
    Vector2f home_xy;                   // Position NE from home (m)
    Vector2f cmd_pos;                   // Position error command in NE referential (N)
    float k_x;                          // Inclination compensation factor for X axis (Body referential)
    float k_y;                          // Inclination compensation factor for Y axis (Body referential)
    Vector3f vect_k_x;                  // Initially in XYZ body referential
    Vector3f vect_k_y;                  // Initially in XYZ body referential
    Vector3f vect_x_NED;
    Vector3f vect_y_NED;
    float cmd_idle = 0.0f;              // Idle thrust setter (N)
    // float side_force = 5.5f;            // Norm of the side force of constant norm and changing orientation (N)
    Vector2f cmd_rf;                    // Command: side force of constant norm and changing orientation North-East (N)

    // Variable initialization Autoyaw. NOTE: Autoyaw calculations are done in the NED reference frame. Not sure if the yaw controller will work if the PCS is upside-down.
    float angular_speed_z = _ahrs.get_gyro_latest().z;            // Gyros smoothed angular velocity in yaw (rad/s)
    float cmd_der_yaw = 0.0f;                                     // Derivative of angular position error term of yaw command
    float cmd_pos_yaw = 0.0f;                                     // Angular position error term of yaw command
    float cmd_plt_yaw = 0.0f;                                     // Pilot command for yaw 
    float cmd_yaw = 0.0f;                                         // yaw command
    Vector2f u_d_to_p;                                            // Position unit vector from aircraft (drone) to PCS 
    float ang_N_u_d_to_p = 0.0f;                                  // Angle in between the North unit vector and u_drone_to_pcs unit vector (rad)
    float yaw_offset = radians(_ayaw_off);                        // Yaw offset in orientation following (rad) 
    float yaw_dir = 0.0f;                                         // Yaw rotation desired direction in regard to the XYZ body reference frame
    float yaw_thrust = 0.0f;                                      // Resultant desired yaw thrust in the XYZ body reference frame (antagonist configuration). (N) 
    Vector3f heading_body;                                        // Yaw heading vector considering the yaw offset parameter in the body reference frame
    heading_body.x = cosf(yaw_offset); heading_body.y = sinf(yaw_offset); heading_body.z = 0.0f;
    heading_body = rot_body_to_NED * heading_body;
    Vector3f heading_ned = Vector3f(heading_body.x, heading_body.y, 0.0f);   // Yaw heading unit vector considering the yaw offset parameter in the NED reference frame (Down component is null)
    heading_ned.x = heading_ned.x / safe_sqrt(heading_ned.x * heading_ned.x + heading_ned.y * heading_ned.y + heading_ned.z * heading_ned.z);
    heading_ned.y = heading_ned.y / safe_sqrt(heading_ned.x * heading_ned.x + heading_ned.y * heading_ned.y + heading_ned.z * heading_ned.z);
    const float dt = AP::scheduler().get_loop_period_s();         // Period of the loop (s)
    static bool reset_yaw_ctrl = true;                                          // Boolean to reset derivative related terms of the yaw controller    

    if (enabled_auto_yaw && ((angular_speed_z * angular_speed_z) <= 625.0f )) {         // Second condition is a safety if angular speed in yaw exceeds 25 rad/s
        cmd_plt_yaw = _ayaw_plt * yaw_pilot;     // Proportional gain on pilot command
    } else {
        cmd_plt_yaw = 0.0f;
    }

    if (enabled_rfc) {
        if (_ahrs.get_velocity_NED(vel_inertial)) {
            cmd_vel_latitude = _rfc_vel_kp * (plt_latitude * _rfc_vel_plt - vel_inertial.x);     // Proportional latitude vel controller. Target comes from pilot.
            cmd_vel_longitude = _rfc_vel_kp * (plt_longitude * _rfc_vel_plt - vel_inertial.y);     // Proportional longitude vel controller. Target comes from pilot.

        } else {
            cmd_vel_latitude = 0.0f;
            cmd_vel_longitude = 0.0f;
        }       

        if (_ahrs.get_relative_position_NE_home(home_xy)) {
            cmd_pos.x = -_rfc_pos_kp * home_xy.x;           // Position target is the home position (0,0)
            cmd_pos.y = -_rfc_pos_kp * home_xy.y;           // Position target is the home position (0,0)

            if (((dist_vec_tar_ned.x > 0.01f) || (dist_vec_tar_ned.x < -0.01f)) && ((dist_vec_tar_ned.y > 0.01f) || (dist_vec_tar_ned.y < -0.01f))) {
                // We are only interested by the horizontal orientation of drone to PCS vector.
                u_d_to_p.x = (home_xy.x - dist_vec_tar_ned.x) / safe_sqrt( (home_xy.x - dist_vec_tar_ned.x) * (home_xy.x - dist_vec_tar_ned.x) + (home_xy.y - dist_vec_tar_ned.y) * (home_xy.y - dist_vec_tar_ned.y) );
                u_d_to_p.y = (home_xy.y - dist_vec_tar_ned.y) / safe_sqrt( (home_xy.x - dist_vec_tar_ned.x) * (home_xy.x - dist_vec_tar_ned.x) + (home_xy.y - dist_vec_tar_ned.y) * (home_xy.y - dist_vec_tar_ned.y) );

                cmd_rf.x = _side_force * u_d_to_p.x;
                cmd_rf.y = _side_force * u_d_to_p.y;
                
                if (enabled_auto_yaw) {
                    ang_N_u_d_to_p = acosf(heading_ned.x * -1.0f * u_d_to_p.x + heading_ned.y * -1.0f * u_d_to_p.y);      // We inverse u_d_to_p vector and then find the angle via dot product formula
                    yaw_dir = u_d_to_p.y * heading_ned.x - u_d_to_p.x * heading_ned.y;         // We inverse u_d_to_p vector and then find the rotation direction via cross product simplified formula
                    
                    if (yaw_dir > 0.0f) {       // We need to inverse the desired rotation direction so that the angular orientation error (ang_N_u_d_to_p) becomes null as the vectors are aligned. 
                        yaw_dir = -1.0f;
                    } else if (yaw_dir < 0.0f) {
                        yaw_dir = 1.0f;
                    }
                    cmd_pos_yaw = _ayaw_kp * ang_N_u_d_to_p * yaw_dir;     // Proportional yaw_rate controller on angular position relative to aircraft

                    if (reset_yaw_ctrl) {
                        reset_yaw_ctrl = false;
                        _error = ang_N_u_d_to_p * yaw_dir;
                    } else {
                        float error_last = _error;                             // Angular position error during previous loop (rad)
                        _error = ang_N_u_d_to_p * yaw_dir;

                        if (dt > 0.0f) {
                            float derivative = (_error - error_last) / dt;     // Derivative of the angular position error (rad/s)
                            cmd_der_yaw = _ayaw_kd * derivative;
                        }
                    }

                } else {
                    cmd_pos_yaw = 0.0f;
                    cmd_der_yaw = 0.0f;
                }
                /* static uint8_t counter = 0;         // Debug JV
                counter++;
                if (counter > 200) {
                    counter = 0;
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "RF_NORTH= %5.3f", (float)cmd_rf.x);
                } */
            } else {
                cmd_rf.x = 0.0f;
                cmd_rf.y = 0.0f;
            }

        } else {
            cmd_pos.x = 0.0f;
            cmd_pos.y = 0.0f;
            cmd_rf.x = 0.0f;
            cmd_rf.y = 0.0f;
        }

        // conversion from NED to body frame (lateral/forward/yaw axes)
        cmd_body.x = cmd_vel_latitude + cmd_pos.x + cmd_rf.x;              // (North). Add cmd_rf.x
        cmd_body.y = cmd_vel_longitude + cmd_pos.y + cmd_rf.y;             // (East). Add cmd_rf.y
        cmd_body.z = 0.0f;                                      // (Down)
        cmd_body = rot_body_to_NED.mul_transpose(cmd_body);     // NED -> XYZ (forward/right/down) (Newtons)

        // Compensation for the inclination of the PCS. Note that these compensations don't account for the fact that the ideal side thrust (to be centered) reduces/augmentes if the PCS is inclined (vertical component of thrust). 
        vect_k_x = Vector3f(1.0f, 0.0f, 0.0f);
        vect_k_x = rot_body_to_NED * vect_k_x;            // Rotation from XYZ to NED referential (Vp vector)
        vect_x_NED =  Vector3f(vect_k_x.x, vect_k_x.y, 0.0f);         // 2D Vector with the same orientation of Vp (Vi vector)
        k_x = vect_k_x * vect_x_NED / safe_sqrt(vect_x_NED.x * vect_x_NED.x + vect_x_NED.y * vect_x_NED.y);
        if (k_x <= 0.0f || k_x > 2.0f) {       // Safety limits. 2.0f equals 60 ° of inclination
            k_x = 1.0f;
        }
        cmd_body.x *= 1.0f / k_x;

        vect_k_y = Vector3f(0.0f, 1.0f, 0.0f);
        vect_k_y = rot_body_to_NED * vect_k_y;            // Rotation from XYZ to NED referential (Vp vector)
        vect_y_NED =  Vector3f(vect_k_y.x, vect_k_y.y, 0.0f);         // 2D Vector with the same orientation of Vp (Vi vector)
        k_y = vect_k_y * vect_y_NED / safe_sqrt(vect_y_NED.x * vect_y_NED.x + vect_y_NED.y * vect_y_NED.y);
        if (k_y <= 0.0f || k_y > 2.0f) {       // Safety limits. 2.0f equals 60 ° of inclination
            k_y = 1.0f;
        }
        cmd_body.y *= 1.0f / k_y;

        if (_idle_on) {                 // Idle thrust setting (N). Idle JV
            cmd_idle = thrust_model_f1507gf3028(_idle_thrust);

            if (cmd_body.x > 0.0f) {                                                       // This compensates for the idling thrust.
                cmd_forward = thrust_model_f1507gf3028(cmd_body.x + _idle_thrust);          // Longitudinal props  
            } else if (cmd_body.x < 0.0f) {
                cmd_forward = thrust_model_f1507gf3028(cmd_body.x - _idle_thrust);          // Longitudinal props  
            }
        } else {
            cmd_forward = thrust_model_f1507gf3028(cmd_body.x);
        }
        

        // conversion from thrust (N) in body frame to a -1 to +1 range
        cmd_lateral = thrust_model_ba2310hq6x35x3(cmd_body.y / 2.0f);       // We divide by 2 because we have 2 main props acting in the same direction. Motor mixing will naturally remultiply by 2.
          

    } else {
        cmd_lateral = 0.0f;
        cmd_forward = 0.0f;
    }

    // A positive value should cause positive angular velocity in yaw. Viewed from the top, the PCS should turn clockwise.
    if (enabled_auto_yaw && ((angular_speed_z * angular_speed_z) <= 625.0f )) {         // Second condition is a safety if angular speed in yaw exceeds 25 rad/s
        yaw_thrust = cmd_der_yaw + cmd_pos_yaw + cmd_plt_yaw;                           // PD controller on angular position relative to aircraft + pilot command (N)
        
        if (_idle_on) {
            if (yaw_thrust > 0.0f) {                                                       // This compensates for the idling thrust.
                cmd_yaw = thrust_model_f1507gf3028(yaw_thrust + _idle_thrust);
            } else if (yaw_thrust < 0.0f) {
                cmd_yaw = thrust_model_f1507gf3028(yaw_thrust - _idle_thrust);
            }
        } else {
            cmd_yaw = thrust_model_f1507gf3028(yaw_thrust);
        }  
        
    } else {
        cmd_yaw = 0.0f;
    }

    // Logging JV
    _pcscmd.pcs_tar_lat = cmd_lateral;
    _pcscmd.pcs_tar_fwd = cmd_forward;                          // Note that this recorded value is: 01range(desired thrust + idle thrust)
    _pcscmd.pcs_tar_yaw = cmd_yaw;                              // Note that this recorded value is: 01range(desired thrust + idle thrust)
    _pcscmd.pcs_pos_lat = cmd_pos.x;
    _pcscmd.pcs_pos_lon = cmd_pos.y;
    _pcscmd.pcs_vel_lat = cmd_vel_latitude;
    _pcscmd.pcs_vel_lon = cmd_vel_longitude;
    _pcscmd.pcs_rf_lat = cmd_rf.x;
    _pcscmd.pcs_rf_lon = cmd_rf.y;

    _pcsoth.pcs_tar_yaw = cmd_yaw;                              // Target yaw (-1 to  +1)
    _pcsoth.pcs_ayaw_plt = cmd_plt_yaw;                         // Pilot command
    _pcsoth.pcs_ayaw_pos = cmd_pos_yaw;                         // Angular position error
    _pcsoth.pcs_ayaw_der = cmd_der_yaw;                         // Derivative of angular position error

    _motors.set_lateral(cmd_lateral);                           // Set lateral. To be used in output()
    _motors.set_forward(cmd_forward);                           // Set forward
    _motors.set_yaw(cmd_yaw);                                   // Set yaw command
    _motors.set_idle(enabled_rfc, _idle_on, cmd_idle);          // Set idle command. Idle JV

    /* static uint8_t counter = 0;         // Use to debug JV
    counter++;
    if (counter > 50) {
        counter = 0;
        gcs().send_text(MAV_SEVERITY_CRITICAL, "yaw= %5.3f", (float)_motors.get_yaw());
    } */
} 

// sanity check parameters.  should be called once before takeoff
void AC_AttitudeControl_Multi::parameter_sanity_check()
{
    // sanity check throttle mix parameters
    if (_thr_mix_man < 0.1f || _thr_mix_man > AC_ATTITUDE_CONTROL_MAN_LIMIT) {
        // parameter description recommends thr-mix-man be no higher than 0.9 but we allow up to 4.0
        // which can be useful for very high powered copters with very low hover throttle
        _thr_mix_man.set_and_save(constrain_float(_thr_mix_man, 0.1, AC_ATTITUDE_CONTROL_MAN_LIMIT));
    }
    if (_thr_mix_min < 0.1f || _thr_mix_min > AC_ATTITUDE_CONTROL_MIN_LIMIT) {
        _thr_mix_min.set_and_save(constrain_float(_thr_mix_min, 0.1, AC_ATTITUDE_CONTROL_MIN_LIMIT));
    }
    if (_thr_mix_max < 0.5f || _thr_mix_max > AC_ATTITUDE_CONTROL_MAX) {
        // parameter description recommends thr-mix-max be no higher than 0.9 but we allow up to 5.0
        // which can be useful for very high powered copters with very low hover throttle
        _thr_mix_max.set_and_save(constrain_float(_thr_mix_max, 0.5, AC_ATTITUDE_CONTROL_MAX));
    }
    if (_thr_mix_min > _thr_mix_max) {
        _thr_mix_min.set_and_save(AC_ATTITUDE_CONTROL_MIN_DEFAULT);
        _thr_mix_max.set_and_save(AC_ATTITUDE_CONTROL_MAX_DEFAULT);
    }
}
