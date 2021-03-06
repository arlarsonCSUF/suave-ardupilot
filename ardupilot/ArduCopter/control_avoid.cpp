/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 * control_avoid.pde - init and run calls for avoid flight mode
 */

// avoid_init - initialise avoid controller
bool Copter::avoid_init(bool ignore_checks)
{
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors.armed() && ap.land_complete && !mode_has_manual_throttle(control_mode) && (g.rc_3.control_in > get_non_takeoff_throttle())) {
        return false;
    }
    // set target altitude to zero for reporting
    pos_control.set_alt_target(0);

    return true;
}

// avoid_run - runs the main avoid controller
// should be called at 100hz or more
void Copter::avoid_run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;
    int16_t pilot_throttle_scaled;

    // if not armed or throttle at zero, set throttle to zero and exit immediately
    if(!motors.armed() || ap.throttle_zero) {
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        // slow start if landed
        if (ap.land_complete) {
            motors.slow_start(true);
        }
        return;
    }

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode(); 

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    
    float mixedInputRoll = channel_roll->control_in * magnitudeUserInput + avoidanceRollForce * (1-magnitudeUserInput);
    float mixedInputPitch = channel_pitch->control_in * magnitudeUserInput + avoidancePitchForce * (1-magnitudeUserInput);
    get_pilot_desired_lean_angles(mixedInputRoll, mixedInputPitch, target_roll, target_pitch, aparm.angle_max);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->control_in);

    // call attitude controller
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(target_roll, target_pitch, target_yaw_rate,get_smoothing_gain());

    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
    attitude_control.set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}
