/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 * control_avoid.pde - init and run calls for avoid flight mode
 *                     the avoid flight mode is a custom flight mode for the SUAVE project 
                       the flight mode is a modified version of the stabilize flight mode
                       
                       the goal of the flight mode is to behave just like stabilize except
                       when an object is detected ignore user input and have the autopilot
                       take over to avoid a collision
 */

// avoid_init - initialise avoid controller
bool Copter::avoid_init(bool ignore_checks)
{
    // set target altitude to zero for reporting
    pos_control.set_alt_target(0);

    // avoid should never be made to fail
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
    
    #if CONFIG_SONAR == ENABLED 
        //if detected obstacle is closer than avoidanceThreshold ignore user input and try to avoid
        if(avoidanceSensorReading > avoidanceThreshold)
            get_pilot_desired_lean_angles(channel_roll->control_in, channel_pitch->control_in, target_roll, target_pitch, aparm.angle_max);
        else
            get_pilot_desired_lean_angles(channel_roll->control_in,avoidanceMagnitude, target_roll, target_pitch, aparm.angle_max);
            //avoidance avoidanceMagnitude is determined in UserCode.cpp
    #else
        get_pilot_desired_lean_angles(channel_roll->control_in,avoidanceMagnitude, target_roll, target_pitch, aparm.angle_max);
    #endif

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->control_in);

    // call attitude controller
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
    attitude_control.set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}