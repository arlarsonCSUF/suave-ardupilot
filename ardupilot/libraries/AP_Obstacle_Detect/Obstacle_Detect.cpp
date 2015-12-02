// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

#include "Obstacle_Detect.h"
#include "AP_RangeFinder_GenericI2C.h"

// table of user settable parameters
const AP_Param::GroupInfo Detector::var_info[] PROGMEM = {
    // @Param: _TYPE
    // @DisplayName: Detector type
    // @Description: What type of Detector device that is connected
    // @Values: 0:None,1:Analog,2:APM2-MaxbotixI2C,3:APM2-PulsedLightI2C,4:PX4-I2C,5:PX4-PWM,6:BBB-PRU,7:LightWareI2C,8:LightWareSerial
    // @User: Standard
    AP_GROUPINFO("_TYPE",    0, Detector, _type[0], 0),

    // @Param: _PIN
    // @DisplayName: Detector pin
    // @Description: Analog pin that Detector is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
    // @Values: -1:Not Used, 0:APM2-A0, 1:APM2-A1, 2:APM2-A2, 3:APM2-A3, 4:APM2-A4, 5:APM2-A5, 6:APM2-A6, 7:APM2-A7, 8:APM2-A8, 9:APM2-A9, 11:PX4-airspeed port, 15:Pixhawk-airspeed port, 64:APM1-airspeed port
    // @User: Standard
    
    AP_GROUPINFO("_SCALING", 1, Detector, _scaling[0], 3.0f),

    // @Param: _OFFSET
    // @DisplayName: Detector offset
    // @Description: Offset in volts for zero distance for analog Detectors. Offset added to distance in centimeters for PWM and I2C Lidars
    // @Units: Volts
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("_OFFSET",  2, Detector, _offset[0], 0.0f),

    // @Param: _MIN_CM
    // @DisplayName: Detector minimum distance
    // @Description: Minimum distance in centimeters that Detector can reliably read
	// @Units: centimeters
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_MIN_CM",  3, Detector, _min_distance_cm[0], 20),

    // @Param: _MAX_CM
    // @DisplayName: Detector maximum distance
    // @Description: Maximum distance in centimeters that Detector can reliably read
	// @Units: centimeters
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_MAX_CM",  4, Detector, _max_distance_cm[0], 700),

    // @Param: _SETTLE
    // @DisplayName: Detector settle time
    // @Description: The time in milliseconds that the Detector reading takes to settle. This is only used when a STOP_PIN is specified. It determines how long we have to wait for the Detector to give a reading after we set the STOP_PIN high. For a sonar Detector with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again.
    // @Units: milliseconds
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_SETTLE", 5, Detector, _settle_time_ms[0], 0),

    AP_GROUPEND
};

Detector::Detector(AP_SerialManager &_serial_manager) :
    primary_instance(0),
    num_instances(0),
    serial_manager(_serial_manager)
{
    AP_Param::setup_object_defaults(this, var_info);

    // init state and drivers
    memset(state,0,sizeof(state));
    memset(drivers,0,sizeof(drivers));
}

/*
  initialise the Detector class. We do detection of attached range
  finders here. For now we won't allow for hot-plugging of
  Detectors.
 */
 
void Detector::init(void)
{
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }
    for (uint8_t i=0; i<DETECTOR_MAX_INSTANCES; i++) {
        detect_instance(i);
        if (drivers[i] != NULL) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
            num_instances = i+1;
        }
        
        // initialise status
        state[i].status = Detector_NotConnected;
        state[i].range_valid_count = 0;
    }
}

/*
  update Detector state for all instances. This should be called at
  around 10Hz by main loop
 */
void Detector::update(void)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != NULL) {
            if (_type[i] == Detector_TYPE_NONE) {
                // allow user to disable a Detector at runtime
                state[i].status = Detector_NotConnected;
                state[i].range_valid_count = 0;
                continue;
            }
            drivers[i]->update();
            //update_pre_arm_check(i);
        }
    }

    // work out primary instance - first sensor returning good data
    for (int8_t i=num_instances-1; i>=0; i--) {
        if (drivers[i] != NULL && (state[i].status == Detector_Good)) {
            primary_instance = i;
        }
    }
}
    
/*
  detect if an instance of a Detector is connected. 
 */
void Detector::detect_instance(uint8_t instance)
{
    uint8_t type = _type[instance];

    if (type == RangeFinder_TYPE_GENI2C){
        if (AP_RangeFinder_GenericI2C::detect(*this, instance)) {
            state[instance].instance = instance;
            drivers[instance] = new AP_RangeFinder_GenericI2C(*this, instance, state[instance]);
            return;
        }
    }
}   
// query status
Dectector::Detector_Status Dectector::status(uint8_t instance) const
{
    // sanity check instance
    if (instance >= DETECTOR_MAX_INSTANCES) {
        return Detector_NotConnected;
    }

    if (drivers[instance] == NULL || _type[instance] == Detector_TYPE_NONE) {
        return Detector_NotConnected;
    }

    return state[instance].status;
}

// true if sensor is returning data
bool Detector::has_data(uint8_t instance) const
{
    // sanity check instance
    if (instance >= DETECTOR_MAX_INSTANCES) {
        return Detector_NotConnected;
    }
    return ((state[instance].status != Detector_NotConnected) && (state[instance].status != Detector_NoData));
}