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

#ifndef __OBSTACLE_DETECT_H__
#define __OBSTACLE_DETECT_H__

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>

// Maximum number of range finder instances available on this platform
#define DETECTOR_MAX_INSTANCES 1
#define DETECTOR_GROUND_CLEARANCE_CM_DEFAULT 10
#define DETECTOR_PREARM_ALT_MAX_CM           200
#define DETECTOR_PREARM_REQUIRED_CHANGE_CM   50

class AP_RangeFinder_Backend; 
 
class Detector
{
public:
    friend class AP_RangeFinder_Backend;
    
    Detector(AP_SerialManager &_serial_manager);
    
    // Detector driver types
    enum Detector_Type {
        Detector_TYPE_NONE   = 0,
        RangeFinder_TYPE_GENI2C = 1
    };

    enum Detector_Function {
        FUNCTION_LINEAR    = 0,
        FUNCTION_INVERTED  = 1,
        FUNCTION_HYPERBOLA = 2
    };

    enum Detector_Status {
        Detector_NotConnected = 0,
        Detector_NoData,
        Detector_OutOfRangeLow,
        Detector_OutOfRangeHigh,
        Detector_Good
    };

    // The Detector_State structure is filled in by the backend driver
    struct Detector_State {
        uint8_t                instance;    // the instance number of this Detector
        uint16_t               distance_cm; // distance: in cm
        uint16_t               voltage_mv;  // voltage in millivolts,
                                            // if applicable, otherwise 0
        enum Detector_Status status;     // sensor status
        uint8_t                range_valid_count;   // number of consecutive valid readings (maxes out at 10)
        bool                   pre_arm_check;   // true if sensor has passed pre-arm checks
        uint16_t               pre_arm_distance_min;    // min distance captured during pre-arm checks
        uint16_t               pre_arm_distance_max;    // max distance captured during pre-arm checks
    };

    // parameters for each instance
    AP_Int8  _type[DETECTOR_MAX_INSTANCES];
    AP_Int16 _settle_time_ms[DETECTOR_MAX_INSTANCES];
    AP_Float _scaling[DETECTOR_MAX_INSTANCES];
    AP_Float _offset[DETECTOR_MAX_INSTANCES];
    AP_Int16 _min_distance_cm[DETECTOR_MAX_INSTANCES];
    AP_Int16 _max_distance_cm[DETECTOR_MAX_INSTANCES];
    AP_Int8  _address[DETECTOR_MAX_INSTANCES];

    static const struct AP_Param::GroupInfo var_info[];
    
    // Return the number of range finder instances
    uint8_t num_sensors(void) const {
        return num_instances;
    }

    // detect and initialise any available Detectors
    void init(void);

    // update state of all Detectors. Should be called at around
    // 10Hz from main loop
    void update(void);
    
#define _Detector_STATE(instance) state[instance]

    uint16_t distance_cm(uint8_t instance) const {
        return (instance<num_instances? _Detector_STATE(instance).distance_cm : 0);
    }
    uint16_t distance_cm() const {
        return distance_cm(primary_instance);
    }

    int16_t max_distance_cm(uint8_t instance) const {
        return _max_distance_cm[instance];
    }
    int16_t max_distance_cm() const {
        return max_distance_cm(primary_instance);
    }

    int16_t min_distance_cm(uint8_t instance) const {
        return _min_distance_cm[instance];
    }
    int16_t min_distance_cm() const {
        return min_distance_cm(primary_instance);
    }
    
    // query status
    Detector_Status status(uint8_t instance) const;
    Detector_Status status(void) const {
        return status(primary_instance);
    }

    // true if sensor is returning data
    bool has_data(uint8_t instance) const;
    bool has_data() const {
        return has_data(primary_instance);
    }

    // returns count of consecutive good readings
    uint8_t range_valid_count() const {
        return range_valid_count(primary_instance);
    }
    uint8_t range_valid_count(uint8_t instance) const {
        return _Detector_STATE(instance).range_valid_count;
    }

private:
    Detector_State state[DETECTOR_MAX_INSTANCES];
    AP_RangeFinder_Backend *drivers[DETECTOR_MAX_INSTANCES];
    uint8_t primary_instance:1;
    uint8_t num_instances:1;
    AP_SerialManager &serial_manager;
    void detect_instance(uint8_t instance);
    void update_instance(uint8_t instance);  

};
#endif // __OBSTACLE_DETECT_H__
