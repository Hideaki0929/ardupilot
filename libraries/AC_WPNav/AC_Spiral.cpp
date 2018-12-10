#include <AP_HAL/AP_HAL.h>
#include "AC_Spiral.h"
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

double angle_gain;

const AP_Param::GroupInfo AC_Spiral::var_info[] = {
    // @Param: RADIUS
    // @DisplayName: Spiral Radius
    // @Description: Defines the radius of the spiral the vehicle will fly when in Spiral flight mode
    // @Units: cm
    // @Range: 0 10000
    // @Increment: 100
    // @User: Standard
    AP_GROUPINFO("RADIUS",  0,  AC_Spiral, _radius, AC_SPIRAL_RADIUS_DEFAULT),

    // @Param: RATE
    // @DisplayName: Spiral rate
    // @Description: Spiral mode's turn rate in deg/sec.  Positive to turn clockwise, negative for counter clockwise
    // @Units: deg/s
    // @Range: -90 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RATE",    1, AC_Spiral, _rate,    AC_SPIRAL_RATE_DEFAULT),

    // @Param: SDIRECT
    // @DisplayName: Spiral DIRECT
    // @Description: TEST
    // @Units: TEST
    // @Range: 1 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SDIRECT",  2, AC_Spiral, _sdirection,  AC_SPIRAL_SDIRECTION_DEFAULT),

    // @Param: SPITCH
    // @DisplayName: Spiral PITCH
    // @Description: TEST
    // @Units: TEST
    // @Range: 1 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SPITCH",  3, AC_Spiral, _spitch,  AC_SPIRAL_SPITCH_DEFAULT),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_Spiral::AC_Spiral(const AP_InertialNav& inav, const AP_AHRS_View& ahrs, AC_PosControl& pos_control) :
    _inav(inav),
    _ahrs(ahrs),
    _pos_control(pos_control),
    _yaw(0.0f),
    _angle(0.0f),
    _angle_total(0.0f),
    _angular_vel(0.0f),
    _angular_vel_max(0.0f),
    _angular_accel(0.0f)
{
    AP_Param::setup_object_defaults(this, var_info);

    // init flags
    _flags.panorama = false;
}

/// init - initialise spiral controller setting center specifically
///     caller should set the position controller's x,y and z speeds and accelerations before calling this
void AC_Spiral::init(const Vector3f& center)
{
    _center = center;

    // initialise position controller (sets target roll angle, pitch angle and I terms based on vehicle current lean angles)
    _pos_control.set_desired_accel_xy(0.0f,0.0f);
    _pos_control.set_desired_velocity_xy(0.0f,0.0f);
    _pos_control.init_xy_controller();

    // set initial position target to reasonable stopping point
    _pos_control.set_target_to_stopping_point_xy();
    _pos_control.set_target_to_stopping_point_z();

    // calculate velocities
    calc_velocities(true);

    // set start angle from position
    init_start_angle(false);
}

/// init - initialise spiral controller setting center using stopping point and projecting out based on the copter's heading
///     caller should set the position controller's x,y and z speeds and accelerations before calling this
void AC_Spiral::init()
{
    // initialise position controller (sets target roll angle, pitch angle and I terms based on vehicle current lean angles)
    _pos_control.set_desired_accel_xy(0.0f,0.0f);
    _pos_control.set_desired_velocity_xy(0.0f,0.0f);
    _pos_control.init_xy_controller();

    // set initial position target to reasonable stopping point
    _pos_control.set_target_to_stopping_point_xy();
    _pos_control.set_target_to_stopping_point_z();

    // get stopping point
    const Vector3f& stopping_point = _pos_control.get_pos_target();

    // set circle center to circle_radius ahead of stopping point
    _center.x = stopping_point.x + _radius * _ahrs.cos_yaw();
    _center.y = stopping_point.y + _radius * _ahrs.sin_yaw();
    _center.z = stopping_point.z;

    // calculate velocities
    calc_velocities(true);

    // set starting angle from vehicle heading
    init_start_angle(true);

    gcs().send_text(MAV_SEVERITY_INFO,"POS_CONTROL,%lf ,%lf",stopping_point.x, stopping_point.y );
    gcs().send_text(MAV_SEVERITY_INFO,"INIT CENTER,%lf ,%lf",_center.x, _center.y);
    gcs().send_text(MAV_SEVERITY_INFO,"INIT ANGLE,%lf",_angle);

    //current_radius = _radius;
    //gcs().send_text(MAV_SEVERITY_INFO,"INIT RADIUS,%lf",current_radius);
}

/// set_circle_rate - set circle rate in degrees per second
void AC_Spiral::set_rate(float deg_per_sec)
{
    if (!is_equal(deg_per_sec, _rate.get())) {
        _rate = deg_per_sec;
        calc_velocities(false);
    }
}

/// update - update circle controller
void AC_Spiral::update()
{
    // calculate dt
    float dt = _pos_control.time_since_last_xy_update();
    if (dt >= 0.2f) {
        dt = 0.0f;
    }

    // ramp angular velocity to maximum
    if (_angular_vel < _angular_vel_max) {
        _angular_vel += fabsf(_angular_accel) * dt;
        _angular_vel = MIN(_angular_vel, _angular_vel_max);
    }
    if (_angular_vel > _angular_vel_max) {
        _angular_vel -= fabsf(_angular_accel) * dt;
        _angular_vel = MAX(_angular_vel, _angular_vel_max);
    }

    // update the target angle and total angle traveled
    float angle_change = _angular_vel * dt;
    //_angle += angle_change;
    //_angle = wrap_PI(_angle);
    //_angle_total += angle_change;
    _angle -= angle_change;

    // if the circle_radius is zero we are doing panorama so no need to update loiter target
    if (_angle >= 0) {
        // calculate target position
        Vector3f target;
        target.x = _center.x + angle_gain * _angle * cosf(-_angle);
        target.y = _center.y - _sdirection * angle_gain * _angle * sinf(-_angle);
        target.z = _pos_control.get_alt_target();

        // update position controller target
        _pos_control.set_xy_target(target.x, target.y);

        // heading is 180 deg from vehicles target position around circle
        _yaw = wrap_PI(_angle-M_PI) * DEGX100;
  
        //gcs().send_text(MAV_SEVERITY_INFO,"%lf",current_radius);
        //gcs().send_text(MAV_SEVERITY_INFO,"S1,%lf", _angle);
        gcs().send_text(MAV_SEVERITY_INFO,"S2,%lf ,%lf ,%lf",target.x, target.y,_angle);                                        
        //gcs().send_text(MAV_SEVERITY_INFO,"S3,%lf ,%lf",_center.x, _center.y);                                        
    } 
    
    // update position controller
    _pos_control.update_xy_controller(1.0f);
}

// get_closest_point_on_circle - returns closest point on the circle
//  circle's center should already have been set
//  closest point on the circle will be placed in result
//  result's altitude (i.e. z) will be set to the circle_center's altitude
//  if vehicle is at the center of the circle, the edge directly behind vehicle will be returned
void AC_Spiral::get_closest_point_on_circle(Vector3f &result)
{
    // return center if radius is zero
    if (_radius <= 0) {
        result = _center;
        return;
    }

    // get current position
    const Vector3f &curr_pos = _inav.get_position();

    // calc vector from current location to circle center
    Vector2f vec;   // vector from circle center to current location
    vec.x = (curr_pos.x - _center.x);
    vec.y = (curr_pos.y - _center.y);
    float dist = norm(vec.x, vec.y);

    // if current location is exactly at the center of the circle return edge directly behind vehicle
    if (is_zero(dist)) {
        result.x = _center.x - _radius * _ahrs.cos_yaw();
        result.y = _center.y - _radius * _ahrs.sin_yaw();
        result.z = _center.z;
        return;
    }

    // calculate closest point on edge of circle
    result.x = _center.x + vec.x / dist * _radius;
    result.y = _center.y + vec.y / dist * _radius;
    result.z = _center.z;
}

// calc_velocities - calculate angular velocity max and acceleration based on radius and rate
//      this should be called whenever the radius or rate are changed
//      initialises the yaw and current position around the circle
void AC_Spiral::calc_velocities(bool init_velocity)
{
    // if we are doing a panorama set the circle_angle to the current heading
    if (_radius <= 0) {
        _angular_vel_max = ToRad(_rate);
        _angular_accel = MAX(fabsf(_angular_vel_max),ToRad(AC_SPIRAL_ANGULAR_ACCEL_MIN));  // reach maximum yaw velocity in 1 second
    }else{
        // calculate max velocity based on waypoint speed ensuring we do not use more than half our max acceleration for accelerating towards the center of the circle
        float velocity_max = MIN(_pos_control.get_speed_xy(), safe_sqrt(0.5f*_pos_control.get_accel_xy()*_radius));

        // angular_velocity in radians per second
        _angular_vel_max = velocity_max/_radius;
        _angular_vel_max = constrain_float(ToRad(_rate),-_angular_vel_max,_angular_vel_max);

        // angular_velocity in radians per second
        _angular_accel = MAX(_pos_control.get_accel_xy()/_radius, ToRad(AC_SPIRAL_ANGULAR_ACCEL_MIN));
    }

    // initialise angular velocity
    if (init_velocity) {
        _angular_vel = 0;
    }
}

// init_start_angle - sets the starting angle around the circle and initialises the angle_total
//  if use_heading is true the vehicle's heading will be used to init the angle causing minimum yaw movement
//  if use_heading is false the vehicle's position from the center will be used to initialise the angle
void AC_Spiral::init_start_angle(bool use_heading)
{
    // initialise angle total
    _angle_total = 0;

    // if the radius is zero we are doing panorama so init angle to the current heading
    if (_radius <= 0) {
        _angle = _ahrs.yaw;
        return;
    }

    // if use_heading is true
    if (use_heading) {
        //change program 1208
        _angle = _spitch * 2 * M_PI + wrap_2PI(_ahrs.yaw-M_PI);
        angle_gain = _radius / _angle;
    } else {
        // if we are exactly at the center of the circle, init angle to directly behind vehicle (so vehicle will backup but not change heading)
        const Vector3f &curr_pos = _inav.get_position();
        if (is_equal(curr_pos.x,_center.x) && is_equal(curr_pos.y,_center.y)) {
            _angle = wrap_PI(_ahrs.yaw-M_PI);
        } else {
            // get bearing from circle center to vehicle in radians
            float bearing_rad = atan2f(curr_pos.y-_center.y,curr_pos.x-_center.x);
            _angle = wrap_PI(bearing_rad);
        }
    }
}
