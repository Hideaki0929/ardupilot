#include "Copter.h"

#if MODE_SPIRALTD_ENABLED == ENABLED

/*
 * Init and run calls for spiral flight mode
 */

// spiral_init - initialise spiral controller flight mode
bool Copter::ModeSpiralTD::init(bool ignore_checks)
{
    if (copter.position_ok() || ignore_checks) {
        pilot_yaw_override = false;

        // initialize speeds and accelerations
        pos_control->set_speed_xy(wp_nav->get_speed_xy());
        pos_control->set_accel_xy(wp_nav->get_wp_acceleration());
        pos_control->set_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
        pos_control->set_accel_z(g.pilot_accel_z);

        // initialise spiral controller including setting the spiral center based on vehicle speed
        copter.spiral_nav->init();
        
        return true;
    }else{
        return false;
    }
}

// spiral_run - runs the spiral flight mode
// should be called at 100hz or more
void Copter::ModeSpiralTD::run()
{
    float target_yaw_rate = 0;
    float target_climb_rate = 0;

    // initialize speeds and accelerations
    pos_control->set_speed_xy(wp_nav->get_speed_xy());
    pos_control->set_accel_xy(wp_nav->get_wp_acceleration());
    pos_control->set_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_accel_z(g.pilot_accel_z);
    
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || ap.land_complete || !motors->get_interlock()) {
        // To-Do: add some initialisation of position controllers
        zero_throttle_and_relax_ac();
        pos_control->set_alt_target_to_current_alt();
        return;
    }

    // process pilot inputs
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            pilot_yaw_override = true;
        }

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());

        // check for pilot requested take-off
        if (ap.land_complete && target_climb_rate > 0) {
            // indicate we are taking off
            set_land_complete(false);
            // clear i term when we're taking off
            set_throttle_takeoff();
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run spiral controller
    copter.spiral_nav->update();

    // call attitude controller
    if (pilot_yaw_override) {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(copter.spiral_nav->get_roll(),
                                                                      copter.spiral_nav->get_pitch(),
                                                                      target_yaw_rate);
    }else{
        attitude_control->input_euler_angle_roll_pitch_yaw(copter.spiral_nav->get_roll(),
                                                           copter.spiral_nav->get_pitch(),
                                                           copter.spiral_nav->get_yaw(), true);
    }

    // adjust climb rate using rangefinder
    target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);

    // update altitude target and call position controller
    pos_control->set_alt_target_from_climb_rate(target_climb_rate, G_Dt, false);
    pos_control->update_z_controller();
}

uint32_t Copter::ModeSpiralTD::wp_distance() const
{
    return copter.spiral_nav->get_distance_to_target();
}

int32_t Copter::ModeSpiralTD::wp_bearing() const
{
    return copter.spiral_nav->get_bearing_to_target();
}

#endif
