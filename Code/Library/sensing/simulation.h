/*
 * simulation.c
 *
 * Onboard Hardware-in-the-loop simulation 
 * Simulates quad-copter dynamics based on simple aerodynamics/physics model and generates simulated raw IMU measurements
 * 
 * Created: 06/08/2013 17:02:56
 *  Author: Felix Schill
 */ 

#ifndef SIMULATION_H_
#define SIMULATION_H_

#include "compiler.h"
#include "qfilter.h"

#include "conf_platform.h"
#include "imu.h"
#include "servo_pwm.h"
#include "bmp085.h"
#include "position_estimation.h"

typedef struct {
	float torques_bf[3], rates_bf[3], lin_forces_bf[3], vel_bf[3], vel[3], pos[3];
	Quat_Attitude_t attitude;
	local_coordinates_t localPosition;
	
	float rotorspeeds[ROTORCOUNT];                              // estimated rotor speeds
	float rotor_lpf, rotor_rpm_gain, rotor_rpm_offset;          // low pass filter to simulate rotor inertia and lag, gain/offset to convert servo commands to rpm
	float rotor_cd, rotor_cl, rotor_diameter, rotor_foil_area;	// rotor lift and drag coefficients, mean rotor diameter (used to calculate rotor lift, torque and drag)
	float rotor_pitch;
	float total_mass;											// vehicle mass in kg
	float vehicle_drag;                                           // coefficient of drag of whole vehicle
	float roll_pitch_momentum,  yaw_momentum;                   // angular momentum constants (assumed to be independent)
	float rotor_momentum;										// angular momentum of rotor (for rotor inertia)
	float rotor_arm_length;							 			// distance between CoG and motor (in meter)
	uint32_t last_update;											// last update in system ticks
	float dt;													// time base of current update
	float wind_x, wind_y;
} simulation_model_t;


void init_simulation(simulation_model_t *sim, Quat_Attitude_t *start_attitude, local_coordinates_t localPos);

// computes artificial gyro and accelerometer values based on motor commands
void simu_update(simulation_model_t *sim, servo_output *servo_commands, Imu_Data_t *imu, position_estimator_t *pos_est);
void simulate_barometer(simulation_model_t *sim, pressure_data *pressure);
void simulate_gps(simulation_model_t *sim, gps_Data_type *gps);

#endif /* SIMULATION_H_ */