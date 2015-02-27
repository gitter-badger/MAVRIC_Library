/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file altitude_estimation.h
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 * \author Nicolas Dousse
 *   
 * \brief 	Altitude estimation
 *
 ******************************************************************************/


#include "altitude_estimation.h"
#include "print_util.h"
#include "time_keeper.h"
#include "constants.h"

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	Initialization of the position estimation from the GPS position
 *
 * \param	estimator		The pointer to the altitude estimation structure
 * \param	gps				The pointer to the GPS structure
 *
 * \return	void
 */
static void gps_position_init(altitude_estimation_t *estimator);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static void gps_position_init(altitude_estimation_t* estimator)
{
	if ( (estimator->init_gps_position == false)&&(estimator->gps->status == GPS_OK) )
	{
		if ( estimator->time_last_gps_msg < estimator->gps->time_last_msg )
		{
			estimator->time_last_gps_msg = estimator->gps->time_last_msg;
		
			estimator->init_gps_position = true;
			
			estimator->kalman_filter.state.v[0] = estimator->gps->altitude;
			estimator->kalman_filter.state.v[2] = estimator->gps->vertical_speed;
			
			estimator->measurement.v[2] = estimator->gps->altitude;
			estimator->measurement.v[3] = estimator->gps->vertical_speed;
			
			print_util_dbg_print("GPS position initialized!\r\n");
		}
	}
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool altitude_estimation_init(altitude_estimation_t* estimator, const altitude_estimation_conf_t* config, const sonar_t* sonar, const barometer_t* barometer, const ahrs_t* ahrs, const gps_t* gps, const imu_t* imu)
{
	bool init_success = true;
	
	// Init dependencies
	estimator->sonar 				= sonar;
	estimator->barometer 			= barometer;
	estimator->ahrs 				= ahrs;
	estimator->gps				= gps;
	
	estimator->imu				= imu;
	
	estimator->time_last_gps_msg = time_keeper_get_millis();
	estimator->time_last_barometer_msg = time_keeper_get_millis();
	estimator->init_gps_position = false;
	
	// Init members for the kalman filter
	estimator->kalman_filter.dt = 0.004;
	
	uint8_t i,j;
	//Matrix F
	for( i=0; i<4; i++ )
	{
		estimator->kalman_filter.control_model.v[i] = config->kalman_filter.control_model.v[i];
		
		for( j=0; j<4; j++)
		{
			estimator->kalman_filter.system_model.v[i][j] = config->kalman_filter.system_model.v[i][j];
			
			estimator->kalman_filter.observation_model.v[i][j] = config->kalman_filter.observation_model.v[i][j];
			
			estimator->kalman_filter.noise_prediction.v[i][j] = config->kalman_filter.noise_prediction.v[i][j];
			
			estimator->kalman_filter.noise_measurement.v[i][j] = config->kalman_filter.noise_measurement.v[i][j];
			
			estimator->kalman_filter.covariance.v[i][j] = config->kalman_filter.covariance.v[i][j];
		}
	}
	
	/*//Matrix F
	estimator->kalman_filter.system_model.v[0][0] = 1.0f;
	estimator->kalman_filter.system_model.v[0][1] = 0.0f;
	estimator->kalman_filter.system_model.v[0][2] = 0.004f;
	estimator->kalman_filter.system_model.v[0][3] = SQR(0.004f)/2.0f;
	estimator->kalman_filter.system_model.v[1][0] = 0.0f;
	estimator->kalman_filter.system_model.v[1][1] = 1.0f;
	estimator->kalman_filter.system_model.v[1][2] = 0.004f;
	estimator->kalman_filter.system_model.v[1][3] = SQR(0.004f)/2.0f;
	estimator->kalman_filter.system_model.v[2][0] = 0.0f;
	estimator->kalman_filter.system_model.v[2][1] = 0.0f;
	estimator->kalman_filter.system_model.v[2][2] = 1.0f;
	estimator->kalman_filter.system_model.v[2][3] = 0.004f;
	estimator->kalman_filter.system_model.v[3][0] = 0.0f;
	estimator->kalman_filter.system_model.v[3][1] = 0.0f;
	estimator->kalman_filter.system_model.v[3][2] = 0.0f;
	estimator->kalman_filter.system_model.v[3][3] = 1.0f;
	
	//Matrix B
	estimator->kalman_filter.control_model.v[0][0] = SQR(estimator->kalman_filter.dt)/2.0f;
	estimator->kalman_filter.control_model.v[0][1] = 0.0f;
	estimator->kalman_filter.control_model.v[0][2] = 0.0f;
	estimator->kalman_filter.control_model.v[0][3] = 0.0f;
	estimator->kalman_filter.control_model.v[1][0] = 0.0f;
	estimator->kalman_filter.control_model.v[1][1] = SQR(estimator->kalman_filter.dt)/2.0f;
	estimator->kalman_filter.control_model.v[1][2] = 0.0f;
	estimator->kalman_filter.control_model.v[1][3] = 0.0f;
	estimator->kalman_filter.control_model.v[2][0] = 0.0f;
	estimator->kalman_filter.control_model.v[2][1] = 0.0f;
	estimator->kalman_filter.control_model.v[2][2] = estimator->kalman_filter.dt;
	estimator->kalman_filter.control_model.v[2][3] = 0.0f;
	estimator->kalman_filter.control_model.v[3][0] = 0.0f;
	estimator->kalman_filter.control_model.v[3][1] = 0.0f;
	estimator->kalman_filter.control_model.v[3][2] = 0.0f;
	estimator->kalman_filter.control_model.v[3][3] = 0.0f;
	
	//Matrix H
	estimator->kalman_filter.observation_model.v[0][0] = 0.0f;
	estimator->kalman_filter.observation_model.v[0][1] = 1.0f;
	estimator->kalman_filter.observation_model.v[0][2] = 1.0f;
	estimator->kalman_filter.observation_model.v[0][3] = 0.0f;
	estimator->kalman_filter.observation_model.v[1][0] = 1.0f;
	estimator->kalman_filter.observation_model.v[1][1] = 0.0f;
	estimator->kalman_filter.observation_model.v[1][2] = 0.0f;
	estimator->kalman_filter.observation_model.v[1][3] = 0.0f;
	estimator->kalman_filter.observation_model.v[2][0] = 0.0f;
	estimator->kalman_filter.observation_model.v[2][1] = 0.0f;
	estimator->kalman_filter.observation_model.v[2][2] = 1.0f;
	estimator->kalman_filter.observation_model.v[2][3] = 0.0f;
	estimator->kalman_filter.observation_model.v[3][0] = 0.0f;
	estimator->kalman_filter.observation_model.v[3][1] = 0.0f;
	estimator->kalman_filter.observation_model.v[3][2] = 0.0f;
	estimator->kalman_filter.observation_model.v[3][3] = 0.0f;
	
	//Matrix Q
	estimator->kalman_filter.noise_prediction.v[0][0] = SQR(var_process)*SQR(dt);
	estimator->kalman_filter.noise_prediction.v[0][1] = SQR(var_process)*SQR(dt);
	estimator->kalman_filter.noise_prediction.v[0][2] = SQR(var_process)*dt;
	estimator->kalman_filter.noise_prediction.v[0][3] = 0.0f;
	estimator->kalman_filter.noise_prediction.v[1][0] = SQR(var_process)*SQR(dt);
	estimator->kalman_filter.noise_prediction.v[1][1] = SQR(var_process)*SQR(dt);
	estimator->kalman_filter.noise_prediction.v[1][2] = SQR(var_process)*dt;
	estimator->kalman_filter.noise_prediction.v[1][3] = 0.0f;
	estimator->kalman_filter.noise_prediction.v[2][0] = SQR(var_process)*dt;
	estimator->kalman_filter.noise_prediction.v[2][1] = SQR(var_process)*dt;
	estimator->kalman_filter.noise_prediction.v[2][2] = SQR(var_process);
	estimator->kalman_filter.noise_prediction.v[2][3] = 0.0f;
	estimator->kalman_filter.noise_prediction.v[3][0] = 0.0f;
	estimator->kalman_filter.noise_prediction.v[3][1] = 0.0f;
	estimator->kalman_filter.noise_prediction.v[3][2] = 0.0f;
	estimator->kalman_filter.noise_prediction.v[3][3] = SQR(var_acc);
	
	//Matrix R
	estimator->kalman_filter.noise_measurement.v[0][0] = 0.1f; // Variance sonar
	estimator->kalman_filter.noise_measurement.v[0][1] = 0.0f;
	estimator->kalman_filter.noise_measurement.v[0][2] = 0.0f;
	estimator->kalman_filter.noise_measurement.v[0][3] = 0.0f;
	estimator->kalman_filter.noise_measurement.v[1][0] = 0.0f;
	estimator->kalman_filter.noise_measurement.v[1][1] = 0.5f; // Variance barometer
	estimator->kalman_filter.noise_measurement.v[1][2] = 0.0f;
	estimator->kalman_filter.noise_measurement.v[1][3] = 0.0f;
	estimator->kalman_filter.noise_measurement.v[2][0] = 0.0f;
	estimator->kalman_filter.noise_measurement.v[2][1] = 0.0f;
	estimator->kalman_filter.noise_measurement.v[2][2] = 5.0f; // Variance GPS position
	estimator->kalman_filter.noise_measurement.v[2][3] = 0.0f;
	estimator->kalman_filter.noise_measurement.v[3][0] = 0.0f;
	estimator->kalman_filter.noise_measurement.v[3][1] = 0.0f;
	estimator->kalman_filter.noise_measurement.v[3][2] = 0.0f;
	estimator->kalman_filter.noise_measurement.v[3][3] = 10.0f; // Variance GPS vertical speed
	
	//Matrix P
	estimator->kalman_filter.covariance.v[0][0] = 10.0f;
	estimator->kalman_filter.covariance.v[0][1] = 0.0f;
	estimator->kalman_filter.covariance.v[0][2] = 0.0f;
	estimator->kalman_filter.covariance.v[0][3] = 0.0f;
	estimator->kalman_filter.covariance.v[1][0] = 0.0f;
	estimator->kalman_filter.covariance.v[1][1] = 10.0f;
	estimator->kalman_filter.covariance.v[1][2] = 0.0f;
	estimator->kalman_filter.covariance.v[1][3] = 0.0f;
	estimator->kalman_filter.covariance.v[2][0] = 0.0f;
	estimator->kalman_filter.covariance.v[2][1] = 0.0f;
	estimator->kalman_filter.covariance.v[2][2] = 10.0f;
	estimator->kalman_filter.covariance.v[3][0] = 0.0f;
	estimator->kalman_filter.covariance.v[3][1] = 0.0f;
	estimator->kalman_filter.covariance.v[3][2] = 0.0f;
	estimator->kalman_filter.covariance.v[3][3] = 10.0f;*/
	
	//State vector initialisation
	estimator->kalman_filter.state.v[0] = 400.0f; // TODO: change to config
	estimator->kalman_filter.state.v[1] = 0.0f;
	estimator->kalman_filter.state.v[2] = 0.0f;
	estimator->kalman_filter.state.v[3] = estimator->imu->calib_accelero.bias[Z];
	
	//Mesures capteurs
	estimator->measurement.v[0] = 0.0f;
	estimator->measurement.v[1] = 0.0f;
	estimator->measurement.v[2] = 0.0f;
	estimator->measurement.v[3] = 0.0f;

	gps_position_init(estimator);

	print_util_dbg_print("[ALTITUDE_ESTIMATION]: initialised.\r\n");
	
	return init_success;
}


void altitude_estimation_update(altitude_estimation_t* estimator)
{
	float acc_global;
	kalman_filter_4D_t* kalman = &estimator->kalman_filter;
	
	quat_t qacc_bf = quaternions_create_from_vector(estimator->ahrs->linear_acc);
	
	quat_t qacc = quaternions_local_to_global(estimator->ahrs->qe, qacc_bf);
	
	acc_global = qacc.v[2];
	
	kalman_4D_prediction(&(estimator->kalman_filter), acc_global);
	
	// sonar correction
	if( estimator->time_last_sonar_msg < estimator->sonar->last_update)
	{
		estimator->time_last_sonar_msg = estimator->sonar->last_update;
		
		estimator->measurement.v[0] = estimator->sonar->current_distance;
		
		kalman_4D_per_component_update(kalman,estimator->measurement,0,1);
	}
	
	// barometer correction
	if ( estimator->time_last_barometer_msg < estimator->barometer->last_update )
	{
		estimator->time_last_barometer_msg = estimator->barometer->last_update;
		
		estimator->measurement.v[1] = estimator->barometer->altitude;
		
		kalman_4D_per_component_update(kalman,estimator->measurement,1,0);
	}
	
	// gps correction
	if (estimator->init_gps_position)
	{
		if ( (estimator->time_last_gps_msg < estimator->gps->time_last_msg) && (estimator->gps->status == GPS_OK) )
		{
			estimator->time_last_gps_msg = estimator->gps->time_last_msg;

			estimator->measurement.v[2] = estimator->gps->altitude;
			
			kalman_4D_per_component_update(kalman,estimator->measurement,2,0);
			
			estimator->measurement.v[3] = estimator->gps->vertical_speed;
			
			kalman_4D_per_component_update(kalman,estimator->measurement,3,2);
		}
	}
	else
	{
		gps_position_init(estimator);
	}
	
	
	
	//kalman_4D_update(&(estimator->kalman_filter), estimator->measurement);
	
	estimator->altitude_estimated.above_sea 	= estimator->kalman_filter.state.v[0];
	estimator->altitude_estimated.above_ground = estimator->kalman_filter.state.v[1];

}

/// Mavlink communication ///

void altitude_estimation_send_dist(const altitude_t* estimator, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)

{

	mavlink_msg_named_value_float_pack(	mavlink_stream->sysid,
										mavlink_stream->compid,
										msg,
										time_keeper_get_millis(),
										"altestimated",
										-100*estimator->above_ground );
}

void ahrs_send_dist(const ahrs_t* estimator, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
	mavlink_msg_named_value_float_pack(	mavlink_stream->sysid,
										mavlink_stream->compid,
										msg,
										time_keeper_get_millis(),
										"ahrsZ",
										estimator->linear_acc[2] );
}

void baro_send_dist(const barometer_t* estimator, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
	mavlink_msg_named_value_float_pack(	mavlink_stream->sysid,
										mavlink_stream->compid,
										msg,
										time_keeper_get_millis(),
										"baroAlt",
										estimator->altitude );
}