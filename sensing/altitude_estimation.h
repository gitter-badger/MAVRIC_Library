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


#ifndef ALTITUDE_ESTIMATION_H_
#define ALTITUDE_ESTIMATION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "altitude.h"
#include "sonar.h"
#include "barometer.h"
#include "ahrs.h"
#include "math.h"
#include "kalman.h"
#include "mavlink_stream.h"
#include "gps_ublox.h"
#include "imu.h"
#include "coord_conventions.h"
#include "small_matrix.h"

#include <stdbool.h>

/**
 * \brief Altitude estimator structure
 */
typedef struct 
{	
	uint32_t time_last_gps_msg;						///< The time at which we received the last GPS message in ms
	uint32_t time_last_barometer_msg;				///< The time at which we received the last barometer message in ms
	uint32_t time_last_sonar_msg;					///< The time at which we received the last sonar message in ms
	bool init_gps_position;							///< The boolean flag ensuring that the GPS was initialized
	
	//Filtre Kalman
	kalman_filter_4D_t  kalman_filter;     	    ///< Kalman structure
	vector_4_t          measurement;				///< Mesures des capteurs
	
	const sonar_t*		sonar;						///< Pointer to sonar (input)
	const barometer_t* 	barometer;					///< Pointer to barometer (input)
	const ahrs_t* 		ahrs;						///< Pointer to estimated attitude and acceleration (input)
	const gps_t*			gps;	
	
	const imu_t*			imu;
	altitude_t* 		altitude_estimated; 			///< Pointer to estimated altitude (output)
} altitude_estimation_t;


/**
 * \brief Altitude estimation configuration
 */
typedef struct
{
	kalman_filter_4D_t kalman_filter;
} altitude_estimation_conf_t;	


/**
 * \brief               		Initialises the altitude estimation structure
 * 
 * \param 	estimator    		Pointer to data structure
 * \param 	config				Pointer to configuration
 */
bool altitude_estimation_init(altitude_estimation_t* estimator, const altitude_estimation_conf_t* config, altitude_t* altitude_estimated, const sonar_t* sonar, const barometer_t* barometer, const ahrs_t* ahrs, const gps_t* gps, const imu_t* imu);


/**
 * \brief               	Main update function
 * 
 * \param 	estimator    	Pointer to data structure
 */
void altitude_estimation_update(altitude_estimation_t* estimator);

//Mavlink communication//
/**
 * \brief	Mavlink tracker - Improve the strategy
 *
 * \param	altitude_estimation		The pointer to the structure of the track following
 */

void altitude_estimation_send_estimation(const altitude_estimation_t* alt_estimation, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg);

#ifdef __cplusplus
}
#endif

#endif /* ALTITUDE_ESTIMATION_H_ */