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
 * \file sonar.h
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief Definition of structure for sonar, independent of the sensor used
 *
 ******************************************************************************/


#ifndef SONAR_H_
#define SONAR_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "quaternions.h"

#define LPF_SONAR_VARIO 0.4f

/**
 * \brief Sonar structure, independent of the sensor used
*/
typedef struct 
{
	uint32_t last_update; 		///< Last time we updated the sensor measurement
	quat_t 	orientation; 		///< Direction the sensor faces from FIXME enum.
	float 	min_distance; 		///< Minimum distance the sensor can measure in centimeters
	float 	max_distance; 		///< Maximum distance the sensor can measure in centimeters
	float 	current_distance;	///< Measured distance in meters
	float	current_velocity;	///< Computed velocity in m/s
	float 	covariance; 		///< Measurement covariance in centimeters, 0 for unknown / invalid readings
	bool 	healthy;			///< Indicates whether the current measurement can be trusted
	bool	healthy_vel;		///< Indicated whether the velocity estimation can be trusted
} sonar_t;


#ifdef __cplusplus
	}
#endif

#endif /* SONAR_H */