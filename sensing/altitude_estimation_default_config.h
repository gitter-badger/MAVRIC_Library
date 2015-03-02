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
 * \file altitude_estimation_default_config.h
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief The configuration for the altitude estimation module
 *
 ******************************************************************************/


#ifndef ALTITUDE_ESTIMATION_DEFAULT_CONFIG_H_
#define ALTITUDE_ESTIMATION_DEFAULT_CONFIG_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include "altitude_estimation.h"
#include "math.h"

static const altitude_estimation_conf_t altitude_estimation_default_config = 
{

	//Matrix F
	.kalman_filter.system_model.v[0][0] = 1.0f,
	.kalman_filter.system_model.v[0][1] = 0.0f,
	.kalman_filter.system_model.v[0][2] = 0.004f,
	.kalman_filter.system_model.v[0][3] = SQR(0.004f)/2.0f,
	.kalman_filter.system_model.v[0][4] = 0.0f,
	.kalman_filter.system_model.v[1][0] = 0.0f,
	.kalman_filter.system_model.v[1][1] = 1.0f,
	.kalman_filter.system_model.v[1][2] = 0.004f,
	.kalman_filter.system_model.v[1][3] = SQR(0.004f)/2.0f,
	.kalman_filter.system_model.v[1][4] = 0.0f,
	.kalman_filter.system_model.v[2][0] = 0.0f,
	.kalman_filter.system_model.v[2][1] = 0.0f,
	.kalman_filter.system_model.v[2][2] = 1.0f,
	.kalman_filter.system_model.v[2][3] = 0.004f,
	.kalman_filter.system_model.v[2][4] = 0.0f,
	.kalman_filter.system_model.v[3][0] = 0.0f,
	.kalman_filter.system_model.v[3][1] = 0.0f,
	.kalman_filter.system_model.v[3][2] = 0.0f,
	.kalman_filter.system_model.v[3][3] = 1.0f,
	.kalman_filter.system_model.v[3][4] = 0.0f,
	.kalman_filter.system_model.v[4][0] = 0.0f,
	.kalman_filter.system_model.v[4][1] = 0.0f,
	.kalman_filter.system_model.v[4][2] = 0.0f,
	.kalman_filter.system_model.v[4][3] = 0.0f,
	.kalman_filter.system_model.v[4][4] = 1.0f,
	
	//Matrix B
	.kalman_filter.control_model.v[0] = SQR(0.004f)/2.0f,
	.kalman_filter.control_model.v[1] = SQR(0.004f)/2.0f,
	.kalman_filter.control_model.v[2] = 0.004f,
	.kalman_filter.control_model.v[3] = 0.0f,
	.kalman_filter.control_model.v[4] = 0.0f,
	
	//Matrix H
	.kalman_filter.observation_model.v[0][0] = 0.0f,
	.kalman_filter.observation_model.v[0][1] = 1.0f,
	.kalman_filter.observation_model.v[0][2] = 0.0f,
	.kalman_filter.observation_model.v[0][3] = 0.0f,
	.kalman_filter.observation_model.v[0][4] = 0.0f,

	.kalman_filter.observation_model.v[1][0] = 1.0f,
	.kalman_filter.observation_model.v[1][1] = 0.0f,
	.kalman_filter.observation_model.v[1][2] = 0.0f,
	.kalman_filter.observation_model.v[1][3] = 0.0f,
	.kalman_filter.observation_model.v[1][4] = 1.0f,

	.kalman_filter.observation_model.v[2][0] = 1.0f,
	.kalman_filter.observation_model.v[2][1] = 0.0f,
	.kalman_filter.observation_model.v[2][2] = 0.0f,
	.kalman_filter.observation_model.v[2][3] = 0.0f,
	.kalman_filter.observation_model.v[2][4] = 0.0f,

	.kalman_filter.observation_model.v[3][0] = 0.0f,
	.kalman_filter.observation_model.v[3][1] = 0.0f,
	.kalman_filter.observation_model.v[3][2] = 1.0f,
	.kalman_filter.observation_model.v[3][3] = 0.0f,
	.kalman_filter.observation_model.v[3][4] = 0.0f,

	.kalman_filter.observation_model.v[4][0] = 0.0f,
	.kalman_filter.observation_model.v[4][1] = 0.0f,
	.kalman_filter.observation_model.v[4][2] = 0.0f,
	.kalman_filter.observation_model.v[4][3] = 0.0f,
	.kalman_filter.observation_model.v[4][4] = 0.0f,	
	
	//Matrix Q
	.kalman_filter.noise_prediction.v[0][0] = 0.25f* 0.000000000256f *SQR(0.0037f), // pow(0.004f,4) = 2.56E-10
	.kalman_filter.noise_prediction.v[0][1] = 0.25f* 0.000000000256f *SQR(0.0037f),
	.kalman_filter.noise_prediction.v[0][2] = 0.5f*0.000000064f*SQR(0.0037f), // pow(0.004f,3) = 6.4E-8
	.kalman_filter.noise_prediction.v[0][3] = 0.5f*SQR(0.004f)*SQR(0.0037f),
	.kalman_filter.noise_prediction.v[0][4] = 0.0f,
	.kalman_filter.noise_prediction.v[1][0] = 0.25f* 0.000000000256f *SQR(0.0037f),
	.kalman_filter.noise_prediction.v[1][1] = 0.25f* 0.000000000256f *SQR(0.0037f),
	.kalman_filter.noise_prediction.v[1][2] = 0.5f*0.000000064f*SQR(0.0037f),
	.kalman_filter.noise_prediction.v[1][3] = 0.5f*SQR(0.004f)*SQR(0.0037f),
	.kalman_filter.noise_prediction.v[1][4] = 0.0f,
	.kalman_filter.noise_prediction.v[2][0] = 0.5f*0.000000064f*SQR(0.0037f),
	.kalman_filter.noise_prediction.v[2][1] = 0.5f*0.000000064f*SQR(0.0037f),
	.kalman_filter.noise_prediction.v[2][2] = SQR(0.004f)*SQR(0.0037f),
	.kalman_filter.noise_prediction.v[2][3] = 0.004f*SQR(0.0037f),
	.kalman_filter.noise_prediction.v[2][4] = 0.0f,
	.kalman_filter.noise_prediction.v[3][0] = 0.5f*SQR(0.004f)*SQR(0.0037f),
	.kalman_filter.noise_prediction.v[3][1] = 0.5f*SQR(0.004f)*SQR(0.0037f),
	.kalman_filter.noise_prediction.v[3][2] = 0.004f*SQR(0.0037f),
	.kalman_filter.noise_prediction.v[3][3] = SQR(0.0037f), // var_acc: = 0.0037f, computed with accelerometer measurements
	.kalman_filter.noise_prediction.v[3][4] = 0.0f,
	.kalman_filter.noise_prediction.v[4][0] = 0.0f,
	.kalman_filter.noise_prediction.v[4][1] = 0.0f,
	.kalman_filter.noise_prediction.v[4][2] = 0.0f,
	.kalman_filter.noise_prediction.v[4][3] = 0.0f,
	.kalman_filter.noise_prediction.v[4][4] = SQR(0.0001f),
	
	//Matrix R
	.kalman_filter.noise_measurement.v[0][0] = 0.0025f, // Variance sonar
	.kalman_filter.noise_measurement.v[0][1] = 0.0f,
	.kalman_filter.noise_measurement.v[0][2] = 0.0f,
	.kalman_filter.noise_measurement.v[0][3] = 0.0f,
	.kalman_filter.noise_measurement.v[1][0] = 0.0f,
	.kalman_filter.noise_measurement.v[1][1] = 0.25f, // Variance barometer
	.kalman_filter.noise_measurement.v[1][2] = 0.0f,
	.kalman_filter.noise_measurement.v[1][3] = 0.0f,
	.kalman_filter.noise_measurement.v[2][0] = 0.0f,
	.kalman_filter.noise_measurement.v[2][1] = 0.0f,
	.kalman_filter.noise_measurement.v[2][2] = 25.0f, // Variance GPS position
	.kalman_filter.noise_measurement.v[2][3] = 0.0f,
	.kalman_filter.noise_measurement.v[3][0] = 0.0f,
	.kalman_filter.noise_measurement.v[3][1] = 0.0f,
	.kalman_filter.noise_measurement.v[3][2] = 0.0f,
	.kalman_filter.noise_measurement.v[3][3] = 100.0f, // Variance GPS vertical speed
	
	//Matrix P
	.kalman_filter.covariance.v[0][0] = 200.0f,
	.kalman_filter.covariance.v[0][1] = 0.0f,
	.kalman_filter.covariance.v[0][2] = 0.0f,
	.kalman_filter.covariance.v[0][3] = 0.0f,
	.kalman_filter.covariance.v[0][4] = 0.0f,
	.kalman_filter.covariance.v[1][0] = 0.0f,
	.kalman_filter.covariance.v[1][1] = 200.0f,
	.kalman_filter.covariance.v[1][2] = 0.0f,
	.kalman_filter.covariance.v[1][3] = 0.0f,
	.kalman_filter.covariance.v[1][4] = 0.0f,
	.kalman_filter.covariance.v[2][0] = 0.0f,
	.kalman_filter.covariance.v[2][1] = 0.0f,
	.kalman_filter.covariance.v[2][2] = 200.0f,
	.kalman_filter.covariance.v[2][3] = 0.0f,
	.kalman_filter.covariance.v[2][4] = 0.0f,
	.kalman_filter.covariance.v[3][0] = 0.0f,
	.kalman_filter.covariance.v[3][1] = 0.0f,
	.kalman_filter.covariance.v[3][2] = 0.0f,
	.kalman_filter.covariance.v[3][3] = 200.0f,
	.kalman_filter.covariance.v[3][4] = 0.0f,
	.kalman_filter.covariance.v[4][0] = 0.0f,
	.kalman_filter.covariance.v[4][1] = 0.0f,
	.kalman_filter.covariance.v[4][2] = 0.0f,
	.kalman_filter.covariance.v[4][3] = 0.0f,
	.kalman_filter.covariance.v[4][4] = 200.0f,
};

#ifdef __cplusplus
}
#endif

#endif /* ALTITUDE_ESTIMATION_DEFAULT_CONFIG_H_ */