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
 * \file kalman.h
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief 2D kalman filter
 *
 ******************************************************************************/


#ifndef KALMAN_H_
#define KALMAN_H_


#ifdef __cplusplus
extern "C" 
{
#endif

#include "small_matrix.h"
#include "linear_algebra.h"


/**
 * @brief 2D Kalman filter 
 */
typedef struct kalman_filter_2D_t 
{
	matrix_2x2_t system_model;			///<	Model matrix
	matrix_2x2_t control_model;			///<	Control matrix
	matrix_2x2_t observation_model;		///<	Observation matrix
	matrix_2x2_t noise_prediction;		///<	Model noise matrix
	matrix_2x2_t noise_measurement;		///<	Measurement noise  matrix
	matrix_2x2_t covariance;			///<	Covariance matrix
	vector_2_t   state;					///<	State vector
	float        dt;                    ///<    Time step between each update
} kalman_filter_2D_t;

/**
 * @brief 4D Kalman filter 
 */
typedef struct kalman_filter_4D_t 
{
	matrix_4x4_t system_model;			///<	Model matrix
	vector_4_t control_model;			///<	Control matrix
	matrix_4x4_t observation_model;		///<	Observation matrix
	matrix_4x4_t noise_prediction;		///<	Model noise matrix
	matrix_4x4_t noise_measurement;		///<	Measurement noise  matrix
	matrix_4x4_t covariance;			///<	Covariance matrix
	vector_4_t   state;					///<	State vector
	float        dt;                    ///<    Time step between each update
} kalman_filter_4D_t;

/**
 * @brief 4D Kalman filter 
 */
typedef struct kalman_filter_5D_t 
{
	matrix_5x5_t system_model;			///<	Model matrix
	vector_5_t control_model;			///<	Control matrix
	matrix_5x5_t observation_model;		///<	Observation matrix
	matrix_5x5_t noise_prediction;		///<	Model noise matrix
	matrix_5x5_t noise_measurement;		///<	Measurement noise  matrix
	matrix_5x5_t covariance;			///<	Covariance matrix
	vector_5_t   state;					///<	State vector
	float        dt;                    ///<    Time step between each update
} kalman_filter_5D_t;

void kalman_4D_prediction(kalman_filter_4D_t *kalman, float control);
void kalman_4D_update(kalman_filter_4D_t *kalman, vector_4_t measurement);

void kalman_4D_per_component_update(kalman_filter_4D_t *kalman, vector_4_t measurement, uint8_t m_index, uint8_t x_index);

void kalman_5D_prediction(kalman_filter_5D_t *kalman, float control);

void kalman_5D_per_component_update(kalman_filter_5D_t *kalman, vector_5_t measurement, uint8_t m_index, uint8_t x_index);

/**
 * \brief 	Kalman prediction step
 * 
 * \param 	kalman 		Pointer to kalman structure
 * \param 	control 	Control vector
 */
void kalman_2D_prediction(kalman_filter_2D_t *kalman, vector_2_t control);


/**
 * \brief 	Kalman update step
 * 
 * \param 	kalman 			Pointer to kalman structure
 * \param 	measurement 	Measurement vector
 */
void kalman_2D_update(kalman_filter_2D_t *kalman, vector_2_t measurement);


#ifdef __cplusplus
}
#endif


#endif /* KALMAN_H_ */