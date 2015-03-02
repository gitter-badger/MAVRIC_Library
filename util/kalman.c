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
 * \file kalman.c
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief 2D kalman filter
 *
 ******************************************************************************/


#include "kalman.h"

void kalman_4D_prediction(kalman_filter_4D_t *kalman, float control)
{
	kalman->state =     vadd4( 	mvmul4(kalman->system_model, kalman->state), 
								svmul4(control, kalman->control_model));

	kalman->covariance = madd4(  mmul4( 	mmul4(	kalman->system_model, 
												kalman->covariance), 
										trans4(kalman->system_model)), 
								kalman->noise_prediction);
}

void kalman_4D_update(kalman_filter_4D_t *kalman, vector_4_t measurement)
{
	vector_4_t innovation = vsub4(	measurement, 
									mvmul4(kalman->observation_model, kalman->state));
	
	matrix_4x4_t innovation_covariance = madd4(	mmul4( 	mmul4(kalman->observation_model, kalman->covariance),
											   			trans4(kalman->observation_model)), 
											  	kalman->noise_measurement);
	
	matrix_4x4_t kalman_gain = mmul4(	mmul4(kalman->covariance, trans4(kalman->observation_model)), 
										inv4(innovation_covariance));
	
	kalman->state = vadd4(kalman->state, mvmul4(kalman_gain, innovation));

	kalman->covariance = mmul4(	msub4(ident_4x4, mmul4(kalman_gain, kalman->observation_model)), 
								kalman->covariance);
}

void kalman_4D_per_component_update(kalman_filter_4D_t *kalman, vector_4_t measurement, uint8_t m_index, uint8_t x_index)
{
	uint8_t i, j;
	
	vector_4_t kalman_gain;
	
	matrix_4x4_t KH = 
		{.v={{0.0f, 0.0f, 0.0f, 0.0f}, 
        {0.0f, 0.0f, 0.0f, 0.0f}, 
        {0.0f, 0.0f, 0.0f, 0.0f}, 
        {0.0f, 0.0f, 0.0f, 0.0f}} };
	
	float Hx = 0.0f;
	for ( i=0; i<4; i++ )
	{
		Hx += kalman->observation_model.v[m_index][i]*kalman->state.v[i];
	}
	
	float innovation = measurement.v[m_index] - Hx;
	
	vector_4_t PHt;
	PHt = mvmul4(kalman->covariance,col4(trans4(kalman->observation_model),m_index));

	float HPHt = 0.0f;
	for ( i=0; i<4; i++)
	{
		HPHt += kalman->observation_model.v[m_index][i] * PHt.v[i];
	} 

	float innovation_covariance_inverse = 1.0f / (HPHt + kalman->noise_measurement.v[m_index][m_index]);
	
	for( i=0; i<4; i++ )
	{
		kalman_gain.v[i] = PHt.v[i] * innovation_covariance_inverse;
	}
	
	kalman->state = vadd4(kalman->state, svmul4(innovation,kalman_gain));
	
	for( i=0; i<4; i++)
	{
		for ( j=0; j<4; j++)
		{
			KH.v[i][j] = kalman_gain.v[i]*kalman->observation_model.v[m_index][j];
		}
	}
	kalman->covariance = mmul4(	msub4(ident_4x4, KH), kalman->covariance);
}

void kalman_5D_prediction(kalman_filter_5D_t *kalman, float control)
{
	kalman->state =     vadd5( 	mvmul5(kalman->system_model, kalman->state), 
								svmul5(control, kalman->control_model));

	kalman->covariance = madd5(  mmul5( 	mmul5(	kalman->system_model, 
												kalman->covariance), 
										trans5(kalman->system_model)), 
								kalman->noise_prediction);
}

void kalman_5D_per_component_update(kalman_filter_5D_t *kalman, vector_5_t measurement, uint8_t m_index, uint8_t x_index)
{
	uint8_t dim = 5;
	uint8_t i, j;
	
	vector_5_t kalman_gain;
	
	matrix_5x5_t KH = 
		{.v={{0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, 
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, 
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, 
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, 
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}} };
	
	float Hx = 0.0f;
	for ( i=0; i<dim; i++ )
	{
		Hx += kalman->observation_model.v[m_index][i]*kalman->state.v[i];
	}
	
	float innovation = measurement.v[m_index] - Hx;
	
	vector_5_t PHt;
	PHt = mvmul5(kalman->covariance,col5(trans4(kalman->observation_model),m_index));

	float HPHt = 0.0f;
	for ( i=0; i<dim; i++)
	{
		HPHt += kalman->observation_model.v[m_index][i] * PHt.v[i];
	} 

	float innovation_covariance_inverse = 1.0f / (HPHt + kalman->noise_measurement.v[m_index][m_index]);
	
	float PHt = 0.0f;
	for( i=0; i<dim; i++ )
	{
		PHt = 0.0f;
		for ( j=0; j<dim; j++ )
		{
			PHt += kalman->covariance.v[i][j]*kalman->observation_model.v[m_index][j];
		}
		kalman_gain.v[i] = PHt * innovation_covariance_inverse;
	}
	
	kalman->state = vadd5(kalman->state, svmul5(innovation,kalman_gain));
	
	for( i=0; i<dim; i++)
	{
		for ( j=0; j<dim; j++)
		{
			KH.v[i][j] = kalman_gain.v[i]*kalman->observation_model.v[m_index][j];
		}
	}
	kalman->covariance = mmul5(	msub4(ident_5x5, KH), kalman->covariance);
}

void kalman_2D_prediction(kalman_filter_2D_t *kalman, vector_2_t control) 
{
	kalman->state =     vadd2( 	mvmul2(kalman->system_model, kalman->state), 
								mvmul2(kalman->control_model, control));

	kalman->covariance= madd2(  mmul2( 	mmul2(	kalman->system_model, 
												kalman->covariance), 
										trans2(kalman->system_model)), 
								kalman->noise_prediction);
}

void kalman_2D_update(kalman_filter_2D_t *kalman, vector_2_t measurement)
{
	vector_2_t innovation = vsub2(	measurement, 
									mvmul2(kalman->observation_model, kalman->state));
	
	matrix_2x2_t innovation_covariance = madd2(	mmul2( 	mmul2(kalman->observation_model, kalman->covariance),
											   			trans2(kalman->observation_model)), 
											  	kalman->noise_measurement);
	
	matrix_2x2_t kalman_gain = mmul2(	mmul2(kalman->covariance, trans2(kalman->observation_model)), 
										inv2(innovation_covariance));
	
	kalman->state = vadd2(kalman->state, mvmul2(kalman_gain, innovation));

	kalman->covariance = mmul2(	msub2(ident_2x2, mmul2(kalman_gain, kalman->observation_model)), 
								kalman->covariance);
}
