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
 * \file attitude_controller_ywing_config.h
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief Configuration for the module attitude_controller for Ywing
 *
 ******************************************************************************/


#ifndef ATTITUDE_CONTROLLER_YWING_CONFIG_H_
#define ATTITUDE_CONTROLLER_YWING_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "attitude_controller.h"

static attitude_controller_conf_t attitude_controller_ywing_config =
{
	// #########################################################################
	// ######  RATE CONTROL  ###################################################
	// #########################################################################
	.rate_pid_config =
	{
		// -----------------------------------------------------------------
		// ------ ROLL RATE PID --------------------------------------------
		// -----------------------------------------------------------------
		{
			.p_gain = 0.2f,
			.clip_min = -1.0f,
			.clip_max = 1.0f,
			.integrator =
			{
				.gain = 0.0f,
				.clip_pre = 1.0f,
				.accumulator = 0.0f,
				.clip = 1.0f,
			},
			.differentiator =
			{
				.gain = 0.002f,
				.previous = 0.0f,
				.clip = 1.0f
			},
			.soft_zone_width = 0.0f,
		},
		// -----------------------------------------------------------------
		// ------ PITCH RATE PID -------------------------------------------
		// -----------------------------------------------------------------
		{
			.p_gain = 0.3f,
			.clip_min = -1.0f,
			.clip_max = 1.0f,
			.integrator =
			{
				.gain = 0.0f,
				.clip_pre = 1.0f,
				.accumulator = 0.0f,
				.clip = 1.0f,
			},
			.differentiator =
			{
				.gain = 0.002f,
				.previous = 0.0f,
				.clip = 1.0f
			},
			.soft_zone_width = 0.0f,
		},
		// -----------------------------------------------------------------
		// ------ YAW RATE PID ---------------------------------------------
		// -----------------------------------------------------------------
		{
			.p_gain = 0.3f,
			.clip_min = -1.0f,
			.clip_max = 1.0f,
			.integrator={
				.gain = 0.0f,
				.clip_pre = 1.0f,
				.accumulator = 0.0f,
				.clip = 1.0f,
			},
			.differentiator={
				.gain = 0.002f,
				.previous = 0.0f,
				.clip = 1.0f
			},
			.soft_zone_width = 0.0f
		}
	},
	// #########################################################################
	// ######  ANGLE CONTROL  ##################################################
	// #########################################################################
	.angle_pid_config =
	{
		// -----------------------------------------------------------------
		// ------ ROLL ANGLE PID -------------------------------------------
		// -----------------------------------------------------------------
		{
			.p_gain = 5.0f,
			.clip_min = -10.0f,
			.clip_max = 10.0f,
			.integrator={
				.gain = 0.0f,
				.clip_pre = 1.0f,
				.accumulator = 0.0f,
				.clip = 10.0f,
			},
			.differentiator={
				.gain = 0.0f,
				.previous = 0.0f,
				.clip = 10.0f
			},
			.soft_zone_width = 0.0f
		},
		// -----------------------------------------------------------------
		// ------ PITCH ANGLE PID ------------------------------------------
		// -----------------------------------------------------------------
		{
			.p_gain = 6.0f,
			.clip_min = -10.0f,
			.clip_max = 10.0f,
			.integrator={
				.gain = 0.0f,
				.clip_pre = 1.0f,
				.accumulator = 0.0f,
				.clip = 10.0f,
			},
			.differentiator={
				.gain = 0.0f,
				.previous = 0.0f,
				.clip = 10.0f
			},
			.soft_zone_width = 0.0f
		},
		// -----------------------------------------------------------------
		// ------ YAW ANGLE PID --------------------------------------------
		// -----------------------------------------------------------------
		{
			.p_gain = 6.0f,
			.clip_min = -10.0f,
			.clip_max = 10.0f,
			.integrator={
				.gain = 0.0f,
				.clip_pre = 1.0f,
				.accumulator = 0.0f,
				.clip = 10.0f,
			},
			.differentiator={
				.gain = 0.0f,
				.previous = 0.0f,
				.clip = 10.0f
			},
			.soft_zone_width = 0.0f
		},
	},
};

#ifdef __cplusplus
}
#endif

#endif // ATTITUDE_CONTROLLER_YWING_CONFIG_H_