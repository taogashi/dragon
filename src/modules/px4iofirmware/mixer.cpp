/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mixer.cpp
 *
 * Control channel input/output mixer and failsafe.
 */

#include <nuttx/config.h>
#include <syslog.h>

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>

#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>

#include <systemlib/pwm_limit/pwm_limit.h>
#include <systemlib/mixer/mixer.h>

extern "C" {
	//#define DEBUG
#include "px4io.h"
}

/*
 * Maximum interval in us before FMU signal is considered lost
 */
#define FMU_INPUT_DROP_LIMIT_US		500000

/* current servo arm/disarm state */
static bool mixer_servos_armed = false;
static bool should_arm = false;
static bool should_always_enable_pwm = false;
static volatile bool in_mixer = false;

/* selected control values and count for mixing */
enum mixer_source {
	MIX_NONE,
	MIX_FMU,
	MIX_OVERRIDE,
	MIX_FAILSAFE,
	MIX_OVERRIDE_FMU_OK
};
static mixer_source source;

static int	mixer_callback(uintptr_t handle,
		uint8_t control_group,
		uint8_t control_index,
		float &control);

static MixerGroup mixer_group(mixer_callback, 0);

/* Set the failsafe values of all mixed channels (based on zero throttle, controls centered) */
static void mixer_set_failsafe();
#define NONLINEAR 1
#if NONLINEAR == 1
int cmpfunc(const float* a, const float* b);
float *bsearch(const float *key, const float *base, size_t num, size_t size,
		int (*cmp)(const float *key, const float *elt));
static void map_desire_moment_to_aoa(float* desire_moment, float* desire_aoa, int mixed);
static void map_aoa_to_pwm(const uint16_t *disarmed_pwm, const uint16_t *min_pwm, const uint16_t *max_pwm, const float* aoa, uint16_t* pwm_out);

static const float moment_aoa_table[] = {
	-2.3429, -2.3114, -2.2799, -2.2484, -2.2169, -2.1854, -2.1539, -2.1224, -2.0909, -2.0594,
	-2.0279, -1.9964, -1.9649, -1.9334, -1.9019, -1.8704, -1.8389, -1.8074, -1.7759, -1.7444,
	-1.7129, -1.6814, -1.6499, -1.6184, -1.5869, -1.5554, -1.5239, -1.4924, -1.4609, -1.4294,
	-1.3979, -1.3664, -1.3349, -1.3034, -1.2719, -1.2404, -1.2089, -1.1774, -1.1459, -1.1144,
	-1.0829, -1.0514, -1.0199, -0.9884, -0.9569, -0.9254, -0.8939, -0.8624, -0.8309, -0.7994,
	-0.7679, -0.7364, -0.7049, -0.6734, -0.6419, -0.6104, -0.5789, -0.5474, -0.5159, -0.4844,
	-0.4529, -0.4214, -0.3899, -0.3877, -0.3854, -0.3802, -0.3783, -0.3633, -0.2772, -0.2327,
	-0.1884, -0.1793, -0.1462, 0.1188, 0.2423, 0.2839, 0.3349, 0.3614, 0.4026, 0.4402,
	0.4686,  0.5050,  0.5267,  0.5604,  0.5799,  0.6063,  0.6291,  0.6444,  0.6791,  0.6804,
	0.7221,  0.7379,  0.7537,  0.7562,  0.7751,  0.7863,  0.8193,  0.8279,  0.8708,  0.8861,
	0.9013,  0.9093,  0.9172,  0.9620,  0.9642,  0.9663,  0.9999,  1.0035, 1.0071,  1.0410,
	1.0412,  1.0414,  1.0695,  1.1010, 1.1325, 1.1640, 1.1955, 1.2270, 1.2585, 1.2900,
	1.3215, 1.3530, 1.3845, 1.4160, 1.4475, 1.4790, 1.5105, 1.5420, 1.5735, 1.6050,
	1.6365, 1.6680, 1.6995, 1.7310, 1.7625, 1.7940, 1.8255, 1.8570, 1.8885, 1.9200,
	1.9515, 1.9830, 2.0145, 2.0460, 2.0775, 2.1090, 2.1405, 2.1720, 2.2035, 2.2350,
	2.2665, 2.2980, 2.3295, 2.3610, 2.3925, 2.4240, 2.4555, 2.4870, 2.5185, 2.5500
};
#endif

void mixer_tick(void)
{

	/* check that we are receiving fresh data from the FMU */
	if ((system_state.fmu_data_received_time == 0) ||
			hrt_elapsed_time(&system_state.fmu_data_received_time) > FMU_INPUT_DROP_LIMIT_US) {

		/* too long without FMU input, time to go to failsafe */
		if (r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK) {
			isr_debug(1, "AP RX timeout");
		}
		r_status_flags &= ~(PX4IO_P_STATUS_FLAGS_FMU_OK);
		r_status_alarms |= PX4IO_P_STATUS_ALARMS_FMU_LOST;

	} else {
		r_status_flags |= PX4IO_P_STATUS_FLAGS_FMU_OK;

		/* this flag is never cleared once OK */
		r_status_flags |= PX4IO_P_STATUS_FLAGS_FMU_INITIALIZED;
	}

	/* default to failsafe mixing - it will be forced below if flag is set */
	source = MIX_FAILSAFE;

	/*
	 * Decide which set of controls we're using.
	 */

	/* do not mix if RAW_PWM mode is on and FMU is good */
	if ((r_status_flags & PX4IO_P_STATUS_FLAGS_RAW_PWM) &&
			(r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK)) {

		/* don't actually mix anything - we already have raw PWM values */
		source = MIX_NONE;

	} else {

		if (!(r_status_flags & PX4IO_P_STATUS_FLAGS_OVERRIDE) &&
				(r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK) &&
				(r_status_flags & PX4IO_P_STATUS_FLAGS_MIXER_OK)) {

			/* mix from FMU controls */
			source = MIX_FMU;
		}

		if ( (r_status_flags & PX4IO_P_STATUS_FLAGS_OVERRIDE) &&
				(r_status_flags & PX4IO_P_STATUS_FLAGS_RC_OK) &&
				(r_status_flags & PX4IO_P_STATUS_FLAGS_MIXER_OK) &&
				!(r_setup_arming & PX4IO_P_SETUP_ARMING_RC_HANDLING_DISABLED) &&
				!(r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK) &&
				/* do not enter manual override if we asked for termination failsafe and FMU is lost */
				!(r_setup_arming & PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE)) {

			/* if allowed, mix from RC inputs directly */
			source = MIX_OVERRIDE;
		} else 	if ( (r_status_flags & PX4IO_P_STATUS_FLAGS_OVERRIDE) &&
				(r_status_flags & PX4IO_P_STATUS_FLAGS_RC_OK) &&
				(r_status_flags & PX4IO_P_STATUS_FLAGS_MIXER_OK) &&
				!(r_setup_arming & PX4IO_P_SETUP_ARMING_RC_HANDLING_DISABLED) &&
				(r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK)) {

			/* if allowed, mix from RC inputs directly up to available rc channels */
			source = MIX_OVERRIDE_FMU_OK;
		}
	}

	/*
	 * Decide whether the servos should be armed right now.
	 *
	 * We must be armed, and we must have a PWM source; either raw from
	 * FMU or from the mixer.
	 *
	 */
	should_arm = (
			/* IO initialised without error */   (r_status_flags & PX4IO_P_STATUS_FLAGS_INIT_OK)
			/* and IO is armed */ 		  && (r_status_flags & PX4IO_P_STATUS_FLAGS_SAFETY_OFF)
			/* and FMU is armed */ 		  && (
				((r_setup_arming & PX4IO_P_SETUP_ARMING_FMU_ARMED)
				 /* and there is valid input via or mixer */         &&   (r_status_flags & PX4IO_P_STATUS_FLAGS_MIXER_OK) )
				/* or direct PWM is set */               || (r_status_flags & PX4IO_P_STATUS_FLAGS_RAW_PWM)
				/* or failsafe was set manually */	 || ((r_setup_arming & PX4IO_P_SETUP_ARMING_FAILSAFE_CUSTOM) && !(r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK))
				)
		     );

	should_always_enable_pwm = (r_setup_arming & PX4IO_P_SETUP_ARMING_ALWAYS_PWM_ENABLE)
		&& (r_status_flags & PX4IO_P_STATUS_FLAGS_INIT_OK)
		&& (r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK);

	/*
	 * Check if failsafe termination is set - if yes,
	 * set the force failsafe flag once entering the first
	 * failsafe condition.
	 */
	if (	/* if we have requested flight termination style failsafe (noreturn) */
			(r_setup_arming & PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE) &&
			/* and we ended up in a failsafe condition */
			(source == MIX_FAILSAFE) &&
			/* and we should be armed, so we intended to provide outputs */
			should_arm &&
			/* and FMU is initialized */
			(r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_INITIALIZED)) {
		r_setup_arming |= PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE;
	}

	/*
	 * Check if we should force failsafe - and do it if we have to
	 */
	if (r_setup_arming & PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE) {
		source = MIX_FAILSAFE;
	}

	/*
	 * Set failsafe status flag depending on mixing source
	 */
	if (source == MIX_FAILSAFE) {
		r_status_flags |= PX4IO_P_STATUS_FLAGS_FAILSAFE;
	} else {
		r_status_flags &= ~(PX4IO_P_STATUS_FLAGS_FAILSAFE);
	}

	/*
	 * Run the mixers.
	 */
	if (source == MIX_FAILSAFE) {

		/* copy failsafe values to the servo outputs */
		for (unsigned i = 0; i < PX4IO_SERVO_COUNT; i++) {
			r_page_servos[i] = r_page_servo_failsafe[i];

			/* safe actuators for FMU feedback */
			r_page_actuators[i] = FLOAT_TO_REG((r_page_servos[i] - 1500) / 600.0f);
		}


	} else if (source != MIX_NONE && (r_status_flags & PX4IO_P_STATUS_FLAGS_MIXER_OK)) {

		float	outputs[PX4IO_SERVO_COUNT];
		unsigned mixed;

		/* mix */

		/* poor mans mutex */
		in_mixer = true;
		mixed = mixer_group.mix(&outputs[0], PX4IO_SERVO_COUNT);
		in_mixer = false;

#if NONLINEAR == 1
		float desire_aoa[4] = {0.0};
		/* outputs -1.0~1.0 */
		map_desire_moment_to_aoa(outputs, desire_aoa, 4);
#endif
#if NONLINEAR == 1
		map_aoa_to_pwm(r_page_servo_disarmed, r_page_servo_control_min, r_page_servo_control_max, desire_aoa, r_page_servos);
#endif
		/* the pwm limit call takes care of out of band errors */
		pwm_limit_calc(should_arm, mixed, r_page_servo_disarmed, r_page_servo_control_min, r_page_servo_control_max, outputs, r_page_servos, &pwm_limit);
		for (unsigned i = mixed; i < PX4IO_SERVO_COUNT; i++)
			r_page_servos[i] = 0;

		for (unsigned i = 0; i < PX4IO_SERVO_COUNT; i++) {
			r_page_actuators[i] = FLOAT_TO_REG(outputs[i]);
		}
	}

	/* set arming */
	bool needs_to_arm = (should_arm || should_always_enable_pwm);

	/* check any conditions that prevent arming */
	if (r_setup_arming & PX4IO_P_SETUP_ARMING_LOCKDOWN) {
		needs_to_arm = false;
	}
	if (!should_arm && !should_always_enable_pwm) {
		needs_to_arm = false;
	}

	if (needs_to_arm && !mixer_servos_armed) {
		/* need to arm, but not armed */
		up_pwm_servo_arm(true);
		mixer_servos_armed = true;
		r_status_flags |= PX4IO_P_STATUS_FLAGS_OUTPUTS_ARMED;
		isr_debug(5, "> PWM enabled");

	} else if (!needs_to_arm && mixer_servos_armed) {
		/* armed but need to disarm */
		up_pwm_servo_arm(false);
		mixer_servos_armed = false;
		r_status_flags &= ~(PX4IO_P_STATUS_FLAGS_OUTPUTS_ARMED);
		isr_debug(5, "> PWM disabled");
	}

	if (mixer_servos_armed && should_arm) {
		/* update the servo outputs. */
		for (unsigned i = 0; i < PX4IO_SERVO_COUNT; i++)
			up_pwm_servo_set(i, r_page_servos[i]);

		/* set S.BUS1 or S.BUS2 outputs */

		if (r_setup_features & PX4IO_P_SETUP_FEATURES_SBUS2_OUT) {
			sbus2_output(r_page_servos, PX4IO_SERVO_COUNT);
		} else if (r_setup_features & PX4IO_P_SETUP_FEATURES_SBUS1_OUT) {
			sbus1_output(r_page_servos, PX4IO_SERVO_COUNT);
		}

	} else if (!mixer_servos_armed && should_always_enable_pwm) {
		/* set the disarmed servo outputs. */
		for (unsigned i = 0; i < PX4IO_SERVO_COUNT; i++)
			up_pwm_servo_set(i, r_page_servo_disarmed[i]);

		/* set S.BUS1 or S.BUS2 outputs */
		if (r_setup_features & PX4IO_P_SETUP_FEATURES_SBUS1_OUT)
			sbus1_output(r_page_servos, PX4IO_SERVO_COUNT);

		if (r_setup_features & PX4IO_P_SETUP_FEATURES_SBUS2_OUT)
			sbus2_output(r_page_servos, PX4IO_SERVO_COUNT);
	}
}

	static int
mixer_callback(uintptr_t handle,
		uint8_t control_group,
		uint8_t control_index,
		float &control)
{
	if (control_group >= PX4IO_CONTROL_GROUPS)
		return -1;

	switch (source) {
		case MIX_FMU:
			if (control_index < PX4IO_CONTROL_CHANNELS && control_group < PX4IO_CONTROL_GROUPS ) {
				control = REG_TO_FLOAT(r_page_controls[CONTROL_PAGE_INDEX(control_group, control_index)]);
				break;
			}
			return -1;

		case MIX_OVERRIDE:
			if (r_page_rc_input[PX4IO_P_RC_VALID] & (1 << CONTROL_PAGE_INDEX(control_group, control_index))) {
				control = REG_TO_FLOAT(r_page_rc_input[PX4IO_P_RC_BASE + control_index]);
				break;
			}
			return -1;

		case MIX_OVERRIDE_FMU_OK:
			/* FMU is ok but we are in override mode, use direct rc control for the available rc channels. The remaining channels are still controlled by the fmu */
			if (r_page_rc_input[PX4IO_P_RC_VALID] & (1 << CONTROL_PAGE_INDEX(control_group, control_index))) {
				control = REG_TO_FLOAT(r_page_rc_input[PX4IO_P_RC_BASE + control_index]);
				break;
			} else if (control_index < PX4IO_CONTROL_CHANNELS && control_group < PX4IO_CONTROL_GROUPS) {
				control = REG_TO_FLOAT(r_page_controls[CONTROL_PAGE_INDEX(control_group, control_index)]);
				break;
			}
			return -1;

		case MIX_FAILSAFE:
		case MIX_NONE:
			control = 0.0f;
			return -1;
	}

	return 0;
}

/*
 * XXX error handling here should be more aggressive; currently it is
 * possible to get STATUS_FLAGS_MIXER_OK set even though the mixer has
 * not loaded faithfully.
 */

static char mixer_text[256];		/* large enough for one mixer */
static unsigned mixer_text_length = 0;

	int
mixer_handle_text(const void *buffer, size_t length)
{
	/* do not allow a mixer change while safety off and FMU armed */
	if ((r_status_flags & PX4IO_P_STATUS_FLAGS_SAFETY_OFF) &&
			(r_setup_arming & PX4IO_P_SETUP_ARMING_FMU_ARMED)) {
		return 1;
	}

	/* disable mixing, will be enabled once load is complete */
	r_status_flags &= ~(PX4IO_P_STATUS_FLAGS_MIXER_OK);

	/* abort if we're in the mixer - the caller is expected to retry */
	if (in_mixer) {
		return 1;
	}

	px4io_mixdata	*msg = (px4io_mixdata *)buffer;

	isr_debug(2, "mix txt %u", length);

	if (length < sizeof(px4io_mixdata)) {
		return 0;
	}

	unsigned text_length = length - sizeof(px4io_mixdata);

	switch (msg->action) {
		case F2I_MIXER_ACTION_RESET:
			isr_debug(2, "reset");

			/* THEN actually delete it */
			mixer_group.reset();
			mixer_text_length = 0;

			/* FALLTHROUGH */
		case F2I_MIXER_ACTION_APPEND:
			isr_debug(2, "append %d", length);

			/* check for overflow - this would be really fatal */
			if ((mixer_text_length + text_length + 1) > sizeof(mixer_text)) {
				r_status_flags &= ~PX4IO_P_STATUS_FLAGS_MIXER_OK;
				return 0;
			}

			/* append mixer text and nul-terminate, guard against overflow */
			memcpy(&mixer_text[mixer_text_length], msg->text, text_length);
			mixer_text_length += text_length;
			mixer_text[mixer_text_length] = '\0';
			isr_debug(2, "buflen %u", mixer_text_length);

			/* process the text buffer, adding new mixers as their descriptions can be parsed */
			unsigned resid = mixer_text_length;
			mixer_group.load_from_buf(&mixer_text[0], resid);

			/* if anything was parsed */
			if (resid != mixer_text_length) {

				/* only set mixer ok if no residual is left over */
				if (resid == 0) {
					r_status_flags |= PX4IO_P_STATUS_FLAGS_MIXER_OK;
				} else {
					/* not yet reached the end of the mixer, set as not ok */
					r_status_flags &= ~PX4IO_P_STATUS_FLAGS_MIXER_OK;
				}

				isr_debug(2, "used %u", mixer_text_length - resid);

				/* copy any leftover text to the base of the buffer for re-use */
				if (resid > 0)
					memcpy(&mixer_text[0], &mixer_text[mixer_text_length - resid], resid);

				mixer_text_length = resid;

				/* update failsafe values */
				mixer_set_failsafe();
			}

			break;
	}

	return 0;
}

	static void
mixer_set_failsafe()
{
	/* 
	 * Check if a custom failsafe value has been written,
	 * or if the mixer is not ok and bail out.
	 */

	if ((r_setup_arming & PX4IO_P_SETUP_ARMING_FAILSAFE_CUSTOM) ||
			!(r_status_flags & PX4IO_P_STATUS_FLAGS_MIXER_OK))
		return;

	/* set failsafe defaults to the values for all inputs = 0 */
	float	outputs[PX4IO_SERVO_COUNT];
	unsigned mixed;

	/* mix */
	mixed = mixer_group.mix(&outputs[0], PX4IO_SERVO_COUNT);

	/* scale to PWM and update the servo outputs as required */
	for (unsigned i = 0; i < mixed; i++) {

		/* scale to servo output */
		r_page_servo_failsafe[i] = (outputs[i] * 600.0f) + 1500;

	}

	/* disable the rest of the outputs */
	for (unsigned i = mixed; i < PX4IO_SERVO_COUNT; i++)
		r_page_servo_failsafe[i] = 0;

}

#if NONLINEAR==1

int cmpfunc(const float* a, const float* b)
{
	if (*a > *b) {
		if (*a - *b < (*(b + 1) - *b)*(float)0.5)
			return 0;
		else
			return 1;
	}
	else {
		if (*b - *a < (*(b + 1) - *b)*(float)0.5)
			return 0;
		else
			return -1;
	}
}

float *bsearch(const float *key, const float *base, size_t num, size_t size,
		int (*cmp)(const float *key, const float *elt))
{
	size_t start = 0, end = num;
	int result;

	while (start < end) {
		size_t mid = start + (end - start) / 2;

		result = cmp(key, base + mid * size);
		if (result < 0)
			end = mid;
		else if (result > 0)
			start = mid + 1;
		else
			return (float *)base + mid * size;
	}

	return NULL;
}

static void map_desire_moment_to_aoa(float* normed_moment, float* desire_aoa, int mixed)
{
	int n = 0;
#define ZERO_AOA_MOMENT (float)0.4686
#define MAX_AOA_MOMENT 	(float)2.1
	for (n = 0; n < mixed; n++) {
		float desire_moment = normed_moment[n] * MAX_AOA_MOMENT + ZERO_AOA_MOMENT;
		float* nearest_mom = (float *)bsearch(&desire_moment, moment_aoa_table, 160, sizeof(float), cmpfunc);

		if (nearest_mom != NULL) {
			if (desire_moment < *nearest_mom) {
				desire_aoa[n] = ((desire_moment - *(nearest_mom - 1))/(*nearest_mom - *(nearest_mom-1))
						+(nearest_mom - moment_aoa_table)-80)*(float)0.25;
			} else {
				desire_aoa[n] = ((desire_moment - *nearest_mom)/(*(nearest_mom+1) - *nearest_mom)
						+(nearest_mom - moment_aoa_table)-80)*(float)0.25;
			}
		}
		else {
			desire_aoa[n] = (float)0.0;
		}
	}
}

static void map_aoa_to_pwm(const uint16_t *disarmed_pwm, const uint16_t *min_pwm, const uint16_t *max_pwm, const float* aoa, uint16_t* pwm_out)
{
#define AOA2PWM_RATIO (float)200.0
	int i = 0;
	for (i = 0; i < 4; i++) {
		pwm_out[i] = (uint16_t)(aoa[i]*AOA2PWM_RATIO + disarmed_pwm[i]);
		if (pwm_out[i] < min_pwm[i]) {
			pwm_out[i] = min_pwm[i];
		}
		else if(pwm_out[i] > max_pwm[i]) {
			pwm_out[i] = max_pwm[i];
		}
	}
}
#endif
