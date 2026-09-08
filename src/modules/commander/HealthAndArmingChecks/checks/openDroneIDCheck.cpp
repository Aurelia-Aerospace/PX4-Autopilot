/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "openDroneIDCheck.hpp"
#include <drivers/drv_hrt.h>

using namespace time_literals;

void OpenDroneIDChecks::checkAndReport(const Context &context, Report &reporter)
{
	// Check to see if the check has been disabled
	if (!_param_com_arm_odid.get()) {
		return;
	}

	NavModes affected_modes{NavModes::None};

	if (_param_com_arm_odid.get() == 2) {
		// disallow arming without the Open Drone ID system
		affected_modes = NavModes::All;
	}

	if (!context.status().open_drone_id_system_present) {
		/* EVENT
		 * @description
		 * Open Drone ID system failed to report. Make sure it is setup and installed properly.
		 *
		 * <profile name="dev">
		 * This check can be configured via <param>COM_ARM_ODID</param> parameter.
		 * </profile>
		 */
		reporter.armingCheckFailure(affected_modes, health_component_t::open_drone_id,
					    events::ID("check_open_drone_id_missing"),
					    events::Log::Error, "Open Drone ID system missing");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Open Drone ID system missing");
		}

	} else if (!context.status().open_drone_id_system_healthy) {
		/* EVENT
		 * @description
		 * Open Drone ID system reported being unhealthy.
		 *
		 * <profile name="dev">
		 * This check can be configured via <param>COM_ARM_ODID</param> parameter.
		 * </profile>
		 */
		reporter.armingCheckFailure(affected_modes, health_component_t::open_drone_id,
					    events::ID("check_open_drone_id_unhealthy"),
					    events::Log::Error, "Open Drone ID system not ready");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Open Drone ID system not ready");
		}
	}

	// In-flight: check ODID module flying_allowed status
	bool flying_not_allowed = false;

	if (_param_com_odid_fs_act.get() > 0) {
		aurelia_odid_status_s aurelia_status{};

		if (_aurelia_odid_status_sub.copy(&aurelia_status)
		    && hrt_elapsed_time(&aurelia_status.timestamp) < 5_s) {
			// MAV_AURELIA_CHECK_STATUS_FAIL_FLYING_NOT_ALLOWED == 3
			flying_not_allowed = (aurelia_status.status == 3);
		}
	}

	reporter.failsafeFlags().odid_flying_not_allowed = flying_not_allowed;

	if (flying_not_allowed && reporter.mavlink_log_pub()) {
		mavlink_log_critical(reporter.mavlink_log_pub(), "ODID: flying not allowed");
	}
}
