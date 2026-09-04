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

	 bool failsafe = false;

	 if (context.status().open_drone_id_system_healthy!=fldsmdfr_status_s::AURELIA_CHECK_STATUS_GOOD_TO_ARM) {
		const hrt_abstime now = hrt_absolute_time();
		_fldsmdfr_status_sub.update(&fldsmdfr_status);
		 //Me quede agregando todo lo de commander
		 switch(context.status().open_drone_id_system_healthy){
			 case fldsmdfr_status_s::AURELIA_CHECK_STATUS_FAIL_GENERIC:
			 case fldsmdfr_status_s::AURELIA_CHECK_STATUS_FAIL_GPS:
					/* EVENT
					* @description
					* FLDSMDFR failure
					*/
					reporter.armingCheckFailure(affected_modes, health_component_t::open_drone_id,
						events::ID("fldsmdfr_failure"),
						events::Log::Error, "FLDSMDFR: Not ready");

					if (reporter.mavlink_log_pub() && hrt_elapsed_time(&_last_warning_message) > 5_s) {
						mavlink_log_critical(reporter.mavlink_log_pub(), "FLDSMDFR: %s", fldsmdfr_status.error);
						_last_warning_message = now;

					}

			 break;
			 case fldsmdfr_status_s::AURELIA_CHECK_STATUS_FAIL_FLYING_NOT_ALLOWED:
					/* EVENT
					* @description
					* FLDSMDFR Flying not allowed
					*/
					reporter.armingCheckFailure(affected_modes, health_component_t::open_drone_id,
						events::ID("fldsmdfr_fna"),
						events::Log::Error, "FLDSMDFR: Flying not allowed");

					if (reporter.mavlink_log_pub() && hrt_elapsed_time(&_last_warning_message) > 5_s) {
						mavlink_log_critical(reporter.mavlink_log_pub(), "FLDSMDFR: %s", fldsmdfr_status.error);
						_last_warning_message = now;
					}


				 failsafe = true;
			 break;
			 case fldsmdfr_status_s::AURELIA_CHECK_STATUS_FAIL_LOST_MODULE:

					/* EVENT
					* @description
					* FLDSMDFR system failed to report. Make sure it is setup and installed properly.
					*/
					reporter.armingCheckFailure(affected_modes, health_component_t::open_drone_id,
					events::ID("fldsmdfr_missing"),
						events::Log::Error, "FLDSMDFR system missing");
					if (reporter.mavlink_log_pub() && hrt_elapsed_time(&_last_warning_message) > 5_s) {
						mavlink_log_critical(reporter.mavlink_log_pub(), "FLDSMDFR: system missing");
						_last_warning_message = now;
					}
			 break;
			 default:
			 break;
		 }
	 }
	 reporter.failsafeFlags().fldsmdfr_flying_not_allowed = failsafe;
 }
