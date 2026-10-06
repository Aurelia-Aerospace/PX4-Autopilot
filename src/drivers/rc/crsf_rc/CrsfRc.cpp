/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "CrsfRc.hpp"
#include "CrsfParser.hpp"
#include "Crc8.hpp"

#include <fcntl.h>

#include <lib/geo/geo.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/wind.h>

using namespace time_literals;

#define CRSF_BAUDRATE 420000

CrsfRc::CrsfRc(const char *device) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(device))
{
	if (device) {
		strncpy(_device, device, sizeof(_device) - 1);
		_device[sizeof(_device) - 1] = '\0';
	}
}

CrsfRc::~CrsfRc()
{
	perf_free(_cycle_interval_perf);
	perf_free(_publish_interval_perf);
}

int CrsfRc::task_spawn(int argc, char *argv[])
{
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	const char *device_name = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_name = myoptarg;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return PX4_ERROR;
	}

	if (!device_name) {
		PX4_ERR("Valid device required");
		return PX4_ERROR;
	}

	CrsfRc *instance = new CrsfRc(device_name);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	instance->ScheduleNow();

	return PX4_OK;
}

void CrsfRc::Run()
{
	if (should_exit()) {
		ScheduleClear();

		if (_uart) {
			(void) _uart->close();
			delete _uart;
			_uart = nullptr;
		}

		exit_and_cleanup();
		return;
	}

	if (_uart == nullptr) {
		// Create the UART port instance
		_uart = new Serial(_device);

		if (_uart == nullptr) {
			PX4_ERR("Error creating serial device %s", _device);
			px4_sleep(1);
			return;
		}
	}

	if (! _uart->isOpen()) {
		// Configure the desired baudrate if one was specified by the user.
		// Otherwise the default baudrate will be used.
		if (! _uart->setBaudrate(CRSF_BAUDRATE)) {
			PX4_ERR("Error setting baudrate to %u on %s", CRSF_BAUDRATE, _device);
			px4_sleep(1);
			return;
		}

		// Open the UART. If this is successful then the UART is ready to use.
		if (! _uart->open()) {
			PX4_ERR("Error opening serial device  %s", _device);
			px4_sleep(1);
			return;
		}

		if (board_rc_swap_rxtx(_device)) {
			_uart->setSwapRxTxMode();
		}

		if (board_rc_singlewire(_device)) {
			_is_singlewire = true;
			_uart->setSingleWireMode();
		}

		PX4_INFO("Crsf serial opened sucessfully");

		if (_is_singlewire) {
			PX4_INFO("Crsf serial is single wire. Telemetry disabled");
		}

		_uart->flush();

		Crc8Init(0xd5);

		_input_rc.rssi_dbm = NAN;
		_input_rc.link_quality = -1;

		CrsfParser_Init();
	}

	const hrt_abstime time_now_us = hrt_absolute_time();
	perf_count_interval(_cycle_interval_perf, time_now_us);

	// Read all available data from the serial RC input UART
	int new_bytes = _uart->readAtLeast(&_rcs_buf[0], RC_MAX_BUFFER_SIZE, 1, 100);

	if (new_bytes > 0) {
		_bytes_rx += new_bytes;

		// Load new bytes into the CRSF parser buffer
		CrsfParser_LoadBuffer(_rcs_buf, new_bytes);

		// Scan the parse buffer for messages, one at a time
		CrsfPacket_t new_crsf_packet;

		while (CrsfParser_TryParseCrsfPacket(&new_crsf_packet, &_packet_parser_statistics)) {
			switch (new_crsf_packet.message_type) {
			case CRSF_MESSAGE_TYPE_RC_CHANNELS:
				_input_rc.timestamp_last_signal = time_now_us;
				_last_packet_seen = time_now_us;

				for (int i = 0; i < CRSF_CHANNEL_COUNT; i++) {
					_input_rc.values[i] = new_crsf_packet.channel_data.channels[i];
				}

				break;

			case CRSF_MESSAGE_TYPE_LINK_STATISTICS:
				_last_packet_seen = time_now_us;
				_input_rc.rssi_dbm = -(float)new_crsf_packet.link_statistics.uplink_rssi_1;
				_input_rc.link_quality = new_crsf_packet.link_statistics.uplink_link_quality;
				break;

			default:
				break;
			}
		}

		if (_param_rc_crsf_tel_en.get() && !_is_singlewire
		    && (_input_rc.timestamp > _telemetry_update_last + 100_ms)) {
			switch (_next_type) {
			case 0:
				battery_status_s battery_status;

				if (_battery_status_sub.update(&battery_status)) {
					uint16_t voltage = battery_status.voltage_filtered_v * 10;
					uint16_t current = battery_status.current_filtered_a * 10;
					int fuel = battery_status.discharged_mah;
					uint8_t remaining = battery_status.remaining * 100;
					this->SendTelemetryBattery(voltage, current, fuel, remaining);
					// 0x5007 paramId=4: batt1Capacity in mAh, used by yaapu for % calculation
					this->SendTelemetryPassthrough(0x5007, ((uint32_t)4 << 24) | (battery_status.capacity & 0xFFFFFF));

					// 0x5003: BATT1 passthrough (yaapu uses it for batt1volt/current/mah display)
					// voltage: bits0-8 in 0.1V (decivolts, max 51.1V); current: bit9=exp + bits10-16=man in dA; mah: bits17-31
					uint16_t volt_enc = (uint16_t)(battery_status.voltage_filtered_v * 10.0f);
					if (volt_enc > 511) { volt_enc = 511; }
					float cur_a = battery_status.current_filtered_a;
					uint8_t cur_exp = (cur_a * 10.0f > 127.0f) ? 1 : 0;
					uint8_t cur_man = cur_exp ? (uint8_t)(cur_a < 127.0f ? cur_a : 127.0f)
							           : (uint8_t)(cur_a * 10.0f);
					uint32_t mah_enc = (uint32_t)battery_status.discharged_mah;
					if (mah_enc > 32767) { mah_enc = 32767; }
					this->SendTelemetryPassthrough(0x5003,
								       (volt_enc & 0x1FF)
								       | ((uint32_t)cur_exp << 9)
								       | ((uint32_t)cur_man << 10)
								       | (mah_enc << 17));
				}

				break;

			case 1:
				sensor_gps_s sensor_gps;

				if (_vehicle_gps_position_sub.update(&sensor_gps)) {
					int32_t latitude = static_cast<int32_t>(round(sensor_gps.latitude_deg * 1e7));
					int32_t longitude = static_cast<int32_t>(round(sensor_gps.longitude_deg * 1e7));
					uint16_t groundspeed = sensor_gps.vel_d_m_s / 3.6f * 10.f;
					uint16_t gps_heading = math::degrees(sensor_gps.cog_rad) * 100.f;
					uint16_t altitude = static_cast<int16_t>(sensor_gps.altitude_msl_m * 1e3) + 1000;
					uint8_t num_satellites = sensor_gps.satellites_used;
					this->SendTelemetryGps(latitude, longitude, groundspeed, gps_heading, altitude, num_satellites);

					// 0x5002: GPS STATUS passthrough (yaapu-compatible)
					// fix_type: 0=no hw, 1=no fix, 2=2D, 3=3D, 4=DGPS, 5=RTK float, 6=RTK fixed
					uint8_t basic_fix, advanced_fix;

					if (sensor_gps.fix_type <= 1) {
						basic_fix = sensor_gps.fix_type; advanced_fix = 0;

					} else if (sensor_gps.fix_type == 2) {
						basic_fix = 2; advanced_fix = 0;

					} else if (sensor_gps.fix_type == 3) {
						basic_fix = 3; advanced_fix = 0;

					} else if (sensor_gps.fix_type == 4) {
						basic_fix = 3; advanced_fix = 1;

					} else if (sensor_gps.fix_type == 5) {
						basic_fix = 3; advanced_fix = 2;

					} else {
						basic_fix = 3; advanced_fix = 3;
					}

					// HDOP: bit6=exp(0→×1, 1→×10), bits7-13=mantissa, value in tenths
					uint32_t hdop_dm = (uint32_t)(sensor_gps.hdop * 10.0f);
					uint8_t hdop_exp = (hdop_dm > 127) ? 1 : 0;
					uint8_t hdop_man = hdop_exp ? (uint8_t)(sensor_gps.hdop < 127.0f ? sensor_gps.hdop : 127.0f)
							           : (uint8_t)hdop_dm;

					// GPS alt: bits22-23=exp(10^n dm), bits24-30=mantissa, bit31=sign
					float alt_m = (float)sensor_gps.altitude_msl_m;
					uint8_t alt_sign = (alt_m < 0.0f) ? 1 : 0;
					float alt_abs_dm = (alt_sign ? -alt_m : alt_m) * 10.0f;
					uint8_t alt_exp, alt_man;

					if (alt_abs_dm <= 127.0f) {
						alt_exp = 0; alt_man = (uint8_t)alt_abs_dm;

					} else if (alt_abs_dm <= 1270.0f) {
						alt_exp = 1; alt_man = (uint8_t)(alt_abs_dm / 10.0f);

					} else if (alt_abs_dm <= 12700.0f) {
						alt_exp = 2; alt_man = (uint8_t)(alt_abs_dm / 100.0f);

					} else {
						alt_exp = 3; alt_man = (uint8_t)(alt_abs_dm / 1000.0f < 127.0f ? alt_abs_dm / 1000.0f : 127.0f);
					}

					uint32_t val_5002 = (uint32_t)(num_satellites & 0xF)
							    | ((uint32_t)(basic_fix & 0x3) << 4)
							    | ((uint32_t)hdop_exp << 6)
							    | ((uint32_t)hdop_man << 7)
							    | ((uint32_t)(advanced_fix & 0x3) << 14)
							    | ((uint32_t)alt_exp << 22)
							    | ((uint32_t)alt_man << 24)
							    | ((uint32_t)alt_sign << 31);
					this->SendTelemetryPassthrough(0x5002, val_5002);
				}

				break;

			case 2:
				vehicle_attitude_s vehicle_attitude;

				if (_vehicle_attitude_sub.update(&vehicle_attitude)) {
					matrix::Eulerf attitude = matrix::Quatf(vehicle_attitude.q);
					int16_t pitch = attitude(1) * 1e4f;
					int16_t roll = attitude(0) * 1e4f;
					int16_t yaw = attitude(2) * 1e4f;
					this->SendTelemetryAttitude(pitch, roll, yaw);

					// 0x5006: ROLLPITCH — bits0-10: roll+900*5, bits11-20: pitch+450*5
					float roll_deg  = math::degrees(attitude(0));
					float pitch_deg = math::degrees(attitude(1));
					int roll_enc  = (int)(roll_deg  * 5.0f + 900.5f);
					int pitch_enc = (int)(pitch_deg * 5.0f + 450.5f);
					if (roll_enc  < 0) { roll_enc  = 0; } if (roll_enc  > 1800) { roll_enc  = 1800; }
					if (pitch_enc < 0) { pitch_enc = 0; } if (pitch_enc > 900)  { pitch_enc = 900;  }
					this->SendTelemetryPassthrough(0x5006,
								       (roll_enc & 0x7FF)
								       | ((uint32_t)(pitch_enc & 0x3FF) << 11));

					// 0x5005: VELANDYAW — vspeed from GPS vel_d, hspeed from GPS vel_m_s, yaw from attitude
					sensor_gps_s gps_copy{};
					_vehicle_gps_position_sub.copy(&gps_copy);
					float vspeed_ms = -gps_copy.vel_d_m_s; // positive = climbing (NED sign flip)
					float vspeed_abs_dm = (vspeed_ms < 0.0f ? -vspeed_ms : vspeed_ms) * 10.0f;
					uint8_t vs_sign = (vspeed_ms < 0.0f) ? 1 : 0;
					uint8_t vs_exp = (vspeed_abs_dm > 127.0f) ? 1 : 0;
					float vspeed_abs_ms = vspeed_ms < 0.0f ? -vspeed_ms : vspeed_ms;
					uint8_t vs_man = vs_exp ? (uint8_t)(vspeed_abs_ms < 127.0f ? vspeed_abs_ms : 127.0f)
							          : (uint8_t)vspeed_abs_dm;
					float hspeed_dm = gps_copy.vel_m_s * 10.0f;
					uint8_t hs_exp = (hspeed_dm > 127.0f) ? 1 : 0;
					uint8_t hs_man = hs_exp ? (uint8_t)(gps_copy.vel_m_s < 127.0f ? gps_copy.vel_m_s : 127.0f)
							          : (uint8_t)hspeed_dm;
					float yaw_deg = math::degrees(attitude(2));
					if (yaw_deg < 0.0f) { yaw_deg += 360.0f; }
					uint16_t yaw_enc = (uint16_t)(yaw_deg / 0.2f);
					if (yaw_enc > 1799) { yaw_enc = 1799; }
					this->SendTelemetryPassthrough(0x5005,
								       (uint32_t)vs_exp
								       | ((uint32_t)vs_man  << 1)
								       | ((uint32_t)vs_sign << 8)
								       | ((uint32_t)hs_exp  << 9)
								       | ((uint32_t)hs_man  << 10)
								       | ((uint32_t)yaw_enc << 17));
				}

				break;

			case 3:
				vehicle_status_s vehicle_status;

				if (_vehicle_status_sub.update(&vehicle_status)) {
					const char *flight_mode = "(unknown)";

					switch (vehicle_status.nav_state) {
					case vehicle_status_s::NAVIGATION_STATE_MANUAL:
						flight_mode = "Manual";
						break;

					case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
						flight_mode = "Altitude";
						break;

					case vehicle_status_s::NAVIGATION_STATE_POSCTL:
						flight_mode = "Position";
						break;

					case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
						flight_mode = "Return";
						break;

					case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
						flight_mode = "Mission";
						break;

					case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
					case vehicle_status_s::NAVIGATION_STATE_DESCEND:
					case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
					case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
					case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
					case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
						flight_mode = "Auto";
						break;

					/*case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL:
						flight_mode = "Failure";
						break;*/

					case vehicle_status_s::NAVIGATION_STATE_ACRO:
						flight_mode = "Acro";
						break;

					case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
						flight_mode = "Terminate";
						break;

					case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
						flight_mode = "Offboard";
						break;

					case vehicle_status_s::NAVIGATION_STATE_STAB:
						flight_mode = "Stabilized";
						break;

					default:
						flight_mode = "Unknown";
					}

					this->SendTelemetryFlightMode(flight_mode);

					// 0x5007: PARAMS — frameType (paramId=1), triggers flight mode table load in yaapu
					// vehicle_type: 1=rotary, 2=fixed_wing, 3=rover → yaapu frameTypes: 0=copter, 1=plane, 10=rover
					uint8_t frame_type_val;

					switch (vehicle_status.vehicle_type) {
					case vehicle_status_s::VEHICLE_TYPE_FIXED_WING: frame_type_val = 1;  break;
					case vehicle_status_s::VEHICLE_TYPE_ROVER:      frame_type_val = 10; break;
					default:                                         frame_type_val = 0;  break; // copter
					}

					this->SendTelemetryPassthrough(0x5007, (1u << 24) | frame_type_val);

					// 0x5001: AP STATUS — armed bit + flight mode index for copter_px4.lua / plane_px4.lua
					uint8_t armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) ? 1 : 0;
					uint8_t fm_index;

					switch (vehicle_status.nav_state) {
					case vehicle_status_s::NAVIGATION_STATE_MANUAL:          fm_index = 0;  break;
					case vehicle_status_s::NAVIGATION_STATE_ALTCTL:          fm_index = 1;  break;
					case vehicle_status_s::NAVIGATION_STATE_POSCTL:          fm_index = 2;  break;
					case vehicle_status_s::NAVIGATION_STATE_ACRO:            fm_index = 3;  break;
					case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:        fm_index = 4;  break;
					case vehicle_status_s::NAVIGATION_STATE_STAB:            fm_index = 5;  break;
					case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:    fm_index = 13; break;
					case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:     fm_index = 14; break;
					case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:    fm_index = 15; break;
					case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:        fm_index = 16; break;
					case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
					case vehicle_status_s::NAVIGATION_STATE_DESCEND:         fm_index = 17; break;
					case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET: fm_index = 19; break;
					case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:   fm_index = 20; break;
					default:                                                  fm_index = 31; break;
					}

					// throttle: bits19-24=magnitude(0-63), bit25=sign; RC stick -1..1 mapped to 0-100%
					manual_control_setpoint_s manual_control{};
					_manual_control_setpoint_sub.copy(&manual_control);
					float thr_pct = (manual_control.throttle + 1.0f) * 50.0f;
					if (thr_pct < 0.0f) { thr_pct = 0.0f; }
					if (thr_pct > 100.0f) { thr_pct = 100.0f; }
					uint8_t thr6 = (uint8_t)(thr_pct * (63.0f / 100.0f));

					this->SendTelemetryPassthrough(0x5001,
								       (fm_index & 0x1F)
								       | ((uint32_t)armed << 8)
								       | ((uint32_t)thr6 << 19));
				}

				break;

			case 4: {
				// 0x5004: HOME — distance, relative altitude and bearing to home
				sensor_gps_s gps4{};
				home_position_s home{};

				if (_vehicle_gps_position_sub.copy(&gps4) && _home_position_sub.copy(&home) && home.valid_alt) {
					float dist_xy, dist_z;
					get_distance_to_point_global_wgs84(
						gps4.latitude_deg, gps4.longitude_deg, (float)gps4.altitude_msl_m,
						home.lat, home.lon, home.alt,
						&dist_xy, &dist_z);

					// distance encoding: bits0-1=exp, bits2-11=mantissa (meters)
					uint8_t d_exp; uint16_t d_man;
					if      (dist_xy <= 1023.0f)       { d_exp = 0; d_man = (uint16_t)dist_xy; }
					else if (dist_xy / 10.0f <= 1023.0f) { d_exp = 1; d_man = (uint16_t)(dist_xy / 10.0f); }
					else if (dist_xy / 100.0f <= 1023.0f){ d_exp = 2; d_man = (uint16_t)(dist_xy / 100.0f); }
					else                                { d_exp = 3; d_man = (uint16_t)(dist_xy / 1000.0f < 1023.0f ? dist_xy / 1000.0f : 1023.0f); }

					// homeAlt: bits12-13=exp, bits14-23=mantissa (mantissa*10^exp*0.1 = meters), bit24=sign
					float home_alt_m = (float)gps4.altitude_msl_m - home.alt;
					float alt_abs_dm = (home_alt_m < 0.0f ? -home_alt_m : home_alt_m) * 10.0f;
					uint8_t alt_sign = (home_alt_m < 0.0f) ? 1 : 0;
					uint8_t a_exp; uint16_t a_man;
					if      (alt_abs_dm <= 1023.0f)        { a_exp = 0; a_man = (uint16_t)alt_abs_dm; }
					else if (alt_abs_dm / 10.0f <= 1023.0f){ a_exp = 1; a_man = (uint16_t)(alt_abs_dm / 10.0f); }
					else if (alt_abs_dm / 100.0f <= 1023.0f){ a_exp = 2; a_man = (uint16_t)(alt_abs_dm / 100.0f); }
					else                                   { a_exp = 3; a_man = (uint16_t)(alt_abs_dm / 1000.0f < 1023.0f ? alt_abs_dm / 1000.0f : 1023.0f); }

					// bearing: bits25-31 = bearing_deg/3 (7 bits, 0-127 → 0-381°)
					float bearing_rad = get_bearing_to_next_waypoint(gps4.latitude_deg, gps4.longitude_deg,
										    home.lat, home.lon);
					float bearing_deg = math::degrees(bearing_rad);
					if (bearing_deg < 0.0f) { bearing_deg += 360.0f; }
					uint8_t angle_enc = (uint8_t)(bearing_deg / 3.0f);

					this->SendTelemetryPassthrough(0x5004,
								       (uint32_t)d_exp
								       | ((uint32_t)d_man   << 2)
								       | ((uint32_t)a_exp   << 12)
								       | ((uint32_t)a_man   << 14)
								       | ((uint32_t)alt_sign << 24)
								       | ((uint32_t)angle_enc << 25));
				}

				break;
			}

			case 5: {
				// 0x5009 + 0x500D: WAYPOINTS v1 and v2
				sensor_gps_s gps5{};
				mission_result_s mission{};
				position_setpoint_triplet_s sp_triplet{};
				_vehicle_gps_position_sub.copy(&gps5);
				_mission_result_sub.copy(&mission);
				_position_setpoint_triplet_sub.copy(&sp_triplet);

				uint16_t wp_num = mission.seq_current;
				uint8_t d_exp = 0; uint16_t d_man = 0;
				uint8_t bearing_enc = 0;

				if (sp_triplet.current.valid) {
					float wp_dist_xy, wp_dist_z;
					get_distance_to_point_global_wgs84(
						gps5.latitude_deg, gps5.longitude_deg, (float)gps5.altitude_msl_m,
						sp_triplet.current.lat, sp_triplet.current.lon, sp_triplet.current.alt,
						&wp_dist_xy, &wp_dist_z);
					if      (wp_dist_xy <= 1023.0f)        { d_exp = 0; d_man = (uint16_t)wp_dist_xy; }
					else if (wp_dist_xy / 10.0f <= 1023.0f){ d_exp = 1; d_man = (uint16_t)(wp_dist_xy / 10.0f); }
					else if (wp_dist_xy / 100.0f <= 1023.0f){ d_exp = 2; d_man = (uint16_t)(wp_dist_xy / 100.0f); }
					else                                   { d_exp = 3; d_man = (uint16_t)(wp_dist_xy / 1000.0f < 1023.0f ? wp_dist_xy / 1000.0f : 1023.0f); }

					float brg_rad = get_bearing_to_next_waypoint(gps5.latitude_deg, gps5.longitude_deg,
										     sp_triplet.current.lat, sp_triplet.current.lon);
					float brg_deg = math::degrees(brg_rad);
					if (brg_deg < 0.0f) { brg_deg += 360.0f; }
					bearing_enc = (uint8_t)(brg_deg / 3.0f); // 7 bits (0-119)

					// 0x5009 v1: cog offset = bearing to WP relative to current COG, 45° resolution
					float cog_deg = math::degrees(gps5.cog_rad);
					if (cog_deg < 0.0f) { cog_deg += 360.0f; }
					float offset = brg_deg - cog_deg;
					if (offset < 0.0f) { offset += 360.0f; }
					uint8_t cog_offset = (uint8_t)(offset / 45.0f) & 0x7;
					this->SendTelemetryPassthrough(0x5009,
								       (uint32_t)(wp_num & 0x3FF)
								       | ((uint32_t)d_exp << 10)
								       | ((uint32_t)d_man << 12)
								       | ((uint32_t)cog_offset << 29));
				}

				// 0x500D v2: absolute bearing
				this->SendTelemetryPassthrough(0x500D,
							       (uint32_t)(wp_num & 0x7FF)
							       | ((uint32_t)d_exp << 11)
							       | ((uint32_t)d_man << 13)
							       | ((uint32_t)bearing_enc << 23));

				break;
			}

			case 6: {
				// 0x500C: WIND — true wind speed/direction from EKF2 wind estimator
				wind_s wind{};

				if (_wind_sub.copy(&wind)) {
					float spd_ms = sqrtf(wind.windspeed_north * wind.windspeed_north
							     + wind.windspeed_east  * wind.windspeed_east);
					float spd_dm = spd_ms * 10.0f;
					uint8_t ws_exp = (spd_dm > 127.0f) ? 1 : 0;
					uint8_t ws_man = ws_exp ? (uint8_t)(spd_ms < 127.0f ? spd_ms : 127.0f)
							          : (uint8_t)spd_dm;
					float angle_deg = math::degrees(atan2f(wind.windspeed_east, wind.windspeed_north));
					if (angle_deg < 0.0f) { angle_deg += 360.0f; }
					uint8_t angle_enc = (uint8_t)(angle_deg / 3.0f); // 7 bits
					this->SendTelemetryPassthrough(0x500C,
								       (uint32_t)angle_enc
								       | ((uint32_t)ws_exp << 7)
								       | ((uint32_t)ws_man << 8));
				}

				break;
			}
			}

			_telemetry_update_last = _input_rc.timestamp;
			_next_type = (_next_type + 1) % num_data_types;
		}
	}

	// If no communication
	if (time_now_us - _last_packet_seen > 100_ms) {
		// Invalidate link statistics
		_input_rc.rssi_dbm = NAN;
		_input_rc.link_quality = -1;
	}

	// If we have not gotten RC updates specifically
	if (time_now_us - _input_rc.timestamp_last_signal > 50_ms) {
		_input_rc.rc_lost = 1;
		_input_rc.rc_failsafe = 1;

	} else {
		_input_rc.rc_lost = 0;
		_input_rc.rc_failsafe = 0;
	}

	_input_rc.channel_count = CRSF_CHANNEL_COUNT;
	_input_rc.rssi = -1;
	_input_rc.rc_ppm_frame_length = 0;
	_input_rc.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_CRSF;
	_input_rc.timestamp = hrt_absolute_time();
	_input_rc_pub.publish(_input_rc);

	perf_count(_publish_interval_perf);

	// 0x5000: GCS messages — event-driven, independent of the round-robin telemetry timer
	if (_param_rc_crsf_tel_en.get() && !_is_singlewire) {
		mavlink_log_s log_msg;

		if (_mavlink_log_sub.update(&log_msg) && log_msg.severity <= 6 // 0=EMERG..6=INFO, skip 7=DEBUG
		    && log_msg.text[0] != '[') {                                // skip internal [module] messages
			this->SendTelemetryMessage(log_msg.severity, log_msg.text);
		}
	}

	ScheduleDelayed(4_ms);
}

/**
 * write an uint8_t value to a buffer at a given offset and increment the offset
 */
static inline void write_uint8_t(uint8_t *buf, int &offset, uint8_t value)
{
	buf[offset++] = value;
}

/**
 * write an uint16_t value to a buffer at a given offset and increment the offset
 */
static inline void write_uint16_t(uint8_t *buf, int &offset, uint16_t value)
{
	// Big endian
	buf[offset] = value >> 8;
	buf[offset + 1] = value & 0xff;
	offset += 2;
}

/**
 * write an uint24_t value to a buffer at a given offset and increment the offset
 */
static inline void write_uint24_t(uint8_t *buf, int &offset, int value)
{
	// Big endian
	buf[offset] = value >> 16;
	buf[offset + 1] = (value >> 8) & 0xff;
	buf[offset + 2] = value & 0xff;
	offset += 3;
}

/**
 * write an int32_t value to a buffer at a given offset and increment the offset
 */
static inline void write_int32_t(uint8_t *buf, int &offset, int32_t value)
{
	// Big endian
	buf[offset] = value >> 24;
	buf[offset + 1] = (value >> 16) & 0xff;
	buf[offset + 2] = (value >> 8) & 0xff;
	buf[offset + 3] = value & 0xff;
	offset += 4;
}

void CrsfRc::WriteFrameHeader(uint8_t *buf, int &offset, const crsf_frame_type_t type, const uint8_t payload_size)
{
	write_uint8_t(buf, offset, 0xc8); // this got changed from the address to the sync byte
	write_uint8_t(buf, offset, payload_size + 2);
	write_uint8_t(buf, offset, (uint8_t)type);
}

void CrsfRc::WriteFrameCrc(uint8_t *buf, int &offset, const int buf_size)
{
	// CRC does not include the address and length
	write_uint8_t(buf, offset, Crc8Calc(buf + 2, buf_size - 3));
}

bool CrsfRc::SendTelemetryBattery(const uint16_t voltage, const uint16_t current, const int fuel,
				  const uint8_t remaining)
{
	uint8_t buf[(uint8_t)crsf_payload_size_t::battery_sensor + 4];
	int offset = 0;
	WriteFrameHeader(buf, offset, crsf_frame_type_t::battery_sensor, (uint8_t)crsf_payload_size_t::battery_sensor);
	write_uint16_t(buf, offset, voltage);
	write_uint16_t(buf, offset, current);
	write_uint24_t(buf, offset, fuel);
	write_uint8_t(buf, offset, remaining);
	WriteFrameCrc(buf, offset, sizeof(buf));
	return _uart->write((void *) buf, (size_t) offset);

}

bool CrsfRc::SendTelemetryGps(const int32_t latitude, const int32_t longitude, const uint16_t groundspeed,
			      const uint16_t gps_heading, const uint16_t altitude, const uint8_t num_satellites)
{
	uint8_t buf[(uint8_t)crsf_payload_size_t::gps + 4];
	int offset = 0;
	WriteFrameHeader(buf, offset, crsf_frame_type_t::gps, (uint8_t)crsf_payload_size_t::gps);
	write_int32_t(buf, offset, latitude);
	write_int32_t(buf, offset, longitude);
	write_uint16_t(buf, offset, groundspeed);
	write_uint16_t(buf, offset, gps_heading);
	write_uint16_t(buf, offset, altitude);
	write_uint8_t(buf, offset, num_satellites);
	WriteFrameCrc(buf, offset, sizeof(buf));
	return _uart->write((void *) buf, (size_t) offset);
}

bool CrsfRc::SendTelemetryAttitude(const int16_t pitch, const int16_t roll, const int16_t yaw)
{
	uint8_t buf[(uint8_t)crsf_payload_size_t::attitude + 4];
	int offset = 0;
	WriteFrameHeader(buf, offset, crsf_frame_type_t::attitude, (uint8_t)crsf_payload_size_t::attitude);
	write_uint16_t(buf, offset, pitch);
	write_uint16_t(buf, offset, roll);
	write_uint16_t(buf, offset, yaw);
	WriteFrameCrc(buf, offset, sizeof(buf));
	return _uart->write((void *) buf, (size_t) offset);
}

bool CrsfRc::SendTelemetryFlightMode(const char *flight_mode)
{
	const int max_length = 16;
	int length = strlen(flight_mode) + 1;

	if (length > max_length) {
		length = max_length;
	}

	uint8_t buf[max_length + 4];
	int offset = 0;
	WriteFrameHeader(buf, offset, crsf_frame_type_t::flight_mode, length);
	memcpy(buf + offset, flight_mode, length);
	offset += length;
	buf[offset - 1] = 0; // ensure null-terminated string
	WriteFrameCrc(buf, offset, length + 4);
	return _uart->write((void *) buf, (size_t) offset);
}

bool CrsfRc::SendTelemetryPassthrough(uint16_t app_id, uint32_t value)
{
	// payload: [0xF0][app_id_lo][app_id_hi][v0][v1][v2][v3] = 7 bytes
	// frame:   [sync][len][0x80][payload...][CRC] = 11 bytes total
	uint8_t buf[11];
	int offset = 0;
	WriteFrameHeader(buf, offset, crsf_frame_type_t::ap_custom_telem, 7);
	write_uint8_t(buf, offset, 0xF0);
	write_uint8_t(buf, offset, app_id & 0xFF);
	write_uint8_t(buf, offset, (app_id >> 8) & 0xFF);
	write_uint8_t(buf, offset, value & 0xFF);
	write_uint8_t(buf, offset, (value >> 8) & 0xFF);
	write_uint8_t(buf, offset, (value >> 16) & 0xFF);
	write_uint8_t(buf, offset, (value >> 24) & 0xFF);
	WriteFrameCrc(buf, offset, sizeof(buf));
	return _uart->write((void *) buf, (size_t) offset);
}

bool CrsfRc::SendTelemetryMessage(uint8_t severity, const char *text)
{
	// 0x5000: send message as consecutive 0xF0 passthrough packets, 4 x 7-bit chars per packet.
	// Each byte: [severity_bit:1][char:7]. Severity is a 3-bit value split across bits 7,15,23.
	// Last packet is the one that contains the null terminator — yaapu assembles from MSB to LSB.
	const uint8_t sev0 = (severity >> 0) & 1;
	const uint8_t sev1 = (severity >> 1) & 1;
	const uint8_t sev2 = (severity >> 2) & 1;
	bool ok = true;
	int i = 0;
	bool is_last = false;

	do {
		uint8_t chars[4] = {0, 0, 0, 0};
		is_last = false;

		for (int j = 0; j < 4; j++) {
			if (text[i] == '\0') {
				is_last = true;
				break;
			}

			chars[j] = (uint8_t)(text[i] & 0x7F);
			i++;
		}

		// Pack 4 bytes MSB-first. Severity bits in bit7 of the last packet only.
		uint32_t val = ((uint32_t)(chars[3] | (is_last ? sev0 << 7 : 0)) <<  0)
			       | ((uint32_t)(chars[2] | (is_last ? sev1 << 7 : 0)) <<  8)
			       | ((uint32_t)(chars[1] | (is_last ? sev2 << 7 : 0)) << 16)
			       | ((uint32_t) chars[0]                               << 24);
		ok &= this->SendTelemetryPassthrough(0x5000, val);

	} while (!is_last && i < 127);

	return ok;
}

int CrsfRc::print_status()
{
	if (_device[0] != '\0') {
		PX4_INFO("UART device: %s", _device);
		PX4_INFO("UART RX bytes: %"  PRIu32, _bytes_rx);
	}

	if (_is_singlewire) {
		PX4_INFO("Telemetry disabled: Singlewire RC port");

	} else {
		PX4_INFO("Telemetry: %s", _param_rc_crsf_tel_en.get() ? "yes" : "no");
	}

	perf_print_counter(_cycle_interval_perf);
	perf_print_counter(_publish_interval_perf);

	PX4_INFO_RAW("Disposed bytes: %" PRIu32 "\n", _packet_parser_statistics.disposed_bytes);
	PX4_INFO_RAW("Valid known packet CRCs: %" PRIu32 "\n", _packet_parser_statistics.crcs_valid_known_packets);
	PX4_INFO_RAW("Valid unknown packet CRCs: %" PRIu32 "\n", _packet_parser_statistics.crcs_valid_unknown_packets);
	PX4_INFO_RAW("Invalid CRCs: %" PRIu32 "\n", _packet_parser_statistics.crcs_invalid);
	PX4_INFO_RAW("Invalid known packet sizes: %" PRIu32 "\n", _packet_parser_statistics.invalid_known_packet_sizes);
	PX4_INFO_RAW("Invalid unknown packet sizes: %" PRIu32 "\n", _packet_parser_statistics.invalid_unknown_packet_sizes);

	return 0;
}

int CrsfRc::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int CrsfRc::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module parses the CRSF RC uplink protocol and generates CRSF downlink telemetry data

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("crsf_rc", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "<file:dev>", "RC device", true);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int crsf_rc_main(int argc, char *argv[])
{
	return CrsfRc::main(argc, argv);
}
