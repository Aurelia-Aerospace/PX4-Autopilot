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

#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_status.h>

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

					uint32_t val_5002 = (uint32_t)(num_satellites & 0xF)
							    | ((uint32_t)(basic_fix & 0x3) << 4)
							    | ((uint32_t)(advanced_fix & 0x3) << 14);
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

					this->SendTelemetryPassthrough(0x5001, ((uint32_t)armed << 8) | (fm_index & 0x1F));
				}

				break;
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
