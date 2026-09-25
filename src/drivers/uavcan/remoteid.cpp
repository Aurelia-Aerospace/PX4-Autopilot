/****************************************************************************
 *
 *   Copyright (C) 2024 PX4 Development Team. All rights reserved.
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

#include "remoteid.hpp"
#include <modules/mavlink/open_drone_id_translations.hpp>
#include <drivers/drv_hrt.h>
#include <fcntl.h>
#include <unistd.h>

#include <px4_platform_common/tasks.h>

#ifdef PX4_CRYPTO
#include <optional/monocypher-ed25519.h>
#include <px4_random.h>
#include <keystore_backend_definitions.h>
#include <lib/parameters/param_security.hpp>
#include <image_toc.h>
#include <stddef.h>
#include <nuttx/progmem.h>
#include <sys/stat.h>
#include <px4_platform_common/board_common.h>
#include <px4_platform_common/log.h>
extern "C" {
keystore_session_handle_t keystore_open(void);
void                      keystore_close(keystore_session_handle_t *handle);
size_t                    keystore_get_key(keystore_session_handle_t handle, uint8_t idx, uint8_t *key_buf, size_t key_buf_size);

static int bl_update_main(int argc, char *argv[])
{
	if (argc < 2) { return 1; }

	const char *path = argv[1];

	struct stat s;
	if (stat(path, &s) != 0) { return 1; }
	if (s.st_size > 128 * 1024) { return 1; }

	const size_t page_sz = up_progmem_pagesize(0);
	const size_t img_sz  = ((size_t)s.st_size + page_sz - 1) & ~(page_sz - 1);

	uint8_t *buf = (uint8_t *)malloc(img_sz);
	if (!buf) { return 1; }
	memset(buf, 0xff, img_sz);

	int fd = open(path, O_RDONLY);
	const bool ok = (fd >= 0) && (read(fd, buf, s.st_size) == (ssize_t)s.st_size);
	if (fd >= 0) { close(fd); }
	if (!ok) { free(buf); return 1; }

	sched_lock();
	ssize_t erase_ret = up_progmem_eraseblock(0);
	ssize_t write_ret = up_progmem_write(0x08000000, buf, img_sz);
	sched_unlock();

	const uint8_t *flash = (const uint8_t *)0x08000000;
	bool verified = (erase_ret >= 0) && (write_ret == (ssize_t)img_sz);
	for (size_t i = 0; i < img_sz && verified; i++) {
		if (flash[i] != buf[i]) { verified = false; }
	}
	free(buf);

	if (verified) {
		px4_usleep(500000);
		board_reset(0);
	}
	return verified ? 0 : 1;
}
}
#endif

using namespace time_literals;

// ponytail: SD-backed key file — upgrade to dedicated MTD partition if SD-less operation is required
static constexpr const char *RID_KEY_PATH = "/fs/microsd/rid_pubkey.bin";
static constexpr size_t RID_KEY_LEN = 32; // Ed25519 public key

static int rid_key_read(uint8_t *buf, size_t len)
{
	int fd = open(RID_KEY_PATH, O_RDONLY);
	if (fd < 0) { return -1; }
	ssize_t n = read(fd, buf, len);
	close(fd);
	return (n == (ssize_t)len) ? 0 : -1;
}

static int rid_key_write(const uint8_t *buf, size_t len)
{
	int fd = open(RID_KEY_PATH, O_WRONLY | O_CREAT | O_TRUNC, 0600);
	if (fd < 0) { return -1; }
	ssize_t n = write(fd, buf, len);
	close(fd);
	return (n == (ssize_t)len) ? 0 : -1;
}

#ifdef PX4_CRYPTO
// ponytail: secret key on SD — upgrade to HSM/eFuse if hardware supports it
static constexpr const char *RID_SECKEY_PATH = "/fs/microsd/rid_seckey.bin";

static int rid_seckey_read(uint8_t *buf, size_t len)
{
	int fd = open(RID_SECKEY_PATH, O_RDONLY);
	if (fd < 0) { return -1; }
	ssize_t n = read(fd, buf, len);
	close(fd);
	return (n == (ssize_t)len) ? 0 : -1;
}

static int rid_seckey_write(const uint8_t *buf, size_t len)
{
	int fd = open(RID_SECKEY_PATH, O_WRONLY | O_CREAT | O_TRUNC, 0600);
	if (fd < 0) { return -1; }
	ssize_t n = write(fd, buf, len);
	close(fd);
	return (n == (ssize_t)len) ? 0 : -1;
}
#endif


UavcanRemoteIDController::UavcanRemoteIDController(uavcan::INode &node) :
	ModuleParams(nullptr),
	_timer(node),
	_ota_poll_timer(node),
	_node(node),
	_uavcan_pub_remoteid_basicid(node),
	_uavcan_pub_remoteid_location(node),
	_uavcan_pub_remoteid_self_id(node),
	_uavcan_pub_remoteid_system(node),
	_uavcan_pub_remoteid_operator_id(node),
	_uavcan_sub_arm_status(node),
	_uavcan_secure_command_server(node),
	_uavcan_secure_command_client(node)
{
}

int UavcanRemoteIDController::init()
{
	// Setup timer and call back function for periodic updates
	_timer.setCallback(TimerCbBinder(this, &UavcanRemoteIDController::periodic_update));
	_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000 / MAX_RATE_HZ));

	_ota_poll_timer.setCallback(OtaTimerCbBinder(this, &UavcanRemoteIDController::ota_poll));
	_ota_poll_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(2)); // 500 Hz — matches ArduPilot 400 Hz cadence

	int res = _uavcan_sub_arm_status.start(ArmStatusBinder(this, &UavcanRemoteIDController::arm_status_sub_cb));

	if (res < 0) {
		PX4_WARN("ArmStatus sub failed %i", res);
		return res;
	}

	res = _uavcan_secure_command_server.start(
		      SecureCommandBinder(this, &UavcanRemoteIDController::secure_command_server_cb));

	if (res < 0) {
		PX4_WARN("SecureCommand server failed %i", res);
		return res;
	}

	_uavcan_secure_command_client.setCallback(
		SecureCommandClientBinder(this, &UavcanRemoteIDController::secure_command_client_cb));

	return 0;
}

void UavcanRemoteIDController::periodic_update(const uavcan::TimerEvent &)
{
	_vehicle_status.update();


	send_basic_id();
	send_location();
	send_self_id();
	send_system();
	send_operator_id();

	if (_secure_command_request_sub.updated()) {
		secure_command_request_s req{};
		_secure_command_request_sub.copy(&req);

		// MAVLink op numbering (ArduPilot-aligned):
		//   0,1 = session key (local)
		//   8   = SET_PARAM          (local, PX4_CRYPTO only)
		//   9   = GENERATE_RID_KEY   → DroneCAN op 8
		//   10  = WRITE_RDCT         (local, PX4_CRYPTO only)
		//   11  = OTA_BEGIN          (local, PX4_CRYPTO only)
		//   12  = OTA_CHUNK          → DroneCAN op 9
		//   13  = TRIGGER_BL_UPDATE  (local, PX4_CRYPTO only)
		//   others → forward as-is
		static constexpr uint32_t MAV_OP_SET_PARAM          = 8;
		static constexpr uint32_t MAV_OP_GENERATE_RID_KEY   = 9;
		static constexpr uint32_t MAV_OP_WRITE_RDCT         = 10;
		static constexpr uint32_t MAV_OP_OTA_BEGIN          = 11;
		static constexpr uint32_t MAV_OP_OTA_CHUNK          = 12;
		static constexpr uint32_t MAV_OP_TRIGGER_BL_UPDATE  = 13;

		const bool is_local_cmd =
			req.operation == dronecan::remoteid::SecureCommand::Request::SECURE_COMMAND_GET_SESSION_KEY ||
			req.operation == dronecan::remoteid::SecureCommand::Request::SECURE_COMMAND_GET_REMOTEID_SESSION_KEY ||
			req.operation == MAV_OP_SET_PARAM ||
			req.operation == MAV_OP_WRITE_RDCT ||
			req.operation == MAV_OP_OTA_BEGIN ||
			req.operation == MAV_OP_TRIGGER_BL_UPDATE;

		if (is_local_cmd) {
#ifdef PX4_CRYPTO
			handle_secure_command_local(req);
#else
			secure_command_reply_s reply{};
			reply.timestamp = hrt_absolute_time();
			reply.sequence  = req.sequence;
			reply.operation = req.operation;
			reply.result    = 3; // MAV_RESULT_UNSUPPORTED
			_secure_command_reply_pub.publish(reply);
#endif

		} else if (req.operation == MAV_OP_OTA_CHUNK) {
			// ota_poll handles OTA_CHUNK at 500 Hz — discard if 1 Hz timer wins the race

		} else if (_rid_node_id != 0) {
			// Non-OTA DroneCAN commands (e.g. GENERATE_RID_KEY)
			uint32_t dronecan_op = req.operation;
			if (req.operation == MAV_OP_GENERATE_RID_KEY) {
				dronecan_op = dronecan::remoteid::SecureCommand::Request::SECURE_COMMAND_GENERATE_RID_KEY;
			}
			dronecan::remoteid::SecureCommand::Request dronecan_req{};
			dronecan_req.sequence   = req.sequence;
			dronecan_req.operation  = dronecan_op;
			dronecan_req.sig_length = req.sig_length;
			for (uint8_t i = 0; i < req.data_length && i < sizeof(req.data); ++i) {
				dronecan_req.data.push_back(req.data[i]);
			}
			_uavcan_secure_command_client.call(uavcan::NodeID(_rid_node_id), dronecan_req);

		} else {
			secure_command_reply_s reply{};
			reply.timestamp = hrt_absolute_time();
			reply.sequence  = req.sequence;
			reply.operation = req.operation;
			reply.result    = 4; // MAV_RESULT_FAILED — no module
			_secure_command_reply_pub.publish(reply);
		}
	}
}

void UavcanRemoteIDController::send_basic_id()
{
	dronecan::remoteid::BasicID basic_id {};
	// basic_id.id_or_mac // supposedly only used for drone ID data from other UAs
	basic_id.id_type = dronecan::remoteid::BasicID::ODID_ID_TYPE_SERIAL_NUMBER;
	basic_id.ua_type = static_cast<uint8_t>(open_drone_id_translations::odidTypeForMavType(
			_vehicle_status.get().system_type));

	// uas_id: UAS (Unmanned Aircraft System) ID following the format specified by id_type
	// TODO: MAV_ODID_ID_TYPE_SERIAL_NUMBER needs to be ANSI/CTA-2063 format

	char uas_id[20] = {};
	board_get_px4_guid_formated((char *)(uas_id), sizeof(uas_id));
	basic_id.uas_id = uas_id;

	_uavcan_pub_remoteid_basicid.broadcast(basic_id);
}

void UavcanRemoteIDController::send_location()
{
	dronecan::remoteid::Location msg {};

	// initialize all fields to unknown
	msg.status = MAV_ODID_STATUS_UNDECLARED;
	msg.direction = 36100; // If unknown: 36100 centi-degrees
	msg.speed_horizontal = 25500; // If unknown: 25500 cm/s
	msg.speed_vertical = 6300; // If unknown: 6300 cm/s
	msg.latitude = 0; // If unknown: 0
	msg.longitude = 0; // If unknown: 0
	msg.altitude_geodetic = -1000; // If unknown: -1000 m
	msg.altitude_geodetic = -1000; // If unknown: -1000 m
	msg.height = -1000; // If unknown: -1000 m
	msg.horizontal_accuracy = MAV_ODID_HOR_ACC_UNKNOWN;
	msg.vertical_accuracy = MAV_ODID_VER_ACC_UNKNOWN;
	msg.barometer_accuracy = MAV_ODID_VER_ACC_UNKNOWN;
	msg.speed_accuracy = MAV_ODID_SPEED_ACC_UNKNOWN;
	msg.timestamp = 0xFFFF; // If unknown: 0xFFFF
	msg.timestamp_accuracy = MAV_ODID_TIME_ACC_UNKNOWN;

	bool updated = false;

	if (_vehicle_land_detected_sub.advertised()) {
		vehicle_land_detected_s vehicle_land_detected{};

		if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)
		    && (hrt_elapsed_time(&vehicle_land_detected.timestamp) < 10_s)) {
			if (vehicle_land_detected.landed) {
				msg.status = MAV_ODID_STATUS_GROUND;

			} else {
				msg.status = MAV_ODID_STATUS_AIRBORNE;
			}

			updated = true;
		}
	}

	if (hrt_elapsed_time(&_vehicle_status.get().timestamp) < 10_s) {
		if (_vehicle_status.get().failsafe && (_vehicle_status.get().arming_state == vehicle_status_s::ARMING_STATE_ARMED)) {
			msg.status = MAV_ODID_STATUS_EMERGENCY;
			updated = true;
		}
	}

	if (_vehicle_gps_position_sub.advertised()) {
		sensor_gps_s vehicle_gps_position{};

		if (_vehicle_gps_position_sub.copy(&vehicle_gps_position)
		    && (hrt_elapsed_time(&vehicle_gps_position.timestamp) < 10_s)) {

			if (vehicle_gps_position.vel_ned_valid) {
				const matrix::Vector3f vel_ned{vehicle_gps_position.vel_n_m_s, vehicle_gps_position.vel_e_m_s, vehicle_gps_position.vel_d_m_s};

				// direction: calculate GPS course over ground angle
				const float course = atan2f(vel_ned(1), vel_ned(0));
				const int course_deg = roundf(math::degrees(matrix::wrap_2pi(course)));
				msg.direction = math::constrain(100 * course_deg, 0, 35999); // 0 - 35999 centi-degrees

				// speed_horizontal: If speed is larger than 25425 cm/s, use 25425 cm/s.
				const int speed_horizontal_cm_s = matrix::Vector2f(vel_ned).length() * 100.f;
				msg.speed_horizontal = math::constrain(speed_horizontal_cm_s, 0, 25425);

				// speed_vertical: Up is positive, If speed is larger than 6200 cm/s, use 6200 cm/s. If lower than -6200 cm/s, use -6200 cm/s.
				const int speed_vertical_cm_s = roundf(-vel_ned(2) * 100.f);
				msg.speed_vertical = math::constrain(speed_vertical_cm_s, -6200, 6200);

				msg.speed_accuracy = open_drone_id_translations::odidSpeedAccForVariance(vehicle_gps_position.s_variance_m_s);

				updated = true;
			}

			if (vehicle_gps_position.fix_type >= 2) {
				msg.latitude = static_cast<int32_t>(round(vehicle_gps_position.latitude_deg * 1e7));
				msg.longitude = static_cast<int32_t>(round(vehicle_gps_position.longitude_deg * 1e7));

				// altitude_geodetic
				if (vehicle_gps_position.fix_type >= 3) {
					msg.altitude_geodetic = static_cast<float>(round(vehicle_gps_position.altitude_msl_m)); // [m]
				}

				msg.horizontal_accuracy = open_drone_id_translations::odidHorAccForEph(vehicle_gps_position.eph);

				msg.vertical_accuracy = open_drone_id_translations::odidVerAccForEpv(vehicle_gps_position.epv);

				updated = true;
			}

			if (vehicle_gps_position.time_utc_usec != 0) {
				// timestamp: UTC then convert for this field using ((float) (time_week_ms % (60*60*1000))) / 1000
				uint64_t utc_time_msec = vehicle_gps_position.time_utc_usec / 1000;
				msg.timestamp = ((float)(utc_time_msec % (60 * 60 * 1000))) / 1000;

				msg.timestamp_accuracy = open_drone_id_translations::odidTimeForElapsed(hrt_elapsed_time(
								 &vehicle_gps_position.timestamp));

				updated = true;
			}
		}
	}

	// altitude_barometric: The altitude calculated from the barometric pressue
	if (_vehicle_air_data_sub.advertised()) {
		vehicle_air_data_s vehicle_air_data{};

		if (_vehicle_air_data_sub.copy(&vehicle_air_data) && (hrt_elapsed_time(&vehicle_air_data.timestamp) < 10_s)) {
			msg.altitude_barometric = vehicle_air_data.baro_alt_meter;
			msg.barometer_accuracy = MAV_ODID_VER_ACC_UNKNOWN; // We just don't without calibration.
			updated = true;
		}
	}

	// height: The current height of the unmanned aircraft above the take-off location or the ground as indicated by height_reference
	if (_home_position_sub.advertised() && _vehicle_local_position_sub.updated()) {
		home_position_s home_position{};
		vehicle_local_position_s vehicle_local_position{};

		if (_home_position_sub.copy(&home_position)
		    && _vehicle_local_position_sub.copy(&vehicle_local_position)
		    && (hrt_elapsed_time(&vehicle_local_position.timestamp) < 1_s)
		   ) {

			if (home_position.valid_alt && vehicle_local_position.z_valid && vehicle_local_position.z_global) {
				float altitude = (-vehicle_local_position.z + vehicle_local_position.ref_alt);

				msg.height = altitude - home_position.alt;
				msg.height_reference = MAV_ODID_HEIGHT_REF_OVER_TAKEOFF;
				updated = true;
			}
		}
	}

	if (updated) {
		_uavcan_pub_remoteid_location.broadcast(msg);
	}
}

void UavcanRemoteIDController::send_system()
{
	open_drone_id_system_s system;

	if (_open_drone_id_system.advertised() && _open_drone_id_system.copy(&system)) {

		// Use what ground station sends us.

		dronecan::remoteid::System msg {};
		msg.timestamp = system.timestamp;

		for (unsigned i = 0; i < sizeof(system.id_or_mac); ++i) {
			msg.id_or_mac.push_back(system.id_or_mac[i]);
		}

		msg.operator_location_type = system.operator_location_type;
		msg.classification_type = system.classification_type;
		msg.operator_latitude = system.operator_latitude;
		msg.operator_longitude = system.operator_longitude;
		msg.area_count = system.area_count;
		msg.area_radius = system.area_radius;
		msg.area_ceiling = system.area_ceiling;
		msg.area_floor = system.area_floor;
		msg.category_eu = system.category_eu;
		msg.class_eu = system.class_eu;
		msg.operator_altitude_geo = system.operator_altitude_geo;

		_uavcan_pub_remoteid_system.broadcast(msg);

	} else {
		// And otherwise, send our home/takeoff location.

		sensor_gps_s vehicle_gps_position;
		home_position_s home_position;

		if (_vehicle_gps_position_sub.copy(&vehicle_gps_position) && _home_position_sub.copy(&home_position)) {
			if (vehicle_gps_position.fix_type >= 3
			    && home_position.valid_alt && home_position.valid_hpos) {

				dronecan::remoteid::System msg {};

				// msg.id_or_mac // Only used for drone ID data received from other UAs.
				msg.operator_location_type = MAV_ODID_OPERATOR_LOCATION_TYPE_TAKEOFF;
				msg.classification_type = MAV_ODID_CLASSIFICATION_TYPE_UNDECLARED;
				msg.operator_latitude = home_position.lat * 1e7;
				msg.operator_longitude = home_position.lon * 1e7;
				msg.area_count = 1;
				msg.area_radius = 0;
				msg.area_ceiling = -1000;
				msg.area_floor = -1000;
				msg.category_eu = MAV_ODID_CATEGORY_EU_UNDECLARED;
				msg.class_eu = MAV_ODID_CLASS_EU_UNDECLARED;
				float wgs84_amsl_offset = vehicle_gps_position.altitude_ellipsoid_m - vehicle_gps_position.altitude_msl_m;
				msg.operator_altitude_geo = home_position.alt + wgs84_amsl_offset;

				// timestamp: 32 bit Unix Timestamp in seconds since 00:00:00 01/01/2019.
				static uint64_t utc_offset_s = 1'546'300'800; // UTC seconds since 00:00:00 01/01/2019
				msg.timestamp = vehicle_gps_position.time_utc_usec / 1e6 - utc_offset_s;

				_uavcan_pub_remoteid_system.broadcast(msg);
			}
		}
	}
}

void UavcanRemoteIDController::send_self_id()
{
	open_drone_id_self_id_s self_id;

	if (_open_drone_id_self_id.copy(&self_id)) {

		dronecan::remoteid::SelfID msg {};

		for (unsigned i = 0; i < sizeof(self_id.id_or_mac); ++i) {
			msg.id_or_mac.push_back(self_id.id_or_mac[i]);
		}

		msg.description_type = self_id.description_type;

		for (unsigned i = 0; i < sizeof(self_id.description); ++i) {
			msg.description.push_back(self_id.description[i]);
		}

		_uavcan_pub_remoteid_self_id.broadcast(msg);
	}
}

void UavcanRemoteIDController::send_operator_id()
{
	open_drone_id_operator_id_s operator_id;

	if (_open_drone_id_operator_id.copy(&operator_id)) {

		dronecan::remoteid::OperatorID msg {};

		for (unsigned i = 0; i < sizeof(operator_id.id_or_mac); ++i) {
			msg.id_or_mac.push_back(operator_id.id_or_mac[i]);
		}

		msg.operator_id_type = operator_id.operator_id_type;

		for (unsigned i = 0; i < sizeof(operator_id.operator_id); ++i) {
			msg.operator_id.push_back(operator_id.operator_id[i]);
		}

		_uavcan_pub_remoteid_operator_id.broadcast(msg);
	}
}

void
UavcanRemoteIDController::arm_status_sub_cb(const uavcan::ReceivedDataStructure<dronecan::remoteid::ArmStatus> &msg)
{
	_rid_node_id = msg.getSrcNodeID().get();

	open_drone_id_arm_status_s arm_status{};
	arm_status.timestamp = hrt_absolute_time();
	arm_status.status = msg.status;
	memcpy(arm_status.error, msg.error.c_str(), sizeof(arm_status.error));
	_open_drone_id_arm_status_pub.publish(arm_status);
}

#ifdef PX4_CRYPTO
void UavcanRemoteIDController::handle_secure_command_local(const secure_command_request_s &req)
{
	secure_command_reply_s reply{};
	reply.timestamp = hrt_absolute_time();
	reply.sequence  = req.sequence;
	reply.operation = req.operation;
	reply.result    = 4; // MAV_RESULT_FAILED

	static constexpr uint32_t MAV_OP_SET_PARAM = 8; // MAVLink op 8 (ArduPilot-aligned)
	if (req.operation == MAV_OP_SET_PARAM) {
		// Data layout: [name: char[16]] [value: uint8[4]] [MAC: uint8[16]]
		if (!_session_valid || req.data_length < 36) {
			reply.result = 2; // MAV_RESULT_DENIED
			_secure_command_reply_pub.publish(reply);
			return;
		}

		char name[17]{};
		memcpy(name, req.data, 16);

		uint8_t value_bytes[4];
		memcpy(value_bytes, req.data + 16, 4);

		// Verify BLAKE2b-16 MAC: BLAKE2b(key=session_key, msg="set_param" || name[16] || value[4])
		static constexpr char label[] = "set_param";
		uint8_t msg[sizeof(label) - 1 + 16 + 4];
		memcpy(msg,                         label,       sizeof(label) - 1);
		memcpy(msg + sizeof(label) - 1,     req.data,    16);
		memcpy(msg + sizeof(label) - 1 + 16, value_bytes, 4);

		uint8_t expected_mac[16];
		crypto_blake2b_general(expected_mac, 16, _session_key, 32, msg, sizeof(msg));

		if (memcmp(expected_mac, req.data + 20, 16) != 0) {
			reply.result = 2; // MAV_RESULT_DENIED
			_secure_command_reply_pub.publish(reply);
			return;
		}

		if (!fw_param_is_secure(name)) {
			reply.result = 3; // MAV_RESULT_UNSUPPORTED
			_secure_command_reply_pub.publish(reply);
			return;
		}

		param_t p = param_find(name);
		if (p == PARAM_INVALID) {
			_secure_command_reply_pub.publish(reply);
			return;
		}

		int32_t int_val;
		memcpy(&int_val, value_bytes, 4);
		if (param_set(p, &int_val) == 0) {
			param_save_default(false);
			reply.result = 0; // MAV_RESULT_ACCEPTED
		}

		_secure_command_reply_pub.publish(reply);
		return;
	}

#ifdef RDCT_CERT_ADDRESS
	static constexpr uint32_t MAV_OP_WRITE_RDCT        = 10;
	static constexpr uint32_t MAV_OP_OTA_BEGIN         = 11;
	static constexpr uint32_t MAV_OP_TRIGGER_BL_UPDATE = 13;

	if (req.operation == MAV_OP_WRITE_RDCT) {
		// Data layout: [MAC: 16 bytes] [image_cert_t: N bytes]
		// MAC = BLAKE2b-16(key=session_key, msg="write_rdct" || cert_bytes)
		size_t cert_len = sizeof(image_cert_t) + 64; // struct + Ed25519 signature
		if (!_session_valid || req.data_length < 16 + cert_len) {
			reply.result = 2; // MAV_RESULT_DENIED
			_secure_command_reply_pub.publish(reply);
			return;
		}

		const uint8_t *cert_bytes = req.data + 16;

		static constexpr char label[] = "write_rdct";
		uint8_t mac_input[sizeof(label) - 1 + sizeof(image_cert_t) + 64];
		memcpy(mac_input,                    label,      sizeof(label) - 1);
		memcpy(mac_input + sizeof(label) - 1, cert_bytes, cert_len);

		uint8_t expected_mac[16];
		crypto_blake2b_general(expected_mac, 16, _session_key, 32,
				       mac_input, sizeof(label) - 1 + cert_len);

		if (memcmp(expected_mac, req.data, 16) != 0) {
			reply.result = 2; // MAV_RESULT_DENIED
			_secure_command_reply_pub.publish(reply);
			return;
		}

		// ponytail: RDCT lives in bootloader sector — cannot erase without killing BL.
		// Refuse if already written (would hardfault trying to flip 0→1 on STM32H7).
		const uint8_t *existing = (const uint8_t *)RDCT_CERT_ADDRESS;
		if (existing[0] != 0xFF) {
			reply.result = 5; // already written, read-only until DFU reflash
			_secure_command_reply_pub.publish(reply);
			return;
		}

		// STM32H7 flash requires 32-byte word alignment; pad remainder with 0xFF
		static constexpr size_t write_len = (144 + 31u) & ~31u; // 160 bytes
		uint8_t write_buf[write_len];
		memcpy(write_buf, cert_bytes, cert_len);
		memset(write_buf + cert_len, 0xFF, write_len - cert_len);
		ssize_t written = up_progmem_write(RDCT_CERT_ADDRESS, write_buf, write_len);
		reply.result = (written == (ssize_t)write_len) ? 0 : 4;
		_secure_command_reply_pub.publish(reply);
		return;
	}

	if (req.operation == MAV_OP_OTA_BEGIN) {
		// Data layout: [MAC: 16 bytes] [fw_size: uint32 LE]
		// MAC = BLAKE2b-16(key=session_key, msg="ota_begin" || fw_size)

		// If erase already done, script can proceed
		if (_ota_active) {
			reply.operation = req.operation;
			reply.result = 0; // MAV_RESULT_ACCEPTED
			_secure_command_reply_pub.publish(reply);
			return;
		}

		// While ESP32 is erasing, drain retries
		if (_ota_begin_pending) {
			reply.operation = req.operation;
			reply.result = 1; // TEMPORARILY_REJECTED
			_secure_command_reply_pub.publish(reply);
			return;
		}

		if (!_session_valid || req.data_length < 16 + 4) {
			reply.result = 2; // MAV_RESULT_DENIED
			_secure_command_reply_pub.publish(reply);
			return;
		}

		static constexpr char label[] = "ota_begin";
		const uint8_t *fw_size_bytes = req.data + 16;
		uint8_t mac_input[sizeof(label) - 1 + 4];
		memcpy(mac_input,                    label,        sizeof(label) - 1);
		memcpy(mac_input + sizeof(label) - 1, fw_size_bytes, 4);

		uint8_t expected_mac[16];
		crypto_blake2b_general(expected_mac, 16, _session_key, 32,
				       mac_input, sizeof(mac_input));

		if (memcmp(expected_mac, req.data, 16) != 0) {
			reply.result = 2; // MAV_RESULT_DENIED
			_secure_command_reply_pub.publish(reply);
			return;
		}

		if (_rid_node_id == 0) {
			reply.result = 4; // MAV_RESULT_FAILED — no RID node yet
			_secure_command_reply_pub.publish(reply);
			return;
		}

		uint32_t fw_size = 0;
		memcpy(&fw_size, fw_size_bytes, sizeof(fw_size));
		dronecan::remoteid::SecureCommand::Request dreq{};
		dreq.sequence  = req.sequence;
		dreq.operation = dronecan::remoteid::SecureCommand::Request::SECURE_COMMAND_OTA_BEGIN;
		for (uint8_t i = 0; i < 4; ++i) { dreq.data.push_back(fw_size_bytes[i]); }
		// OTA_BEGIN may block up to ~10s while ESP32 erases partition
		_uavcan_secure_command_client.setRequestTimeout(
			uavcan::MonotonicDuration::fromMSec(15000));
		_uavcan_secure_command_client.call(uavcan::NodeID(_rid_node_id), dreq);

		_ota_begin_pending = true;
		_ota_begin_seq     = req.sequence;
		_ota_begin_epoch   = _session_epoch;
		reply.result       = 1; // MAV_RESULT_TEMPORARILY_REJECTED
		_secure_command_reply_pub.publish(reply);
		return;
	}

	if (req.operation == MAV_OP_TRIGGER_BL_UPDATE) {
		if (!_session_valid || req.data_length < 16) {
			reply.result = 2; // MAV_RESULT_DENIED
			_secure_command_reply_pub.publish(reply);
			return;
		}

		static constexpr uint8_t bl_label[] = "bl_update";
		uint8_t expected_mac[16];
		crypto_blake2b_general(expected_mac, 16, _session_key, 32,
				       bl_label, sizeof(bl_label) - 1);

		if (memcmp(expected_mac, req.data, 16) != 0) {
			reply.result = 2; // MAV_RESULT_DENIED
			_secure_command_reply_pub.publish(reply);
			return;
		}

		reply.result = 0; // MAV_RESULT_ACCEPTED
		_secure_command_reply_pub.publish(reply);

		static const char *const romfs_paths[] = {
			"/etc/extras/secure_bootloader.bin",
			nullptr
		};
		const char *bl_path = "/fs/microsd/bootloader.bin";
		for (auto p = romfs_paths; *p; ++p) {
			if (access(*p, F_OK) == 0) { bl_path = *p; break; }
		}
		char *argv_bl[] = {(char *)bl_path, nullptr};
		px4_task_spawn_cmd("bl_update", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT,
				   2048, bl_update_main, argv_bl);
		return;
	}
#endif // RDCT_CERT_ADDRESS

	// Session key operations (op 0 and 1 — GET_SESSION_KEY / GET_REMOTEID_SESSION_KEY)
	keystore_session_handle_t ks = keystore_open();
	uint8_t ed25519_pub[32]{};
	size_t  got = keystore_get_key(ks, 0, ed25519_pub, sizeof(ed25519_pub));
	keystore_close(&ks);

	if (got != 32) {
		_secure_command_reply_pub.publish(reply);
		return;
	}

	uint8_t operator_x25519[32];
	crypto_from_ed25519_public(operator_x25519, ed25519_pub);

	uint8_t eph_priv[32], eph_pub[32];
	if (px4_get_secure_random(eph_priv, 32) == 32) {
		crypto_x25519_public_key(eph_pub, eph_priv);

		uint8_t shared[32];
		crypto_x25519(shared, eph_priv, operator_x25519);
		crypto_blake2b_general(_session_key, 32, nullptr, 0, shared, 32);
		_session_valid = true;
		_session_epoch++;

		crypto_wipe(shared,   sizeof(shared));
		crypto_wipe(eph_priv, sizeof(eph_priv));

		memcpy(reply.data, eph_pub, 32);
		reply.data_length = 32;
		reply.result = 0; // MAV_RESULT_ACCEPTED
	}

	crypto_wipe(operator_x25519, sizeof(operator_x25519));
	_secure_command_reply_pub.publish(reply);
}
#endif

void
UavcanRemoteIDController::secure_command_server_cb(
	const uavcan::ReceivedDataStructure<dronecan::remoteid::SecureCommand::Request> &req,
	dronecan::remoteid::SecureCommand::Response &rsp)
{
	rsp.sequence  = req.sequence;
	rsp.operation = req.operation;
	rsp.result    = dronecan::remoteid::SecureCommand::Response::RESULT_UNSUPPORTED;

	switch (req.operation) {
	case dronecan::remoteid::SecureCommand::Request::SECURE_COMMAND_GET_PUBLIC_KEYS: {
		uint8_t key[RID_KEY_LEN] {};
		if (rid_key_read(key, sizeof(key)) == 0) {
			for (uint8_t b : key) { rsp.data.push_back(b); }
			rsp.result = dronecan::remoteid::SecureCommand::Response::RESULT_ACCEPTED;
		} else {
			rsp.result = dronecan::remoteid::SecureCommand::Response::RESULT_FAILED;
		}
		break;
	}

	case dronecan::remoteid::SecureCommand::Request::SECURE_COMMAND_SET_PUBLIC_KEYS: {
		if (req.data.size() == RID_KEY_LEN) {
			uint8_t key[RID_KEY_LEN];
			for (size_t i = 0; i < RID_KEY_LEN; ++i) { key[i] = req.data[i]; }
			rsp.result = (rid_key_write(key, sizeof(key)) == 0)
				     ? dronecan::remoteid::SecureCommand::Response::RESULT_ACCEPTED
				     : dronecan::remoteid::SecureCommand::Response::RESULT_FAILED;
		} else {
			rsp.result = dronecan::remoteid::SecureCommand::Response::RESULT_DENIED;
		}
		break;
	}

	case dronecan::remoteid::SecureCommand::Request::SECURE_COMMAND_GENERATE_RID_KEY: {
#ifdef PX4_CRYPTO
		uint8_t seed[32], public_key[32];
		if (px4_get_secure_random(seed, sizeof(seed)) != sizeof(seed)) {
			rsp.result = dronecan::remoteid::SecureCommand::Response::RESULT_FAILED;
			break;
		}
		crypto_ed25519_public_key(public_key, seed);
		if (rid_seckey_write(seed, sizeof(seed)) != 0 || rid_key_write(public_key, sizeof(public_key)) != 0) {
			rsp.result = dronecan::remoteid::SecureCommand::Response::RESULT_FAILED;
			break;
		}
		for (uint8_t b : public_key) { rsp.data.push_back(b); }
		rsp.result = dronecan::remoteid::SecureCommand::Response::RESULT_ACCEPTED;
		crypto_wipe(seed, sizeof(seed));
#endif
		break;
	}

	case dronecan::remoteid::SecureCommand::Request::SECURE_COMMAND_AUTH_CHALLENGE: {
#ifdef PX4_CRYPTO
		uint8_t secret_key[32], public_key[32], signature[64];
		if (rid_seckey_read(secret_key, sizeof(secret_key)) != 0 || rid_key_read(public_key, sizeof(public_key)) != 0) {
			rsp.result = dronecan::remoteid::SecureCommand::Response::RESULT_FAILED;
			break;
		}
		const uint8_t *msg = req.data.begin();
		const size_t   msg_len = req.data.size();
		crypto_ed25519_sign(signature, secret_key, public_key, msg, msg_len);
		for (uint8_t b : signature) { rsp.data.push_back(b); }
		rsp.result = dronecan::remoteid::SecureCommand::Response::RESULT_ACCEPTED;
		crypto_wipe(secret_key, sizeof(secret_key));
#endif
		break;
	}

	case dronecan::remoteid::SecureCommand::Request::SECURE_COMMAND_OTA_CHUNK: {
		// ponytail: no signature verification on chunks — add HMAC check if tampering is a concern
		static constexpr const char *OTA_STAGING = "/fs/microsd/ota_staging.px4";

		if (req.sequence == 0) {
			// First chunk: (re)open staging file
			if (_ota_fd >= 0) { close(_ota_fd); }
			_ota_fd = open(OTA_STAGING, O_WRONLY | O_CREAT | O_TRUNC, 0644);
		}

		if (req.data.empty()) {
			// Empty data signals end of transfer — close and stage for bootloader
			if (_ota_fd >= 0) { close(_ota_fd); _ota_fd = -1; }
			// ponytail: rename triggers bootloader pickup on next reboot — add px4_reboot_request() if auto-reboot is needed
			rename(OTA_STAGING, "/fs/microsd/ota_firmware.px4");
			rsp.result = dronecan::remoteid::SecureCommand::Response::RESULT_ACCEPTED;

		} else if (_ota_fd < 0) {
			rsp.result = dronecan::remoteid::SecureCommand::Response::RESULT_FAILED;

		} else {
			ssize_t n = write(_ota_fd, req.data.begin(), req.data.size());
			rsp.result = (n == (ssize_t)req.data.size())
				     ? dronecan::remoteid::SecureCommand::Response::RESULT_ACCEPTED
				     : dronecan::remoteid::SecureCommand::Response::RESULT_FAILED;
		}
		break;
	}

	default:
		break;
	}
}

void UavcanRemoteIDController::secure_command_client_cb(
	const uavcan::ServiceCallResult<dronecan::remoteid::SecureCommand> &result)
{
#ifdef PX4_CRYPTO
	if (_ota_begin_pending) {
		// Restore default request timeout
		_uavcan_secure_command_client.setRequestTimeout(
			uavcan::MonotonicDuration::fromMSec(1000));
		_ota_begin_pending = false;
		// Drop stale callback from a previous session (new GET_SESSION_KEY incremented epoch)
		if (_ota_begin_epoch != _session_epoch) {
			return;
		}
		secure_command_reply_s reply{};
		reply.timestamp = hrt_absolute_time();
		reply.sequence  = _ota_begin_seq;
		reply.operation = 11; // MAV_OP_OTA_BEGIN
		if (result.isSuccessful() && result.getResponse().result == 0) {
			_ota_active  = true;
			reply.result = 0; // ACCEPTED
		} else {
			reply.result = 4; // FAILED
		}
		_secure_command_reply_pub.publish(reply);
		return;
	}

	if (_ota_active) {
		// OTA path: hand all state transitions to ota_poll to avoid re-entrancy
		_dronecan_pending       = false;
		_ota_dronecan_success   = result.isSuccessful() && (result.getResponse().result == 0);
		_ota_dronecan_done      = true;
		return;
	}
#endif

	// Non-OTA path: publish reply immediately
	secure_command_reply_s reply{};
	reply.timestamp = hrt_absolute_time();

	if (!result.isSuccessful()) {
		reply.result = 4; // MAV_RESULT_FAILED
		_secure_command_reply_pub.publish(reply);
		return;
	}

	const auto &rsp = result.getResponse();
	reply.sequence    = rsp.sequence;
	reply.data_length = rsp.data.size();
	memcpy(reply.data, rsp.data.begin(), reply.data_length);

	uint32_t mavlink_op = rsp.operation;
	if (rsp.operation == dronecan::remoteid::SecureCommand::Request::SECURE_COMMAND_GENERATE_RID_KEY) {
		mavlink_op = 9;
	}
	reply.operation = mavlink_op;

	static constexpr uint8_t to_mav_result[4] = {0, 2, 4, 3};
	reply.result = (rsp.result < 4) ? to_mav_result[rsp.result] : 4;

	_secure_command_reply_pub.publish(reply);
}

void UavcanRemoteIDController::ota_poll(const uavcan::TimerEvent &)
{
#ifdef PX4_CRYPTO
	if (!_ota_active) { return; }

	static constexpr uint32_t MAV_OP_OTA_CHUNK = 12;

	// Process DroneCAN result (set by callback; handled here to avoid re-entrancy)
	if (_ota_dronecan_done) {
		_ota_dronecan_done = false;

		if (_dronecan_is_last_chunk) {
			// Forward ESP32 validation result — success or failure — to script.
			// Never retry last chunk: ESP32 is validating; re-sending interrupts it.
			_uavcan_secure_command_client.setRequestTimeout(
				uavcan::MonotonicDuration::fromMSec(1000)); // restore default
			secure_command_reply_s reply{};
			reply.timestamp = hrt_absolute_time();
			reply.sequence  = _ota_inflight.sequence;
			reply.operation = MAV_OP_OTA_CHUNK;
			reply.result    = _ota_dronecan_success ? 0 : 4;
			_secure_command_reply_pub.publish(reply);
			_ota_active             = false;
			_dronecan_is_last_chunk = false;
			return;
		}

		if (!_ota_dronecan_success) {
			// Non-last chunk DroneCAN timeout — retry same chunk
			_dronecan_pending = true;
			dronecan::remoteid::SecureCommand::Request dreq{};
			dreq.sequence  = _ota_inflight.sequence;
			dreq.operation = dronecan::remoteid::SecureCommand::Request::SECURE_COMMAND_OTA_CHUNK;
			for (uint8_t i = 0; i < _ota_inflight.length; ++i) { dreq.data.push_back(_ota_inflight.data[i]); }
			_uavcan_secure_command_client.call(uavcan::NodeID(_rid_node_id), dreq);
			return;
		}

		// Non-last ACK: dispatch buffered chunk if queued
		if (_ota_buf.valid) {
			_dronecan_pending       = true;
			_dronecan_is_last_chunk = _ota_buf.is_last;
			_ota_inflight           = _ota_buf;
			_ota_buf.valid          = false;
			dronecan::remoteid::SecureCommand::Request dreq{};
			dreq.sequence  = _ota_inflight.sequence;
			dreq.operation = dronecan::remoteid::SecureCommand::Request::SECURE_COMMAND_OTA_CHUNK;
			for (uint8_t i = 0; i < _ota_inflight.length; ++i) { dreq.data.push_back(_ota_inflight.data[i]); }
			_uavcan_secure_command_client.call(uavcan::NodeID(_rid_node_id), dreq);
			return;
		}
		// No buffer: fall through to read next chunk from uORB
	}

	// While last chunk is in-flight: drain script retries with TEMPORARILY_REJECTED
	if (_dronecan_is_last_chunk) {
		if (_secure_command_request_sub.updated()) {
			secure_command_request_s req{};
			_secure_command_request_sub.copy(&req);
			if (req.operation == MAV_OP_OTA_CHUNK) {
				secure_command_reply_s reply{};
				reply.timestamp = hrt_absolute_time();
				reply.sequence  = req.sequence;
				reply.operation = MAV_OP_OTA_CHUNK;
				reply.result    = 1; // TEMPORARILY_REJECTED — waiting for ESP32 validation
				_secure_command_reply_pub.publish(reply);
			}
		}
		return;
	}

	// Buffer full: reply TEMP_REJECTED so script retries in ~2ms instead of timing out for 5s
	if (_dronecan_pending && _ota_buf.valid) {
		if (_secure_command_request_sub.updated()) {
			secure_command_request_s req{};
			_secure_command_request_sub.copy(&req);
			if (req.operation == MAV_OP_OTA_CHUNK) {
				secure_command_reply_s reply{};
				reply.timestamp = hrt_absolute_time();
				reply.sequence  = req.sequence;
				reply.operation = MAV_OP_OTA_CHUNK;
				reply.result    = 1; // TEMPORARILY_REJECTED
				_secure_command_reply_pub.publish(reply);
			}
		}
		return;
	}

	if (_dronecan_pending) { return; }
	if (_ota_buf.valid)    { return; }

	if (!_secure_command_request_sub.updated()) { return; }

	secure_command_request_s req{};
	_secure_command_request_sub.copy(&req);

	if (req.operation != MAV_OP_OTA_CHUNK || _rid_node_id == 0) { return; }

	static constexpr uint8_t FLAG_LAST_BIT = 0x02;
	const bool is_last = req.data_length > 0 && (req.data[0] & FLAG_LAST_BIT);

	// Non-last: ACCEPTED immediately. Last: TEMPORARILY_REJECTED until ESP32 validates.
	secure_command_reply_s fast_reply{};
	fast_reply.timestamp = hrt_absolute_time();
	fast_reply.sequence  = req.sequence;
	fast_reply.operation = MAV_OP_OTA_CHUNK;
	fast_reply.result    = is_last ? 1 : 0;
	_secure_command_reply_pub.publish(fast_reply);

	if (!_dronecan_pending) {
		_dronecan_pending       = true;
		_dronecan_is_last_chunk = is_last;
		_ota_inflight.sequence  = req.sequence;
		_ota_inflight.is_last   = is_last;
		_ota_inflight.length    = req.data_length < sizeof(_ota_inflight.data) ? req.data_length : sizeof(_ota_inflight.data);
		memcpy(_ota_inflight.data, req.data, _ota_inflight.length);
		// Last chunk: ESP32 validates entire firmware — give it 60s before declaring failure
		_uavcan_secure_command_client.setRequestTimeout(
			uavcan::MonotonicDuration::fromMSec(is_last ? 60000 : 1000));
		dronecan::remoteid::SecureCommand::Request dreq{};
		dreq.sequence  = req.sequence;
		dreq.operation = dronecan::remoteid::SecureCommand::Request::SECURE_COMMAND_OTA_CHUNK;
		for (uint8_t i = 0; i < req.data_length; ++i) { dreq.data.push_back(req.data[i]); }
		_uavcan_secure_command_client.call(uavcan::NodeID(_rid_node_id), dreq);
	} else {
		_ota_buf.valid    = true;
		_ota_buf.is_last  = is_last;
		_ota_buf.sequence = req.sequence;
		_ota_buf.length   = req.data_length < sizeof(_ota_buf.data) ? req.data_length : sizeof(_ota_buf.data);
		memcpy(_ota_buf.data, req.data, _ota_buf.length);
	}
#endif
}
