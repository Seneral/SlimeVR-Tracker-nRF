/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include "globals.h"
#include "connection.h"
#include "util.h"
#include "esb.h"
#include "build_defines.h"

static uint8_t tracker_id, batt, batt_v, sensor_temp, imu_id, mag_id, tracker_status;
static uint8_t tracker_svr_status = SVR_STATUS_OK;
static float sensor_q[4], sensor_a[3];
static uint64_t sensor_timestamp_us = 0;

LOG_MODULE_REGISTER(connection, LOG_LEVEL_INF);

uint8_t connection_get_id(void)
{
	return tracker_id;
}

void connection_set_id(uint8_t id)
{
	tracker_id = id;
}

void connection_update_sensor_ids(int imu, int mag)
{
	imu_id = get_server_constant_imu_id(imu);
	mag_id = get_server_constant_mag_id(mag);
}

void connection_update_sensor_data(float *q, float *a, uint64_t timestamp_us)
{
	memcpy(sensor_q, q, sizeof(sensor_q));
	memcpy(sensor_a, a, sizeof(sensor_a));
	sensor_timestamp_us = timestamp_us;
}

void connection_update_sensor_temp(float temp)
{
	// sensor_temp == zero means no data
	if (temp < -38.5f)
		sensor_temp = 1;
	else if (temp > 88.5f)
		sensor_temp = 255;
	else
		sensor_temp = ((temp - 25) * 2 + 128.5f); // -38.5 - +88.5 -> 1-255
}

void connection_update_battery(bool battery_available, bool plugged, uint32_t battery_pptt, int battery_mV) // format for packet send
{
	if (!battery_available) // No battery, and voltage is <=1500mV
	{
		batt = 0;
		batt_v = 0;
		return;
	}

	battery_pptt /= 100;
	batt = battery_pptt;
	batt |= 0x80; // battery_available, server will show a battery indicator

	if (plugged) // Charging
	{
		batt_v = 255; // server will show a charging indicator
		return;
	}

	battery_mV /= 10;
	battery_mV -= 245;
	if (battery_mV < 0) // Very dead but it is what it is
		batt_v = 0;
	else if (battery_mV > 255)
		batt_v = 255;
	else
		batt_v = battery_mV; // 0-255 -> 2.45-5.00V
}

void connection_update_status(int status)
{
	tracker_status = status;
	tracker_svr_status = get_server_constant_tracker_status(status);
}

// Building blocks: Status (3B), Info (10B), Timestamps (3B), Data(1-14B) - e.g. IMU(12B)
// LEN:  |t:3|id:5|b1      |b2      |b3      |b4      |b5      |b6      |b7      |b8      |b9      |b10     |b11     |b12     |b13     |b14     |b15     |b16     |b17     |b18     |
//    8: |00000000|Checksum|pairing adress                                       |
//   14: |001|id  |brd_id  |mcu_id  |RESV    |imu_id  |mag_id  |fw_date          |major   |minor   |patch   |batt    |batt_v  |temp    |
// <=15: |XXX|id  |DATA (up to 14B)                                                                                                             |
//   16: |XXX|id  |DATA (12B)                                                                                                 |timestamp imu[12] last[12]|
//   19: |XXX|id  |DATA (12B)                                                                                                 |timestamp imu[12] last[12]|batt    |batt_v  |temp    |
// DATA for TYPE_IMU_CAYLEY (12B, compatible with SIZE_TIMESTAMPED and SIZE_TIMESTAMPED_STATUS):
//                |q0               |q1               |q2               |a0               |a1               |a2               |
// DATA for TYPE_GENERIC_HID (example using full 14B using SIZE_MAX_NORMAL):
//                |001     |Joystick X       |Joystick Y       |Trigger |Buttons |Capacitive Sensors (8x8B?)                                    |

// See  PACKET_HEADER_TYPE and PACKET_RESERVED_SIZES

static inline void write_header(uint8_t data[1], enum PACKET_HEADER_TYPE type)
{
	data[0] = (type << 5) | (tracker_id & 0b11111);
}

static inline void write_info(uint8_t data[10])
{
	data[0] = FW_BOARD; // brd_id
	data[1] = FW_MCU; // mcu_id
	data[2] = 0; // resv
	data[3] = imu_id; // imu_id
	data[4] = mag_id; // mag_id
	uint16_t *buf = (uint16_t *)&data[5];
	buf[0] = ((BUILD_YEAR - 2020) & 127) << 9 | (BUILD_MONTH & 15) << 5 | (BUILD_DAY & 31); // fw_date
	data[7] = FW_VERSION_MAJOR & 255; // fw_major
	data[8] = FW_VERSION_MINOR & 255; // fw_minor
	data[9] = FW_VERSION_PATCH & 255; // fw_patch
}

static inline void write_status(uint8_t data[3])
{
	data[0] = batt;
	data[1] = batt_v;
	data[2] = sensor_temp;
}

static inline void write_imu_cayley(uint8_t data[12])
{
	uint16_t *buf = (uint16_t *)data;
	float v[3];
	q_cayley_f(sensor_q, v); // cayley transform
	buf[0] = TO_FIXED_15(v[0]);
	buf[1] = TO_FIXED_15(v[1]);
	buf[2] = TO_FIXED_15(v[2]);
	buf[3] = TO_FIXED_7(sensor_a[0]);
	buf[4] = TO_FIXED_7(sensor_a[1]);
	buf[5] = TO_FIXED_7(sensor_a[2]);
}

static inline void write_timestamps(uint8_t data[3])
{
	uint16_t ts_imu = (sensor_timestamp_us >> 2) & 0x0FFF;
	uint16_t ts_last = (tx_timestamp >> 2) & 0x0FFF;
	if (tx_errors) ts_last = 0;
	data[0] = ts_imu >> 4;
	data[1] = ((ts_imu&0xF) << 4) | (ts_last >> 8);
	data[2] = ts_last & 0xFF;
}

void connection_write_info_status()
{
	uint8_t data[14];
	write_header(data, TYPE_INFO_STATUS);
	write_info(data+1);
	write_status(data+1+10);
	esb_write(data, sizeof(data));
}

void connection_write_sensors()
{
	uint8_t data[13];
	write_header(data, TYPE_IMU_CAYLEY);
	write_imu_cayley(data+1);
	esb_write(data, sizeof(data));
}

void connection_write_sensors_timestamped()
{
	uint8_t data[SIZE_TIMESTAMPED];
	write_header(data, TYPE_IMU_CAYLEY);
	write_imu_cayley(data+1);
	write_timestamps(data+1+12);
	esb_write(data, sizeof(data));
}

void connection_write_sensors_timestamped_status()
{
	uint8_t data[SIZE_TIMESTAMPED_STATUS];
	write_header(data, TYPE_IMU_CAYLEY);
	write_imu_cayley(data+1);
	write_timestamps(data+1+12);
	write_status(data+1+12+3);
	esb_write(data, sizeof(data));
}