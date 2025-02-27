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
#ifndef SLIMENRF_CONNECTION
#define SLIMENRF_CONNECTION

enum PACKET_HEADER_TYPE
{
	HEADER_PAIR = 0b00000000, // blocks type 0b000 as tracker_id 0 does exist
	TYPE_INFO_STATUS = 0b001,
	TYPE_IMU_CAYLEY = 0b010,
	// Remaining types are for future uses / revisions
	TYPE_GENERIC_HID = 0b100, // First byte is used to specify exact HID format (tbd)
	TYPE_REGISTER = 0b111,
	HEADER_SKIP = 0b11111111,
};

enum PACKET_RESERVED_SIZES
{
	SIZE_MAX_NORMAL = 15,
	SIZE_TIMESTAMPED = 16,
	SIZE_TIMESTAMPED_STATUS = 19,
	// All packets of size 15 or below are forwarded as (1B header, 14B Data & 0-Padding)
	// Packets of SIZE_TIMESTAMPED are forwarded as (1B header, 12B Data & 0-Padding, 2B Timestamp)
	// Packets of SIZE_TIMESTAMPED_STATUS are the same except they also update a 3-byte status
};

uint8_t connection_get_id(void);
void connection_set_id(uint8_t id);

void connection_update_sensor_ids(int imu_id, int mag_id);
void connection_update_sensor_data(float *q, float *a, uint64_t timestamp_us);
void connection_update_sensor_temp(float temp);
void connection_update_battery(
	bool battery_available,
	bool plugged,
	uint32_t battery_pptt,
	int battery_mV
);
void connection_update_status(int status);

void connection_write_info_status();
void connection_write_sensors();
void connection_write_sensors_timestamped();
void connection_write_sensors_timestamped_status();

#endif
