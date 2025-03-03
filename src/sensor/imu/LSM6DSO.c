#include <math.h>

#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>

#include "LSM6DSO.h"
#include "LSM6DSV.h" // Common functions

#define PACKET_SIZE 7

static const float accel_sensitivity = 16.0f / 32768.0f; // Always 16G (FS = ±16 g: 0.488 mg/LSB)
static const float gyro_sensitivity = 0.070f; // Always 2000dps (FS = ±2000 dps: 70 mdps/LSB)

static uint8_t last_accel_mode = 0xff;
static uint8_t last_gyro_mode = 0xff;
static uint8_t last_accel_odr = 0xff;
static uint8_t last_gyro_odr = 0xff;

static uint8_t ext_addr = 0xff;
static uint8_t ext_reg = 0xff;
static bool use_ext_fifo = false;

static uint8_t last_accel_bdr = 0xff;
static uint8_t last_gyro_bdr = 0xff;
static uint8_t last_ext_bdr = 0xff;

static uint32_t time_slot_factor = 1;
static uint8_t cur_time_slot = 0;
static uint32_t track_time_slot = 0;

LOG_MODULE_REGISTER(LSM6DSO, LOG_LEVEL_DBG);

static int update_time_scale()
{
	float max_odr = MAX(DSO_ODR_GYRO_MAP[last_gyro_odr], DSO_ODR_ACCEL_MAP[last_accel_odr]);
	float max_bdr = MAX(DSO_BDR_GYRO_MAP[last_gyro_bdr], DSO_BDR_ACCEL_MAP[last_accel_bdr]);
	time_slot_factor = (int)round(max_odr/max_bdr);
}

int lsm6dso_init(const struct i2c_dt_spec *dev_i2c, float clock_rate, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time, float *timestep_us)
{
	int err = i2c_reg_write_byte_dt(dev_i2c, LSM6DSO_CTRL3, 0x44); // freeze register until done reading, increment register address during multi-byte access (BDU, IF_INC)
	err |= i2c_reg_write_byte_dt(dev_i2c, LSM6DSO_CTRL8, 0x00); // Old mode (allows 16g) | XL_FS_MODE = 0
	// Initialise ODRs
	last_accel_odr = last_gyro_odr = 0xff; // reset last odr
	last_accel_bdr = last_gyro_bdr = last_ext_bdr = 0xff; // reset last bdr
	err |= lsm6dso_update_odr(dev_i2c, accel_time, gyro_time, accel_actual_time, gyro_actual_time);
	// Set initial BDR, any later changes should be detected via FIFO packets
	last_accel_bdr = last_accel_odr;
	last_gyro_bdr = last_gyro_odr;
	// Init timesync
	cur_time_slot = 0;
	track_time_slot = 0;
	err |= update_time_scale();
	*timestep_us = 1000000.0f/MAX(DSO_ODR_GYRO_MAP[last_gyro_odr], DSO_ODR_ACCEL_MAP[last_accel_odr]);
	// Enable Continuous mode, with lowest ODR for timestamp and temperature
	err |= i2c_reg_write_byte_dt(dev_i2c, LSM6DSO_FIFO_CTRL4, 0xC0 | 0x10 | 0x06);
	// Enable timestamp (else timestamp packets will be 0)
	//err |= i2c_reg_write_byte_dt(dev_i2c, LSM6DSO_CTRL10, 0x20);
	if (use_ext_fifo)
		err |= lsm_ext_init(dev_i2c, ext_addr, ext_reg);
	if (err)
		LOG_ERR("I2C error");
	return (err < 0 ? err : 0);
}

void lsm6dso_shutdown(const struct i2c_dt_spec *dev_i2c)
{
	last_accel_odr = last_gyro_odr = 0xff; // reset last odr
	last_accel_bdr = last_gyro_bdr = last_ext_bdr = 0xff; // reset last bdr
	int err = i2c_reg_write_byte_dt(dev_i2c, LSM6DSO_CTRL3, 0x01); // SW_RESET
	if (err)
		LOG_ERR("I2C error");
}

int lsm6dso_update_odr(const struct i2c_dt_spec *dev_i2c, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	uint8_t OP_MODE_XL;
	uint8_t OP_MODE_G;
	uint8_t ODR_XL;
	uint8_t ODR_G;
	uint8_t GYRO_SLEEP = DSO_OP_MODE_G_AWAKE;

	// Calculate accel
	if (accel_time <= 0 || accel_time == INFINITY)
	{ // off, standby interpreted as off
		OP_MODE_XL = DSO_OP_MODE_XL_HP;
		ODR_XL = DSO_ODR_OFF;
		accel_time = 0;
	}
	else
	{ // set High perf mode and select odr on XL
		OP_MODE_XL = DSO_OP_MODE_XL_HP;
		ODR_XL = DSO_ODR_12_5Hz;
		float desiredODR = 1 / accel_time;
		for (int i = 0; i < sizeof(DSO_ODR_ACCEL_MAP)/sizeof(float); i++)
		{
			if (desiredODR > DSO_ODR_ACCEL_MAP[i] && DSO_ODR_ACCEL_MAP[i] > DSO_ODR_ACCEL_MAP[ODR_XL])
				ODR_XL = i;
		}
		accel_time = 1.0f / DSO_ODR_ACCEL_MAP[ODR_XL];
	}

	// Calculate gyro
	if (gyro_time <= 0)
	{ // off
		OP_MODE_G = DSO_OP_MODE_G_HP;
		ODR_G = DSO_ODR_OFF;
		gyro_time = 0;
	}
	else if (gyro_time == INFINITY)
	{ // sleep
		OP_MODE_G = DSO_OP_MODE_G_LP;
		GYRO_SLEEP = DSO_OP_MODE_G_SLEEP;
		ODR_G = last_gyro_odr; // using last ODR
		gyro_time = 0; // off
	}
	else
	{
		OP_MODE_G = DSO_OP_MODE_G_HP;
		ODR_G = DSO_ODR_12_5Hz;
		float desiredODR = 1 / gyro_time;
		for (int i = 0; i < sizeof(DSO_ODR_GYRO_MAP)/sizeof(float); i++)
		{
			if (desiredODR > DSO_ODR_GYRO_MAP[i] && DSO_ODR_GYRO_MAP[i] > DSO_ODR_GYRO_MAP[ODR_G])
				ODR_G = i;
		}
		gyro_time = 1.0f / DSO_ODR_GYRO_MAP[ODR_G];
	}

	if (last_accel_mode == OP_MODE_XL && last_gyro_mode == OP_MODE_G && last_accel_odr == ODR_XL && last_gyro_odr == ODR_G) // if both were already configured
		return 1;

	last_accel_mode = OP_MODE_XL;
	last_gyro_mode = OP_MODE_G;
	last_accel_odr = ODR_XL;
	last_gyro_odr = ODR_G;

	int err = i2c_reg_write_byte_dt(dev_i2c, LSM6DSO_CTRL1, (ODR_XL << 4) | DSO_FS_XL_16G); // set accel ODR and FS
	err |= i2c_reg_write_byte_dt(dev_i2c, LSM6DSO_CTRL6, OP_MODE_XL); // set accelerator perf mode

	err |= i2c_reg_write_byte_dt(dev_i2c, LSM6DSO_CTRL2, (ODR_G << 4) | DSO_FS_G_2000DPS); // set gyro ODR and mode
	err |= i2c_reg_write_byte_dt(dev_i2c, LSM6DSO_CTRL7, OP_MODE_G); // set gyroscope perf mode
	err |= i2c_reg_write_byte_dt(dev_i2c, LSM6DSO_CTRL4, GYRO_SLEEP); // set gyroscope awake/sleep mode

	err |= i2c_reg_write_byte_dt(dev_i2c, LSM6DSO_FIFO_CTRL3, ODR_XL << 0 | ODR_G << 4); // set same BDR
	if (err)
		LOG_ERR("I2C error");

	*accel_actual_time = accel_time;
	*gyro_actual_time = gyro_time;

	return 0;
}

uint16_t lsm6dso_fifo_read(const struct i2c_dt_spec *dev_i2c, uint8_t *data, uint16_t len)
{
	int err = 0;
	uint16_t total = 0;
	uint16_t count = UINT16_MAX;
	while (count > 0 && len >= PACKET_SIZE)
	{
		uint8_t rawCount[2];
		err |= i2c_burst_read_dt(dev_i2c, LSM6DSO_FIFO_STATUS1, &rawCount[0], 2);
		count = (uint16_t)((rawCount[1] & 3) << 8 | rawCount[0]); // Turn the 16 bits into a unsigned 16-bit value
		uint16_t limit = len / PACKET_SIZE;
		if (count > limit)
			count = limit;
		for (int i = 0; i < count; i++)
			err |= i2c_burst_read_dt(dev_i2c, LSM6DSO_FIFO_DATA_OUT_TAG, &data[i * PACKET_SIZE], PACKET_SIZE);
		if (err)
			LOG_ERR("I2C error");
		data += count * PACKET_SIZE;
		len -= count * PACKET_SIZE;
		total += count;
	}
	return total;
}

bool lsm_parse_fifo_packet(sensor_packet_t *packet, uint8_t data[PACKET_SIZE])
{
	{ // TODO: Configure timestamps?
		// Track advances in timestamp via a 3-bit timeslot in packets
		int time_slot = (data[0] >> 1) & 0x3;
		int diff_slots = time_slot-cur_time_slot;
		if (diff_slots < -1) diff_slots += 4;
		cur_time_slot = time_slot;
		// Update timestamp with advance in timeslot
		track_time_slot += diff_slots * time_slot_factor;
		//uint32_t slot_to_us = 1000000/MAX(DSO_ODR_GYRO_MAP[last_gyro_odr], DSO_ODR_ACCEL_MAP[last_accel_odr]);
		packet->timestep = track_time_slot; // Could convert to us, but overflow behaviour would become less predictable
	}

	int sensorTag = data[0] >> 3;
	switch (sensorTag)
	{
		case 0x01:
			packet->tag = SENSOR_GYRO;
			for (int i = 0; i < 3; i++) // x, y, z
			{
				packet->data.gyro[i] = (int16_t)(data[i*2 + 1] | (((uint16_t)data[i*2 + 2]) << 8));
				packet->data.gyro[i] *= gyro_sensitivity;
			}
			return true;
		case 0x02:
			packet->tag = SENSOR_ACCEL;
			for (int i = 0; i < 3; i++) // x, y, z
			{
				packet->data.accel[i] = (int16_t)(data[i*2 + 1] | (((uint16_t)data[i*2 + 2]) << 8));
				packet->data.accel[i] *= accel_sensitivity;
			}
			return true;
		case 0x03:
			packet->tag = SENSOR_TEMP;
			packet->data.temp[0] = (int16_t)(data[1] | (((uint16_t)data[2]) << 8));
			packet->data.temp[0] = packet->data.temp[0] / 256 + 25;
			return true;
		case 0x04:
		{
			// Not using timestamps, doesn't make much sense if we can track time slots
			// Needs synchronisation and estimation of drift either way
			// Test implementation at https://github.com/Seneral/SlimeVR-Tracker-nRF/tree/timestamps-us 
			//uint32_t timestamp = data[1] | (((uint32_t)data[2]) << 8)
			//	| (((uint32_t)data[3]) << 16) | (((uint32_t)data[4]) << 24);
			// Check if BDRs changed
			uint8_t ext_bdr = data[5] & 0x0F, accel_bdr = data[6] & 0x0F, gyro_bdr = (data[6] >> 4) & 0x0F;
			if (gyro_bdr != last_gyro_bdr || accel_bdr != last_accel_bdr || ext_bdr != last_ext_bdr)
			{
				LOG_INF("Set Gyro BDR from %f to %f with ODR at %f!",
					DSO_BDR_GYRO_MAP[last_gyro_bdr], DSO_BDR_GYRO_MAP[gyro_bdr], DSO_ODR_GYRO_MAP[last_gyro_odr]);
				last_gyro_bdr = gyro_bdr;
				last_accel_bdr = accel_bdr;
				last_ext_bdr = ext_bdr;
				packet->tag = SENSOR_UPDATE;
				packet->data.BDRupdate[0] = DSO_BDR_GYRO_MAP[last_gyro_bdr];
				packet->data.BDRupdate[1] = DSO_BDR_ACCEL_MAP[last_accel_bdr];
				packet->data.BDRupdate[2] = DSO_BDR_EXT_MAP[last_ext_bdr];
				return true;
			}
			return false;
		}
		case 0x05:
			// TODO: Add CFG-change fifo packets to detect ODR & BDR changes
			// Relying on timestamp packet for BDR changes for now
			// These are currently not enabled, set ODRCHG_EN
			LOG_ERR("IMU CFG-Change packet not implemented!");
			return false;
		case 0x0E:
			packet->tag = SENSOR_EXT;
			memcpy(packet->data.ext, &data[1], 6);
			return true;
		default:
			// NOT IMPLEMENTED
			LOG_ERR("IMU FIFO Packet %d not implemented!", sensorTag);
			return false;
	}
}

int lsm6dso_fetch_sensor_packets(const struct i2c_dt_spec *dev_i2c, int max_count, handle_sensor_packet_t cb, void *userdata)
{
	int err = 0;
	uint16_t total = 0;
	uint16_t count = UINT16_MAX;
	uint8_t rawCount[2];
	err |= i2c_burst_read_dt(dev_i2c, LSM6DSO_FIFO_STATUS1, &rawCount[0], 2);
	count = (uint16_t)((rawCount[1] & 3) << 8 | rawCount[0]); // Turn the 16 bits into a unsigned 16-bit value
	if (count > max_count) count = max_count;

	uint8_t fifoBuffer[PACKET_SIZE];
	uint8_t fifoRegister = LSM6DSO_FIFO_DATA_OUT_TAG;
	struct i2c_msg fifoMsgs[2] = {
		{ &fifoRegister, 1, I2C_MSG_WRITE },
		{ fifoBuffer, PACKET_SIZE, I2C_MSG_RESTART | I2C_MSG_READ | I2C_MSG_STOP }
	};
	// TODO: Make use of wraparound and direct access to dev_i2c->bus->api->transfer to NOT send I2C_MSG_STOP
	// That way, we can just not send the fifoRegister again, and just keep reading continuously
	sensor_packet_t fifoPacket;
	for (int i = 0; i < count; i++)
	{
		err |= i2c_transfer_dt(dev_i2c, fifoMsgs, 2);
		if (lsm_parse_fifo_packet(&fifoPacket, fifoBuffer))
		{ // Parsed a sensor_packet that should be exposed
			cb(userdata, fifoPacket);
			total++;
		}
	}
	if (err)
		LOG_ERR("I2C error");
	return total;
}

void lsm6dso_setup_WOM(const struct i2c_dt_spec *dev_i2c)
{ // TODO: should be off by the time WOM will be setup
//	i2c_reg_write_byte_dt(dev_i2c, LSM6DSO_CTRL1, (DSO_ODR_OFF << 4)); // set accel off
//	i2c_reg_write_byte_dt(dev_i2c, LSM6DSO_CTRL2, (DSO_ODR_OFF << 4)); // set gyro off

	int err = i2c_reg_write_byte_dt(dev_i2c, LSM6DSO_CTRL1, (DSO_ODR_208Hz << 4) | DSO_FS_XL_8G); // set accel ODR and FS
	err |= i2c_reg_write_byte_dt(dev_i2c, LSM6DSO_CTRL6, DSO_OP_MODE_XL_LP | 0x08); // set accel perf mode, set offset weight to 2^-6 g/LSB
	err |= i2c_reg_write_byte_dt(dev_i2c, LSM6DSO_CTRL5, 0x80); // enable accel ULP // TODO: for LSM6DSR/ISM330DHCX this bit may be required to be 0
	err |= i2c_reg_write_byte_dt(dev_i2c, LSM6DSO_TAP_CFG0, 0x10); // set SLOPE_FDS (using user offset for wake-up)
	err |= i2c_reg_write_byte_dt(dev_i2c, LSM6DSO_WAKE_UP_THS, 0x40 | 0x01); // use offset correction for wake-up, set threshold, 1 * 31.25 mg is ~31.25 mg
	err |= i2c_reg_write_byte_dt(dev_i2c, LSM6DSO_WAKE_UP_DUR, 0x10); // set 1 LSB threshold to FS_XL / 256 (31.25mg)
	k_msleep(12); // need to wait for accel to settle

	float accel_reference[3] = {0};
	lsm_accel_read(dev_i2c, accel_reference); // need to read a reference value to set offset
	int8_t offset[3] = {0};
	for (int i = 0; i < 3; i++) // calculate offset
	{
		accel_reference[i] /= 2; // FS_XL_8G to FS_XL_16G
		// dont invert, for some reason
		accel_reference[i] *= 64; // offset is 2^-6 g/LSB
		accel_reference[i] += accel_reference[i] < 0 ? -0.5f : 0.5f; // round
		offset[i] = CLAMP(accel_reference[i], -127, 127); // value must be in the range -127 to 127
	}
	err |= i2c_burst_write_dt(dev_i2c, LSM6DSO_X_OFS_USR, offset, 3); // set offset correction

	err |= i2c_reg_write_byte_dt(dev_i2c, LSM6DSO_TAP_CFG2, 0x80); // enable interrupts
	err |= i2c_reg_write_byte_dt(dev_i2c, LSM6DSO_MD1_CFG, 0x20); // route wake-up to INT1
	if (err)
		LOG_ERR("I2C error");
}

int lsm6dso_ext_setup(uint8_t addr, uint8_t reg)
{
	ext_addr = addr;
	ext_reg = reg;
	if (addr != 0xff && addr != 0xff)
	{
		use_ext_fifo = true;
		return 0;
	}
	else
	{
		use_ext_fifo = false;
		return 1;
	}
}

extern const sensor_imu_t sensor_imu_lsm6dso = {
	*lsm6dso_init,
	*lsm6dso_shutdown,

	*lsm6dso_update_odr,

	*lsm6dso_fifo_read,
	*lsm_fifo_process,
	*lsm_accel_read,
	*lsm_gyro_read,
	*lsm_temp_read,

	*lsm6dso_setup_WOM,

	*lsm6dso_fetch_sensor_packets,
	
	*lsm6dso_ext_setup,
	*lsm_fifo_process_ext,
	*lsm_ext_read,
	*lsm_ext_passthrough
};
