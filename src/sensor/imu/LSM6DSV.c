#include <math.h>

#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>

#include "LSM6DSV.h"

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

static float maximum_bdr = 0xff;
static uint32_t time_slot_us;
static uint32_t timestamp_to_us;
static uint64_t overflow_us;

static uint64_t cur_time_us = 0;
static uint8_t cur_time_slot = 0;

LOG_MODULE_REGISTER(LSM6DSV, LOG_LEVEL_DBG);

static void update_time_scale()
{
	// TODO: Does changing bdr actually change time slot scale?
	// Seems kinda messed up if timestamp itself stays the same but suddenly means something completely different
	maximum_bdr = MAX(DSV_ODR_GYRO_MAP[last_gyro_bdr], DSV_ODR_ACCEL_MAP[last_accel_bdr]);
	time_slot_us = (uint32_t)(1000000/maximum_bdr); // Only loose neglibible precision here
	// TODO: Timestamp scale is slightly off, but can be modelled as drift
	timestamp_to_us = 25;
	//timestamp_to_us = 25.0f/(1 + 0.0015f * FREQ_FINE);
	// IF we fetch FREQ_FINE, consider we might be in fifo fetch and doing I2C here might mess up address wraparound
	overflow_us = (uint64_t)0xFFFFFFFF * timestamp_to_us;
}

int lsm_init(const struct i2c_dt_spec *dev_i2c, float clock_rate, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	int err = i2c_reg_write_byte_dt(dev_i2c, LSM6DSV_CTRL6, DSV_FS_G_2000DPS); // set gyro FS
	err |= i2c_reg_write_byte_dt(dev_i2c, LSM6DSV_CTRL8, DSV_FS_XL_16G); // set accel FS
	if (err)
		LOG_ERR("I2C error");
	last_accel_odr = last_gyro_odr = 0xff; // reset last odr
	last_accel_bdr = last_gyro_bdr = last_ext_bdr = 0; // reset last bdr
	err |= lsm_update_odr(dev_i2c, accel_time, gyro_time, accel_actual_time, gyro_actual_time);
	// Set initial BDR, any later changes should be detected via FIFO packets
	last_accel_bdr = last_accel_odr;
	last_gyro_bdr = last_gyro_odr;
	update_time_scale();
	// Enable Continuous mode, with lowest ODR for timestamp and temperature
	err |= i2c_reg_write_byte_dt(dev_i2c, LSM6DSV_FIFO_CTRL4, 0xC0 | 0x10 | 0x06);
	// Enable timestamp (else timestamp packets will be 0)
	err |= i2c_reg_write_byte_dt(dev_i2c, LSM6DSV_FUNCTIONS_ENABLE, 0x40);
	if (err)
		LOG_ERR("I2C error");
	if (use_ext_fifo)
		err |= lsm_ext_init(dev_i2c, ext_addr, ext_reg);
	return (err < 0 ? err : 0);
}

void lsm_shutdown(const struct i2c_dt_spec *dev_i2c)
{
	last_accel_odr = last_gyro_odr = 0xff; // reset last odr
	last_accel_bdr = last_gyro_bdr = last_ext_bdr = 0xff; // reset last bdr
	int err = i2c_reg_write_byte_dt(dev_i2c, LSM6DSV_CTRL3, 0x01); // SW_RESET
	if (err)
		LOG_ERR("I2C error");
}

int lsm_update_odr(const struct i2c_dt_spec *dev_i2c, float accel_time, float gyro_time, float *accel_actual_time, float *gyro_actual_time)
{
	uint8_t OP_MODE_XL;
	uint8_t OP_MODE_G;
	uint8_t ODR_XL;
	uint8_t ODR_G;

	// Calculate accel
	if (accel_time <= 0 || accel_time == INFINITY)
	{ // off, standby interpreted as off
		OP_MODE_XL = DSV_OP_MODE_XL_HP;
		ODR_XL = DSV_ODR_OFF;
		accel_time = 0;
	}
	else
	{ // set High perf mode and select odr on XL
		OP_MODE_XL = DSV_OP_MODE_XL_HP;
		ODR_XL = DSV_ODR_1_875Hz;
		float desiredODR = 1 / accel_time;
		for (int i = 0; i < sizeof(DSV_ODR_ACCEL_MAP)/sizeof(float); i++)
	{
			if (desiredODR > DSV_ODR_ACCEL_MAP[i] && DSV_ODR_ACCEL_MAP[i] > DSV_ODR_ACCEL_MAP[ODR_XL])
				ODR_XL = i;
		}
		accel_time = 1.0f / DSV_ODR_ACCEL_MAP[ODR_XL];
	}

	// Calculate gyro
	if (gyro_time <= 0)
	{ // off
		OP_MODE_G = DSV_OP_MODE_G_HP;
		ODR_G = DSV_ODR_OFF;
		gyro_time = 0;
	}
	else if (gyro_time == INFINITY)
	{ // sleep
		OP_MODE_G = DSV_OP_MODE_G_SLEEP;
		ODR_G = last_gyro_odr; // using last ODR
		gyro_time = 0; // off
	}
	else
	{
		OP_MODE_G = DSV_OP_MODE_G_HP;
		ODR_G = DSV_ODR_7_5Hz;
		float desiredODR = 1 / gyro_time;
		for (int i = 0; i < sizeof(DSV_ODR_GYRO_MAP)/sizeof(float); i++)
		{
			if (desiredODR > DSV_ODR_GYRO_MAP[i] && DSV_ODR_GYRO_MAP[i] > DSV_ODR_GYRO_MAP[ODR_G])
				ODR_G = i;
		}
		gyro_time = 1.0f / DSV_ODR_GYRO_MAP[ODR_G];
	}

	if (last_accel_mode == OP_MODE_XL && last_gyro_mode == OP_MODE_G && last_accel_odr == ODR_XL && last_gyro_odr == ODR_G) // if both were already configured
		return 1;

	last_accel_mode = OP_MODE_XL;
	last_gyro_mode = OP_MODE_G;
	last_accel_odr = ODR_XL;
	last_gyro_odr = ODR_G;

	int err = i2c_reg_write_byte_dt(dev_i2c, LSM6DSV_CTRL1, OP_MODE_XL << 4 | ODR_XL); // set accel ODR and mode
	err |= i2c_reg_write_byte_dt(dev_i2c, LSM6DSV_CTRL2, OP_MODE_G << 4 | ODR_G); // set gyro ODR and mode

	err |= i2c_reg_write_byte_dt(dev_i2c, LSM6DSV_FIFO_CTRL3, ODR_XL | (ODR_G << 4)); // set accel and gyro batch rate
	if (err)
		LOG_ERR("I2C error");

	*accel_actual_time = accel_time;
	*gyro_actual_time = gyro_time;

	return 0;
}

uint16_t lsm_fifo_read(const struct i2c_dt_spec *dev_i2c, uint8_t *data, uint16_t len)
{
	int err = 0;
	uint16_t total = 0;
	uint16_t count = UINT16_MAX;
	while (count > 0 && len >= PACKET_SIZE)
	{
		uint8_t rawCount[2];
		err |= i2c_burst_read_dt(dev_i2c, LSM6DSV_FIFO_STATUS1, &rawCount[0], 2);
		count = (uint16_t)((rawCount[1] & 3) << 8 | rawCount[0]); // Turn the 16 bits into a unsigned 16-bit value. Only LSB on FIFO_STATUS2 is used, but we mask 2nd bit too
		uint16_t limit = len / PACKET_SIZE;
		if (count > limit)
			count = limit;
		for (int i = 0; i < count; i++)
			err |= i2c_burst_read_dt(dev_i2c, LSM6DSV_FIFO_DATA_OUT_TAG, &data[i * PACKET_SIZE], PACKET_SIZE);
		if (err)
			LOG_ERR("I2C error");
		data += count * PACKET_SIZE;
		len -= count * PACKET_SIZE;
		total += count;
	}
	return total;
}

int lsm_fifo_process(uint16_t index, uint8_t *data, float a[3], float g[3])
{
	index *= PACKET_SIZE;
	if ((data[index] >> 3) == 0x02) // Accelerometer NC (Accelerometer uncompressed data)
	{
		for (int i = 0; i < 3; i++) // x, y, z
		{
			a[i] = (int16_t)((((uint16_t)data[index + 2 + (i * 2)]) << 8) | data[index + 1 + (i * 2)]);
			a[i] *= accel_sensitivity;
		}
		return 0;
	}
	if ((data[index] >> 3) == 0x01) // Gyroscope NC (Gyroscope uncompressed data)
	{
		for (int i = 0; i < 3; i++) // x, y, z
		{
			g[i] = (int16_t)((((uint16_t)data[index + 2 + (i * 2)]) << 8) | data[index + 1 + (i * 2)]);
			g[i] *= gyro_sensitivity;
		}
		return 0;
	}
	// TODO: need to skip invalid data
	return 1;
}

void lsm_accel_read(const struct i2c_dt_spec *dev_i2c, float a[3])
{
	uint8_t rawAccel[6];
	int err = i2c_burst_read_dt(dev_i2c, LSM6DSV_OUTX_L_A, &rawAccel[0], 6);
	if (err)
		LOG_ERR("I2C error");
	for (int i = 0; i < 3; i++) // x, y, z
	{
		a[i] = (int16_t)((((uint16_t)rawAccel[1 + (i * 2)]) << 8) | rawAccel[i * 2]);
		a[i] *= accel_sensitivity;
	}
	// TODO: for ISM330BX, the accelerometer data is in ZYX order
}

void lsm_gyro_read(const struct i2c_dt_spec *dev_i2c, float g[3])
{
	uint8_t rawGyro[6];
	int err = i2c_burst_read_dt(dev_i2c, LSM6DSV_OUTX_L_G, &rawGyro[0], 6);
	if (err)
		LOG_ERR("I2C error");
	for (int i = 0; i < 3; i++) // x, y, z
	{
		g[i] = (int16_t)((((uint16_t)rawGyro[1 + (i * 2)]) << 8) | rawGyro[i * 2]);
		g[i] *= gyro_sensitivity;
	}
}

float lsm_temp_read(const struct i2c_dt_spec *dev_i2c)
{
	uint8_t rawTemp[2];
	int err = i2c_burst_read_dt(dev_i2c, LSM6DSV_OUT_TEMP_L, &rawTemp[0], 2);
	if (err)
		LOG_ERR("I2C error");
	// TSen Temperature sensitivity 256 LSB/°C
	// The output of the temperature sensor is 0 LSB (typ.) at 25°C
	float temp = (int16_t)((((uint16_t)rawTemp[1]) << 8) | rawTemp[0]);
	temp /= 256;
	temp += 25;
	return temp;
}

static inline bool lsm6dsv_parse_fifo_packet(sensor_packet_t *packet, uint8_t data[PACKET_SIZE])
{
	if (true)
	{ // TODO: Configure timestamps?
		// Track advances in timestamp via a 3-bit timeslot in packets
		int timeSlot = (data[0] >> 1) & 0x3;
		int diffTimeSlots = timeSlot-cur_time_slot;
		if (diffTimeSlots < 0) diffTimeSlots += 4;
		cur_time_slot = timeSlot;
		// Update timestamp with advance in timeslot
		cur_time_us += diffTimeSlots*time_slot_us;
		//LOG_INF("Updated timeslots by %d and timestamp to %f!", diffTimeSlots, cur_time_us);
		packet->timestampUS = cur_time_us;
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
			// Synchronise timestamp
			uint32_t timestamp = data[1] | (((uint32_t)data[2]) << 8)
				| (((uint32_t)data[3]) << 16) | (((uint32_t)data[4]) << 24);
			uint64_t timestamp_us = (uint64_t)timestamp*timestamp_to_us;
			long diff = (long)cur_time_us-timestamp_us;
			// TODO: Handle overflows at overflow_us
			if (abs((int)diff) > 30)
				LOG_WRN("Updating timestamp from %lldus to %lldus!", cur_time_us, timestamp_us);
			cur_time_us = timestamp_us;
			// Check if BDRs changed
			uint8_t ext_bdr = data[5] & 0x0F, accel_bdr = data[6] & 0x0F, gyro_bdr = (data[6] >> 4) & 0x0F;
			if (gyro_bdr != last_gyro_bdr || accel_bdr != last_accel_bdr || ext_bdr != last_ext_bdr)
			{
				LOG_WRN("Set Gyro BDR from %f to %f with ODR at %f!",
					DSV_BDR_GYRO_MAP[last_gyro_bdr], DSV_BDR_GYRO_MAP[gyro_bdr], DSV_ODR_GYRO_MAP[last_gyro_odr]);
				last_gyro_bdr = gyro_bdr;
				last_accel_bdr = accel_bdr;
				last_ext_bdr = ext_bdr;
				update_time_scale();
				packet->tag = SENSOR_UPDATE;
				packet->data.BDRupdate[0] = DSV_BDR_GYRO_MAP[last_gyro_bdr];
				packet->data.BDRupdate[1] = DSV_BDR_ACCEL_MAP[last_accel_bdr];
				packet->data.BDRupdate[2] = DSV_BDR_EXT_MAP[last_ext_bdr];
				return true;
			}
			return false;
		}
		case 0x05:
			// TODO: Add CFG-change fifo packets to detect BDR changes
			// Relying on timestamp packet for BDR changes for now
			// These are currently not even enabled, set ODRCHG_EN
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

int lsm6dsv_fetch_sensor_packets(const struct i2c_dt_spec *dev_i2c, int max_count, handle_sensor_packet_t cb, void *userdata)
{
	int err = 0;
	uint16_t total = 0;
	uint16_t count = UINT16_MAX;
	uint8_t rawCount[2];
	err |= i2c_burst_read_dt(dev_i2c, LSM6DSV_FIFO_STATUS1, &rawCount[0], 2);
	count = (uint16_t)((rawCount[1] & 3) << 8 | rawCount[0]); // Turn the 16 bits into a unsigned 16-bit value
	if (count > max_count) count = max_count;

	uint8_t fifoBuffer[PACKET_SIZE];
	uint8_t fifoRegister = LSM6DSV_FIFO_DATA_OUT_TAG;
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
		if (lsm6dsv_parse_fifo_packet(&fifoPacket, fifoBuffer))
		{ // Parsed a sensor_packet that should be exposed
			cb(userdata, fifoPacket);
			total++;
		}
	}
	if (err)
		LOG_ERR("I2C error");
	return total;
}

void lsm_setup_WOM(const struct i2c_dt_spec *dev_i2c)
{ // TODO: should be off by the time WOM will be setup
//	i2c_reg_write_byte_dt(dev_i2c, LSM6DSV_CTRL1, DSV_ODR_OFF); // set accel off
//	i2c_reg_write_byte_dt(dev_i2c, LSM6DSV_CTRL2, DSV_ODR_OFF); // set gyro off

	int err = i2c_reg_write_byte_dt(dev_i2c, LSM6DSV_CTRL8, DSV_FS_XL_8G); // set accel FS
	err |= i2c_reg_write_byte_dt(dev_i2c, LSM6DSV_CTRL1, DSV_OP_MODE_XL_LP1 << 4 | DSV_ODR_240Hz); // set accel low power mode 1, set accel ODR (enable accel)
	err |= i2c_reg_write_byte_dt(dev_i2c, LSM6DSV_CTRL9, 0x02); // set offset weight to 2^-6 g/LSB
	err |= i2c_reg_write_byte_dt(dev_i2c, LSM6DSV_TAP_CFG0, 0x10); // set SLOPE_FDS (using user offset for wake-up)
	err |= i2c_reg_write_byte_dt(dev_i2c, LSM6DSV_WAKE_UP_THS, 0x40 | 0x04); // use offset correction for wake-up, set threshold, 4 * 7.8125 mg is ~31.25 mg
	k_msleep(11); // need to wait for accel to settle

	float accel_reference[3] = {0};
	lsm_accel_read(dev_i2c, accel_reference); // need to read a reference value to set offset
	int8_t offset[3] = {0};
	for (int i = 0; i < 3; i++) // calculate offset
	{
		accel_reference[i] /= 2; // FS_XL_8G to FS_XL_16G
		accel_reference[i] *= -1; // negate
		accel_reference[i] *= 64; // offset is 2^-6 g/LSB
		accel_reference[i] += accel_reference[i] < 0 ? -0.5f : 0.5f; // round
		offset[i] = CLAMP(accel_reference[i], -127, 127); // value must be in the range -127 to 127
	}
	err |= i2c_burst_write_dt(dev_i2c, LSM6DSV_X_OFS_USR, offset, 3); // set offset correction

	err |= i2c_reg_write_byte_dt(dev_i2c, LSM6DSV_FUNCTIONS_ENABLE, 0x80 | 0x40); // enable interrupts and timestamp
	err |= i2c_reg_write_byte_dt(dev_i2c, LSM6DSV_MD1_CFG, 0x20); // route wake-up to INT1
	if (err)
		LOG_ERR("I2C error");
}

int lsm_ext_setup(uint8_t addr, uint8_t reg)
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

int lsm_fifo_process_ext(uint16_t index, uint8_t *data, float a[3], float g[3], uint8_t *raw_m)
{
	if (!lsm_fifo_process(index, data, a, g)) // try processing a+g first
		return 0;
	index *= PACKET_SIZE;
	if ((data[index] >> 3) == 0x0E)
	{
		memcpy(raw_m, &data[index + 1], 6);
		return 0;
	}
	// TODO: need to skip invalid data
	return 1;
}

void lsm_ext_read(const struct i2c_dt_spec *dev_i2c, uint8_t *raw_m)
{
	int err = i2c_burst_read_dt(dev_i2c, LSM6DSV_SENSOR_HUB_1, raw_m, 6);
	if (err)
		LOG_ERR("I2C error");
}

int lsm_ext_passthrough(const struct i2c_dt_spec *dev_i2c, bool passthrough)
{
	int err = i2c_reg_write_byte_dt(dev_i2c, LSM6DSV_FUNC_CFG_ACCESS, 0x40); // switch to sensor hub registers
	err |= i2c_reg_write_byte_dt(dev_i2c, LSM6DSV_MASTER_CONFIG, passthrough ? 0x10 : 0x24); // toggle passthrough (trigger from INT2, MASTER_ON)
	err |= i2c_reg_write_byte_dt(dev_i2c, LSM6DSV_FUNC_CFG_ACCESS, 0x00); // switch to normal registers
	if (err)
		LOG_ERR("I2C error");
	return err;
}

int lsm_ext_init(const struct i2c_dt_spec *dev_i2c, uint8_t ext_addr, uint8_t ext_reg)
{
	int err = i2c_reg_write_byte_dt(dev_i2c, LSM6DSV_FUNC_CFG_ACCESS, 0x80); // enable sensor hub
	err |= i2c_reg_write_byte_dt(dev_i2c, LSM6DSV_MASTER_CONFIG, 0x24); // trigger from INT2, MASTER_ON
	err |= i2c_reg_write_byte_dt(dev_i2c, LSM6DSV_SLV0_ADD, (ext_addr << 1) | 0x01); // set external address, read mode
	err |= i2c_reg_write_byte_dt(dev_i2c, LSM6DSV_SLV0_SUBADD, ext_reg); // set external register
	err |= i2c_reg_write_byte_dt(dev_i2c, LSM6DSV_SLV0_CONFIG, 0x08 | 0x06); // enable external sensor fifo and set 6 read operations
	err |= i2c_reg_write_byte_dt(dev_i2c, LSM6DSV_FUNC_CFG_ACCESS, 0x00); // switch to normal registers
	if (err)
		LOG_ERR("I2C error");
	return err;
}

extern const sensor_imu_t sensor_imu_lsm6dsv = {
	*lsm_init,
	*lsm_shutdown,

	*lsm_update_odr,

	*lsm_fifo_read,
	*lsm_fifo_process,
	*lsm_accel_read,
	*lsm_gyro_read,
	*lsm_temp_read,

	*lsm_setup_WOM,

	*lsm6dsv_fetch_sensor_packets,
	
	*lsm_ext_setup,
	*lsm_fifo_process_ext,
	*lsm_ext_read,
	*lsm_ext_passthrough
};
