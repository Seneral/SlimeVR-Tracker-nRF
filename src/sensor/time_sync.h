
#ifndef TIME_SYNC_H
#define TIME_SYNC_H

#include <stdint.h>

#include <zephyr/logging/log.h>

/*
 * Time Sync
 * Determines a best-estimate time sync between two systems
 * This variant is tuned for embedded systems and high-rate wire protocols (e.g. I2C):
 * - 32Bit integer operation if possible
 * - expecting low latencies
 * - expecting consistent communication
 */

typedef struct time_sync
{
	uint32_t measurements;
	uint64_t last_timestep;
	uint64_t last_timestamp;
	float factor; // Includes conversion ratio and drift
	float correction_up;
	float correction_down;
	int const_offset_us;
	uint16_t init_period;
} time_sync_t;

static inline void init_time_sync(time_sync_t *time)
{
	time->measurements = 0;
	time->last_timestep = 0;
	time->last_timestamp = 0;
	time->factor = 1.0f; // Includes conversion ratio and drift
	time->correction_up = 0.0005f;
	time->correction_down = 0.1f;
	time->const_offset_us = 0;
	time->init_period = 10;
	LOG_INF("Init time sync with factor %f", time->factor);
}

static inline uint64_t get_time_synced_past(const time_sync_t *time, uint64_t timestep)
{
	uint32_t passed = (uint32_t)(time->last_timestep-timestep);
	passed = (uint32_t)((float)passed * time->factor);
	return time->last_timestamp - passed;
}

static inline uint64_t get_time_synced_future(const time_sync_t *time, uint64_t timestep)
{
	uint32_t passed = (uint32_t)(timestep - time->last_timestep);
	passed = (uint32_t)((float)passed * time->factor);
	return time->last_timestamp + passed;
}

static inline uint64_t get_time_synced(const time_sync_t *time, uint64_t timestep)
{
	if (timestep < time->last_timestep)
		return get_time_synced_past(time, timestep);
	else
		return get_time_synced_future(time, timestep);
}

static uint64_t update_time_synced(time_sync_t *time, uint64_t timestep, uint64_t measurement)
{ // Received packet timestep with best real-time estimation being measurement

	if (time->measurements++ < time->init_period)
	{ // Initialise first measurement
		float lerp = 1.0f - 1.0f/time->init_period;
		if (time->measurements > 2)
			time->factor = time->factor*lerp + (float)(measurement-time->last_timestamp)/(float)(timestep-time->last_timestep) * (1-lerp);
		time->last_timestamp = measurement;
		time->last_timestep = timestep;
		return measurement;
	}
	/* LOG_INF("Received time sync measurement %lld (diff %lld) and steps %lld (diff %lld) with factor %f!",
		measurement, (int64_t)measurement-(int64_t)time->last_timestamp, timestep, (int64_t)timestep-(int64_t)time->last_timestep, time->factor); */

	if (timestep < time->last_timestep)
	{ // Can't update past timestamp
		LOG_WRN("Cannot update synced time from past timestamp %lld < %lld!", timestep, time->last_timestep);
		return get_time_synced_past(time, timestep);
	}

	// Predict real-time of timestep based on time-local estimation of time synchronisation
	uint32_t passed = (uint32_t)(timestep - time->last_timestep);
	float passed_us = (float)passed * time->factor;
	uint64_t time_pred = time->last_timestamp + (uint32_t)passed_us;
	//LOG_INF("Predicted update time to be %llu (last %llu, passed %u/%fus, factor %f)!", time_pred, time->last_timestamp, passed, passed_us, time->factor);

	// Update time-local estimation of time synchronisation with new measurement
	if (measurement < time_pred)
	{ // New minimum, upper bound for the actual time sync, correct and adapt factor slowly
		uint32_t diff_us = (uint32_t)(time_pred-measurement);
		time->factor -= (float)diff_us * time->correction_down / (float)passed;
		time_pred = measurement;
	}
	else
	{ // Higher than predicted, assume not significantly delayed, but primarily factor, so adapt slowly
		uint32_t diff_us = (uint32_t)(measurement-time_pred);
		float correction_us = (float)diff_us * time->correction_up;
		time->factor += correction_us / (float)passed;
		time_pred += (uint32_t)correction_us; // Probably unnecessary, will be small
	}

	//LOG_INF("Updated synced time from measurement %lld to synced %lld with factor %f", measurement, time_pred, time->factor);

	time->last_timestamp = time_pred;
	time->last_timestep = timestep;
	return time_pred + time->const_offset_us;
}

#endif // TIME_SYNC_H