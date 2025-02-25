
#ifndef TIME_SYNC_H
#define TIME_SYNC_H

#include <stdint.h>

/*
 * Time Sync
 * Determines a best-estimate time sync between two systems (here over USB)
 * Assumes no inherent transfer time, but variable latency
 * Estimates from lowest-latency transfer
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
} time_sync_t;

static inline void init_time_sync(time_sync_t *time)
{
    time->measurements = 0;
    time->last_timestep = 0;
    time->last_timestamp = 0;
    time->factor = 0.0f; // Includes conversion ratio and drift
    time->correction_up = 0.0005f;
    time->correction_down = 0.1f;
    time->const_offset_us = 0;
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

	if (timestep < time->last_timestep) return measurement;
	// Won't deal with that case here

	// Predict real-time of timestep based on time-local estimation of time synchronisation
	uint32_t passed = (uint32_t)(timestep - time->last_timestep);
	float drift_us = (float)passed * time->factor;
	uint64_t time_pred = time->last_timestamp + passed + (uint32_t)drift_us;

	if (time->measurements++ == 0)
	{ // Initialise first measurement
		time->last_timestamp = measurement;
		time->last_timestep = timestep;
		return measurement;
	}

	// Update time-local estimation of time synchronisation with new measurement
	if (measurement < time_pred)
	{ // New minimum, upper bound for the actual time sync, correct and adapt factor slowly
		uint32_t diffUS = (uint32_t)(time_pred-measurement);
		time->factor -= (float)diffUS * time->correction_down / (float)passed;
		time_pred = measurement;
	}
	else
	{ // Higher than predicted, assume not significantly delayed, but primarily factor, so adapt slowly
		uint32_t diffUS = (uint32_t)(measurement-time_pred);
        float correction_us = (float)diffUS * time->correction_up;
		time->factor += correction_us / (float)passed;
		//time_pred += (uint32_t)correction_us; // Probably unnecessary, will be small
	}

	time->last_timestamp = time_pred;
	time->last_timestep = timestep;
	return time_pred + time->const_offset_us;
}

#endif // TIME_SYNC_H