//
// PHL Functions
//
#ifndef PHL_FUNCTION
#define PHL_FUNCTION

// Include
#include <stdint.h>

/// @brief Calculate time difference, including rollover (us or ms)
/// - Rollover (us) = ~4295s -> ~1H 11',
/// - Rollover (ms) = ~71580' -> ~49 days, 17h,
/// @param cur Curent time
/// @param last Last time
/// @return Delta value, (including overroll)
static uint32_t pTimeDiff (uint32_t cur, uint32_t last)
{
    unsigned long delta_t;

	// Delta time (us) 
	if (cur < last) 
		delta_t = (4294967295 - last) + cur; 	// Rollover
	else 
		delta_t = cur - last;

    return delta_t;
}

#endif
//
// EOF
//