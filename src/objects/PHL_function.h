//
// PHL Functions
//

#ifndef PHL_FUNCTION_H
#define PHL_FUNCTION_H

#include <stdint.h>

// Time
uint32_t pTimeDiff (uint32_t cur, uint32_t last);
//#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))
//#define pTimeDiff (cur, last) ((cur) < (last) ? (4294967295 - (last)) + (cur) : (last) - (cur))

#endif
//
// EOF
//