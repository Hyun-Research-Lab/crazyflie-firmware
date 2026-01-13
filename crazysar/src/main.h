#include "stabilizer_types.h"

#define FILTER_SIZE 50
#define NETWORK_RATE RATE_100_HZ
#define NODE_UNSET UINT8_MAX

void disturbance_observer_step(struct vec* u, const struct vec* re, const struct vec* re_dot, const struct vec* b1);