#include "stabilizer_types.h"

#define FILTER_SIZE 50
#define NETWORK_RATE RATE_100_HZ
#define NODE_UNSET UINT8_MAX

struct vec disturbance_observer_step(struct vec re, struct vec re_dot, struct vec u, struct vec b1);