#include "stabilizer_types.h"

#define FILTER_SIZE 50
#define NETWORK_RATE RATE_100_HZ

#define LED_LEADER   0b10110101 // red and blue
#define LED_ROOT     0b10101011 // green and blue
#define LED_FOLLOWER 0b10000000 // all off

void disturbance_observer_step(struct vec* u, const struct vec* re, const struct vec* re_dot, const struct vec* b1);