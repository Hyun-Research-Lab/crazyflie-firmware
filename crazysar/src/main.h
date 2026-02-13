#include "stabilizer_types.h"

#define FILTER_SIZE 50

#define CRAZYSAR_NETWORK_RATE RATE_100_HZ
#define CRAZYSAR_ATTITUDE_RATE RATE_500_HZ

#define LED_LEADER   0b10110101 // red and blue
#define LED_ROOT     0b10101011 // green and blue
#define LED_FOLLOWER 0b10000000 // all off

#define LEADER_FOLLOWER_DATA_SIZE 10

typedef struct LeaderFollowerData_s {
  union {
    struct {
      float m;
      struct vec F_d_bar;
      struct vec x;
      struct vec v;
    };
    float raw[LEADER_FOLLOWER_DATA_SIZE];
  };
} LeaderFollowerData_t;

void disturbance_observer_step(struct vec* u, const struct vec* re, const struct vec* re_dot, const struct vec* b1);