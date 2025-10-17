#ifndef GP_MODEL_PARAMS_H
#define GP_MODEL_PARAMS_H

#define GP_MODEL_NUM_SAMPLES 400
#define GP_MODEL_THRUST_DATA_DIM 3
#define GP_MODEL_TORQUE_DATA_DIM 6

typedef struct gp_thrust_params_s {
  float X_train[GP_MODEL_NUM_SAMPLES * GP_MODEL_THRUST_DATA_DIM];
  float alpha_times_outputscale[GP_MODEL_NUM_SAMPLES];
  float neg_gamma[GP_MODEL_THRUST_DATA_DIM];
} gp_thrust_params_t;

typedef struct gp_torque_params_s {
  float X_train[GP_MODEL_NUM_SAMPLES * GP_MODEL_TORQUE_DATA_DIM];
  float alpha_times_outputscaleX[GP_MODEL_NUM_SAMPLES];
  float alpha_times_outputscaleY[GP_MODEL_NUM_SAMPLES];
  float alpha_times_outputscaleZ[GP_MODEL_NUM_SAMPLES];
  float neg_gammaX[GP_MODEL_TORQUE_DATA_DIM];
  float neg_gammaY[GP_MODEL_TORQUE_DATA_DIM];
  float neg_gammaZ[GP_MODEL_TORQUE_DATA_DIM];
} gp_torque_params_t;

#endif /* GP_MODEL_PARAMS_H */