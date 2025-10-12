#ifndef GP_MODEL_PARAMS_H
#define GP_MODEL_PARAMS_H

#define MAX_NUM_SAMPLES 300
#define MAX_NUM_DIMS 3

typedef struct gp_model_params_s {
  unsigned int NUM_SAMPLES;
  unsigned int NUM_DIMS;
  float X_train[MAX_NUM_SAMPLES*MAX_NUM_DIMS];
  float alpha_times_outputscale[MAX_NUM_SAMPLES];
  float neg_gamma[MAX_NUM_DIMS];
  float noise;
} gp_model_params_t;

#endif /* GP_MODEL_PARAMS_H */