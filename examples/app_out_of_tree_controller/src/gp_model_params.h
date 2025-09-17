#ifndef __GP_MODEL_PARAMS_H__
#define __GP_MODEL_PARAMS_H__

#define SAMPLING_PERIOD 0.05f
#define MAX_NUM_SAMPLES 275
#define MAX_NUM_DIMS 6

typedef struct gp_model_params_s {
  unsigned int NUM_SAMPLES;
  unsigned int NUM_DIMS;
  float X_train[MAX_NUM_SAMPLES*MAX_NUM_DIMS];
  float y_mean;
  float y_std;
  float alpha_times_outputscale[MAX_NUM_SAMPLES];
  float lengthscale_sq[MAX_NUM_DIMS];
  float noise;
} gp_model_params_t;

#endif /* __GP_MODEL_PARAMS_H__ */