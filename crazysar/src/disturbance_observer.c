#include "arm_math.h"
#include "math3d.h"
#include "main.h"
#include "static_mem.h"
#include "log.h"

#define DEBUG_MODULE "DISTOBS"
#include "debug.h"

arm_matrix_instance_f32 aug_state_est = { 9, 1, (float32_t[]){ 0, 0, 0, 0, 0, 0, 0, 0, 0 } };

struct vec disturbance_est;

const arm_matrix_instance_f32 A = { 9, 9, (float32_t[]){ 
	1.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0001f, 0.0f, 0.0f,
	0.0f, 1.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0001f, 0.0f,
	0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0001f,
	0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f,
	0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.01f, 0.0f,
	0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.01f,
	0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
	0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
	0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f
} };
const arm_matrix_instance_f32 B = { 9, 3, (float32_t[]){ 
	0.0001f, 0.0f, 0.0f,
	0.0f, 0.0001f, 0.0f,
	0.0f, 0.0f, 0.0001f,
	0.01f, 0.0f, 0.0f,
	0.0f, 0.01f, 0.0f,
	0.0f, 0.0f, 0.01f,
	0.0f, 0.0f, 0.0f,
	0.0f, 0.0f, 0.0f,
	0.0f, 0.0f, 0.0f
} };
const arm_matrix_instance_f32 C = { 6, 9, (float32_t[]){ 
	1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
	0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
	0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
	0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
	0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
	0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f
} };
const arm_matrix_instance_f32 L = { 9, 6, (float32_t[]){ 
	  0.02f,    0.0f,    0.0f, 0.01f,  0.0f,  0.0f,
	   0.0f,   0.02f,    0.0f,  0.0f, 0.01f,  0.0f,
	   0.0f,    0.0f,   0.02f,  0.0f,  0.0f, 0.01f,
	0.0001f,    0.0f,    0.0f, 0.05f,  0.0f,  0.0f,
	   0.0f, 0.0001f,    0.0f,  0.0f, 0.05f,  0.0f,
	   0.0f,    0.0f, 0.0001f,  0.0f,  0.0f, 0.05f,
	0.0003f,    0.0f,    0.0f, 0.06f,  0.0f,  0.0f,
	   0.0f, 0.0003f,    0.0f,  0.0f, 0.06f,  0.0f,
	   0.0f,    0.0f,  0.0003f, 0.0f,  0.0f, 0.06f
} };

void disturbance_observer_step(struct vec* u, const struct vec* re, const struct vec* re_dot, const struct vec* b1) {
	disturbance_est = mkvec(
		aug_state_est.pData[6],
		aug_state_est.pData[7],
		aug_state_est.pData[8]
	);
	disturbance_est = vsub(disturbance_est, vscl(0.5f / CRAZYSAR_NETWORK_RATE * vdot(disturbance_est, *b1), *b1)); // Reduce the component along b1 over time
	aug_state_est.pData[6] = disturbance_est.x;
	aug_state_est.pData[7] = disturbance_est.y;
	aug_state_est.pData[8] = disturbance_est.z;

	*u = vsub(*u, disturbance_est);

	static float input_data[3];
	static arm_matrix_instance_f32 input = { 3, 1, input_data };
	input.pData[0] = u->x;
	input.pData[1] = u->y;
	input.pData[2] = u->z;

	static float output_data[6];
	static arm_matrix_instance_f32 output = { 6, 1, output_data };
	output.pData[0] = re->x;
	output.pData[1] = re->y;
	output.pData[2] = re->z;
	output.pData[3] = re_dot->x;
	output.pData[4] = re_dot->y;
	output.pData[5] = re_dot->z;

	// Temporary matrices
	static float Ax_data[9];
	static arm_matrix_instance_f32 Ax = { 9, 1, Ax_data };

	static float Bu_data[9];
	static arm_matrix_instance_f32 Bu = { 9, 1, Bu_data };

	static float AxBu_data[9];
	static arm_matrix_instance_f32 AxBu = { 9, 1, AxBu_data };

	static float Cx_data[6];
	static arm_matrix_instance_f32 Cx = { 6, 1, Cx_data };

	static float yCx_data[6];
	static arm_matrix_instance_f32 yCx = { 6, 1, yCx_data };

	static float LC_data[9];
	static arm_matrix_instance_f32 LC = { 9, 1, LC_data };

	arm_mat_mult_f32(&A, &aug_state_est, &Ax);
	arm_mat_mult_f32(&B, &input, &Bu);
	arm_mat_add_f32(&Ax, &Bu, &AxBu);

	arm_mat_mult_f32(&C, &aug_state_est, &Cx);
	arm_mat_sub_f32(&output, &Cx, &yCx);
	arm_mat_mult_f32(&L, &yCx, &LC);
	arm_mat_add_f32(&AxBu, &LC, &aug_state_est);

	// DEBUG_PRINT("Disturbance estimate: %.4f, %.4f, %.4f\n", (double)disturbance_est.x, (double)disturbance_est.y, (double)disturbance_est.z);
}

LOG_GROUP_START(distObs)

LOG_ADD(LOG_FLOAT, distEstX, &disturbance_est.x)
LOG_ADD(LOG_FLOAT, distEstY, &disturbance_est.y)
LOG_ADD(LOG_FLOAT, distEstZ, &disturbance_est.z)

LOG_GROUP_STOP(distObs)