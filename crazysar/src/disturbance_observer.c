#include "arm_math.h"
#include "math3d.h"
#include "main.h"

arm_matrix_instance_f32 aug_state_est = { 9, 1, (float32_t[]){ 0, 0, 0, 0, 0, 0, 0, 0, 0 } };

arm_matrix_instance_f32 A = { 9, 9, (float32_t[]){ 
	1.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0001f, 0.0f, 0.0f,
	0.0f, 1.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0001f, 0.0f,
	0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0001f,
	0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f,
	0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.01f, 0.0f,
	0, 0, 0, 0, 0, 1, 0, 0, 0.01,
	0, 0, 0, 0, 0, 0, 1, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 1, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 1
} };
arm_matrix_instance_f32 B = { 9, 3, (float32_t[]){ 
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
arm_matrix_instance_f32 C = { 6, 9, (float32_t[]){ 
	1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
	0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
	0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
	0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
	0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
	0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f
} };
arm_matrix_instance_f32 L = { 9, 6, (float32_t[]){ 
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

float Ax_data[9];
arm_matrix_instance_f32 Ax = { 9, 1, Ax_data };

float Bu_data[9];
arm_matrix_instance_f32 Bu = { 9, 1, Bu_data };

float AxBu_data[9];
arm_matrix_instance_f32 AxBu = { 9, 1, AxBu_data };

float Cx_data[6];
arm_matrix_instance_f32 Cx = { 6, 1, Cx_data };

float yCx_data[6];
arm_matrix_instance_f32 yCx = { 6, 1, yCx_data };

float LC_data[9];
arm_matrix_instance_f32 LC = { 9, 1, LC_data };

struct vec disturbance_observer_step(struct vec re, struct vec re_dot, struct vec u, struct vec b1) {
	struct vec disturbance_est = mkvec(
		aug_state_est.pData[6],
		aug_state_est.pData[7],
		aug_state_est.pData[8]
	);
	disturbance_est = vsub(disturbance_est, vscl(0.5f / NETWORK_RATE * vdot(disturbance_est, b1), b1)); // Reduce the component along b1 over time
	aug_state_est.pData[6] = disturbance_est.x;
	aug_state_est.pData[7] = disturbance_est.y;
	aug_state_est.pData[8] = disturbance_est.z;

	arm_matrix_instance_f32 input = { 3, 1, (float32_t[]){ u.x, u.y, u.z } };
	arm_matrix_instance_f32 output = { 6, 1, (float32_t[]){ re.x, re.y, re.z, re_dot.x, re_dot.y, re_dot.z } };

	arm_mat_mult_f32(&A, &aug_state_est, &Ax);
	arm_mat_mult_f32(&B, &input, &Bu);
	arm_mat_add_f32(&Ax, &Bu, &AxBu);

	arm_mat_mult_f32(&C, &aug_state_est, &Cx);
	arm_mat_sub_f32(&output, &Cx, &yCx);
	arm_mat_mult_f32(&L, &yCx, &LC);
	arm_mat_add_f32(&AxBu, &LC, &aug_state_est);

	return disturbance_est;
}