#ifndef __MADGWICK_H
#define __MADGWICK_H
static float invSqrt(float x);
struct madgwick;
struct madgwick_cfg;
struct quat_data;
struct nodegg;
angle_handle_t angle_answer;

typedef struct quat_data
{
    float q1;
    float q2;
    float q3;
    float q0;
} quat_data_t;

typedef struct madgwick_cfg
{
    float beta;
    float freq;
} madgwick_cfg_t;

typedef struct madgwick
{
    float beta;
    float freq;
    float q1;
    float q2;
    float q3;
    float q0;
} madgwick_t;
typedef struct nodegg
{
    float gx, gy, gz, ax, ay, az, mx, my, mz;
} nodegg_t;
typedef struct madgwick *madgwick_handle_t;
typedef struct quat_data *quat_data_handle_t;
typedef struct nodegg *nodegg_handle_t;
typedef struct quat_data quat_data_t;
// madgwick_handle_t madgwick_handle;
madgwick_handle_t madgwick_init(struct madgwick_cfg *config);
typedef struct angle
{
    float roll, pitch, yaw;
} angle_t;

typedef struct angle *angle_handle_t;
quat_data_t quat_answer;
void madgwick_get_quaternion(madgwick_handle_t handle, quat_data_handle_t quat);
void madgwick_test();
void madgwick_display1(angle_handle_t angle_answer_it);
void madgwick_display2(quat_data_t quat_data_it);
void madgwick_displayraw(nodegg_handle_t it);

#endif