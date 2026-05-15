/**
 * @file    ekf.cpp
 * @brief   Quaternion EKF — full implementation with 4×4 covariance algebra.
 *
 * State vector
 * ------------
 *     x = q = [q0, q1, q2, q3]^T,    ||q|| = 1.
 *
 * The quaternion rotates a vector from the body frame to the world frame:
 *     v_world = q ⊗ v_body ⊗ q*.
 *
 * Process model
 * -------------
 * Gyroscope angular velocity ω = (ωx, ωy, ωz) in the body frame propagates
 * the quaternion forward in time. The continuous-time kinematics are
 *
 *     q̇ = 0.5 · Ω(ω) · q,
 *
 * where Ω(ω) is the 4×4 skew-symmetric matrix
 *
 *           ┌  0   -ωx  -ωy  -ωz ┐
 *     Ω =   │ ωx    0    ωz  -ωy │.
 *           │ ωy  -ωz    0    ωx │
 *           └ ωz   ωy   -ωx    0 ┘
 *
 * Using a first-order (small-angle) discretisation,
 *
 *     q_{k+1} = (I + 0.5 · Ω · dt) · q_k        (predict).
 *
 * The corresponding state-transition Jacobian is F = I + 0.5·Ω·dt, and the
 * covariance propagates as P⁻ = F·P·Fᵀ + Q with process-noise Q.
 *
 * Measurement model
 * -----------------
 * With the device stationary the accelerometer measures gravity rotated into
 * the body frame. The predicted gravity vector in body coordinates is
 *
 *     h(q) = g · [ 2(q1 q3 − q0 q2),
 *                  2(q0 q1 + q2 q3),
 *                  q0² − q1² − q2² + q3² ].
 *
 * The 3×4 measurement Jacobian H = ∂h/∂q is differentiated analytically below.
 * We normalise the measured accelerometer vector to a unit gravity direction
 * before comparing to h(q) — this throws away magnitude information (which
 * is corrupted by limb motion anyway) and keeps the update well-conditioned.
 *
 * Innovation
 * ----------
 *     y = z − h(q),                        (3×1)
 *     S = H · P⁻ · Hᵀ + R,                 (3×3)
 *     K = P⁻ · Hᵀ · S⁻¹,                   (4×3)
 *     q⁺ = q⁻ + K · y,                     (4×1, then re-normalised)
 *     P⁺ = (I − K · H) · P⁻.               (4×4)
 *
 * Adaptive R
 * ----------
 * The accel update is only trustworthy near gravity. We inflate R when the
 * measured magnitude departs from 9.81 m/s² — this dynamically down-weights
 * the update during fast arm swings without needing extra states.
 */

#include "ekf.h"
#include "config.h"
#include <math.h>

namespace ekf {

// ---------------------------------------------------------------------------
// State + covariance
// ---------------------------------------------------------------------------
static float q[4]    = { 1.0f, 0.0f, 0.0f, 0.0f }; ///< x
static float P[4][4] = {{0}};                      ///< P
static Vec3  s_linAccel = { 0.0f, 0.0f, 0.0f };

// ---------------------------------------------------------------------------
// Small matrix helpers — fixed dims, no heap, FPU-friendly floats.
// ---------------------------------------------------------------------------
static inline void mat4_identity(float M[4][4]) {
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) M[i][j] = (i == j) ? 1.0f : 0.0f;
}

static void mat4_mul(const float A[4][4], const float B[4][4], float C[4][4]) {
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 4; ++k) s += A[i][k] * B[k][j];
            C[i][j] = s;
        }
}

static void mat4_mulT(const float A[4][4], const float B[4][4], float C[4][4]) {
    // C = A * Bᵀ
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 4; ++k) s += A[i][k] * B[j][k];
            C[i][j] = s;
        }
}

static void mat4_add(const float A[4][4], const float B[4][4], float C[4][4]) {
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) C[i][j] = A[i][j] + B[i][j];
}

/// @brief Invert a 3×3 SPD matrix via cofactor expansion. Returns false if singular.
static bool mat3_inv(const float M[3][3], float Inv[3][3]) {
    const float a = M[0][0], b = M[0][1], c = M[0][2];
    const float d = M[1][0], e = M[1][1], f = M[1][2];
    const float g = M[2][0], h = M[2][1], i = M[2][2];

    const float A =  (e*i - f*h);
    const float B = -(d*i - f*g);
    const float C =  (d*h - e*g);
    const float det = a*A + b*B + c*C;
    if (fabsf(det) < 1e-9f) return false;
    const float inv = 1.0f / det;

    Inv[0][0] =  A * inv;
    Inv[0][1] = -(b*i - c*h) * inv;
    Inv[0][2] =  (b*f - c*e) * inv;
    Inv[1][0] =  B * inv;
    Inv[1][1] =  (a*i - c*g) * inv;
    Inv[1][2] = -(a*f - c*d) * inv;
    Inv[2][0] =  C * inv;
    Inv[2][1] = -(a*h - b*g) * inv;
    Inv[2][2] =  (a*e - b*d) * inv;
    return true;
}

// ---------------------------------------------------------------------------

void reset() {
    q[0] = 1.0f; q[1] = q[2] = q[3] = 0.0f;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            P[i][j] = (i == j) ? EKF_P_INIT : 0.0f;
    s_linAccel = { 0.0f, 0.0f, 0.0f };
}

// ---------------------------------------------------------------------------
// Predict
// ---------------------------------------------------------------------------
//   F = I + 0.5 · Ω(ω) · dt
//   q⁻ = F · q
//   P⁻ = F · P · Fᵀ + Q
static void predict(float wx, float wy, float wz, float dt) {
    const float half_dt = 0.5f * dt;

    // F = I + 0.5·Ω·dt (small-angle approximation; valid for ω·dt « 1 rad).
    float F[4][4] = {
        { 1.0f,           -wx * half_dt, -wy * half_dt, -wz * half_dt },
        { wx * half_dt,    1.0f,          wz * half_dt, -wy * half_dt },
        { wy * half_dt,   -wz * half_dt,  1.0f,          wx * half_dt },
        { wz * half_dt,    wy * half_dt, -wx * half_dt,  1.0f         },
    };

    // q⁻ = F · q
    float qn[4];
    for (int i = 0; i < 4; ++i) {
        qn[i] = F[i][0]*q[0] + F[i][1]*q[1] + F[i][2]*q[2] + F[i][3]*q[3];
    }
    // Re-normalise — numerical drift accumulates otherwise.
    const float n = sqrtf(qn[0]*qn[0] + qn[1]*qn[1] + qn[2]*qn[2] + qn[3]*qn[3]);
    if (n > 1e-9f) {
        const float inv = 1.0f / n;
        q[0] = qn[0]*inv; q[1] = qn[1]*inv; q[2] = qn[2]*inv; q[3] = qn[3]*inv;
    }

    // P⁻ = F·P·Fᵀ + Q
    float FP[4][4], FPFt[4][4];
    mat4_mul(F, P, FP);
    mat4_mulT(FP, F, FPFt);

    float Q[4][4]; mat4_identity(Q);
    for (int i = 0; i < 4; ++i) Q[i][i] = EKF_Q_GYRO * dt;

    mat4_add(FPFt, Q, P);
}

// ---------------------------------------------------------------------------
// Update — fuse accelerometer gravity vector
// ---------------------------------------------------------------------------
static void update(float ax, float ay, float az) {
    // 1) Normalise the measured accel to a unit gravity direction.
    const float amag = sqrtf(ax*ax + ay*ay + az*az);
    if (amag < 1e-3f) return;       // free fall / sensor fault — skip update
    const float inv_a = 1.0f / amag;
    const float zx = ax * inv_a;
    const float zy = ay * inv_a;
    const float zz = az * inv_a;

    // 2) Predicted gravity direction in the body frame, h(q) (unit vector).
    const float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    const float hx = 2.0f * (q1*q3 - q0*q2);
    const float hy = 2.0f * (q0*q1 + q2*q3);
    const float hz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // 3) Innovation y = z − h(q).
    const float y0 = zx - hx;
    const float y1 = zy - hy;
    const float y2 = zz - hz;

    // 4) Measurement Jacobian H = ∂h/∂q  (3×4).
    float H[3][4] = {
        { -2.0f*q2,  2.0f*q3, -2.0f*q0,  2.0f*q1 },
        {  2.0f*q1,  2.0f*q0,  2.0f*q3,  2.0f*q2 },
        {  2.0f*q0, -2.0f*q1, -2.0f*q2,  2.0f*q3 },
    };

    // 5) Adaptive measurement noise — inflate R if |a| ≠ g.
    const float gerr = fabsf(amag - GRAVITY_MPS2) / GRAVITY_MPS2;
    const float r    = EKF_R_ACCEL * (1.0f + 10.0f * gerr * gerr);

    // 6) S = H·P·Hᵀ + R  (3×3).
    float HP[3][4];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 4; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 4; ++k) s += H[i][k] * P[k][j];
            HP[i][j] = s;
        }
    float S[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 4; ++k) s += HP[i][k] * H[j][k]; // Hᵀ at (k,j)
            S[i][j] = s + (i == j ? r : 0.0f);
        }

    // 7) Kalman gain K = P·Hᵀ·S⁻¹  (4×3).
    float Sinv[3][3];
    if (!mat3_inv(S, Sinv)) return;

    float PHt[4][3];
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 3; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 4; ++k) s += P[i][k] * H[j][k];
            PHt[i][j] = s;
        }
    float K[4][3];
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 3; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 3; ++k) s += PHt[i][k] * Sinv[k][j];
            K[i][j] = s;
        }

    // 8) State update q⁺ = q + K · y.
    float dq[4];
    for (int i = 0; i < 4; ++i)
        dq[i] = K[i][0]*y0 + K[i][1]*y1 + K[i][2]*y2;
    q[0] += dq[0]; q[1] += dq[1]; q[2] += dq[2]; q[3] += dq[3];

    // Re-normalise after the additive correction.
    const float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n > 1e-9f) {
        const float inv = 1.0f / n;
        q[0] *= inv; q[1] *= inv; q[2] *= inv; q[3] *= inv;
    }

    // 9) Covariance update P⁺ = (I − K·H) · P.
    float KH[4][4] = {{0}};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 3; ++k) s += K[i][k] * H[k][j];
            KH[i][j] = s;
        }
    float ImKH[4][4];
    mat4_identity(ImKH);
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) ImKH[i][j] -= KH[i][j];

    float Pnew[4][4];
    mat4_mul(ImKH, P, Pnew);
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) P[i][j] = Pnew[i][j];
}

// ---------------------------------------------------------------------------
// Linear acceleration (gravity removed) — computed each step from the
// post-update quaternion so downstream rep detection has a clean signal.
// ---------------------------------------------------------------------------
static void updateLinearAccel(float ax, float ay, float az) {
    const float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    // Gravity in body frame, scaled by g.
    const float gx_b = GRAVITY_MPS2 * 2.0f * (q1*q3 - q0*q2);
    const float gy_b = GRAVITY_MPS2 * 2.0f * (q0*q1 + q2*q3);
    const float gz_b = GRAVITY_MPS2 * (q0*q0 - q1*q1 - q2*q2 + q3*q3);
    s_linAccel.x = ax - gx_b;
    s_linAccel.y = ay - gy_b;
    s_linAccel.z = az - gz_b;
}

// ---------------------------------------------------------------------------

void step(float gx, float gy, float gz,
          float ax, float ay, float az,
          float dt) {
    predict(gx, gy, gz, dt);
    update(ax, ay, az);
    updateLinearAccel(ax, ay, az);
}

// ---------------------------------------------------------------------------
// Output — quaternion to Tait–Bryan ZYX Euler angles (deg).
// ---------------------------------------------------------------------------
float getRoll() {
    const float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    return atan2f(2.0f * (q0*q1 + q2*q3), 1.0f - 2.0f * (q1*q1 + q2*q2))
           * (180.0f / PI);
}

float getPitch() {
    const float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    float s = 2.0f * (q0*q2 - q3*q1);
    if (s >  1.0f) s =  1.0f;
    if (s < -1.0f) s = -1.0f;
    return asinf(s) * (180.0f / PI);
}

float getYaw() {
    const float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    return atan2f(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3))
           * (180.0f / PI);
}

void getQuaternion(float out[4]) {
    out[0] = q[0]; out[1] = q[1]; out[2] = q[2]; out[3] = q[3];
}

Vec3 getLinearAccel() { return s_linAccel; }

} // namespace ekf
