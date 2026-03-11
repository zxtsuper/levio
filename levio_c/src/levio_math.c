/**
 * @file levio_math.c
 * @brief Small fixed-size linear algebra helpers (no heap).
 */
#include "levio_math.h"
#include <math.h>
#include <string.h>

/* --------------------------------------------------------------------------
 * vec3 operations
 * -------------------------------------------------------------------------- */

vec3f_t levio_vec3_add(vec3f_t a, vec3f_t b)
{
    vec3f_t r = { a.x + b.x, a.y + b.y, a.z + b.z };
    return r;
}

vec3f_t levio_vec3_sub(vec3f_t a, vec3f_t b)
{
    vec3f_t r = { a.x - b.x, a.y - b.y, a.z - b.z };
    return r;
}

vec3f_t levio_vec3_scale(vec3f_t v, float s)
{
    vec3f_t r = { v.x * s, v.y * s, v.z * s };
    return r;
}

float levio_vec3_dot(vec3f_t a, vec3f_t b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

vec3f_t levio_vec3_cross(vec3f_t a, vec3f_t b)
{
    vec3f_t r;
    r.x = a.y * b.z - a.z * b.y;
    r.y = a.z * b.x - a.x * b.z;
    r.z = a.x * b.y - a.y * b.x;
    return r;
}

float levio_vec3_norm(vec3f_t v)
{
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

vec3f_t levio_vec3_normalize(vec3f_t v)
{
    float n = levio_vec3_norm(v);
    if (n < 1e-12f) {
        vec3f_t zero = {0.0f, 0.0f, 0.0f};
        return zero;
    }
    return levio_vec3_scale(v, 1.0f / n);
}

/* --------------------------------------------------------------------------
 * mat3 operations
 * -------------------------------------------------------------------------- */

mat3f_t levio_mat3_identity(void)
{
    mat3f_t m;
    memset(m.m, 0, sizeof(m.m));
    m.m[0] = m.m[4] = m.m[8] = 1.0f;
    return m;
}

mat3f_t levio_mat3_mul(mat3f_t a, mat3f_t b)
{
    mat3f_t c;
    int i, j, k;
    for (i = 0; i < 3; ++i)
        for (j = 0; j < 3; ++j) {
            float s = 0.0f;
            for (k = 0; k < 3; ++k)
                s += a.m[i * 3 + k] * b.m[k * 3 + j];
            c.m[i * 3 + j] = s;
        }
    return c;
}

mat3f_t levio_mat3_transpose(mat3f_t m)
{
    mat3f_t t;
    int i, j;
    for (i = 0; i < 3; ++i)
        for (j = 0; j < 3; ++j)
            t.m[i * 3 + j] = m.m[j * 3 + i];
    return t;
}

vec3f_t levio_mat3_mul_vec3(mat3f_t m, vec3f_t v)
{
    vec3f_t r;
    r.x = m.m[0] * v.x + m.m[1] * v.y + m.m[2] * v.z;
    r.y = m.m[3] * v.x + m.m[4] * v.y + m.m[5] * v.z;
    r.z = m.m[6] * v.x + m.m[7] * v.y + m.m[8] * v.z;
    return r;
}

float levio_mat3_det(mat3f_t m)
{
    return  m.m[0] * (m.m[4] * m.m[8] - m.m[5] * m.m[7])
          - m.m[1] * (m.m[3] * m.m[8] - m.m[5] * m.m[6])
          + m.m[2] * (m.m[3] * m.m[7] - m.m[4] * m.m[6]);
}

mat3f_t levio_mat3_inv(mat3f_t m)
{
    float det = levio_mat3_det(m);
    mat3f_t inv;
    if (fabsf(det) < 1e-12f)
        return levio_mat3_identity();
    float id = 1.0f / det;
    inv.m[0] =  (m.m[4] * m.m[8] - m.m[5] * m.m[7]) * id;
    inv.m[1] = -(m.m[1] * m.m[8] - m.m[2] * m.m[7]) * id;
    inv.m[2] =  (m.m[1] * m.m[5] - m.m[2] * m.m[4]) * id;
    inv.m[3] = -(m.m[3] * m.m[8] - m.m[5] * m.m[6]) * id;
    inv.m[4] =  (m.m[0] * m.m[8] - m.m[2] * m.m[6]) * id;
    inv.m[5] = -(m.m[0] * m.m[5] - m.m[2] * m.m[3]) * id;
    inv.m[6] =  (m.m[3] * m.m[7] - m.m[4] * m.m[6]) * id;
    inv.m[7] = -(m.m[0] * m.m[7] - m.m[1] * m.m[6]) * id;
    inv.m[8] =  (m.m[0] * m.m[4] - m.m[1] * m.m[3]) * id;
    return inv;
}

mat3f_t levio_mat3_skew(vec3f_t v)
{
    mat3f_t s;
    s.m[0] =  0.0f;  s.m[1] = -v.z;   s.m[2] =  v.y;
    s.m[3] =  v.z;   s.m[4] =  0.0f;  s.m[5] = -v.x;
    s.m[6] = -v.y;   s.m[7] =  v.x;   s.m[8] =  0.0f;
    return s;
}

/* --------------------------------------------------------------------------
 * Generic NxN routines
 * -------------------------------------------------------------------------- */

int levio_chol(float *A, int n)
{
    int i, j, k;
    for (i = 0; i < n; ++i) {
        for (j = 0; j <= i; ++j) {
            float s = A[i * n + j];
            for (k = 0; k < j; ++k)
                s -= A[i * n + k] * A[j * n + k];
            if (i == j) {
                if (s <= 0.0f)
                    return -1;
                A[i * n + i] = sqrtf(s);
            } else {
                A[i * n + j] = s / A[j * n + j];
            }
        }
        /* Zero upper triangle */
        for (j = i + 1; j < n; ++j)
            A[i * n + j] = 0.0f;
    }
    return 0;
}

void levio_fwd_sub(const float *L, float *b, int n)
{
    int i, k;
    for (i = 0; i < n; ++i) {
        float s = b[i];
        for (k = 0; k < i; ++k)
            s -= L[i * n + k] * b[k];
        b[i] = s / L[i * n + i];
    }
}

void levio_back_sub(const float *L, float *b, int n)
{
    int i, k;
    for (i = n - 1; i >= 0; --i) {
        float s = b[i];
        for (k = i + 1; k < n; ++k)
            s -= L[k * n + i] * b[k];
        b[i] = s / L[i * n + i];
    }
}

int levio_solve_chol(float *A, float *b, int n)
{
    if (levio_chol(A, n) != 0)
        return -1;
    levio_fwd_sub(A, b, n);
    levio_back_sub(A, b, n);
    return 0;
}

void levio_matmul(const float *A, const float *B, float *C, int m, int k, int n)
{
    int i, j, l;
    memset(C, 0, (size_t)(m * n) * sizeof(float));
    for (i = 0; i < m; ++i)
        for (l = 0; l < k; ++l) {
            float a = A[i * k + l];
            for (j = 0; j < n; ++j)
                C[i * n + j] += a * B[l * n + j];
        }
}

void levio_matadd_scaled(float *C, const float *A, float s, int m, int n)
{
    int i, sz = m * n;
    for (i = 0; i < sz; ++i)
        C[i] += s * A[i];
}
