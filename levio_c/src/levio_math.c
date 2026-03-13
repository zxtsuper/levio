/**
 * @file levio_math.c
 * @brief Small fixed-size linear algebra helpers (no heap).
 */
#include "levio_math.h"
#include <math.h>
#include <string.h>
#include <stdint.h>

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

/* --------------------------------------------------------------------------
 * Jacobi symmetric eigendecomposition
 * -------------------------------------------------------------------------- */

int levio_sym_eig(float *A, float *V, int n)
{
    int i, j, k, iter;
    float c, s, t, tau;
    float off;

    if (n > LEVIO_MAX_EIG_DIM)
        return -1;

    /* Initialise V = identity */
    for (i = 0; i < n * n; ++i)
        V[i] = 0.0f;
    for (i = 0; i < n; ++i)
        V[i * n + i] = 1.0f;

    /* Cyclic Jacobi sweeps */
    for (iter = 0; iter < 50 * n * n; ++iter) {
        /* Check convergence: sum of squared off-diagonal elements */
        off = 0.0f;
        for (i = 0; i < n - 1; ++i)
            for (j = i + 1; j < n; ++j)
                off += A[i * n + j] * A[i * n + j];
        if (off < 1e-20f)
            break;

        /* Find largest off-diagonal element */
        int p = 0, q = 1;
        float max_off = fabsf(A[0 * n + 1]);
        for (i = 0; i < n - 1; ++i) {
            for (j = i + 1; j < n; ++j) {
                float val = fabsf(A[i * n + j]);
                if (val > max_off) {
                    max_off = val;
                    p = i;
                    q = j;
                }
            }
        }
        if (max_off < 1e-10f)
            break;

        /* Compute rotation */
        tau = (A[q * n + q] - A[p * n + p]) / (2.0f * A[p * n + q]);
        if (tau >= 0.0f)
            t = 1.0f / (tau + sqrtf(1.0f + tau * tau));
        else
            t = 1.0f / (tau - sqrtf(1.0f + tau * tau));
        c = 1.0f / sqrtf(1.0f + t * t);
        s = t * c;

        /* Update diagonal elements */
        float App = A[p * n + p];
        float Aqq = A[q * n + q];
        float Apq = A[p * n + q];
        A[p * n + p] = c * c * App + s * s * Aqq - 2.0f * s * c * Apq;
        A[q * n + q] = s * s * App + c * c * Aqq + 2.0f * s * c * Apq;
        A[p * n + q] = 0.0f;
        A[q * n + p] = 0.0f;

        /* Update off-diagonal elements for rows/cols != p, q */
        for (k = 0; k < n; ++k) {
            if (k == p || k == q)
                continue;
            float Akp = A[k * n + p];
            float Akq = A[k * n + q];
            A[k * n + p] = A[p * n + k] = c * Akp - s * Akq;
            A[k * n + q] = A[q * n + k] = s * Akp + c * Akq;
        }

        /* Accumulate rotation in V */
        for (k = 0; k < n; ++k) {
            float Vkp = V[k * n + p];
            float Vkq = V[k * n + q];
            V[k * n + p] = c * Vkp - s * Vkq;
            V[k * n + q] = s * Vkp + c * Vkq;
        }
    }
    return 0;
}

/* --------------------------------------------------------------------------
 * 3×3 SVD via Jacobi on A^T*A
 * -------------------------------------------------------------------------- */

void levio_svd3(const float A[9], float U[9], float s[3], float Vt[9])
{
    /* Step 1: compute M = A^T * A (3×3 symmetric) */
    float M[9];
    int i, j, k;
    for (i = 0; i < 3; ++i)
        for (j = 0; j < 3; ++j) {
            float sum = 0.0f;
            for (k = 0; k < 3; ++k)
                sum += A[k * 3 + i] * A[k * 3 + j];
            M[i * 3 + j] = sum;
        }

    /* Step 2: Jacobi eigendecomposition of M → V (columns = right sing. vecs.) */
    float V[9];
    levio_sym_eig(M, V, 3);
    /* M diagonal now holds eigenvalues λ_i = s_i^2 */

    /* Step 3: sort descending by eigenvalue (simple 3-element bubble sort) */
    int idx[3] = {0, 1, 2};
    float ev[3] = {M[0], M[4], M[8]};
    if (ev[idx[0]] < ev[idx[1]]) { int tmp = idx[0]; idx[0] = idx[1]; idx[1] = tmp; }
    if (ev[idx[1]] < ev[idx[2]]) { int tmp = idx[1]; idx[1] = idx[2]; idx[2] = tmp; }
    if (ev[idx[0]] < ev[idx[1]]) { int tmp = idx[0]; idx[0] = idx[1]; idx[1] = tmp; }

    /* Step 4: singular values and Vt (right singular vectors as rows) */
    for (i = 0; i < 3; ++i) {
        float lambda = ev[idx[i]];
        s[i] = (lambda > 0.0f) ? sqrtf(lambda) : 0.0f;
        /* i-th row of Vt = idx[i]-th eigenvector (column of V) */
        for (j = 0; j < 3; ++j)
            Vt[i * 3 + j] = V[j * 3 + idx[i]];
    }

    /* Step 5: left singular vectors U = A * V / sigma_i */
    for (i = 0; i < 3; ++i) {
        /* U[:,i] = A * Vt[i,:] / s[i] = A * v_i / s[i] */
        float v[3] = {Vt[i * 3 + 0], Vt[i * 3 + 1], Vt[i * 3 + 2]};
        for (j = 0; j < 3; ++j) {
            float sum = 0.0f;
            for (k = 0; k < 3; ++k)
                sum += A[j * 3 + k] * v[k];
            U[j * 3 + i] = (s[i] > 1e-10f) ? sum / s[i] : 0.0f;
        }
    }

    /* Step 6: if det(U) < 0, flip last column to ensure proper rotation */
    {
        float detU = U[0]*(U[4]*U[8]-U[5]*U[7])
                   - U[1]*(U[3]*U[8]-U[5]*U[6])
                   + U[2]*(U[3]*U[7]-U[4]*U[6]);
        if (detU < 0.0f) {
            for (j = 0; j < 3; ++j)
                U[j * 3 + 2] = -U[j * 3 + 2];
            s[2] = -s[2]; /* technically s[2] stays non-negative, but flip sign */
        }
    }
    /* Ensure s[2] >= 0 */
    if (s[2] < 0.0f) s[2] = -s[2];
}

/* --------------------------------------------------------------------------
 * LCG pseudo-random generator
 * -------------------------------------------------------------------------- */

uint32_t levio_rand_next(uint32_t *state)
{
    /* Park-Miller LCG: multiplier 48271, modulus 2^31-1 */
    *state = (uint32_t)(((uint64_t)(*state) * 48271u) % 2147483647u);
    return *state;
}
