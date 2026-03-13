/**
 * @file levio_math.h
 * @brief Small fixed-size linear algebra helpers for LEVIO embedded VIO.
 *
 * All routines operate on stack-allocated arrays to avoid dynamic memory
 * allocation (paper §3.4 embedded constraints).
 *
 * Naming convention:
 *   levio_mat3_*   – 3×3 float matrices
 *   levio_vec3_*   – 3-element float vectors
 *   levio_matN_*   – generic NxN routines (N passed as parameter)
 */
#ifndef LEVIO_MATH_H
#define LEVIO_MATH_H

#include <stddef.h>
#include <stdint.h>
#include "levio_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * vec3 operations
 * -------------------------------------------------------------------------- */
vec3f_t levio_vec3_add(vec3f_t a, vec3f_t b);
vec3f_t levio_vec3_sub(vec3f_t a, vec3f_t b);
vec3f_t levio_vec3_scale(vec3f_t v, float s);
float   levio_vec3_dot(vec3f_t a, vec3f_t b);
vec3f_t levio_vec3_cross(vec3f_t a, vec3f_t b);
float   levio_vec3_norm(vec3f_t v);
vec3f_t levio_vec3_normalize(vec3f_t v);

/* --------------------------------------------------------------------------
 * mat3 operations (row-major, 9 floats)
 * -------------------------------------------------------------------------- */
mat3f_t levio_mat3_identity(void);
mat3f_t levio_mat3_mul(mat3f_t a, mat3f_t b);
mat3f_t levio_mat3_transpose(mat3f_t m);
vec3f_t levio_mat3_mul_vec3(mat3f_t m, vec3f_t v);

/** Determinant of 3×3 matrix */
float   levio_mat3_det(mat3f_t m);

/** Inverse of 3×3 matrix (returns identity on singular input) */
mat3f_t levio_mat3_inv(mat3f_t m);

/** Skew-symmetric matrix from 3-vector: [v]× */
mat3f_t levio_mat3_skew(vec3f_t v);

/* --------------------------------------------------------------------------
 * Generic NxN dense routines (used for preintegration covariance, Schur)
 * All matrices stored row-major in caller-provided float arrays.
 * -------------------------------------------------------------------------- */

/**
 * @brief In-place Cholesky decomposition L of A (A = L L^T).
 * @param A     [in/out] symmetric positive-definite n×n matrix → lower L
 * @param n     matrix dimension
 * @return 0 on success, -1 if not positive-definite
 */
int levio_chol(float *A, int n);

/**
 * @brief Solve L x = b in-place (lower triangular forward substitution).
 * @param L   n×n lower triangular matrix
 * @param b   [in/out] right-hand side → solution x
 * @param n   dimension
 */
void levio_fwd_sub(const float *L, float *b, int n);

/**
 * @brief Solve L^T x = b in-place (back substitution).
 */
void levio_back_sub(const float *L, float *b, int n);

/**
 * @brief Solve A x = b via Cholesky (A must be SPD).
 *        Modifies A in place.
 * @return 0 on success, -1 on failure
 */
int levio_solve_chol(float *A, float *b, int n);

/**
 * @brief Matrix multiply C = A * B.
 * @param A   m×k row-major
 * @param B   k×n row-major
 * @param C   m×n row-major output (must not alias A or B)
 */
void levio_matmul(const float *A, const float *B, float *C, int m, int k, int n);

/**
 * @brief Add scaled matrix: C += s * A.
 */
void levio_matadd_scaled(float *C, const float *A, float s, int m, int n);

/* --------------------------------------------------------------------------
 * Symmetric eigendecomposition (Jacobi) – max dimension 12
 * -------------------------------------------------------------------------- */

/** Maximum matrix dimension supported by levio_sym_eig(). */
#define LEVIO_MAX_EIG_DIM 12

/**
 * @brief Jacobi cyclic eigendecomposition of a real symmetric n×n matrix.
 *
 * On entry  A holds the upper/lower symmetric matrix.
 * On exit   A holds eigenvalues on the diagonal (off-diagonal is zeroed).
 *           V holds the eigenvectors as columns (V is orthogonal).
 *
 * @param A   n×n symmetric matrix (row-major, overwritten).
 * @param V   n×n output eigenvector matrix (row-major).
 * @param n   matrix dimension (n <= LEVIO_MAX_EIG_DIM).
 * @return    0 on success, -1 if n > LEVIO_MAX_EIG_DIM.
 */
int levio_sym_eig(float *A, float *V, int n);

/* --------------------------------------------------------------------------
 * 3×3 Singular Value Decomposition
 * -------------------------------------------------------------------------- */

/**
 * @brief SVD of a 3×3 matrix: A = U * diag(s) * Vt.
 *
 * Singular values s[0] >= s[1] >= s[2] >= 0 (sorted descending).
 * U and Vt are orthogonal (det may be -1; caller must handle sign if needed).
 *
 * @param A   Input 3×3 matrix (row-major, not modified).
 * @param U   Output 3×3 left singular vector matrix (row-major).
 * @param s   Output 3 singular values (sorted descending).
 * @param Vt  Output 3×3 right singular vector matrix transposed (row-major).
 */
void levio_svd3(const float A[9], float U[9], float s[3], float Vt[9]);

/* --------------------------------------------------------------------------
 * Simple LCG pseudo-random (for RANSAC)
 * -------------------------------------------------------------------------- */

/**
 * @brief Linear congruential generator step.
 *
 * @param state  LCG state (updated in place).
 * @return       Next pseudo-random uint32.
 */
uint32_t levio_rand_next(uint32_t *state);

#ifdef __cplusplus
}
#endif

#endif /* LEVIO_MATH_H */
