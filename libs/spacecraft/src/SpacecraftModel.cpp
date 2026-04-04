#include "apeiron/spacecraft/SpacecraftModel.h"

#include <glm/glm.hpp>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <numeric>

// ---------------------------------------------------------------------------
// Pseudoinverse of a 6×N matrix via eigen-decomposition of B*B^T (6×6).
//
// For M=6 rows and N thrusters (N >> 6), it is far more efficient to
// decompose the 6×6 product B*B^T than the N×N product B^T*B.
//
// B = U Σ V^T   →   B*B^T = U Σ² U^T   (6×6 eigen-problem)
// B^+ = Σ_j  (1/σⱼ²)  (B^T uⱼ)  uⱼ^T   (N×6)
//
// Layout: column-major throughout, index = row + nRows*col.
// ---------------------------------------------------------------------------

namespace {

// 6×6 symmetric Jacobi eigen-decomposition.
// On entry:  C = symmetric 6×6 matrix (col-major).
// On exit:   C is diagonal (eigenvalues), U holds orthonormal eigenvectors.
static void jacobi6(std::vector<double>& C, std::vector<double>& U)
{
    constexpr int M = 6;
    U.assign(M * M, 0.0);
    for (int i = 0; i < M; ++i) U[i + M * i] = 1.0;

    for (int sweep = 0; sweep < 200; ++sweep) {
        double offSq = 0.0;
        for (int p = 0; p < M; ++p)
            for (int q = p + 1; q < M; ++q)
                offSq += C[p + M * q] * C[p + M * q];
        if (offSq < 1e-28) break;

        for (int p = 0; p < M - 1; ++p) {
            for (int q = p + 1; q < M; ++q) {
                double cpq = C[p + M * q];
                if (std::abs(cpq) < 1e-15) continue;
                double cpp = C[p + M * p], cqq = C[q + M * q];
                double tau = (cqq - cpp) / (2.0 * cpq);
                double t   = (tau >= 0.0) ?  1.0 / ( tau + std::sqrt(1.0 + tau*tau))
                                           : -1.0 / (-tau + std::sqrt(1.0 + tau*tau));
                double c = 1.0 / std::sqrt(1.0 + t * t);
                double s = t * c;
                C[p + M * p] = cpp - t * cpq;
                C[q + M * q] = cqq + t * cpq;
                C[p + M * q] = C[q + M * p] = 0.0;
                for (int r = 0; r < M; ++r) {
                    if (r == p || r == q) continue;
                    double crp = C[r + M * p], crq = C[r + M * q];
                    C[r + M * p] = C[p + M * r] =  c * crp - s * crq;
                    C[r + M * q] = C[q + M * r] =  s * crp + c * crq;
                }
                for (int r = 0; r < M; ++r) {
                    double urp = U[r + M * p], urq = U[r + M * q];
                    U[r + M * p] =  c * urp - s * urq;
                    U[r + M * q] =  s * urp + c * urq;
                }
            }
        }
    }
}

// Compute B^+ (N×6) for a 6×N effectiveness matrix B (col-major).
static void computePseudoinverse(const std::vector<double>& B, int N,
                                  std::vector<double>& Bpinv)
{
    constexpr int M = 6;

    // C = B * B^T  (6×6, col-major).
    std::vector<double> C(M * M, 0.0);
    for (int i = 0; i < M; ++i)
        for (int j = 0; j < M; ++j)
            for (int k = 0; k < N; ++k)
                C[i + M * j] += B[i + M * k] * B[j + M * k];

    std::vector<double> U;
    jacobi6(C, U);  // C is now diagonal with eigenvalues; U holds eigenvectors.

    // Sort eigenpairs descending by eigenvalue.
    std::vector<int> idx(M);
    std::iota(idx.begin(), idx.end(), 0);
    std::sort(idx.begin(), idx.end(),
              [&](int a, int b){ return C[a + M*a] > C[b + M*b]; });

    std::vector<double> Us(M * M);
    std::vector<double> sigma(M);
    for (int j = 0; j < M; ++j) {
        sigma[j] = std::sqrt(std::max(0.0, C[idx[j] + M * idx[j]]));
        for (int i = 0; i < M; ++i)
            Us[i + M * j] = U[i + M * idx[j]];
    }
    U = Us;

    const double tol = (sigma[0] > 0.0) ? 1e-9 * sigma[0] : 1e-12;

    // B^+ = sum_j  (B^T uⱼ) * uⱼ^T / σⱼ²
    // Bpinv[r, c]  (N×6 col-major, index = r + N*c).
    Bpinv.assign(N * M, 0.0);
    for (int j = 0; j < M; ++j) {
        if (sigma[j] < tol) continue;
        double inv_s2 = 1.0 / (sigma[j] * sigma[j]);

        // v = B^T * U_j  (N-vector).
        for (int r = 0; r < N; ++r) {
            double v_r = 0.0;
            for (int k = 0; k < M; ++k)
                v_r += B[k + M * r] * U[k + M * j];

            for (int c = 0; c < M; ++c)
                Bpinv[r + N * c] += v_r * U[c + M * j] * inv_s2;
        }
    }
}

} // anonymous namespace

namespace spacecraft {

void SpacecraftModel::buildEffectivenessMatrix()
{
    m_N = static_cast<int>(thrusters.size());
    if (m_N == 0) return;

    // Build 6×N matrix B (col-major).
    m_B.assign(6 * m_N, 0.0);
    for (int col = 0; col < m_N; ++col) {
        const auto& t   = thrusters[col];
        glm::vec3   arm = t.position - centerOfMass;
        glm::vec3   f   = t.direction * t.thrustN;
        glm::vec3   tau = glm::cross(arm, f);
        m_B[0 + 6 * col] = f.x;
        m_B[1 + 6 * col] = f.y;
        m_B[2 + 6 * col] = f.z;
        m_B[3 + 6 * col] = tau.x;
        m_B[4 + 6 * col] = tau.y;
        m_B[5 + 6 * col] = tau.z;
    }

    computePseudoinverse(m_B, m_N, m_Bpinv);
}

void SpacecraftModel::solveAllocation(const Wrench& desired)
{
    if (m_N == 0) return;

    // t = B⁺ * desired  (N×6 × 6×1 = N×1)
    for (int r = 0; r < m_N; ++r) {
        double val = 0.0;
        for (int c = 0; c < 6; ++c)
            val += m_Bpinv[r + m_N * c] * desired[c];
        thrusters[r].throttle = static_cast<float>(std::clamp(val, 0.0, 1.0));
    }
}

void SpacecraftModel::stepPWM(double dt, double pwmPeriod, double minDutyCycleFraction)
{
    float maxThrottle = 0.0f;
    for (const auto& t : thrusters)
        maxThrottle = std::max(maxThrottle, t.throttle);

    const float cutoff = static_cast<float>(minDutyCycleFraction) * maxThrottle;

    for (auto& t : thrusters) {
        if (t.throttle < cutoff) {
            t.firing   = false;
            t.pwmPhase = 0.0f;
            continue;
        }
        t.pwmPhase += static_cast<float>(dt);
        if (t.pwmPhase >= static_cast<float>(pwmPeriod))
            t.pwmPhase -= static_cast<float>(pwmPeriod);
        t.firing = (t.pwmPhase < t.throttle * static_cast<float>(pwmPeriod));
    }
}

void SpacecraftModel::accumulateWrench(glm::vec3& forceOut, glm::vec3& torqueOut) const
{
    for (const auto& t : thrusters) {
        if (!t.firing) continue;
        glm::vec3 f   = t.direction * t.thrustN;
        glm::vec3 arm = t.position - centerOfMass;
        forceOut  += f;
        torqueOut += glm::cross(arm, f);
    }
}

} // namespace spacecraft
