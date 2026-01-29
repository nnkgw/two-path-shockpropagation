#include "Simulation.h"

#define GLM_ENABLE_EXPERIMENTAL

#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/norm.hpp>
#include <iostream>
#include <algorithm>


namespace {
constexpr float kSatEps = 1e-6f;

static glm::mat3 computeInvInertiaWorld(const Box& b, float weight) {
    if (weight <= 0.0f) return glm::mat3(0.0f);
    float m = b.getMass();
    if (m <= 0.0f) return glm::mat3(0.0f);

    glm::vec3 s = b.getSize(); // full lengths
    float Ixx = (m / 12.0f) * (s.y * s.y + s.z * s.z);
    float Iyy = (m / 12.0f) * (s.x * s.x + s.z * s.z);
    float Izz = (m / 12.0f) * (s.x * s.x + s.y * s.y);

    glm::mat3 invLocal(0.0f);
    invLocal[0][0] = (Ixx > 0.0f) ? (1.0f / Ixx) : 0.0f;
    invLocal[1][1] = (Iyy > 0.0f) ? (1.0f / Iyy) : 0.0f;
    invLocal[2][2] = (Izz > 0.0f) ? (1.0f / Izz) : 0.0f;

    glm::mat3 R = glm::mat3_cast(b.getOrientation());
    glm::mat3 invWorld = R * invLocal * glm::transpose(R);
    return invWorld * weight;
}

static glm::vec3 supportPoint(const Box& b, const glm::vec3& dir) {
    glm::vec3 half = b.getSize() * 0.5f;
    glm::mat3 R = glm::mat3_cast(b.getOrientation());

    glm::vec3 local(
        (glm::dot(dir, R[0]) >= 0.0f) ? half.x : -half.x,
        (glm::dot(dir, R[1]) >= 0.0f) ? half.y : -half.y,
        (glm::dot(dir, R[2]) >= 0.0f) ? half.z : -half.z
    );
    return b.getPosition() + R * local;
}

static void integrateOrientation(Box& b, float dt) {
    glm::vec3 w = b.getAngularVelocity();
    glm::quat q = b.getOrientation();
    glm::quat dq = 0.5f * glm::quat(0.0f, w.x, w.y, w.z) * q;
    b.setOrientation(glm::normalize(q + dq * dt));
}

static float computeMinYVertex(const Box& b, glm::vec3* outPoint) {
    glm::vec3 half = b.getSize() * 0.5f;
    glm::mat3 R = glm::mat3_cast(b.getOrientation());
    glm::vec3 c = b.getPosition();

    float minY = 1e30f;
    glm::vec3 minP(0.0f);
    for (int sx = -1; sx <= 1; sx += 2) {
        for (int sy = -1; sy <= 1; sy += 2) {
            for (int sz = -1; sz <= 1; sz += 2) {
                glm::vec3 local((float)sx * half.x, (float)sy * half.y, (float)sz * half.z);
                glm::vec3 p = c + R * local;
                if (p.y < minY) { minY = p.y; minP = p; }
            }
        }
    }
    if (outPoint) *outPoint = minP;
    return minY;
}

static int computeVertices(const Box& b, glm::vec3 outV[8]) {
    glm::vec3 half = b.getSize() * 0.5f;
    glm::mat3 R = glm::mat3_cast(b.getOrientation());
    glm::vec3 c = b.getPosition();

    int idx = 0;
    for (int sx = -1; sx <= 1; sx += 2) {
        for (int sy = -1; sy <= 1; sy += 2) {
            for (int sz = -1; sz <= 1; sz += 2) {
                glm::vec3 local((float)sx * half.x, (float)sy * half.y, (float)sz * half.z);
                outV[idx++] = c + R * local;
            }
        }
    }
    return idx;
}
} // namespace

Simulation::Simulation()
    : currentMode(FIG1_UPWARD), currentStep(0), isAutoPlay(false),
      autoPlayTimer(0.0f), autoPlayInterval(1.5f),
      gravity(0.0f, -9.81f, 0.0f), numSubsteps(25), numIterations(40) {
    initializeBoxes();
    updateStep();
}

void Simulation::initializeBoxes() {
    boxes.clear();
    // X (blue), Y (orange), Z (yellow)
    // Fig.1(a) initial state: Only X penetrates ground.
    // X center at y=0.3, size=1.0 -> bottom at -0.2 (penetrates ground by 0.2), top at 0.8.
    // Y center at y=1.3, size=1.0 -> bottom at 0.8 (exactly touches X's top).
    // Z center at y=2.3, size=1.0 -> bottom at 1.8 (exactly touches Y's top).
    // Note: To ensure they touch, Y should be at 0.8 + 0.5 = 1.3, Z at 1.3 + 0.5 + 0.5 = 2.3.
    // Wait, if X top is 0.8, Y bottom should be 0.8. Y center = 0.8 + 0.5 = 1.3. Correct.
    // If Y top is 1.3 + 0.5 = 1.8, Z bottom should be 1.8. Z center = 1.8 + 0.5 = 2.3. Correct.
    boxes.push_back(Box("X", glm::vec3(0.0f, 0.3f, 0.0f), glm::vec3(1.0f), glm::vec3(0.3f, 0.5f, 0.9f), 1.0f));
    boxes.push_back(Box("Y", glm::vec3(0.0f, 1.3f, 0.0f), glm::vec3(1.0f), glm::vec3(0.9f, 0.5f, 0.3f), 1.0f));
    boxes.push_back(Box("Z", glm::vec3(0.0f, 2.3f, 0.0f), glm::vec3(1.0f), glm::vec3(0.9f, 0.9f, 0.3f), 1.0f));
}

void Simulation::setMode(Mode mode) {
    currentMode = mode;
    currentStep = 0;
    reset();
}

void Simulation::reset() {
    initializeBoxes();
    currentStep = 0;
    updateStep();
}

void Simulation::nextStep() {
    int totalSteps = getTotalSteps();
    if (currentStep < totalSteps - 1) {
        currentStep++;
        updateStep();
    }
}

void Simulation::prevStep() {
    if (currentStep > 0) {
        currentStep--;
        updateStep();
    }
}

void Simulation::autoPlay(bool enable) {
    isAutoPlay = enable;
    autoPlayTimer = 0.0f;
}

void Simulation::update(float deltaTime) {
    // Limit deltaTime to prevent instability
    float dt = std::min(deltaTime, 0.016f); 
    stepPhysics(dt);

    if (isAutoPlay) {
        autoPlayTimer += deltaTime;
        if (autoPlayTimer >= autoPlayInterval) {
            autoPlayTimer = 0.0f;
            nextStep();
            if (currentStep >= getTotalSteps() - 1) {
                currentStep = 0;
                updateStep();
            }
        }
    }
}

void Simulation::stepPhysics(float dt) {
    if (dt <= 0.0f) return;
    float sdt = dt / numSubsteps;

    for (int s = 0; s < numSubsteps; s++) {
        // 1. Predict positions
        for (auto& box : boxes) {
            if (box.isInfiniteMass()) continue;
            
            // Apply gravity
            if (currentMode == FIG3_DOWNWARD_DYNAMIC || currentMode == PHYSICS_SIM) {
                box.setVelocity(box.getVelocity() + gravity * sdt);
            }
            
            box.prevPosition = box.getPosition();
            box.setPosition(box.getPosition() + box.getVelocity() * sdt);
            
                        box.prevOrientation = box.getOrientation();
            integrateOrientation(box, sdt);
        }

        // 2. Solve constraints
        // Fig.1 uses its own per-layer convergence loop inside solveConstraints().
        // Calling it numIterations times would effectively multiply the intended work
        // and can lead to over-correction.
        int iterations = numIterations;
        if (currentMode == FIG1_UPWARD) iterations = 1;
        for (int i = 0; i < iterations; i++) {
            solveConstraints(sdt);
        }

        // 3. Update velocities and apply damping
        for (auto& box : boxes) {
            if (box.isInfiniteMass()) continue;

            // Fig.1/Fig.2 are presented as quasi-static, step-by-step constraint projections.
            // Do not accumulate the "velocity" induced by positional correction, or the
            // demonstration can drift/explode.
            if (currentMode == FIG1_UPWARD || currentMode == FIG2_DOWNWARD_STATIC) {
                box.setVelocity(glm::vec3(0.0f));
                box.setAngularVelocity(glm::vec3(0.0f));
                continue;
            }
            
            glm::vec3 newVel = (box.getPosition() - box.prevPosition) / sdt;
            
            // Velocity Clamping to prevent "flying away"
            float maxVel = 10.0f;
            if (glm::length(newVel) > maxVel) {
                newVel = glm::normalize(newVel) * maxVel;
            }
            float velDamping = (currentMode == FIG3_DOWNWARD_DYNAMIC || currentMode == PHYSICS_SIM) ? 0.9995f : 0.95f;
            glm::vec3 v = newVel * velDamping;

            // Angular velocity from the orientation change during constraint projection.
            glm::quat dq = box.getOrientation() * glm::inverse(box.prevOrientation);
            glm::vec3 axis(dq.x, dq.y, dq.z);
            float angle = 2.0f * acos(glm::clamp(dq.w, -1.0f, 1.0f));

            glm::vec3 angVel(0.0f);
            if (angle > 0.001f && glm::length2(axis) > 1e-12f) {
                angVel = glm::normalize(axis) * (angle / sdt) * 0.98f;
            }

            // Ground friction (kinetic + simple static) for the dynamic modes.
            if (currentMode == FIG3_DOWNWARD_DYNAMIC || currentMode == PHYSICS_SIM) {
                glm::vec3 groundP;
                float minY = computeMinYVertex(box, &groundP);
                bool onGround = (minY <= 0.01f);

                // Prevent "sinking" by canceling downward velocity when grounded.
                if (onGround && v.y < 0.0f) v.y = 0.0f;

                // Kinetic friction: decelerate the horizontal velocity by mu*g.
                // Also apply the corresponding torque around the contact point so the box can
                // naturally rotate while sliding and settle flat.
                if (onGround) {
                    glm::vec3 vt0(v.x, 0.0f, v.z);

                    float mu = 0.7f;
                    float g = glm::abs(gravity.y);
                    float maxDelta = mu * g * sdt;
                    float speed = glm::length(vt0);
                    if (speed <= maxDelta) {
                        v.x = 0.0f;
                        v.z = 0.0f;
                    } else if (speed > 1e-8f) {
                        float newSpeed = speed - maxDelta;
                        glm::vec3 vt = vt0 * (newSpeed / speed);
                        v.x = vt.x;
                        v.z = vt.z;
                    }

                    glm::vec3 dv(v.x - vt0.x, 0.0f, v.z - vt0.z);
                    float m = box.getMass();
                    if (m > 0.0f && glm::length2(dv) > 1e-12f) {
                        glm::mat3 invI = computeInvInertiaWorld(box, 1.0f);
                        glm::vec3 r = groundP - box.getPosition();
                        glm::vec3 J = m * dv;
                        angVel += invI * glm::cross(r, J);
                    }

                    float maxAng = 25.0f;
                    float angLen = glm::length(angVel);
                    if (angLen > maxAng) angVel = (angVel / angLen) * maxAng;
                }

                // Rolling / angular friction when grounded.
                if (onGround) {
                    angVel *= 0.985f;
                }
            }

            box.setVelocity(v);
            box.setAngularVelocity(angVel);
        }
    }
}

void Simulation::solveConstraints(float dt) {
    // NOTE:
    //  - FIG1_UPWARD is a pedagogical, step-by-step reproduction of Fig.1 in the paper.
    //    In (a) and (b), we must NOT resolve the ground contact yet; otherwise X is pushed up
    //    immediately and Y starts interpenetrating X, which contradicts Fig.1(a).
    //  - Therefore, in FIG1 we only activate each constraint on the corresponding "Apply" step.
    if (currentMode == FIG1_UPWARD) {
        // Step mapping (Fig.1):
        //  0: (a) Initial
        //  1: (b) Eval: Ground-X          (no position change)
        //  2: (c) Apply to X (Top)        (resolve Ground-X)
        //  3: (d) Eval: X-Y               (no position change)
        //  4: (e) Apply to Y (Top)        (resolve X-Y with X treated as infinite)
        //  5: (f) Eval: Y-Z               (no position change)
        //  6: (g) Apply to Z (Top)        (resolve Y-Z with Y treated as infinite)
        // In the paper, each layer is iterated until convergence.
        // Here we emulate that by iterating each active constraint until the remaining
        // penetration is below a small tolerance.
        constexpr int kMaxLayerIters = 75;
        constexpr float kTolPen = 5e-4f;

        auto solveGroundUntil = [&](Box& box) {
            for (int it = 0; it < kMaxLayerIters; ++it) {
                float minY = computeMinYVertex(box, nullptr);
                if (minY >= -kTolPen) break;
                solveGround(box);
            }
        };

        auto solvePairUntil = [&](Box& a, Box& b, float wA, float wB) {
            for (int it = 0; it < kMaxLayerIters; ++it) {
                Contact c;
                if (!checkCollision(a, b, c)) break;
                if (c.penetration <= kTolPen) break;
                solveContact(a, b, c, wA, wB);
            }
        };

        if (currentStep >= 2) solveGroundUntil(boxes[0]);
        if (currentStep >= 4) solvePairUntil(boxes[0], boxes[1], 0.0f, 1.0f);
        if (currentStep >= 6) solvePairUntil(boxes[1], boxes[2], 0.0f, 1.0f);
        return;
    }

    // 1. Upward Pass (Shock Propagation)
    // Solve from bottom to top, treating the bottom object as static (infinite mass)

    // Ground constraints for all boxes
    for (auto& box : boxes) solveGround(box);

    // In Downward Pass or Physics Sim, we perform the full Upward Pass first
    // Check all pairs to prevent skipping collisions when boxes move horizontally
    solveContact(boxes[0], boxes[1], {}, 0.0f, 1.0f);
    solveContact(boxes[1], boxes[2], {}, 0.0f, 1.0f);
    solveContact(boxes[0], boxes[2], {}, 0.0f, 1.0f); // X-Z direct contact if Y is pushed out

    // 2. Downward Pass (Standard Gauss-Seidel)
    // Solve with normal mass ratios to allow settling
    solveContact(boxes[1], boxes[2], {}, 0.5f, 0.5f);
    solveContact(boxes[0], boxes[1], {}, 0.5f, 0.5f);
    solveContact(boxes[0], boxes[2], {}, 0.5f, 0.5f);

    for (auto& box : boxes) solveGround(box);
}


void Simulation::solveGround(Box& box) {
    if (box.isInfiniteMass()) return;

    glm::vec3 n(0.0f, 1.0f, 0.0f);
    bool allowRotation = (currentMode == FIG3_DOWNWARD_DYNAMIC || currentMode == PHYSICS_SIM);

    // Static / step-by-step modes: resolve using only the minimum vertex.
    // This avoids multi-vertex over-correction that can break Fig.1/2.
    if (!allowRotation) {
        glm::vec3 pMin;
        float minY = computeMinYVertex(box, &pMin);
        if (minY < 0.0f) {
            // Full correction is safe here because we are using only the minimum vertex.
            float dy = (-minY);
            box.setPosition(box.getPosition() + glm::vec3(0.0f, dy, 0.0f));
        }
        return;
    }

    // Dynamic modes: use a Gauss-Seidel style ground solve.
    // Each mini-iteration resolves only the currently most-penetrating vertex,
    // recomputing vertices after applying the correction. This prevents "pushing" the box
    // multiple times by the same penetration amount.
    float invMass = box.getInvMass();
    glm::mat3 invI = computeInvInertiaWorld(box, 1.0f);

    // Clamp per-iteration correction to avoid explosive position updates -> large velocities.
    constexpr float kMaxCorr = 0.05f;

    for (int it = 0; it < 4; ++it) {
        glm::vec3 V[8];
        computeVertices(box, V);

        int best = -1;
        float bestPen = 0.0f;
        for (int i = 0; i < 8; ++i) {
            float pen = -V[i].y;
            if (pen > bestPen) { bestPen = pen; best = i; }
        }
        if (best < 0 || bestPen <= 0.0f) break;

        glm::vec3 p = V[best];
        glm::vec3 r = p - box.getPosition();
        glm::vec3 rn = glm::cross(r, n);
        float k = glm::dot(n, glm::cross(invI * rn, r));
        float denom = invMass + k;
        if (denom <= 0.0f) break;

        float lambda = bestPen / denom;
        glm::vec3 corr = lambda * n;
        float corrLen = glm::length(corr);
        if (corrLen > kMaxCorr) corr *= (kMaxCorr / corrLen);

        box.setPosition(box.getPosition() + corr * invMass);

        glm::vec3 dtheta = invI * glm::cross(r, corr);
        glm::quat q = box.getOrientation();
        glm::quat dq = 0.5f * glm::quat(0.0f, dtheta.x, dtheta.y, dtheta.z) * q;
        box.setOrientation(glm::normalize(q + dq));
    }
}


bool Simulation::checkCollision(const Box& a, const Box& b, Contact& contact) {
    // Oriented box vs oriented box using SAT (15 axes).
    glm::vec3 aHalf = a.getSize() * 0.5f;
    glm::vec3 bHalf = b.getSize() * 0.5f;

    glm::mat3 RA = glm::mat3_cast(a.getOrientation());
    glm::mat3 RB = glm::mat3_cast(b.getOrientation());

    glm::vec3 Aaxis[3] = { RA[0], RA[1], RA[2] };
    glm::vec3 Baxis[3] = { RB[0], RB[1], RB[2] };

    float R[3][3];
    float AbsR[3][3];

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R[i][j] = glm::dot(Aaxis[i], Baxis[j]);
            AbsR[i][j] = glm::abs(R[i][j]) + kSatEps;
        }
    }

    glm::vec3 tV = b.getPosition() - a.getPosition();
    float t[3] = {
        glm::dot(tV, Aaxis[0]),
        glm::dot(tV, Aaxis[1]),
        glm::dot(tV, Aaxis[2])
    };

    float minPen = 1e30f;
    glm::vec3 bestAxis(0.0f);

    auto considerAxis = [&](const glm::vec3& axisWorld, float pen) {
        if (pen < minPen) {
            minPen = pen;
            bestAxis = axisWorld;
        }
    };

    // Test axes L = A0, A1, A2
    for (int i = 0; i < 3; i++) {
        float ra = aHalf[i];
        float rb = bHalf.x * AbsR[i][0] + bHalf.y * AbsR[i][1] + bHalf.z * AbsR[i][2];
        float dist = glm::abs(t[i]);
        float pen = (ra + rb) - dist;
        if (pen < 0.0f) return false;
        considerAxis(Aaxis[i], pen);
    }

    // Test axes L = B0, B1, B2
    for (int j = 0; j < 3; j++) {
        float ra = aHalf.x * AbsR[0][j] + aHalf.y * AbsR[1][j] + aHalf.z * AbsR[2][j];
        float rb = bHalf[j];
        float dist = glm::abs(t[0] * R[0][j] + t[1] * R[1][j] + t[2] * R[2][j]);
        float pen = (ra + rb) - dist;
        if (pen < 0.0f) return false;
        considerAxis(Baxis[j], pen);
    }

    // Test axes L = Ai x Bj
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            glm::vec3 axis = glm::cross(Aaxis[i], Baxis[j]);
            float len2 = glm::length2(axis);
            if (len2 < 1e-10f) continue;

            float ra = aHalf[(i + 1) % 3] * AbsR[(i + 2) % 3][j] + aHalf[(i + 2) % 3] * AbsR[(i + 1) % 3][j];
            float rb = bHalf[(j + 1) % 3] * AbsR[i][(j + 2) % 3] + bHalf[(j + 2) % 3] * AbsR[i][(j + 1) % 3];
            float dist = glm::abs(t[(i + 2) % 3] * R[(i + 1) % 3][j] - t[(i + 1) % 3] * R[(i + 2) % 3][j]);

            float pen = (ra + rb) - dist;
            if (pen < 0.0f) return false;

            float invLen = 1.0f / glm::sqrt(len2);
            considerAxis(axis * invLen, pen * invLen);
        }
    }

    if (glm::length2(bestAxis) < 1e-12f) return false;

    glm::vec3 n = glm::normalize(bestAxis);
    if (glm::dot(n, tV) < 0.0f) n = -n;

    contact.normal = n;
    contact.penetration = minPen;

    // Approximate contact point using support points along +/- normal.
    glm::vec3 pA = supportPoint(a, n);
    glm::vec3 pB = supportPoint(b, -n);
    contact.point = 0.5f * (pA + pB);

    return true;
}

void Simulation::solveContact(Box& a, Box& b, const Contact& contactIn, float weightA, float weightB) {
    (void)contactIn;

    Contact c;
    if (!checkCollision(a, b, c)) return;

    bool allowRotation = (currentMode == FIG3_DOWNWARD_DYNAMIC || currentMode == PHYSICS_SIM);

    glm::vec3 n = c.normal;
    float penetration = c.penetration;
    glm::vec3 p = c.point;

    float invMassA = a.getInvMass() * weightA;
    float invMassB = b.getInvMass() * weightB;

    glm::mat3 invIA = computeInvInertiaWorld(a, weightA);
    glm::mat3 invIB = computeInvInertiaWorld(b, weightB);

    glm::vec3 rA = p - a.getPosition();
    glm::vec3 rB = p - b.getPosition();

    float kA = 0.0f;
    float kB = 0.0f;
    if (allowRotation && invMassA > 0.0f) {
        glm::vec3 rnA = glm::cross(rA, n);
        kA = glm::dot(n, glm::cross(invIA * rnA, rA));
    }
    if (allowRotation && invMassB > 0.0f) {
        glm::vec3 rnB = glm::cross(rB, n);
        kB = glm::dot(n, glm::cross(invIB * rnB, rB));
    }

    float denom = invMassA + invMassB + (allowRotation ? (kA + kB) : 0.0f);
    if (denom <= 0.0f) return;

    // Soft positional correction to avoid "explosive" push-outs.
    // In the real-time modes, both the upward and downward passes can be applied
    // many times (iterations * substeps). Applying full penetration correction each
    // time can inject large positional changes -> large velocities.
    float slop = (currentMode == FIG3_DOWNWARD_DYNAMIC || currentMode == PHYSICS_SIM) ? 0.001f : 0.0f;
    float pen = penetration - slop;
    if (pen <= 0.0f) return;

    float relax = 1.0f;
    if (currentMode == PHYSICS_SIM) relax = 0.25f;
    else if (currentMode == FIG3_DOWNWARD_DYNAMIC) relax = 0.35f;
    else if (currentMode == FIG2_DOWNWARD_STATIC) relax = 0.6f;

    float lambda = (pen / denom) * relax;
    glm::vec3 corr = lambda * n;

    // Clamp per-call correction to keep the solver stable.
    float maxCorr = (currentMode == PHYSICS_SIM) ? 0.03f : 0.06f;
    float corrLen = glm::length(corr);
    if (corrLen > maxCorr) corr *= (maxCorr / corrLen);

    // Positional correction (non-penetration).
    if (invMassA > 0.0f) a.setPosition(a.getPosition() - corr * invMassA);
    if (invMassB > 0.0f) b.setPosition(b.getPosition() + corr * invMassB);

    // Rotational correction (dynamic modes only).
    if (allowRotation) {
        if (invMassA > 0.0f) {
            glm::vec3 dtheta = -(invIA * glm::cross(rA, corr));
            glm::quat q = a.getOrientation();
            glm::quat dq = 0.5f * glm::quat(0.0f, dtheta.x, dtheta.y, dtheta.z) * q;
            a.setOrientation(glm::normalize(q + dq));
        }
        if (invMassB > 0.0f) {
            glm::vec3 dtheta = (invIB * glm::cross(rB, corr));
            glm::quat q = b.getOrientation();
            glm::quat dq = 0.5f * glm::quat(0.0f, dtheta.x, dtheta.y, dtheta.z) * q;
            b.setOrientation(glm::normalize(q + dq));
        }
    } else {
        // For the static figure modes, keep the stack visually vertical.
        if (glm::abs(n.y) > 0.9f) {
            glm::vec3 vA = a.getVelocity();
            glm::vec3 vB = b.getVelocity();
            a.setVelocity(glm::vec3(vA.x * 0.8f, vA.y, vA.z * 0.8f));
            b.setVelocity(glm::vec3(vB.x * 0.8f, vB.y, vB.z * 0.8f));
        }
    }
}

int Simulation::getTotalSteps() const {
    switch (currentMode) {
        case FIG1_UPWARD: return 7;
        case FIG2_DOWNWARD_STATIC: return 8;
        case FIG3_DOWNWARD_DYNAMIC: return 8;
        case PHYSICS_SIM: return 1;
        default: return 7;
    }
}

std::string Simulation::getModeName() const {
    switch (currentMode) {
        case FIG1_UPWARD: return "Fig.1: Upward Pass (Static)";
        case FIG2_DOWNWARD_STATIC: return "Fig.2: Downward Pass (Static)";
        case FIG3_DOWNWARD_DYNAMIC: return "Fig.3: Downward Pass (Dynamic)";
        case PHYSICS_SIM: return "Real-time Physics (Two-Pass)";
        default: return "Unknown";
    }
}

std::string Simulation::getCurrentStepDescription() const {
    switch (currentMode) {
        case FIG1_UPWARD:
            switch (currentStep) {
                case 0: return "(a) Initial";
                case 1: return "(b) Eval: Ground-X";
                case 2: return "(c) Apply to T (X)";
                case 3: return "(d) Eval: X-Y";
                case 4: return "(e) Apply to T (Y)";
                case 5: return "(f) Eval: Y-Z";
                case 6: return "(g) Apply to T (Z)";
                default: return "";
            }
        case FIG2_DOWNWARD_STATIC:
            switch (currentStep) {
                case 0: return "(a) Eval: Z-Y";
                case 1: return "(b) Apply to T (Z)";
                case 2: return "(c) Apply to B (Y)";
                case 3: return "(d) Eval: Y-X";
                case 4: return "(e) Apply to T (Y)";
                case 5: return "(f) Apply to B (X)";
                case 6: return "(g) Eval: X-Ground";
                case 7: return "(h) Apply to T (X)";
                default: return "";
            }
        case FIG3_DOWNWARD_DYNAMIC:
            switch (currentStep) {
                case 0: return "(a) Eval: Z-Y";
                case 1: return "(b) Apply to T (Z)";
                case 2: return "(c) Apply to B (Y)";
                case 3: return "(d) Eval: Y-X";
                case 4: return "(e) Apply to T (Y)";
                case 5: return "(f) Apply to B (X)";
                case 6: return "(g) Eval: X-Ground";
                case 7: return "(h) Apply to T (X)";
                default: return "";
            }
        case PHYSICS_SIM:
            return "(sim)";
        default: return "";
    }
}

void Simulation::updateStep() {
    impulses.clear();
    printStepExplanation();
    
    glm::vec3 yellowImpulse(1.0f, 0.8f, 0.0f);
    glm::vec3 redImpulse(1.0f, 0.1f, 0.1f);
    glm::vec3 blueImpulse(0.2f, 0.4f, 1.0f);
    
    for (auto& box : boxes) box.setInfiniteMass(false);

    if (currentMode == PHYSICS_SIM) {
        boxes[0].setPosition(glm::vec3(0.0f, 0.3f, 0.0f));
        // Match Fig.1(a): only X penetrates the ground; Y/Z are initially just touching.
        boxes[1].setPosition(glm::vec3(0.0f, 1.3f, 0.0f));
        boxes[2].setPosition(glm::vec3(0.15f, 2.3f, 0.0f));
        return;
    }

    if (currentMode == FIG1_UPWARD) {
        switch (currentStep) {
            case 0: 
                boxes[0].setPosition(glm::vec3(0.0f, 0.3f, 0.0f)); 
                // Match Fig.1(a): only X penetrates the ground; Y/Z are initially just touching.
                boxes[1].setPosition(glm::vec3(0.0f, 1.3f, 0.0f));
                boxes[2].setPosition(glm::vec3(0.0f, 2.3f, 0.0f));
                break;
            case 2: impulses.push_back(Impulse(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), yellowImpulse, 0.5f)); break;
            case 3: boxes[0].setInfiniteMass(true); break;
            case 4: impulses.push_back(Impulse(glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), yellowImpulse, 0.5f));
                    impulses.push_back(Impulse(glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f), redImpulse, 0.5f)); break;
            case 5: boxes[1].setInfiniteMass(true); break;
            case 6: impulses.push_back(Impulse(glm::vec3(0.0f, 2.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), yellowImpulse, 0.5f));
                    impulses.push_back(Impulse(glm::vec3(0.0f, 2.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f), redImpulse, 0.5f));
                    impulses.push_back(Impulse(glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f), blueImpulse, 0.5f)); break;
        }
    } else if (currentMode == FIG2_DOWNWARD_STATIC) {
        switch (currentStep) {
            case 0:
                // Start of the downward pass (Fig.2): this begins from the end of the upward pass,
                // where all penetrations have been resolved assuming the Bottom body is infinite.
                boxes[0].setPosition(glm::vec3(0.0f, 0.5f, 0.0f));
                boxes[1].setPosition(glm::vec3(0.0f, 1.5f, 0.0f));
                boxes[2].setPosition(glm::vec3(0.0f, 2.5f, 0.0f));
                break;
            case 1: impulses.push_back(Impulse(glm::vec3(0.0f, 2.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), yellowImpulse, 0.1f)); break;
            case 2: impulses.push_back(Impulse(glm::vec3(0.0f, 2.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f), redImpulse, 0.5f)); break;
            case 3: boxes[0].setInfiniteMass(true); break;
            case 4: impulses.push_back(Impulse(glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), yellowImpulse, 0.5f)); break;
            case 5: impulses.push_back(Impulse(glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f), blueImpulse, 0.5f)); break;
            case 7: impulses.push_back(Impulse(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), yellowImpulse, 0.5f)); break;
        }
    } else if (currentMode == FIG3_DOWNWARD_DYNAMIC) {
        if (currentStep == 0) {
            // Start of the dynamic downward pass (Fig.3): begin from the end of the upward pass,
            // and introduce a small horizontal perturbation on Z.
            boxes[0].setPosition(glm::vec3(0.0f, 0.5f, 0.0f));
            boxes[1].setPosition(glm::vec3(0.0f, 1.5f, 0.0f));
            // Overhang + small lateral drift so Z slides off Y and falls.
            boxes[2].setPosition(glm::vec3(0.45f, 2.5f, 0.0f));
            boxes[0].setVelocity(glm::vec3(0.0f));
            boxes[1].setVelocity(glm::vec3(0.0f));
            boxes[2].setVelocity(glm::vec3(1.2f, 0.0f, 0.0f));
            boxes[0].setAngularVelocity(glm::vec3(0.0f));
            boxes[1].setAngularVelocity(glm::vec3(0.0f));
            boxes[2].setAngularVelocity(glm::vec3(0.0f));
        }
        switch (currentStep) {
            case 1: impulses.push_back(Impulse(glm::vec3(0.45f, 2.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), yellowImpulse, 0.1f)); break;
            case 2: impulses.push_back(Impulse(glm::vec3(0.45f, 2.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f), redImpulse, 0.5f)); break;
            case 5: impulses.push_back(Impulse(glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f), blueImpulse, 0.5f)); break;
            case 7: impulses.push_back(Impulse(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), yellowImpulse, 0.5f)); break;
        }
    }
}

void Simulation::printStepExplanation() const {
    std::cout << "\n--- " << getModeName() << " ---" << std::endl;
    std::cout << "Step " << (currentStep + 1) << "/" << getTotalSteps() << ": " << getCurrentStepDescription() << std::endl;
}
