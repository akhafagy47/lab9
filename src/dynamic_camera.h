#ifndef DYNAMIC_CAMERA_H
#define DYNAMIC_CAMERA_H

#include <iostream>
#include <limits>

#include "camera.h"

constexpr float INF = std::numeric_limits<float>::infinity();

// AABB in 1D
struct Interval {
    float min;
    float max;

    constexpr bool contains(const Interval& other) const {
        return (min <= other.min) && (max >= other.max);
    }

    constexpr bool contains(float x) const { return (min <= x) && (x <= max); }

    // signedDistance returns a positive smallest distance of x from the interval boundary
    // if it is not contained, else it returns a smallest (in magnitude) negative distance
    // from the interval boundary.
    constexpr float signedDistance(float x) const {
        if (contains(x)) {
            // 0 or negative
            return std::max(x - max, min - x);
        }
        // positive
        return std::max(x - max, min - x);
    }
};

struct DynamicCamera {
    void update(Camera& camera,                           // the camera to update
                const sf::Vector2f& trackedPosWorld,      // world coords of player to track
                const sf::Vector2f& trackedVelocityWorld  // world velocity of player to track
    ) {
        // Player position in normalized window coords, relative to the camera *without*
        // lookahead (the actual camera center is shifted by lookaheadState, so we can't use
        // camera.worldToWindowPercent directly). The unadjusted center maps to 0.5.
        const float playerX =
            camera.worldToWindowPercentDist(trackedPosWorld - cameraUnadjustedCenter).x + 0.5f;

        // Lookahead target in world coords: ahead in the direction of motion, 0 when stopped.
        float lookAheadTargetWorld = 0.f;
        if (trackedVelocityWorld.x > 0.f) {
            lookAheadTargetWorld = camera.windowPercentToWorldDist({lookAhead, 0.f}).x;
        } else if (trackedVelocityWorld.x < 0.f) {
            lookAheadTargetWorld = -camera.windowPercentToWorldDist({lookAhead, 0.f}).x;
        }
        lookaheadState = alpha * lookAheadTargetWorld + (1.f - alpha) * lookaheadState;

        // Signed amount (window percent) by which x lies outside the interval; 0 if inside.
        auto overshoot = [](const Interval& interval, float x) {
            if (x > interval.max) {
                return x - interval.max;
            }
            if (x < interval.min) {
                return x - interval.min;
            }
            return 0.f;
        };
        const float deadOvershoot = overshoot(deadZoneX, playerX);
        const float softOvershoot = overshoot(softZoneX, playerX);

        // Dead zone: no movement (deadOvershoot == 0).
        // Soft zone: EMA the camera toward bringing the player back to the dead zone edge.
        float displacementPercent = alpha * deadOvershoot;
        if (softOvershoot != 0.f) {
            // Hard zone: immediately move enough to put the player back on the soft zone edge,
            // then EMA the remainder toward the dead zone edge as in the soft zone.
            displacementPercent = softOvershoot + alpha * (deadOvershoot - softOvershoot);
        }

        sf::Vector2f newUnadjustedCenter = cameraUnadjustedCenter;
        newUnadjustedCenter.x += camera.windowPercentToWorldDist({displacementPercent, 0.f}).x;

        camera.setCameraCenter(newUnadjustedCenter + sf::Vector2f{lookaheadState, 0.f});
        cameraUnadjustedCenter = newUnadjustedCenter;
    }

    float alpha;  // alpha to use for damping of camera position, *as well as* for smoothing of
                  // lookahead offset.
    Interval deadZoneX;  // percentage interval along x-axis to use for dead zone in [0, 1]
    // percentage interval along x-axis to use for soft rejection zone in [0, 1]
    Interval softZoneX = {-INF, INF};
    // (everything beyond softZone is considered hard zone).
    float lookAhead;  // screen-percentage lookahead
    mutable sf::Vector2f
        cameraUnadjustedCenter;      // the camera world center from last frame, ignoring lookahead
    mutable float lookaheadState{};  // the EMA of the accumulated lookahead offset in world coords
};

#endif // DYNAMIC_CAMERA_H
