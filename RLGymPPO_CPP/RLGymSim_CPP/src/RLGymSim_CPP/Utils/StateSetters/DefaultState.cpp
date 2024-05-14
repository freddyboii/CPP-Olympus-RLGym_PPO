// DefaultState.cpp
#include "DefaultState.h"
#include "../../Math.h"
#include <algorithm>
#include <random>

namespace RLGSC {
    const std::vector<Vec> DefaultState::SPAWN_BLUE_POS = {
        Vec(-2048.0f, -2560.0f, 17.0f),
        Vec(2048.0f, -2560.0f, 17.0f),
        Vec(-256.0f, -3840.0f, 17.0f),
        Vec(256.0f, -3840.0f, 17.0f),
        Vec(0.0f, -4608.0f, 17.0f)
    };

    const std::vector<float> DefaultState::SPAWN_BLUE_YAW = {
        0.25f * M_PI,
        0.75f * M_PI,
        0.5f * M_PI,
        0.5f * M_PI,
        0.5f * M_PI
    };

    const std::vector<Vec> DefaultState::SPAWN_ORANGE_POS = {
        Vec(2048.0f, 2560.0f, 17.0f),
        Vec(-2048.0f, 2560.0f, 17.0f),
        Vec(256.0f, 3840.0f, 17.0f),
        Vec(-256.0f, 3840.0f, 17.0f),
        Vec(0.0f, 4608.0f, 17.0f)
    };

    const std::vector<float> DefaultState::SPAWN_ORANGE_YAW = {
        -0.75f * M_PI,
        -0.25f * M_PI,
        -0.5f * M_PI,
        -0.5f * M_PI,
        -0.5f * M_PI
    };

    GameState DefaultState::ResetState(Arena* arena) {
        // Set the ball position to (0, 0, BALL_RADIUS)
        BallState bs = {};
        bs.pos = Vec(0.0f, 0.0f, CommonValues::BALL_RADIUS);
        bs.vel = Vec(0.0f, 0.0f, 0.0f);
        bs.angVel = Vec(0.0f, 0.0f, 0.0f); // Set ball angular velocity
        arena->ball->SetState(bs);

        // Shuffle the spawn indices
        std::vector<int> spawnInds = {0, 1, 2, 3, 4};
        std::random_device rd;
        std::mt19937 g(rd());
        std::shuffle(spawnInds.begin(), spawnInds.end(), g);

        int blueCount = 0, orangeCount = 0;

        for (Car* car : arena->_cars) {
            Vec pos = Vec(0.0f, 0.0f, 0.0f);
            float yaw = 0.0f;
            float pitch = 0.0f; // Initialize pitch to 0
            float roll = 0.0f; // Initialize roll to 0

            if (car->team == Team::BLUE) {
                // Spawn for the blue team
                pos = SPAWN_BLUE_POS[spawnInds[blueCount]];
                yaw = SPAWN_BLUE_YAW[spawnInds[blueCount]];
                blueCount++;
            } else if (car->team == Team::ORANGE) {
                // Spawn for the orange team
                pos = SPAWN_ORANGE_POS[spawnInds[orangeCount]];
                yaw = SPAWN_ORANGE_YAW[spawnInds[orangeCount]];
                orangeCount++;
            }

            CarState cs = {};
            cs.pos = pos;
            cs.vel = Vec(0.0f, 0.0f, 0.27099997f); // Set linear velocity
            cs.angVel = Vec(-0.00061f, 0.0f, 0.0f); // Set angular velocity
            cs.boost = 0.33f;
            cs.rotMat[0] = Vec(4.37E-08, 0.999954041, -0.009587233); // Set forward vector
            cs.rotMat.up = Vec(4.19E-10, 0.009587233, -0.009587233); // Set up vector
            cs.rotMat = Angle(yaw, 0, 0).ToRotMat(); // Set the rotation matrix with pitch, yaw, and roll
            car->SetState(cs);
        }

        return GameState(arena);
    }
}