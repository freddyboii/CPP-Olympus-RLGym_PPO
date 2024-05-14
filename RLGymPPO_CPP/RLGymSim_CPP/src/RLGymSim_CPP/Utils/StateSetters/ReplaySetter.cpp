//ReplaySetter.cpp
#include "ReplaySetter.h"
#include "../../Math.h"
#include "../CommonValues.h"
#include <iostream>
#include <random>

#include "data/ssl_1v1.h"
#include "data/ssl_2v2.h"
#include "data/ssl_3v3.h"

using namespace RLGSC;

ReplaySetter::ReplaySetter(int gameMode) {
    switch (gameMode) {
        case 1:
            numStates = kNumStates1;
            states = reinterpret_cast<const float*>(kStates1);
            stateSize = 35;
            break;
        case 2:
            numStates = kNumStates2;
            states = reinterpret_cast<const float*>(kStates2);
            stateSize = 61;
            break;
        case 3:
            numStates = kNumStates3;
            states = reinterpret_cast<const float*>(kStates3);
            stateSize = 87;
            break;
        default:
            std::cerr << "Invalid game mode: " << gameMode << std::endl;
            break;
    }
}

GameState ReplaySetter::ResetState(Arena* arena) {
    // Choose a random state index
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<size_t> dis(0, numStates - 1);

    size_t randomStateIndex;
    const float* randomState;
    bool success = false;
    int retryCount = 0;

    while (!success && retryCount < 3) {
        try {
            randomStateIndex = dis(gen);
            randomState = states + randomStateIndex * stateSize;

            size_t numCars = (stateSize - 9) / 13;

            size_t offset = 0;
            ballPosition = std::vector<float>(randomState + offset, randomState + offset + 3);
            offset += 3;
            ballLinearVelocity = std::vector<float>(randomState + offset, randomState + offset + 3);
            offset += 3;
            ballAngularVelocity = std::vector<float>(randomState + offset, randomState + offset + 3);
            offset += 3;

            carStatesData.clear();
            for (size_t i = 0; i < numCars; ++i) {
                std::vector<float> carState(randomState + offset, randomState + offset + 13);
                carStatesData.push_back(carState);
                offset += 13;
            }

            BallState bs = {};
            bs.pos = Vec(ballPosition[0], ballPosition[1], ballPosition[2]);
            bs.vel = Vec(ballLinearVelocity[0], ballLinearVelocity[1], ballLinearVelocity[2]);
            bs.angVel = Vec(ballAngularVelocity[0], ballAngularVelocity[1], ballAngularVelocity[2]);
            arena->ball->SetState(bs);

            int carIndex = 0;
            for (Car* car : arena->_cars) {
                const std::vector<float>& carState = carStatesData[carIndex];

                CarState cs = {};
                cs.pos = Vec(carState[0], carState[1], carState[2]);
                cs.vel = Vec(carState[6], carState[7], carState[8]);
                cs.angVel = Vec(carState[9], carState[10], carState[11]);
                cs.boost = carState[12];
                cs.rotMat = Angle(carState[4], carState[3], carState[5]).ToRotMat();

                car->SetState(cs);

                carIndex++;
            }

            success = true;
        } catch (const std::exception& e) {
            std::cerr << "Error reading file: " << e.what() << std::endl;
            retryCount++;
            if (retryCount < 3) {
                std::cerr << "Retrying with a new random state..." << std::endl;
            } else {
                std::cerr << "Maximum number of retries reached. Aborting." << std::endl;
            }
        }
    }

    if (!success) {
        std::cerr << "Failed to load a valid game state." << std::endl;
    }

    return GameState(arena);
}