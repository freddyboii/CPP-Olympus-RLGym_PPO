//ReplaySetter.h
#pragma once

#include "StateSetter.h"
#include <vector>

namespace RLGSC {
    class ReplaySetter : public StateSetter {
    public:
        explicit ReplaySetter(int gameMode);
        GameState ResetState(Arena* arena) override;

    private:
        size_t numStates;
        size_t stateSize;
        const float* states;
        std::vector<float> ballPosition;
        std::vector<float> ballLinearVelocity;
        std::vector<float> ballAngularVelocity;
        std::vector<std::vector<float>> carStatesData;
    };
}