// DefaultState.h 
#pragma once
#include "StateSetter.h"
#include <vector>

namespace RLGSC {
    class DefaultState : public StateSetter {
    public:
        DefaultState() = default;
        virtual GameState ResetState(Arena* arena) override;

    private:
        static const std::vector<Vec> SPAWN_BLUE_POS;
        static const std::vector<float> SPAWN_BLUE_YAW;
        static const std::vector<Vec> SPAWN_ORANGE_POS;
        static const std::vector<float> SPAWN_ORANGE_YAW;
    };
}